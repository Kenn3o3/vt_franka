from __future__ import annotations

import time
from collections import deque
from contextlib import asynccontextmanager
from threading import Event, Lock, Thread

import numpy as np
from fastapi import FastAPI
from scipy.spatial.transform import Rotation

from vt_franka_shared.models import ControllerState, UnityTeleopMessage
from vt_franka_shared.timing import precise_sleep
from vt_franka_shared.transforms import SingleArmCalibration

from ..controller.client import ControllerClient
from ..recording.raw_recorder import JsonlStreamRecorder
from ..settings import TeleopSettings


class QuestTeleopService:
    def __init__(
        self,
        settings: TeleopSettings,
        controller: ControllerClient,
        calibration: SingleArmCalibration,
        quest_message_recorder: JsonlStreamRecorder | None = None,
        command_recorder: JsonlStreamRecorder | None = None,
    ) -> None:
        self.settings = settings
        self.controller = controller
        self.calibration = calibration
        self.quest_message_recorder = quest_message_recorder
        self.command_recorder = command_recorder

        self._running = Event()
        self._message_lock = Lock()
        self._latest_message: UnityTeleopMessage | None = None
        self._loop_thread: Thread | None = None

        self._tracking = False
        self._start_real_tcp: np.ndarray | None = None
        self._start_hand_tcp: np.ndarray | None = None
        self._gripper_closed = False
        self._gripper_force = 0.0
        self._gripper_width_history = deque(maxlen=self.settings.gripper_stability_window)

    def submit_message(self, message: UnityTeleopMessage) -> None:
        with self._message_lock:
            self._latest_message = message
        if self.quest_message_recorder is not None:
            self.quest_message_recorder.record_event(
                {
                    "quest_timestamp": message.timestamp,
                    "source_wall_time": time.time(),
                    "message": message.model_dump(mode="json"),
                }
            )

    def start(self) -> None:
        if self._running.is_set():
            return
        self._running.set()
        self._loop_thread = Thread(target=self._control_loop, name="quest-teleop-loop", daemon=True)
        self._loop_thread.start()

    def stop(self) -> None:
        self._running.clear()
        if self._loop_thread is not None:
            self._loop_thread.join(timeout=2.0)

    def get_gripper_status(self) -> dict[str, bool]:
        stable_closed = self._is_gripper_stable_closed()
        stable_open = self._is_gripper_stable_open()
        return {
            "left_gripper_stable_closed": stable_closed,
            "right_gripper_stable_closed": False,
            "left_gripper_stable_open": stable_open,
            "right_gripper_stable_open": True,
        }

    def _control_loop(self) -> None:
        period = 1.0 / self.settings.loop_hz
        while self._running.is_set():
            state = self.controller.get_state()
            self._update_gripper_state(state)
            message = self._latest_message_copy()
            if message is None:
                precise_sleep(period)
                continue

            hand_pose_robot = self.calibration.unity_to_robot_pose(
                np.asarray(message.leftHand.wristPos + message.leftHand.wristQuat, dtype=np.float64)
            )
            tracking_pressed = self._button_pressed(message, self.settings.tracking_button_index)
            if tracking_pressed and not self._tracking:
                self._tracking = True
                self._start_real_tcp = np.asarray(state.tcp_pose, dtype=np.float64)
                self._start_hand_tcp = hand_pose_robot
            elif not tracking_pressed and self._tracking:
                self._tracking = False
                self.controller.stop_gripper()

            if self._tracking:
                self._handle_gripper_toggle(message)
                target_pose = self._calculate_relative_target(hand_pose_robot)
                current_pose = np.asarray(state.tcp_pose, dtype=np.float64)
                if np.linalg.norm(target_pose[:3] - current_pose[:3]) > self.settings.max_tracking_position_error_m:
                    self._tracking = False
                else:
                    self.controller.queue_tcp(target_pose.tolist(), source="teleop")
                    if self.command_recorder is not None:
                        self.command_recorder.record_event(
                            {
                                "source_wall_time": time.time(),
                                "target_tcp": target_pose.tolist(),
                                "gripper_closed": self._gripper_closed,
                            }
                        )

            precise_sleep(period)

    def _latest_message_copy(self) -> UnityTeleopMessage | None:
        with self._message_lock:
            return self._latest_message.model_copy(deep=True) if self._latest_message is not None else None

    def _handle_gripper_toggle(self, message: UnityTeleopMessage) -> None:
        wants_closed = message.leftHand.triggerState > self.settings.trigger_close_threshold
        if wants_closed and not self._gripper_closed:
            if self.settings.use_force_control_for_gripper:
                self.controller.grasp_gripper(
                    velocity=self.settings.gripper_velocity,
                    force_limit=self.settings.grasp_force,
                    source="teleop",
                )
            else:
                self.controller.move_gripper(
                    width=self.settings.min_gripper_width,
                    velocity=self.settings.gripper_velocity,
                    force_limit=self.settings.grasp_force,
                    source="teleop",
                )
            self._gripper_closed = True
        elif not wants_closed and self._gripper_closed:
            self.controller.move_gripper(
                width=self.settings.max_gripper_width,
                velocity=self.settings.gripper_velocity,
                force_limit=self.settings.grasp_force,
                source="teleop",
            )
            self._gripper_closed = False

    def _calculate_relative_target(self, current_hand_pose: np.ndarray) -> np.ndarray:
        if self._start_real_tcp is None or self._start_hand_tcp is None:
            raise RuntimeError("tracking start pose is not initialized")

        target = np.zeros(7, dtype=np.float64)
        target[:3] = (
            self.settings.relative_translation_scale * (current_hand_pose[:3] - self._start_hand_tcp[:3])
            + self._start_real_tcp[:3]
        )
        current_rotation = Rotation.from_quat(_wxyz_to_xyzw(current_hand_pose[3:]))
        start_rotation = Rotation.from_quat(_wxyz_to_xyzw(self._start_hand_tcp[3:]))
        robot_start_rotation = Rotation.from_quat(_wxyz_to_xyzw(self._start_real_tcp[3:]))
        target_rotation = current_rotation * start_rotation.inv() * robot_start_rotation
        quat_xyzw = target_rotation.as_quat()
        target[3:] = [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]
        return target

    def _update_gripper_state(self, state: ControllerState) -> None:
        self._gripper_force = state.gripper_force
        self._gripper_width_history.append(state.gripper_width)

    def _is_gripper_stable_open(self) -> bool:
        if self._gripper_force >= self.settings.gripper_force_open_threshold:
            return False
        if len(self._gripper_width_history) < self.settings.gripper_stability_window:
            return False
        width_variation = max(self._gripper_width_history) - min(self._gripper_width_history)
        return width_variation < self.settings.gripper_width_vis_precision

    def _is_gripper_stable_closed(self) -> bool:
        if self._gripper_force < self.settings.gripper_force_close_threshold:
            return False
        if len(self._gripper_width_history) < self.settings.gripper_stability_window:
            return False
        width_variation = max(self._gripper_width_history) - min(self._gripper_width_history)
        return width_variation < self.settings.gripper_width_vis_precision

    @staticmethod
    def _button_pressed(message: UnityTeleopMessage, index: int) -> bool:
        if index >= len(message.leftHand.buttonState):
            return False
        return bool(message.leftHand.buttonState[index])


def create_teleop_app(service: QuestTeleopService) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app: FastAPI):
        service.start()
        try:
            yield
        finally:
            service.stop()

    app = FastAPI(title="VT Franka Teleop", version="0.1.0", lifespan=lifespan)

    @app.post("/unity")
    def unity(message: UnityTeleopMessage):
        service.submit_message(message)
        return {"status": "ok"}

    @app.get("/get_current_gripper_state")
    def get_current_gripper_state():
        return service.get_gripper_status()

    return app


def _wxyz_to_xyzw(quaternion_wxyz):
    return np.array([quaternion_wxyz[1], quaternion_wxyz[2], quaternion_wxyz[3], quaternion_wxyz[0]], dtype=np.float64)

