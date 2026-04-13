from __future__ import annotations

import logging
import threading
import time
from collections import deque
from typing import Deque, Optional

import numpy as np

from vt_franka_shared.interpolation import PoseTrajectoryInterpolator
from vt_franka_shared.models import (
    ControllerState,
    GripperGraspCommand,
    GripperWidthCommand,
    HealthStatus,
    TcpTargetCommand,
)
from vt_franka_shared.pose_math import pose7d_to_pose6d
from vt_franka_shared.timing import precise_wait

from ..settings import ControllerSettings
from ..backends.base import FrankaBackend

LOGGER = logging.getLogger(__name__)


class ControllerService:
    def __init__(self, settings: ControllerSettings, backend: FrankaBackend) -> None:
        self.settings = settings
        self.backend = backend
        self.command_queue: Deque[dict] = deque(maxlen=settings.control.max_queue_size)
        self._state_lock = threading.Lock()
        self._cached_state: Optional[ControllerState] = None
        self._control_thread: Optional[threading.Thread] = None
        self._running = threading.Event()
        self._gripper_lock = threading.Lock()
        self._pose_interp: Optional[PoseTrajectoryInterpolator] = None
        self._last_waypoint_time: Optional[float] = None

    def start(self) -> None:
        if self._running.is_set():
            return
        self._running.set()
        self._control_thread = threading.Thread(target=self._control_loop, name="controller-loop", daemon=True)
        self._control_thread.start()

    def shutdown(self) -> None:
        self._running.clear()
        if self._control_thread is not None:
            self._control_thread.join(timeout=2.0)
        self.backend.shutdown()

    def queue_tcp_command(self, command: TcpTargetCommand) -> None:
        target_pose = pose7d_to_pose6d(command.target_tcp)
        target_time = time.monotonic() + 1.0 / self.settings.control.teleop_command_hz
        self.command_queue.append({"target_pose": target_pose, "target_time": target_time})

    def queue_gripper_width_command(self, command: GripperWidthCommand) -> None:
        def move() -> None:
            with self._gripper_lock:
                self.backend.move_gripper(command.width, command.velocity, command.force_limit)

        threading.Thread(target=move, name="gripper-width", daemon=True).start()

    def queue_gripper_grasp_command(self, command: GripperGraspCommand) -> None:
        def grasp() -> None:
            with self._gripper_lock:
                self.backend.grasp(command.velocity, command.force_limit)

        threading.Thread(target=grasp, name="gripper-grasp", daemon=True).start()

    def stop_gripper(self) -> None:
        self.backend.stop_gripper()

    def go_home(self) -> None:
        self.backend.go_home(
            self.settings.control.home_joint_positions,
            self.settings.control.home_duration_sec,
        )

    def get_state(self) -> ControllerState:
        with self._state_lock:
            if self._cached_state is not None:
                return self._cached_state
        state = self.backend.get_controller_state(self.settings.control.control_frequency_hz)
        with self._state_lock:
            self._cached_state = state
        return state

    def get_health(self) -> HealthStatus:
        with self._state_lock:
            last_state = self._cached_state
        return HealthStatus(
            ok=self._running.is_set(),
            backend=self.backend.name,
            queue_depth=len(self.command_queue),
            control_loop_running=self._running.is_set(),
            last_state_monotonic_time=last_state.monotonic_time if last_state else None,
        )

    def _refresh_state(self) -> None:
        state = self.backend.get_controller_state(self.settings.control.control_frequency_hz)
        with self._state_lock:
            self._cached_state = state

    def _control_loop(self) -> None:
        current_pose = pose7d_to_pose6d(self.backend.get_tcp_pose())
        start_time = time.monotonic()
        self._pose_interp = PoseTrajectoryInterpolator(times=np.array([start_time]), poses=np.array([current_pose]))
        self._last_waypoint_time = start_time
        self.backend.start_cartesian_impedance(
            self.settings.control.cartesian_stiffness,
            self.settings.control.cartesian_damping,
        )

        iteration = 0
        period = 1.0 / self.settings.control.control_frequency_hz
        while self._running.is_set():
            now = time.monotonic()
            target_pose = self._pose_interp(now)
            self.backend.update_desired_tcp(target_pose)
            self._refresh_state()

            try:
                command = self.command_queue.popleft()
            except IndexError:
                command = None

            if command is not None:
                current_time = now + period
                self._pose_interp = self._pose_interp.schedule_waypoint(
                    pose=command["target_pose"],
                    time=command["target_time"],
                    curr_time=current_time,
                    last_waypoint_time=self._last_waypoint_time,
                )
                self._last_waypoint_time = command["target_time"]

            deadline = start_time + (iteration + 1) * period
            precise_wait(deadline, time_func=time.monotonic)
            iteration += 1
