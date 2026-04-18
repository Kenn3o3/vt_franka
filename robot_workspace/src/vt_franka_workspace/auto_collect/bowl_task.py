from __future__ import annotations

import logging
import random
import socket
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from threading import Event
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from vt_franka_shared.models import ControllerState
from vt_franka_shared.timing import precise_sleep

from ..collect.controller_state import ControllerStateMonitor
from ..controller.client import ControllerClient
from ..recording import JsonlStreamRecorder, RunSessionManager, align_episode, analyze_episode
from ..sensors.rgb_camera import RgbCameraSpec, build_rgb_camera_recorder, resolve_rgb_camera_specs
from ..settings import WorkspaceSettings
from .state_recorder import ControllerStateRecorderLoop

LOGGER = logging.getLogger(__name__)

_OBJECT_LABEL = "bowl"
_COLLECTION_MODE = "auto_expert_bowl"
_STATE_X_VALUES = (0.03, 0.13, 0.23)
_STATE_Y_VALUES = (0.4, 0.5)
_STATE_YAW_VALUES = (45.0, 135.0, -45.0)
_STATE_Z_M = 0.14
_DIRECTION_TO_DELTA = {
    "front": (0.0, 0.1),
    "back": (0.0, -0.1),
    "right": (0.1, 0.0),
    "left": (-0.1, 0.0),
}
_ROTATION_DELTAS = {
    "clockwise_90": 90.0,
    "anticlockwise_90": -90.0,
    "clockwise_180": 180.0,
    "anticlockwise_180": -180.0,
}
_INSTRUCTION_TEMPLATES = {
    "translate_one": (
        f"Move the {_OBJECT_LABEL} to the {{direction}} 10 cm.",
        f"Shift the {_OBJECT_LABEL} {{direction}} by 10 cm.",
    ),
    "translate_two": (
        f"Move the {_OBJECT_LABEL} to the {{first_direction}} 10 cm and to the {{second_direction}} 10 cm.",
        f"Move the {_OBJECT_LABEL} {{first_direction}} 10 cm, then {{second_direction}} 10 cm.",
    ),
    "rotate": (
        f"Rotate the {_OBJECT_LABEL} {{rotation_direction}} {{rotation_degrees}} degree.",
        f"Turn the {_OBJECT_LABEL} {{rotation_direction}} by {{rotation_degrees}} degree.",
    ),
    "translate_rotate": (
        f"Move the {_OBJECT_LABEL} to the {{direction}} 10 cm and Rotate it {{rotation_direction}} {{rotation_degrees}} degree.",
        f"Move the {_OBJECT_LABEL} {{direction}} 10 cm and then rotate it {{rotation_direction}} by {{rotation_degrees}} degree.",
    ),
}


@dataclass(frozen=True)
class BowlState:
    x: float
    y: float
    yaw_deg: float

    def to_pose7(self, z_m: float, *, roll_deg: float = -180.0, pitch_deg: float = 0.0) -> list[float]:
        return xyz_rpy_deg_to_pose7([self.x, self.y, z_m, roll_deg, pitch_deg, self.yaw_deg])

    def to_metadata(self, *, z_m: float) -> dict[str, float]:
        return {"x": self.x, "y": self.y, "z": z_m, "yaw_deg": self.yaw_deg}


@dataclass(frozen=True)
class BowlInstruction:
    family: str
    text: str
    program: list[dict[str, Any]]


@dataclass(frozen=True)
class BowlEpisodePlan:
    instruction: BowlInstruction
    start_state: BowlState
    goal_state: BowlState


@dataclass(frozen=True)
class TimedCommand:
    target_tcp: list[float]
    gripper_closed: bool
    duration_sec: float
    phase: str


def _round_grid_value(value: float) -> float:
    return round(float(value), 2)


def _normalize_yaw_deg(yaw_deg: float) -> float:
    return float(((float(yaw_deg) + 180.0) % 360.0) - 180.0)


def _yaw_error_deg(yaw_a_deg: float, yaw_b_deg: float) -> float:
    return abs(_normalize_yaw_deg(yaw_a_deg - yaw_b_deg))


def canonical_bowl_seed_state() -> BowlState:
    return BowlState(0.03, 0.4, 45.0)


def iter_valid_bowl_states() -> list[BowlState]:
    return [BowlState(x=x, y=y, yaw_deg=yaw_deg) for yaw_deg in _STATE_YAW_VALUES for y in _STATE_Y_VALUES for x in _STATE_X_VALUES]


def is_valid_bowl_state(state: BowlState) -> bool:
    return (
        _round_grid_value(state.x) in {round(value, 2) for value in _STATE_X_VALUES}
        and _round_grid_value(state.y) in {round(value, 2) for value in _STATE_Y_VALUES}
        and float(state.yaw_deg) in _STATE_YAW_VALUES
    )


def apply_translation(state: BowlState, direction: str) -> BowlState | None:
    dx, dy = _DIRECTION_TO_DELTA[direction]
    candidate = BowlState(
        x=_round_grid_value(state.x + dx),
        y=_round_grid_value(state.y + dy),
        yaw_deg=state.yaw_deg,
    )
    return candidate if is_valid_bowl_state(candidate) else None


def apply_rotation(state: BowlState, rotation_key: str) -> BowlState | None:
    if rotation_key == "clockwise_180" and state.yaw_deg != -45.0:
        return None
    if rotation_key == "anticlockwise_180" and state.yaw_deg != 135.0:
        return None
    candidate = BowlState(
        x=state.x,
        y=state.y,
        yaw_deg=_normalize_yaw_deg(state.yaw_deg + _ROTATION_DELTAS[rotation_key]),
    )
    return candidate if is_valid_bowl_state(candidate) else None


def _rotation_label(rotation_key: str) -> tuple[str, int]:
    return (
        "clockwise" if rotation_key.startswith("clockwise") else "anticlockwise",
        int(abs(_ROTATION_DELTAS[rotation_key])),
    )


def _build_instruction(family: str, template_fields: dict[str, Any], rng: random.Random) -> str:
    template = rng.choice(_INSTRUCTION_TEMPLATES[family])
    return template.format(**template_fields)


def enumerate_bowl_episode_plans(start_state: BowlState, rng: random.Random | None = None) -> list[BowlEpisodePlan]:
    rng = rng or random.Random()
    plans: list[BowlEpisodePlan] = []

    for direction in _DIRECTION_TO_DELTA:
        goal_state = apply_translation(start_state, direction)
        if goal_state is None:
            continue
        plans.append(
            BowlEpisodePlan(
                instruction=BowlInstruction(
                    family="translate_one",
                    text=_build_instruction("translate_one", {"direction": direction}, rng),
                    program=[{"type": "translate", "direction": direction, "distance_m": 0.1}],
                ),
                start_state=start_state,
                goal_state=goal_state,
            )
        )

    for first_direction in _DIRECTION_TO_DELTA:
        intermediate = apply_translation(start_state, first_direction)
        if intermediate is None:
            continue
        for second_direction in _DIRECTION_TO_DELTA:
            goal_state = apply_translation(intermediate, second_direction)
            if goal_state is None:
                continue
            plans.append(
                BowlEpisodePlan(
                    instruction=BowlInstruction(
                        family="translate_two",
                        text=_build_instruction(
                            "translate_two",
                            {"first_direction": first_direction, "second_direction": second_direction},
                            rng,
                        ),
                        program=[
                            {"type": "translate", "direction": first_direction, "distance_m": 0.1},
                            {"type": "translate", "direction": second_direction, "distance_m": 0.1},
                        ],
                    ),
                    start_state=start_state,
                    goal_state=goal_state,
                )
            )

    for rotation_key in _ROTATION_DELTAS:
        goal_state = apply_rotation(start_state, rotation_key)
        if goal_state is None:
            continue
        rotation_direction, rotation_degrees = _rotation_label(rotation_key)
        plans.append(
            BowlEpisodePlan(
                instruction=BowlInstruction(
                    family="rotate",
                    text=_build_instruction(
                        "rotate",
                        {
                            "rotation_direction": rotation_direction,
                            "rotation_degrees": rotation_degrees,
                        },
                        rng,
                    ),
                    program=[{"type": "rotate", "direction": rotation_direction, "degrees": rotation_degrees}],
                ),
                start_state=start_state,
                goal_state=goal_state,
            )
        )

    for direction in _DIRECTION_TO_DELTA:
        translated = apply_translation(start_state, direction)
        if translated is None:
            continue
        for rotation_key in _ROTATION_DELTAS:
            goal_state = apply_rotation(translated, rotation_key)
            if goal_state is None:
                continue
            rotation_direction, rotation_degrees = _rotation_label(rotation_key)
            plans.append(
                BowlEpisodePlan(
                    instruction=BowlInstruction(
                        family="translate_rotate",
                        text=_build_instruction(
                            "translate_rotate",
                            {
                                "direction": direction,
                                "rotation_direction": rotation_direction,
                                "rotation_degrees": rotation_degrees,
                            },
                            rng,
                        ),
                        program=[
                            {"type": "translate", "direction": direction, "distance_m": 0.1},
                            {"type": "rotate", "direction": rotation_direction, "degrees": rotation_degrees},
                        ],
                    ),
                    start_state=start_state,
                    goal_state=goal_state,
                )
            )

    return plans


def sample_bowl_episode_plan(rng: random.Random, start_state: BowlState) -> BowlEpisodePlan:
    plans = enumerate_bowl_episode_plans(start_state, rng)
    if not plans:
        raise RuntimeError(f"No valid {_OBJECT_LABEL} plan can be sampled from start state: {start_state}")
    return rng.choice(plans)


def xyz_rpy_deg_to_pose7(values: list[float]) -> list[float]:
    if len(values) != 6:
        raise ValueError("Expected [x, y, z, roll_deg, pitch_deg, yaw_deg]")
    quat_xyzw = Rotation.from_euler("xyz", values[3:], degrees=True).as_quat()
    return [
        float(values[0]),
        float(values[1]),
        float(values[2]),
        float(quat_xyzw[3]),
        float(quat_xyzw[0]),
        float(quat_xyzw[1]),
        float(quat_xyzw[2]),
    ]


def pose7_to_position_yaw(pose7: list[float]) -> tuple[np.ndarray, float]:
    pose = np.asarray(pose7, dtype=np.float64)
    quat_xyzw = np.array([pose[4], pose[5], pose[6], pose[3]], dtype=np.float64)
    yaw_deg = float(Rotation.from_quat(quat_xyzw).as_euler("xyz", degrees=True)[2])
    return pose[:3], yaw_deg


def controller_state_to_position_yaw(state: ControllerState) -> tuple[np.ndarray, float]:
    return pose7_to_position_yaw(state.tcp_pose)


def _build_linear_segment(
    start_pose: list[float],
    end_pose: list[float],
    *,
    duration_sec: float,
    gripper_closed: bool,
    phase: str,
    command_hz: float,
) -> list[TimedCommand]:
    duration_sec = max(float(duration_sec), 0.0)
    step_count = max(1, int(round(max(duration_sec, 1.0 / max(command_hz, 1e-6)) * command_hz)))
    start_array = np.asarray(start_pose, dtype=np.float64)
    end_array = np.asarray(end_pose, dtype=np.float64)
    start_quat_xyzw = np.array([start_array[4], start_array[5], start_array[6], start_array[3]], dtype=np.float64)
    end_quat_xyzw = np.array([end_array[4], end_array[5], end_array[6], end_array[3]], dtype=np.float64)
    slerp = Slerp([0.0, 1.0], Rotation.from_quat([start_quat_xyzw, end_quat_xyzw]))

    commands: list[TimedCommand] = []
    for step_index in range(step_count):
        alpha = (step_index + 1) / step_count
        pose = (1.0 - alpha) * start_array + alpha * end_array
        interp_quat_xyzw = slerp([alpha]).as_quat()[0]
        pose[3:] = [interp_quat_xyzw[3], interp_quat_xyzw[0], interp_quat_xyzw[1], interp_quat_xyzw[2]]
        commands.append(
            TimedCommand(
                target_tcp=pose.astype(float).tolist(),
                gripper_closed=gripper_closed,
                duration_sec=1.0 / max(command_hz, 1e-6),
                phase=phase,
            )
        )
    return commands


def _build_hold_segment(
    pose: list[float],
    *,
    duration_sec: float,
    gripper_closed: bool,
    phase: str,
    command_hz: float,
) -> list[TimedCommand]:
    duration_sec = max(float(duration_sec), 0.0)
    step_count = max(1, int(round(max(duration_sec, 1.0 / max(command_hz, 1e-6)) * command_hz)))
    return [
        TimedCommand(
            target_tcp=list(pose),
            gripper_closed=gripper_closed,
            duration_sec=1.0 / max(command_hz, 1e-6),
            phase=phase,
        )
        for _ in range(step_count)
    ]


def build_dense_bowl_trajectory(plan: BowlEpisodePlan, settings: WorkspaceSettings) -> list[TimedCommand]:
    auto_settings = settings.auto_collect
    durations = auto_settings.segment_durations_sec
    command_hz = auto_settings.command_hz

    init_pose = xyz_rpy_deg_to_pose7(auto_settings.init_pose_xyz_rpy_deg)
    start_hover = plan.start_state.to_pose7(auto_settings.hover_z_m)
    start_table = plan.start_state.to_pose7(auto_settings.table_z_m)
    start_lift = plan.start_state.to_pose7(auto_settings.lift_z_m)
    goal_hover = plan.goal_state.to_pose7(auto_settings.hover_z_m)
    goal_table = plan.goal_state.to_pose7(auto_settings.table_z_m)

    commands: list[TimedCommand] = []
    commands.extend(
        _build_hold_segment(
            init_pose,
            duration_sec=auto_settings.settle_dwell_sec,
            gripper_closed=False,
            phase="init_hold",
            command_hz=command_hz,
        )
    )
    commands.extend(
        _build_linear_segment(
            init_pose,
            start_hover,
            duration_sec=durations.approach,
            gripper_closed=False,
            phase="approach_hover",
            command_hz=command_hz,
        )
    )
    commands.extend(
        _build_linear_segment(
            start_hover,
            start_table,
            duration_sec=durations.descend,
            gripper_closed=False,
            phase="descend_to_grasp",
            command_hz=command_hz,
        )
    )
    commands.extend(
        _build_hold_segment(
            start_table,
            duration_sec=auto_settings.post_grasp_dwell_sec,
            gripper_closed=True,
            phase="grasp_hold",
            command_hz=command_hz,
        )
    )
    commands.extend(
        _build_linear_segment(
            start_table,
            start_lift,
            duration_sec=durations.lift,
            gripper_closed=True,
            phase="lift_hover",
            command_hz=command_hz,
        )
    )
    commands.extend(
        _build_linear_segment(
            start_lift,
            goal_hover,
            duration_sec=durations.transfer,
            gripper_closed=True,
            phase="transfer_hover",
            command_hz=command_hz,
        )
    )
    commands.extend(
        _build_linear_segment(
            goal_hover,
            goal_table,
            duration_sec=durations.place_descend,
            gripper_closed=True,
            phase="descend_to_place",
            command_hz=command_hz,
        )
    )
    commands.extend(
        _build_hold_segment(
            goal_table,
            duration_sec=auto_settings.post_release_dwell_sec,
            gripper_closed=False,
            phase="release_hold",
            command_hz=command_hz,
        )
    )
    commands.extend(
        _build_linear_segment(
            goal_table,
            goal_hover,
            duration_sec=durations.retreat,
            gripper_closed=False,
            phase="retreat_hover",
            command_hz=command_hz,
        )
    )
    return commands


class AutoCollectBowlRunner:
    def __init__(
        self,
        settings: WorkspaceSettings,
        controller: ControllerClient,
        *,
        run_name: str,
        num_episodes: int,
        seed: int | None = None,
        auto_continue: bool | None = None,
    ) -> None:
        self.settings = settings
        self.controller = controller
        self.run_name = run_name
        self.num_episodes = int(num_episodes)
        self.seed = seed
        self.auto_continue = settings.auto_collect.auto_continue if auto_continue is None else bool(auto_continue)
        self.rng = random.Random(seed)

        self.sessions = RunSessionManager(settings.recording.run_root)
        self.state_monitor = ControllerStateMonitor(
            controller,
            poll_hz=settings.auto_collect.controller_state_poll_hz,
        )
        self.state_recorder = ControllerStateRecorderLoop(
            self.state_monitor,
            JsonlStreamRecorder(self.sessions, "controller_state", record_hz=settings.quest_feedback.record_hz),
            record_hz=settings.auto_collect.controller_state_poll_hz,
            max_age_sec=settings.collect.controller_state_max_age_sec,
        )
        self.command_recorder = JsonlStreamRecorder(
            self.sessions,
            "teleop_commands",
            record_hz=settings.teleop.command_record_hz,
        )
        self.rgb_camera_specs = resolve_rgb_camera_specs(settings.rgb_cameras)
        self.rgb_camera_stop_events: dict[str, Event] = {}
        self.rgb_camera_threads: dict[str, threading.Thread] = {}
        self._current_state = canonical_bowl_seed_state()

    def run(self) -> None:
        if self.num_episodes <= 0:
            raise ValueError("num_episodes must be positive")

        run_dir = self.sessions.start_run(
            self.run_name,
            metadata={
                "workspace_hostname": socket.gethostname(),
                "controller_host": self.settings.controller.host,
                "collection_mode": _COLLECTION_MODE,
                "num_episodes_requested": self.num_episodes,
                "seed": self.seed,
                "config": self.settings.model_dump(mode="json"),
            },
        )
        LOGGER.info("Auto %s collection run started: %s", _OBJECT_LABEL, run_dir)

        completed_episodes = 0
        self.state_monitor.start()
        self.state_recorder.start()
        try:
            self._wait_for_initial_controller_state()
            self._start_rgb_cameras()
            self._move_robot_to_init_pose()
            print(
                f"Place the {_OBJECT_LABEL} at [0.03, 0.4, {_STATE_Z_M}], yaw=45 deg, then press Enter to start auto collection.",
                flush=True,
            )
            input()

            for episode_index in range(self.num_episodes):
                plan = sample_bowl_episode_plan(self.rng, self._current_state)
                LOGGER.info(
                    "Episode %d/%d: %s | start=%s goal=%s",
                    episode_index + 1,
                    self.num_episodes,
                    plan.instruction.text,
                    plan.start_state,
                    plan.goal_state,
                )
                episode_dir = self._run_single_episode(episode_index, plan)
                LOGGER.info("Episode saved: %s", episode_dir)
                completed_episodes += 1
                self._current_state = plan.goal_state

                if episode_index + 1 < self.num_episodes and not self.auto_continue:
                    print("Press Enter to continue to the next auto-collected episode.", flush=True)
                    input()
        finally:
            self._stop_rgb_cameras()
            self.state_recorder.stop()
            self.state_monitor.stop()
            self.sessions.stop_run(metadata_updates={"completed_episodes": completed_episodes})

    def _run_single_episode(self, episode_index: int, plan: BowlEpisodePlan) -> Path:
        self._move_robot_to_init_pose()
        metadata = {
            "collection_mode": _COLLECTION_MODE,
            "instruction": plan.instruction.text,
            "instruction_family": plan.instruction.family,
            "program": plan.instruction.program,
            "start_state": plan.start_state.to_metadata(z_m=self.settings.auto_collect.table_z_m),
            "goal_state": plan.goal_state.to_metadata(z_m=self.settings.auto_collect.table_z_m),
            "episode_chain_source": "seed" if episode_index == 0 else "previous_goal",
        }
        episode_dir = self.sessions.start_episode(name=plan.instruction.family, metadata=metadata)
        try:
            commands = build_dense_bowl_trajectory(plan, self.settings)
            self._stream_timed_commands(commands, record=True)
            saved_episode_dir = self.sessions.stop_episode(outcome="saved")
            if saved_episode_dir is None:
                raise RuntimeError("Episode stop returned no saved episode directory")
            self._finalize_episode(saved_episode_dir)
            return saved_episode_dir
        except Exception:
            self.sessions.stop_episode(outcome="discarded", metadata_updates={"failure": "auto_collect_exception"})
            raise

    def _stream_timed_commands(self, commands: list[TimedCommand], *, record: bool) -> None:
        last_gripper_closed = False
        previous_phase: str | None = None
        for command in commands:
            if command.phase != previous_phase and command.phase in {"grasp_hold", "release_hold"}:
                self._wait_until_pose(command.target_tcp, label=command.phase)

            self.controller.queue_tcp(
                command.target_tcp,
                source="auto_expert",
                target_duration_sec=command.duration_sec,
            )
            event_time = time.time()
            if record:
                self.command_recorder.record_event(
                    {
                        "source_wall_time": event_time,
                        "target_tcp": command.target_tcp,
                        "gripper_closed": command.gripper_closed,
                        "command_source": "auto_expert",
                        "phase": command.phase,
                    },
                    event_time=event_time,
                )

            if command.gripper_closed != last_gripper_closed:
                if command.gripper_closed:
                    self.controller.grasp_gripper(
                        velocity=self.settings.teleop.gripper_velocity,
                        force_limit=self.settings.auto_collect.grasp_force,
                        source="auto_expert",
                    )
                    self._hold_pose_during_gripper_wait(
                        command,
                        duration_sec=self.settings.auto_collect.gripper_close_wait_sec,
                        record=record,
                    )
                    self._warn_if_gripper_still_open()
                else:
                    self.controller.move_gripper(
                        width=self.settings.teleop.max_gripper_width,
                        velocity=self.settings.teleop.gripper_velocity,
                        force_limit=self.settings.auto_collect.grasp_force,
                        source="auto_expert",
                    )
                    self._wait_for_gripper_open(timeout_sec=self.settings.auto_collect.gripper_settle_timeout_sec)
                last_gripper_closed = command.gripper_closed

            precise_sleep(command.duration_sec)
            previous_phase = command.phase

    def _hold_pose_during_gripper_wait(self, command: TimedCommand, *, duration_sec: float, record: bool) -> None:
        step_count = max(1, int(round(duration_sec * self.settings.auto_collect.command_hz)))
        for _ in range(step_count):
            self.controller.queue_tcp(
                command.target_tcp,
                source="auto_expert",
                target_duration_sec=1.0 / max(self.settings.auto_collect.command_hz, 1e-6),
            )
            event_time = time.time()
            if record:
                self.command_recorder.record_event(
                    {
                        "source_wall_time": event_time,
                        "target_tcp": command.target_tcp,
                        "gripper_closed": command.gripper_closed,
                        "command_source": "auto_expert",
                        "phase": f"{command.phase}_gripper_wait",
                    },
                    event_time=event_time,
                )
            precise_sleep(1.0 / max(self.settings.auto_collect.command_hz, 1e-6))

    def _move_robot_to_init_pose(self) -> None:
        init_pose = xyz_rpy_deg_to_pose7(self.settings.auto_collect.init_pose_xyz_rpy_deg)
        current_pose = self._get_current_state().tcp_pose
        self.controller.move_gripper(
            width=self.settings.teleop.max_gripper_width,
            velocity=self.settings.teleop.gripper_velocity,
            force_limit=self.settings.auto_collect.grasp_force,
            source="auto_expert",
        )
        commands = _build_linear_segment(
            current_pose,
            init_pose,
            duration_sec=self.settings.auto_collect.segment_durations_sec.approach,
            gripper_closed=False,
            phase="return_to_init",
            command_hz=self.settings.auto_collect.command_hz,
        )
        self._stream_timed_commands(commands, record=False)
        self._wait_until_pose(
            init_pose,
            label="init_pose",
            timeout_sec=self.settings.auto_collect.pose_settle_timeout_sec,
        )

    def _wait_until_pose(self, target_pose: list[float], *, label: str, timeout_sec: float | None = None) -> None:
        timeout_sec = float(
            timeout_sec
            if timeout_sec is not None
            else self.settings.auto_collect.pose_settle_timeout_sec
        )
        target_position, target_yaw_deg = pose7_to_position_yaw(target_pose)
        deadline = time.time() + timeout_sec
        last_position_error = float("nan")
        last_yaw_error = float("nan")
        while time.time() < deadline:
            state = self._get_current_state()
            current_position, current_yaw_deg = controller_state_to_position_yaw(state)
            last_position_error = float(np.linalg.norm(current_position - target_position))
            last_yaw_error = _yaw_error_deg(current_yaw_deg, target_yaw_deg)
            if (
                last_position_error <= self.settings.auto_collect.pose_tolerance_m
                and last_yaw_error <= self.settings.auto_collect.yaw_tolerance_deg
            ):
                return
            precise_sleep(0.02)
        raise RuntimeError(
            f"Timed out waiting for robot to reach {label}: "
            f"position_error_m={last_position_error:.4f} "
            f"yaw_error_deg={last_yaw_error:.2f} "
            f"tolerances=({self.settings.auto_collect.pose_tolerance_m:.4f}m, "
            f"{self.settings.auto_collect.yaw_tolerance_deg:.1f}deg)"
        )

    def _wait_for_initial_controller_state(self, timeout_sec: float = 5.0) -> ControllerState:
        deadline = time.time() + timeout_sec
        last_error = "Controller state is not available yet"
        while time.time() < deadline:
            state = self.state_monitor.get_state_optional(max_age_sec=self.settings.collect.controller_state_max_age_sec)
            if state is not None:
                return state
            snapshot = self.state_monitor.snapshot()
            if snapshot.get("last_error"):
                last_error = str(snapshot["last_error"])
            precise_sleep(0.05)
        raise RuntimeError(f"Timed out waiting for initial controller state: {last_error}")

    def _get_current_state(self) -> ControllerState:
        state = self.state_monitor.get_state_optional(max_age_sec=self.settings.collect.controller_state_max_age_sec)
        if state is not None:
            return state
        return self._wait_for_initial_controller_state()

    def _warn_if_gripper_still_open(self) -> None:
        state = self._get_current_state()
        still_open_threshold = self.settings.teleop.max_gripper_width - self.settings.auto_collect.gripper_open_tolerance_m
        if state.gripper_width >= still_open_threshold:
            LOGGER.warning(
                "Gripper still appears open after close wait: width=%.4f threshold=%.4f force=%.2f. Continuing after fixed wait.",
                state.gripper_width,
                still_open_threshold,
                state.gripper_force,
            )

    def _wait_for_gripper_open(self, *, timeout_sec: float) -> None:
        deadline = time.time() + timeout_sec
        open_width_threshold = self.settings.teleop.max_gripper_width - self.settings.auto_collect.gripper_open_tolerance_m
        last_width = float("nan")
        while time.time() < deadline:
            state = self._get_current_state()
            last_width = state.gripper_width
            if state.gripper_width >= open_width_threshold:
                return
            precise_sleep(0.02)
        raise RuntimeError(
            "Timed out waiting for gripper to open before retreat: "
            f"width={last_width:.4f} threshold={open_width_threshold:.4f}"
        )

    def _finalize_episode(self, episode_dir: Path) -> None:
        align_episode(episode_dir, target_hz=self.settings.collect.sync_hz)
        analyze_episode(
            episode_dir,
            controller_gap_warn_sec=self.settings.collect.controller_gap_warn_sec,
            controller_gap_fail_sec=self.settings.collect.controller_gap_fail_sec,
        )

    def _start_rgb_cameras(self) -> None:
        for spec in self.rgb_camera_specs:
            service = self._build_rgb_camera_service(spec)
            stop_event = Event()
            thread = threading.Thread(
                target=service.run,
                kwargs={"stop_event": stop_event},
                name=f"auto-collect-rgb-{spec.role}",
                daemon=True,
            )
            self.rgb_camera_stop_events[spec.role] = stop_event
            self.rgb_camera_threads[spec.role] = thread
            thread.start()

    def _build_rgb_camera_service(self, spec: RgbCameraSpec):
        return build_rgb_camera_recorder(
            spec,
            recorder=JsonlStreamRecorder(self.sessions, spec.stream_name, record_hz=spec.settings.record_hz),
            image_format=self.settings.recording.image_format,
        )

    def _stop_rgb_cameras(self) -> None:
        for stop_event in self.rgb_camera_stop_events.values():
            stop_event.set()
        for thread in self.rgb_camera_threads.values():
            thread.join(timeout=2.0)
        self.rgb_camera_stop_events.clear()
        self.rgb_camera_threads.clear()
