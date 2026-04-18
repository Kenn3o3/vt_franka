from __future__ import annotations

import json
import logging
import select
import socket
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from vt_franka_shared.timing import precise_sleep
from vt_franka_shared.transforms import SingleArmCalibration

from ..collect.controller_state import ControllerStateMonitor
from ..controller.client import ControllerClient
from ..operator import ManagedUvicornServer, OperatorActionError, OperatorLogBuffer, OperatorSnapshot, create_operator_app
from ..publishers.quest_udp import QuestUdpPublisher
from ..recording import RunSessionManager
from ..recording.raw_recorder import JsonlStreamRecorder
from ..sensors.rgb_camera import build_rgb_camera_recorder, resolve_rgb_camera_specs
from ..settings import WorkspaceSettings
from .live_buffer import LiveSampleBuffer
from .observation import ObservationAssembler
from .real_runner import RealRunner

LOGGER = logging.getLogger(__name__)


@dataclass
class _ThreadWorker:
    name: str
    thread: threading.Thread
    stop_event: threading.Event
    required: bool
    error: Exception | None = None

    def is_alive(self) -> bool:
        return self.thread.is_alive()


class _KeyReader:
    def __init__(self) -> None:
        self._fd: int | None = None
        self._old_attrs = None

    def __enter__(self) -> "_KeyReader":
        if sys.stdin.isatty():
            self._fd = sys.stdin.fileno()
            self._old_attrs = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._fd is not None and self._old_attrs is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_attrs)

    def read_key(self, timeout_sec: float) -> str | None:
        if self._fd is None:
            time.sleep(timeout_sec)
            return None
        ready, _, _ = select.select([sys.stdin], [], [], timeout_sec)
        if not ready:
            return None
        return sys.stdin.read(1)


class GripperStatusEstimator:
    def __init__(self, settings) -> None:
        self.settings = settings
        self._force = 0.0
        self._width_history: list[float] = []

    def update(self, state) -> None:
        self._force = float(state.gripper_force)
        self._width_history.append(float(state.gripper_width))
        if len(self._width_history) > self.settings.gripper_stability_window:
            self._width_history = self._width_history[-self.settings.gripper_stability_window :]

    def get_status(self) -> dict[str, bool]:
        stable_open = False
        stable_closed = False
        if len(self._width_history) >= self.settings.gripper_stability_window:
            width_variation = max(self._width_history) - min(self._width_history)
            stable_open = self._force < self.settings.gripper_force_open_threshold and width_variation < self.settings.gripper_width_vis_precision
            stable_closed = self._force >= self.settings.gripper_force_close_threshold and width_variation < self.settings.gripper_width_vis_precision
        return {
            "left_gripper_stable_closed": stable_closed,
            "right_gripper_stable_closed": False,
            "left_gripper_stable_open": stable_open,
            "right_gripper_stable_open": True,
        }


class RolloutSupervisor:
    def __init__(
        self,
        settings: WorkspaceSettings,
        controller: ControllerClient,
        calibration: SingleArmCalibration,
        *,
        run_name: str,
        policy_spec: str,
        policy_kwargs: dict[str, Any],
        control_hz_override: float | None = None,
        log_buffer: OperatorLogBuffer | None = None,
    ) -> None:
        self.settings = settings
        self.controller = controller
        self.calibration = calibration
        self.run_name = run_name
        self.policy_spec = policy_spec
        self.policy_kwargs = policy_kwargs
        self.control_hz_override = control_hz_override
        self.log_buffer = log_buffer or OperatorLogBuffer(settings.operator_ui.log_buffer_size)

        self.sessions = RunSessionManager(settings.recording.run_root)
        self.quest_publisher = QuestUdpPublisher(
            quest_ip=settings.quest_feedback.quest_ip,
            robot_state_udp_port=settings.quest_feedback.robot_state_udp_port,
            tactile_udp_port=settings.quest_feedback.tactile_udp_port,
            image_udp_port=settings.quest_feedback.image_udp_port,
            force_udp_port=settings.quest_feedback.force_udp_port,
            calibration=calibration,
            force_scale_factor=settings.quest_feedback.force_scale_factor,
        )
        self.state_monitor = ControllerStateMonitor(
            controller,
            poll_hz=max(settings.collect.controller_state_poll_hz, settings.rollout.control_hz),
        )
        self.gripper_status = GripperStatusEstimator(settings.teleop)
        self.rgb_camera_buffers: dict[str, LiveSampleBuffer] = {}
        self.gelsight_marker_buffer: LiveSampleBuffer | None = None
        self.gelsight_frame_buffer: LiveSampleBuffer | None = None
        self.workers: dict[str, _ThreadWorker] = {}
        self.operator_server: ManagedUvicornServer | None = None
        self._current_episode_dir: Path | None = None
        self._latest_saved_episode_dir: Path | None = None
        self._reset_completed = False
        self._last_status_print_wall_time = 0.0
        self._episode_thread: threading.Thread | None = None
        self._episode_stop_event = threading.Event()
        self._episode_error: Exception | None = None
        self._policy_terminated = False
        self._timeout_reached = False
        self._operator_lock = threading.RLock()
        self._quit_requested = threading.Event()
        self._frozen_rgb_snapshots: dict[str, OperatorSnapshot] = {}

        self.policy = RealRunner.load_policy(policy_spec, policy_kwargs=policy_kwargs)
        self.control_hz = control_hz_override if control_hz_override is not None else float(
            getattr(self.policy, "__vt_franka_control_hz__", settings.rollout.control_hz)
        )
        self.max_duration_sec = float(getattr(self.policy, "__vt_franka_max_duration_sec__", settings.rollout.max_duration_sec))
        self.assembler = ObservationAssembler(
            input_settings=settings.rollout.policy.inputs,
            state_provider=self._get_state_for_observation,
            rgb_camera_buffers=self.rgb_camera_buffers,
            gelsight_marker_buffer=self.gelsight_marker_buffer,
            gelsight_frame_buffer=self.gelsight_frame_buffer,
            image_format=settings.recording.image_format,
        )

    def run(self) -> None:
        run_dir = self.sessions.start_run(
            self.run_name,
            metadata={
                "workspace_hostname": socket.gethostname(),
                "controller_host": self.settings.controller.host,
                "policy_spec": self.policy_spec,
                "rollout_config": self.settings.rollout.model_dump(mode="json"),
            },
        )
        self.sessions.record_operator_event("run_started", {"run_dir": str(run_dir), "policy_spec": self.policy_spec})
        self._latest_saved_episode_dir = self.sessions.get_latest_saved_episode_dir()
        try:
            self._start_workers()
            self._print_banner(run_dir)
            with _KeyReader() as key_reader:
                try:
                    self._run_event_loop(key_reader)
                except KeyboardInterrupt:
                    LOGGER.info("Rollout interrupted by user")
        finally:
            self._shutdown()

    def get_operator_status(self) -> dict:
        with self._operator_lock:
            self._poll_episode_status_locked()
            status = self._build_status_locked()
        self.sessions.write_latest_status(status)
        return status

    def get_operator_snapshot(self, name: str) -> OperatorSnapshot | None:
        with self._operator_lock:
            return self._frozen_rgb_snapshots.get(name)

    def operator_reset_ready_pose(self) -> None:
        with self._operator_lock:
            self._reset_ready_pose_locked()

    def operator_start_episode(self) -> None:
        with self._operator_lock:
            self._start_episode_locked()

    def operator_stop_episode(self) -> None:
        with self._operator_lock:
            self._stop_episode_locked()

    def operator_discard_latest_episode(self) -> None:
        with self._operator_lock:
            self._discard_latest_episode_locked()

    def operator_quit(self) -> None:
        with self._operator_lock:
            if self._current_episode_dir is not None:
                raise OperatorActionError("Cannot quit while a rollout episode is active. Stop/save it first.")
            self.sessions.record_operator_event("run_quit_requested")
            self._quit_requested.set()

    def _start_workers(self) -> None:
        self.state_monitor.start()
        if self.settings.operator_ui.enabled:
            operator_app = create_operator_app(self, self.log_buffer)
            self.operator_server = ManagedUvicornServer(
                operator_app,
                self.settings.operator_ui.host,
                self.settings.operator_ui.port,
            )
            self.operator_server.start()
        rgb_camera_specs = {
            spec.role: spec for spec in resolve_rgb_camera_specs(self.settings.rgb_cameras)
        }
        for role in self.settings.rollout.policy.inputs.rgb_cameras:
            if role not in rgb_camera_specs:
                raise RuntimeError(f"rollout policy requested RGB camera role not configured: {role}")
            spec = rgb_camera_specs[role]
            live_buffer = LiveSampleBuffer(spec.stream_name)
            self.rgb_camera_buffers[role] = live_buffer
            self.workers[f"rgb_camera:{role}"] = self._start_thread_worker(
                f"rgb_camera:{role}",
                lambda stop_event, spec=spec, live_buffer=live_buffer: build_rgb_camera_recorder(
                    spec,
                    recorder=None,
                    live_buffer=live_buffer,
                    quest_publisher=self.quest_publisher,
                    image_format=self.settings.recording.image_format,
                ).run(stop_event=stop_event),
                required=True,
            )
        if self.settings.rollout.policy.inputs.gelsight_markers or self.settings.rollout.policy.inputs.gelsight_frame:
            from ..sensors.gelsight.publisher import GelsightPublisher

            self.gelsight_marker_buffer = LiveSampleBuffer("gelsight_markers")
            self.gelsight_frame_buffer = LiveSampleBuffer("gelsight_frame")
            self.workers["gelsight"] = self._start_thread_worker(
                "gelsight",
                lambda stop_event: GelsightPublisher(
                    self.settings.gelsight,
                    self.quest_publisher,
                    marker_recorder=None,
                    frame_recorder=None,
                    marker_buffer=self.gelsight_marker_buffer,
                    frame_buffer=self.gelsight_frame_buffer,
                    image_format=self.settings.recording.image_format,
                    gripper_status_provider=self.gripper_status.get_status,
                ).run(stop_event=stop_event),
                required=True,
            )
        self.assembler = ObservationAssembler(
            input_settings=self.settings.rollout.policy.inputs,
            state_provider=self._get_state_for_observation,
            rgb_camera_buffers=self.rgb_camera_buffers,
            gelsight_marker_buffer=self.gelsight_marker_buffer,
            gelsight_frame_buffer=self.gelsight_frame_buffer,
            image_format=self.settings.recording.image_format,
        )

    def _start_thread_worker(self, name: str, target: Callable[[threading.Event], None], *, required: bool) -> _ThreadWorker:
        stop_event = threading.Event()
        worker = _ThreadWorker(
            name=name,
            thread=threading.Thread(target=lambda: self._run_thread_worker(name, target, stop_event), name=name, daemon=True),
            stop_event=stop_event,
            required=required,
        )
        self.workers[name] = worker
        worker.thread.start()
        time.sleep(0.2)
        return worker

    def _run_thread_worker(self, name: str, target: Callable[[threading.Event], None], stop_event: threading.Event) -> None:
        try:
            target(stop_event)
        except Exception as exc:  # pragma: no cover - worker failure path
            LOGGER.exception("%s worker failed", name)
            self.workers[name].error = exc

    def _run_event_loop(self, key_reader: _KeyReader) -> None:
        while not self._quit_requested.is_set():
            with self._operator_lock:
                self._poll_episode_status_locked()
            self._print_status_if_needed()
            key = key_reader.read_key(0.1)
            if key is None:
                continue
            command = key.lower()
            if command == "r":
                self._run_terminal_action(self.operator_start_episode)
            elif command == "e":
                self._run_terminal_action(self.operator_stop_episode)
            elif command == "d":
                self._handle_terminal_discard(key_reader)
            elif command == "h":
                self._run_terminal_action(self.operator_reset_ready_pose)
            elif command == "q":
                self._run_terminal_action(self.operator_quit)

    def _run_terminal_action(self, action: Callable[[], None]) -> None:
        try:
            action()
        except OperatorActionError as exc:
            LOGGER.warning("%s", exc)

    def _handle_terminal_discard(self, key_reader: _KeyReader) -> None:
        with self._operator_lock:
            self._poll_episode_status_locked()
            episode_dir = self._latest_saved_episode_dir or self.sessions.get_latest_saved_episode_dir()
            if self._current_episode_dir is not None:
                LOGGER.warning("Cannot discard while a rollout episode is active. Press E first.")
                return
            if episode_dir is None:
                LOGGER.warning("No saved rollout episode to discard")
                return
        print(f"Press Enter to confirm discarding {episode_dir.name}, or any other key to cancel.", flush=True)
        key = key_reader.read_key(30.0)
        if key not in ("\n", "\r"):
            LOGGER.info("Discard cancelled")
            return
        self._run_terminal_action(self.operator_discard_latest_episode)

    def _start_episode_locked(self) -> None:
        self._poll_episode_status_locked()
        ready, reasons = self._is_ready_for_episode_locked()
        if not ready:
            raise OperatorActionError(f"Cannot start rollout episode: {'; '.join(reasons)}")
        if self._current_episode_dir is not None:
            raise OperatorActionError("A rollout episode is already active.")
        countdown = self.settings.rollout.start_countdown_sec
        self.sessions.record_operator_event("episode_start_requested", {"countdown_sec": countdown})
        if countdown > 0.0:
            LOGGER.info("Starting rollout episode in %.1f seconds", countdown)
            time.sleep(countdown)
        episode_index = self.sessions.get_next_episode_index()
        episode_dir = self.sessions.start_episode(
            name=f"episode_{episode_index:04d}",
            metadata={
                "policy_spec": self.policy_spec,
                "control_hz": self.control_hz,
                "max_duration_sec": self.max_duration_sec,
            },
        )
        self._current_episode_dir = episode_dir
        self._reset_completed = False
        self._episode_error = None
        self._policy_terminated = False
        self._timeout_reached = False
        self._episode_stop_event = threading.Event()
        self._clear_snapshot_locked()
        if hasattr(self.policy, "reset") and callable(getattr(self.policy, "reset")):
            self.policy.reset()
        self._episode_thread = threading.Thread(target=self._episode_loop, name="rollout-episode", daemon=True)
        self._episode_thread.start()
        self.sessions.record_operator_event("episode_started", {"episode_dir": str(episode_dir)})
        LOGGER.info("Rollout episode started: %s", episode_dir)

    def _stop_episode_locked(self) -> None:
        self._poll_episode_status_locked()
        if self._current_episode_dir is None:
            raise OperatorActionError("No active rollout episode to stop.")
        self._request_episode_stop_locked()
        self._wait_for_episode_finish_locked(manual_stop=True)

    def _request_episode_stop_locked(self) -> None:
        self._episode_stop_event.set()

    def _wait_for_episode_finish_locked(self, manual_stop: bool = False) -> None:
        episode_thread = self._episode_thread
        if episode_thread is not None:
            episode_thread.join(timeout=max(self.max_duration_sec, 5.0))
        self._finalize_current_episode_locked(manual_stop=manual_stop)

    def _poll_episode_status_locked(self) -> None:
        if self._current_episode_dir is None or self._episode_thread is None:
            return
        if self._episode_thread.is_alive():
            return
        self._finalize_current_episode_locked(manual_stop=False)

    def _finalize_current_episode_locked(self, manual_stop: bool) -> None:
        if self._current_episode_dir is None:
            return
        episode_dir = self._current_episode_dir
        self._current_episode_dir = None
        self._episode_thread = None
        self._clear_snapshot_locked()
        if self._episode_error is not None:
            outcome = "failed"
            termination_reason = "error"
        elif self._timeout_reached:
            outcome = "saved"
            termination_reason = "timeout"
        elif self._policy_terminated:
            outcome = "saved"
            termination_reason = "policy_terminate"
        elif manual_stop or self._episode_stop_event.is_set():
            outcome = "saved"
            termination_reason = "manual_stop"
        else:
            outcome = "saved"
            termination_reason = "completed"
        self.sessions.stop_episode(
            outcome=outcome,
            metadata_updates={"termination_reason": termination_reason},
        )
        self._latest_saved_episode_dir = episode_dir if outcome == "saved" else self._latest_saved_episode_dir
        self.sessions.record_operator_event(
            "episode_stopped",
            {"episode_dir": str(episode_dir), "outcome": outcome, "termination_reason": termination_reason},
        )
        if self._episode_error is not None:
            LOGGER.warning("Rollout episode failed: %s", self._episode_error)
        else:
            LOGGER.info("Rollout episode saved: %s (%s)", episode_dir, termination_reason)

    def _discard_latest_episode_locked(self) -> None:
        self._poll_episode_status_locked()
        if self._current_episode_dir is not None:
            raise OperatorActionError("Cannot discard while a rollout episode is active. Stop/save it first.")
        episode_dir = self._latest_saved_episode_dir or self.sessions.get_latest_saved_episode_dir()
        if episode_dir is None:
            raise OperatorActionError("No saved rollout episode to discard.")
        self.sessions.discard_episode(episode_dir)
        self.sessions.record_operator_event("episode_discarded", {"episode_dir": str(episode_dir)})
        LOGGER.info("Discarded rollout episode: %s", episode_dir)
        self._latest_saved_episode_dir = self.sessions.get_latest_saved_episode_dir()

    def _reset_ready_pose_locked(self) -> None:
        self._poll_episode_status_locked()
        if self._current_episode_dir is not None:
            raise OperatorActionError("Cannot reset while a rollout episode is active. Stop/save it first.")
        LOGGER.info("Resetting robot pose to controller ready pose")
        try:
            self.controller.ready()
        except Exception as exc:
            raise OperatorActionError(f"Failed to move robot to ready pose: {exc}") from exc
        self._reset_completed = True
        self._clear_snapshot_locked()
        self.sessions.record_operator_event("ready_pose_requested")
        LOGGER.info("Robot pose reset finished. Ready.")

    def _is_ready_for_episode_locked(self) -> tuple[bool, list[str]]:
        reasons: list[str] = []
        if not self.state_monitor.is_healthy(max_age_sec=self.settings.rollout.policy.inputs.controller_state_max_age_sec):
            reasons.append("controller state is not healthy")
        ready, modality_reasons = self.assembler.assert_ready()
        del ready
        reasons.extend(modality_reasons)
        if not self._reset_completed:
            reasons.append("robot pose has not been reset with H")
        for name, worker in self.workers.items():
            if worker.required and worker.error is not None:
                reasons.append(f"{name} failed: {worker.error}")
        return not reasons, reasons

    def _episode_loop(self) -> None:
        assert self._current_episode_dir is not None
        episode_dir = self._current_episode_dir
        recorder = JsonlStreamRecorder(self.sessions, "policy_steps")
        period = 1.0 / max(self.control_hz, 1e-6)
        start_monotonic = time.monotonic()
        step_index = 0
        while not self._episode_stop_event.is_set():
            step_wall_time = time.time()
            loop_start = time.monotonic()
            elapsed = loop_start - start_monotonic
            if elapsed >= self.max_duration_sec:
                self._timeout_reached = True
                break
            try:
                observation, recorded_observation = self.assembler.assemble(episode_dir, step_index)
                action = self.policy(observation)
                self._execute_action(action)
                recorder.record_event(
                    {
                        "step_index": step_index,
                        "policy_wall_time": step_wall_time,
                        "policy_monotonic_time": loop_start,
                        "episode_elapsed_sec": elapsed,
                        "observation": recorded_observation,
                        "action": _json_safe(action),
                        "timing": {"loop_duration_sec": time.monotonic() - loop_start},
                        "termination_flag": bool(action.get("terminate") is True),
                    },
                    event_time=step_wall_time,
                )
                if action.get("terminate") is True:
                    self._policy_terminated = True
                    break
                step_index += 1
            except Exception as exc:  # pragma: no cover - hardware dependent failure path
                self._episode_error = exc
                LOGGER.exception("Rollout episode iteration failed")
                break
            precise_sleep(max(0.0, period - (time.monotonic() - loop_start)))

    def _execute_action(self, action: dict[str, Any]) -> None:
        target_tcp = action.get("target_tcp")
        if target_tcp is not None:
            self.controller.queue_tcp(list(target_tcp), source="rollout")
        gripper_velocity = float(action.get("gripper_velocity", 0.1))
        gripper_force_limit = float(action.get("gripper_force_limit", 5.0))
        if action.get("gripper_closed") is True:
            self.controller.grasp_gripper(
                velocity=gripper_velocity,
                force_limit=gripper_force_limit,
                source="rollout",
            )
        elif "gripper_width" in action:
            self.controller.move_gripper(
                width=float(action["gripper_width"]),
                velocity=gripper_velocity,
                force_limit=gripper_force_limit,
                source="rollout",
            )

    def _get_state_for_observation(self, max_age_sec: float | None = None):
        state = self.state_monitor.get_state(max_age_sec=max_age_sec)
        self.gripper_status.update(state)
        return state

    def _build_status_locked(self) -> dict:
        ready, reasons = self._is_ready_for_episode_locked()
        self._update_frozen_snapshot_locked(ready=ready)
        next_episode_index = self.sessions.get_next_episode_index()
        run_dir = self.sessions.get_active_run_dir()
        status = {
            "mode": "rollout",
            "run_name": self.run_name,
            "run_dir": None if run_dir is None else str(run_dir),
            "ready": ready,
            "reasons": reasons,
            "active_episode": None if self._current_episode_dir is None else str(self._current_episode_dir),
            "active_episode_name": None if self._current_episode_dir is None else self._current_episode_dir.name,
            "latest_saved_episode": None if self._latest_saved_episode_dir is None else str(self._latest_saved_episode_dir),
            "latest_saved_episode_name": None if self._latest_saved_episode_dir is None else self._latest_saved_episode_dir.name,
            "next_episode_index": next_episode_index,
            "next_episode_name": f"episode_{next_episode_index:04d}",
            "quest_connected": None,
            "teleop_enabled": None,
            "controller_state": self.state_monitor.snapshot(),
            "rgb_cameras": {role: buffer.snapshot() for role, buffer in self.rgb_camera_buffers.items()},
            "gelsight_markers": None if self.gelsight_marker_buffer is None else self.gelsight_marker_buffer.snapshot(),
            "gelsight_frame": None if self.gelsight_frame_buffer is None else self.gelsight_frame_buffer.snapshot(),
            "workers": {
                name: {"alive": worker.is_alive(), "error": None if worker.error is None else str(worker.error)}
                for name, worker in self.workers.items()
            },
            "allowed_actions": {
                "reset": self._current_episode_dir is None,
                "start": ready and self._current_episode_dir is None,
                "stop": self._current_episode_dir is not None,
                "discard": self._current_episode_dir is None and (self._latest_saved_episode_dir or self.sessions.get_latest_saved_episode_dir()) is not None,
                "quit": self._current_episode_dir is None,
            },
            "snapshots": self._snapshot_metadata_locked(),
            "preview_note": None if not self._frozen_rgb_snapshots else "frozen idle RGB snapshots",
        }
        return status

    def _snapshot_metadata_locked(self) -> dict[str, dict[str, object]]:
        metadata: dict[str, dict[str, object]] = {}
        for role in self.rgb_camera_buffers:
            snapshot = self._frozen_rgb_snapshots.get(role)
            if snapshot is None:
                metadata[role] = {"available": False}
                continue
            metadata[role] = {
                "available": True,
                "token": snapshot.token,
                "captured_wall_time": snapshot.captured_wall_time,
                "label": snapshot.label,
            }
        return metadata

    def _clear_snapshot_locked(self) -> None:
        self._frozen_rgb_snapshots.clear()

    def _update_frozen_snapshot_locked(self, *, ready: bool) -> None:
        if self._current_episode_dir is not None or not ready or not self.rgb_camera_buffers:
            self._clear_snapshot_locked()
            return
        for role, buffer in self.rgb_camera_buffers.items():
            if role in self._frozen_rgb_snapshots:
                continue
            sample = buffer.get_latest_optional(max_age_sec=self.settings.operator_ui.snapshot_max_age_sec)
            if sample is None:
                continue
            self._frozen_rgb_snapshots[role] = OperatorSnapshot(
                name=role,
                image=sample.data.copy(),
                captured_wall_time=sample.captured_wall_time,
                label=f"Frozen pre-rollout RGB view ({role}): {sample.name}",
                image_format=self.settings.recording.image_format,
            )

    def _print_status_if_needed(self) -> None:
        now = time.time()
        if now - self._last_status_print_wall_time < 1.0 / max(self.settings.rollout.status_print_hz, 1e-6):
            return
        self._last_status_print_wall_time = now
        with self._operator_lock:
            self._poll_episode_status_locked()
            status = self._build_status_locked()
        self.sessions.write_latest_status(status)
        summary = (
            f"[{'READY' if status['ready'] else 'WAIT'}] "
            f"next={status['next_episode_name']} "
            f"recording={status['active_episode_name'] or 'off'} "
            f"controller_age={status['controller_state']['age_sec'] if status['controller_state']['age_sec'] is not None else 'n/a'} "
            f"rgb_cameras={len(self.rgb_camera_buffers)} "
            f"gelsight={'on' if self.gelsight_marker_buffer is not None or self.gelsight_frame_buffer is not None else 'off'}"
        )
        if status["reasons"]:
            summary = f"{summary} reasons={'; '.join(status['reasons'])}"
        print(summary, flush=True)

    def _print_banner(self, run_dir: Path) -> None:
        print(f"Run started: {run_dir}", flush=True)
        print(f"Task: {self.run_name}", flush=True)
        print(f"Policy: {self.policy_spec}", flush=True)
        print("Checklist:", flush=True)
        print("- Controller PC: launch robot, launch gripper, vt-franka-controller run", flush=True)
        print("- Required policy inputs are producing fresh samples", flush=True)
        print("- Press H to reset pose before each episode", flush=True)
        print("Hotkeys: H=reset pose  R=start rollout  E=end/save  D=discard last saved  Q=quit", flush=True)
        if self.settings.operator_ui.enabled:
            print(
                f"Operator UI: http://{self.settings.operator_ui.host}:{self.settings.operator_ui.port}/operator",
                flush=True,
            )

    def _shutdown(self) -> None:
        with self._operator_lock:
            if self._current_episode_dir is not None:
                self._request_episode_stop_locked()
                self._wait_for_episode_finish_locked(manual_stop=True)
        self.sessions.record_operator_event("run_stopped")
        for worker in self.workers.values():
            worker.stop_event.set()
        for worker in self.workers.values():
            worker.thread.join(timeout=2.0)
        if self.operator_server is not None:
            self.operator_server.stop()
        self.state_monitor.stop()
        self.sessions.stop_run()


def _json_safe(value: Any) -> Any:
    try:
        json.dumps(value)
        return value
    except TypeError:
        if isinstance(value, dict):
            return {str(key): _json_safe(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [_json_safe(item) for item in value]
        return str(value)
