from __future__ import annotations

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
from typing import Callable

from vt_franka_shared.transforms import SingleArmCalibration

from ..controller.client import ControllerClient
from ..operator import ManagedUvicornServer, OperatorActionError, OperatorLogBuffer, OperatorSnapshot, create_operator_app
from ..publishers.quest_udp import QuestUdpPublisher
from ..publishers.state_bridge import StateBridge
from ..recording import JsonlStreamRecorder, RunSessionManager, align_episode, analyze_episode
from ..rollout.live_buffer import LiveSampleBuffer
from ..sensors.rgb_camera import build_rgb_camera_recorder, resolve_rgb_camera_specs
from ..settings import WorkspaceSettings
from ..teleop.quest_server import QuestTeleopService, create_teleop_app
from .controller_state import ControllerStateMonitor

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


class CollectSupervisor:
    def __init__(
        self,
        settings: WorkspaceSettings,
        controller: ControllerClient,
        calibration: SingleArmCalibration,
        *,
        run_name: str,
        use_gelsight: bool,
        log_buffer: OperatorLogBuffer | None = None,
    ) -> None:
        self.settings = settings
        self.controller = controller
        self.calibration = calibration
        self.run_name = run_name
        self.use_gelsight = use_gelsight
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
            poll_hz=settings.collect.controller_state_poll_hz,
        )
        self.teleop_service: QuestTeleopService | None = None
        self.teleop_server: ManagedUvicornServer | None = None
        self.operator_server: ManagedUvicornServer | None = None
        self.state_bridge: StateBridge | None = None
        self.workers: dict[str, _ThreadWorker] = {}
        self._last_status_print_wall_time = 0.0
        self._operator_lock = threading.RLock()
        self._quit_requested = threading.Event()
        self._reset_completed = False
        self._current_episode_dir: Path | None = None
        self._latest_saved_episode_dir: Path | None = None
        self._rgb_camera_buffers: dict[str, LiveSampleBuffer] = {}
        self._frozen_rgb_snapshots: dict[str, OperatorSnapshot] = {}

    def run(self) -> None:
        run_dir = self.sessions.start_run(
            self.run_name,
            metadata={
                "workspace_hostname": socket.gethostname(),
                "controller_host": self.settings.controller.host,
                "rgb_cameras": list(self.settings.rgb_cameras),
                "use_gelsight": self.use_gelsight,
                "config": self.settings.model_dump(mode="json"),
            },
        )
        self._latest_saved_episode_dir = self.sessions.get_latest_saved_episode_dir()
        self.sessions.record_operator_event("run_started", {"run_dir": str(run_dir)})
        try:
            self._start_workers()
            self._print_banner(run_dir)
            with _KeyReader() as key_reader:
                try:
                    self._run_event_loop(key_reader)
                except KeyboardInterrupt:
                    LOGGER.info("Collect interrupted by user")
        finally:
            self._shutdown()

    def get_operator_status(self) -> dict:
        with self._operator_lock:
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
                raise OperatorActionError("Cannot quit while recording. Stop/save the active episode first.")
            self.sessions.record_operator_event("run_quit_requested")
            self._quit_requested.set()

    def _start_workers(self) -> None:
        self.state_monitor.start()
        state_provider = lambda: self.state_monitor.get_state(max_age_sec=self.settings.collect.controller_state_max_age_sec)

        quest_recorder = None
        if self.settings.collect.record_raw_quest_messages:
            quest_recorder = JsonlStreamRecorder(
                self.sessions,
                "quest_messages",
                record_hz=self.settings.teleop.quest_message_record_hz,
            )
        command_recorder = JsonlStreamRecorder(
            self.sessions,
            "teleop_commands",
            record_hz=self.settings.teleop.command_record_hz,
        )
        self.teleop_service = QuestTeleopService(
            self.settings.teleop,
            self.controller,
            self.calibration,
            quest_message_recorder=quest_recorder,
            command_recorder=command_recorder,
            state_provider=state_provider,
        )
        self.teleop_service.set_teleop_enabled(False)
        teleop_app = create_teleop_app(self.teleop_service)
        self.teleop_server = ManagedUvicornServer(teleop_app, self.settings.teleop.host, self.settings.teleop.port)
        self.teleop_server.start()

        if self.settings.operator_ui.enabled:
            operator_app = create_operator_app(self, self.log_buffer)
            self.operator_server = ManagedUvicornServer(
                operator_app,
                self.settings.operator_ui.host,
                self.settings.operator_ui.port,
            )
            self.operator_server.start()

        state_recorder = JsonlStreamRecorder(
            self.sessions,
            "controller_state",
            record_hz=self.settings.quest_feedback.record_hz,
        )
        self.state_bridge = StateBridge(
            self.controller,
            self.quest_publisher,
            self.settings.quest_feedback,
            recorder=state_recorder,
            state_provider=state_provider,
        )
        self.state_bridge.start()

        rgb_camera_specs = resolve_rgb_camera_specs(self.settings.rgb_cameras)
        for spec in rgb_camera_specs:
            recorder = JsonlStreamRecorder(
                self.sessions,
                spec.stream_name,
                record_hz=spec.settings.record_hz,
            )
            live_buffer = LiveSampleBuffer(spec.stream_name)
            self._rgb_camera_buffers[spec.role] = live_buffer
            service = build_rgb_camera_recorder(
                spec,
                recorder=recorder,
                live_buffer=live_buffer,
                quest_publisher=self.quest_publisher,
                image_format=self.settings.recording.image_format,
            )
            self.workers[f"rgb_camera:{spec.role}"] = self._start_thread_worker(
                f"rgb_camera:{spec.role}",
                lambda stop_event: service.run(stop_event=stop_event),
                required=True,
            )

        if self.use_gelsight:
            from ..sensors.gelsight.publisher import GelsightPublisher

            marker_recorder = JsonlStreamRecorder(
                self.sessions,
                "gelsight_markers",
                record_hz=self.settings.gelsight.record_hz,
            )
            frame_recorder = JsonlStreamRecorder(
                self.sessions,
                "gelsight_frames",
                record_hz=self.settings.gelsight.record_hz,
            )
            service = GelsightPublisher(
                self.settings.gelsight,
                self.quest_publisher,
                marker_recorder=marker_recorder,
                frame_recorder=frame_recorder,
                image_format=self.settings.recording.image_format,
                gripper_status_provider=self.teleop_service.get_gripper_status,
            )
            self.workers["gelsight"] = self._start_thread_worker(
                "gelsight",
                lambda stop_event: service.run(stop_event=stop_event),
                required=True,
            )

    def _start_thread_worker(
        self,
        name: str,
        target: Callable[[threading.Event], None],
        *,
        required: bool,
    ) -> _ThreadWorker:
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
        except Exception as exc:  # pragma: no cover - thread failure path
            LOGGER.exception("%s worker failed", name)
            self.workers[name].error = exc

    def _run_event_loop(self, key_reader: _KeyReader) -> None:
        while not self._quit_requested.is_set():
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
            episode_dir = self._latest_saved_episode_dir or self.sessions.get_latest_saved_episode_dir()
            if self._current_episode_dir is not None:
                LOGGER.warning("Cannot discard while recording. Press E first.")
                return
            if episode_dir is None:
                LOGGER.warning("No saved episode to discard")
                return
        print(f"Press Enter to confirm discarding {episode_dir.name}, or any other key to cancel.", flush=True)
        key = key_reader.read_key(30.0)
        if key not in ("\n", "\r"):
            LOGGER.info("Discard cancelled")
            return
        self._run_terminal_action(self.operator_discard_latest_episode)

    def _start_episode_locked(self) -> None:
        ready, reasons = self._is_ready_for_episode_locked()
        if not ready:
            raise OperatorActionError(f"Cannot start episode: {'; '.join(reasons)}")
        if self._current_episode_dir is not None:
            raise OperatorActionError("An episode is already active.")
        countdown = self.settings.collect.start_countdown_sec
        self.sessions.record_operator_event("episode_start_requested", {"countdown_sec": countdown})
        if countdown > 0.0:
            LOGGER.info("Starting episode in %.1f seconds", countdown)
            time.sleep(countdown)
        episode_index = self.sessions.get_next_episode_index()
        episode_dir = self.sessions.start_episode(
            name=f"episode_{episode_index:04d}",
            metadata={
                "rgb_cameras": list(self.settings.rgb_cameras),
                "use_gelsight": self.use_gelsight,
                "controller_status": self.state_monitor.snapshot(),
            },
        )
        self._current_episode_dir = episode_dir
        self._reset_completed = False
        self._clear_snapshot_locked()
        if self.teleop_service is not None:
            self.teleop_service.set_teleop_enabled(True)
        self.sessions.record_operator_event("episode_started", {"episode_dir": str(episode_dir)})
        LOGGER.info("Episode started: %s", episode_dir)

    def _stop_episode_locked(self) -> None:
        if self._current_episode_dir is None:
            raise OperatorActionError("No active episode to stop.")
        episode_dir = self.sessions.stop_episode(outcome="saved")
        self._current_episode_dir = None
        self._latest_saved_episode_dir = episode_dir
        self._clear_snapshot_locked()
        if self.teleop_service is not None:
            self.teleop_service.set_teleop_enabled(False)
        self.sessions.record_operator_event("episode_stopped", {"episode_dir": str(episode_dir), "outcome": "saved"})
        LOGGER.info("Episode saved: %s", episode_dir)
        self._finalize_episode(episode_dir)

    def _discard_latest_episode_locked(self) -> None:
        if self._current_episode_dir is not None:
            raise OperatorActionError("Cannot discard while recording. Stop/save the active episode first.")
        episode_dir = self._latest_saved_episode_dir or self.sessions.get_latest_saved_episode_dir()
        if episode_dir is None:
            raise OperatorActionError("No saved episode to discard.")
        self.sessions.discard_episode(episode_dir)
        self.sessions.record_operator_event("episode_discarded", {"episode_dir": str(episode_dir)})
        LOGGER.info("Discarded episode: %s", episode_dir)
        self._latest_saved_episode_dir = self.sessions.get_latest_saved_episode_dir()

    def _reset_ready_pose_locked(self) -> None:
        if self._current_episode_dir is not None:
            raise OperatorActionError("Cannot reset while recording. Stop/save the active episode first.")
        if self.teleop_service is not None:
            self.teleop_service.set_teleop_enabled(False)
        LOGGER.info("Resetting robot pose to controller ready pose")
        try:
            self.controller.ready()
        except Exception as exc:
            raise OperatorActionError(f"Failed to move robot to ready pose: {exc}") from exc
        self._reset_completed = True
        self._clear_snapshot_locked()
        self.sessions.record_operator_event("ready_pose_requested")
        LOGGER.info("Robot pose reset finished. Ready.")

    def _finalize_episode(self, episode_dir: Path) -> None:
        if self.settings.collect.auto_postprocess:
            try:
                aligned = align_episode(episode_dir, target_hz=self.settings.collect.sync_hz)
                LOGGER.info("Aligned episode written to %s", aligned)
            except Exception as exc:
                LOGGER.warning("Postprocess failed for %s: %s", episode_dir, exc)
        if self.settings.collect.auto_qc:
            try:
                qc_path = analyze_episode(
                    episode_dir,
                    controller_gap_warn_sec=self.settings.collect.controller_gap_warn_sec,
                    controller_gap_fail_sec=self.settings.collect.controller_gap_fail_sec,
                )
                LOGGER.info("QC written to %s", qc_path)
            except Exception as exc:
                LOGGER.warning("QC failed for %s: %s", episode_dir, exc)

    def _is_ready_for_episode_locked(self) -> tuple[bool, list[str]]:
        reasons: list[str] = []
        if self.teleop_server is None or not self.teleop_server.is_alive():
            reasons.append("teleop server is not running")
        if not self.state_monitor.is_healthy(max_age_sec=self.settings.collect.controller_state_max_age_sec):
            reasons.append("controller state is not healthy")
        if not self._reset_completed:
            reasons.append("robot pose has not been reset with H")
        if self.settings.collect.require_quest_connection and (
            self.teleop_service is None
            or not self.teleop_service.has_recent_message(self.settings.collect.quest_message_timeout_sec)
        ):
            reasons.append("Quest connection is not active")
        for name, worker in self.workers.items():
            if worker.required and worker.error is not None:
                reasons.append(f"{name} failed: {worker.error}")
        return not reasons, reasons

    def _build_status_locked(self) -> dict:
        ready, reasons = self._is_ready_for_episode_locked()
        self._update_frozen_snapshot_locked(ready=ready)
        next_episode_index = self.sessions.get_next_episode_index()
        state_snapshot = self.state_monitor.snapshot()
        quest_connected = self.teleop_service is not None and self.teleop_service.has_recent_message(
            self.settings.collect.quest_message_timeout_sec
        )
        teleop_enabled = self.teleop_service is not None and self.teleop_service.is_teleop_enabled()
        run_dir = self.sessions.get_active_run_dir()
        status = {
            "mode": "collect",
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
            "quest_connected": quest_connected,
            "teleop_enabled": teleop_enabled,
            "controller_state": state_snapshot,
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
            "rgb_camera_enabled": bool(self._rgb_camera_buffers),
            "rgb_cameras": {role: buffer.snapshot() for role, buffer in self._rgb_camera_buffers.items()},
            "gelsight_enabled": self.use_gelsight,
        }
        return status

    def _snapshot_metadata_locked(self) -> dict[str, dict[str, object]]:
        metadata: dict[str, dict[str, object]] = {}
        for role in self._rgb_camera_buffers:
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
        if self._current_episode_dir is not None or not ready or not self._rgb_camera_buffers:
            self._clear_snapshot_locked()
            return
        for role, buffer in self._rgb_camera_buffers.items():
            if role in self._frozen_rgb_snapshots:
                continue
            sample = buffer.get_latest_optional(max_age_sec=self.settings.operator_ui.snapshot_max_age_sec)
            if sample is None:
                continue
            self._frozen_rgb_snapshots[role] = OperatorSnapshot(
                name=role,
                image=sample.data.copy(),
                captured_wall_time=sample.captured_wall_time,
                label=f"Frozen pre-episode RGB view ({role}): {sample.name}",
                image_format=self.settings.recording.image_format,
            )

    def _print_status_if_needed(self) -> None:
        now = time.time()
        if now - self._last_status_print_wall_time < 1.0 / max(self.settings.collect.status_print_hz, 1e-6):
            return
        self._last_status_print_wall_time = now

        with self._operator_lock:
            status = self._build_status_locked()
        self.sessions.write_latest_status(status)
        summary = (
            f"[{'READY' if status['ready'] else 'WAIT'}] "
            f"next={status['next_episode_name']} "
            f"recording={status['active_episode_name'] or 'off'} "
            f"quest={'ok' if status['quest_connected'] else 'missing'} "
            f"teleop={'on' if status['teleop_enabled'] else 'blocked'} "
            f"controller_age={status['controller_state']['age_sec'] if status['controller_state']['age_sec'] is not None else 'n/a'} "
            f"rgb_cameras={len(self._rgb_camera_buffers)} "
            f"gelsight={'on' if self.use_gelsight else 'off'}"
        )
        if status["reasons"]:
            summary = f"{summary} reasons={'; '.join(status['reasons'])}"
        print(summary, flush=True)

    def _print_banner(self, run_dir: Path) -> None:
        print(f"Run started: {run_dir}", flush=True)
        print(f"Task: {self.run_name}", flush=True)
        print("Checklist:", flush=True)
        print("- Controller PC: launch robot, launch gripper, vt-franka-controller run", flush=True)
        print("- Workspace PC: Quest connected and streaming", flush=True)
        print("- Press H to reset pose before each episode", flush=True)
        print("Hotkeys: H=reset pose  R=start recording  E=end/save  D=discard last saved  Q=quit", flush=True)
        if self.settings.operator_ui.enabled:
            print(
                f"Operator UI: http://{self.settings.operator_ui.host}:{self.settings.operator_ui.port}/operator",
                flush=True,
            )

    def _shutdown(self) -> None:
        if self.teleop_service is not None:
            self.teleop_service.set_teleop_enabled(False)
        self.sessions.record_operator_event("run_stopped")

        for worker in self.workers.values():
            worker.stop_event.set()
        for worker in self.workers.values():
            worker.thread.join(timeout=2.0)

        if self.state_bridge is not None:
            self.state_bridge.stop()
        if self.operator_server is not None:
            self.operator_server.stop()
        if self.teleop_server is not None:
            self.teleop_server.stop()
        self.state_monitor.stop()
        self.sessions.stop_run()
