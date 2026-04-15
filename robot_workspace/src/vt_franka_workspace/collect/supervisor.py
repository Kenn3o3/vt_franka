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

import uvicorn

from vt_franka_shared.transforms import SingleArmCalibration

from ..controller.client import ControllerClient
from ..publishers.quest_udp import QuestUdpPublisher
from ..publishers.state_bridge import StateBridge
from ..recording import JsonlStreamRecorder, RunSessionManager, align_episode, analyze_episode
from ..sensors.gelsight.publisher import GelsightPublisher
from ..sensors.orbbec import OrbbecRgbRecorder
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


class _UvicornWorker:
    def __init__(self, app, host: str, port: int) -> None:
        self.name = "teleop_server"
        self.required = True
        self.error: Exception | None = None
        config = uvicorn.Config(app, host=host, port=port, log_level="info", access_log=False)
        self.server = uvicorn.Server(config)
        self.server.install_signal_handlers = lambda: None
        self.thread = threading.Thread(target=self._run, name="teleop-server", daemon=True)

    def start(self, timeout_sec: float = 5.0) -> None:
        self.thread.start()
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self.server.started:
                return
            if self.error is not None:
                raise self.error
            if not self.thread.is_alive():
                raise RuntimeError("Teleop server exited before startup completed")
            time.sleep(0.05)
        raise RuntimeError("Timed out waiting for the teleop server to start")

    def stop(self) -> None:
        self.server.should_exit = True
        self.thread.join(timeout=2.0)

    def is_alive(self) -> bool:
        return self.thread.is_alive()

    def _run(self) -> None:
        try:
            self.server.run()
        except Exception as exc:  # pragma: no cover - thread failure path
            self.error = exc


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
        use_orbbec: bool,
        use_gelsight: bool,
    ) -> None:
        self.settings = settings
        self.controller = controller
        self.calibration = calibration
        self.run_name = run_name
        self.use_orbbec = use_orbbec
        self.use_gelsight = use_gelsight

        self.sessions = RunSessionManager(settings.recording.run_root)
        self.quest_publisher = QuestUdpPublisher(
            quest_ip=settings.quest_feedback.quest_ip,
            robot_state_udp_port=settings.quest_feedback.robot_state_udp_port,
            tactile_udp_port=settings.quest_feedback.tactile_udp_port,
            force_udp_port=settings.quest_feedback.force_udp_port,
            calibration=calibration,
            force_scale_factor=settings.quest_feedback.force_scale_factor,
        )
        self.state_monitor = ControllerStateMonitor(
            controller,
            poll_hz=settings.collect.controller_state_poll_hz,
        )
        self.teleop_service: QuestTeleopService | None = None
        self.teleop_server: _UvicornWorker | None = None
        self.state_bridge: StateBridge | None = None
        self.workers: dict[str, _ThreadWorker] = {}
        self._last_status_print_wall_time = 0.0
        self._status_lock = threading.Lock()

    def run(self) -> None:
        run_dir = self.sessions.start_run(
            self.run_name,
            metadata={
                "workspace_hostname": socket.gethostname(),
                "controller_host": self.settings.controller.host,
                "use_orbbec": self.use_orbbec,
                "use_gelsight": self.use_gelsight,
                "config": self.settings.model_dump(mode="json"),
            },
        )
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
        teleop_app = create_teleop_app(self.teleop_service)
        self.teleop_server = _UvicornWorker(teleop_app, self.settings.teleop.host, self.settings.teleop.port)
        self.teleop_server.start()

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

        if self.use_orbbec:
            recorder = JsonlStreamRecorder(
                self.sessions,
                "orbbec_rgb",
                record_hz=self.settings.orbbec.record_hz,
            )
            service = OrbbecRgbRecorder(
                self.settings.orbbec,
                recorder=recorder,
                image_format=self.settings.recording.image_format,
            )
            self.workers["orbbec"] = self._start_thread_worker(
                "orbbec",
                lambda stop_event: service.run(stop_event=stop_event),
                required=True,
            )

        if self.use_gelsight:
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
        while True:
            self._print_status_if_needed()
            key = key_reader.read_key(0.1)
            if key is None:
                continue
            command = key.lower()
            if command == "r":
                self._handle_start_episode()
            elif command == "e":
                self._handle_stop_episode(outcome="saved")
            elif command == "d":
                self._handle_stop_episode(outcome="discarded")
            elif command == "h":
                self._handle_ready_pose()
            elif command == "q":
                self.sessions.record_operator_event("run_quit_requested")
                break

    def _handle_start_episode(self) -> None:
        ready, reasons = self._is_ready_for_episode()
        if not ready:
            LOGGER.warning("Cannot start episode: %s", "; ".join(reasons))
            return
        if self.sessions.get_active_episode_dir() is not None:
            LOGGER.warning("An episode is already active")
            return

        countdown = self.settings.collect.start_countdown_sec
        self.sessions.record_operator_event("episode_start_requested", {"countdown_sec": countdown})
        if countdown > 0.0:
            LOGGER.info("Starting episode in %.1f seconds", countdown)
            time.sleep(countdown)
        episode_dir = self.sessions.start_episode(
            metadata={
                "use_orbbec": self.use_orbbec,
                "use_gelsight": self.use_gelsight,
                "controller_status": self.state_monitor.snapshot(),
            }
        )
        self.sessions.record_operator_event("episode_started", {"episode_dir": str(episode_dir)})
        LOGGER.info("Episode started: %s", episode_dir)

    def _handle_stop_episode(self, outcome: str) -> None:
        episode_dir = self.sessions.stop_episode(outcome=outcome)
        if episode_dir is None:
            LOGGER.warning("No active episode to stop")
            return
        self.sessions.record_operator_event("episode_stopped", {"episode_dir": str(episode_dir), "outcome": outcome})
        LOGGER.info("Episode %s: %s", outcome, episode_dir)
        if outcome == "saved":
            self._finalize_episode(episode_dir)

    def _handle_ready_pose(self) -> None:
        try:
            self.controller.ready()
        except Exception as exc:
            LOGGER.warning("Failed to move robot to ready pose: %s", exc)
            return
        self.sessions.record_operator_event("ready_pose_requested")
        LOGGER.info("Ready pose requested")

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

    def _is_ready_for_episode(self) -> tuple[bool, list[str]]:
        reasons: list[str] = []
        if self.teleop_server is None or not self.teleop_server.is_alive():
            reasons.append("teleop server is not running")
        if not self.state_monitor.is_healthy(max_age_sec=self.settings.collect.controller_state_max_age_sec):
            reasons.append("controller state is not healthy")
        if self.settings.collect.require_quest_connection and (
            self.teleop_service is None
            or not self.teleop_service.has_recent_message(self.settings.collect.quest_message_timeout_sec)
        ):
            reasons.append("Quest connection is not active")
        for name, worker in self.workers.items():
            if worker.required and worker.error is not None:
                reasons.append(f"{name} failed: {worker.error}")
        return not reasons, reasons

    def _print_status_if_needed(self) -> None:
        now = time.time()
        if now - self._last_status_print_wall_time < 1.0 / max(self.settings.collect.status_print_hz, 1e-6):
            return
        self._last_status_print_wall_time = now

        ready, reasons = self._is_ready_for_episode()
        episode_dir = self.sessions.get_active_episode_dir()
        state_snapshot = self.state_monitor.snapshot()
        quest_connected = self.teleop_service is not None and self.teleop_service.has_recent_message(
            self.settings.collect.quest_message_timeout_sec
        )
        status = {
            "ready": ready,
            "reasons": reasons,
            "active_episode": str(episode_dir) if episode_dir is not None else None,
            "quest_connected": quest_connected,
            "controller_state": state_snapshot,
            "workers": {
                name: {"alive": worker.is_alive(), "error": None if worker.error is None else str(worker.error)}
                for name, worker in self.workers.items()
            },
        }
        self.sessions.write_latest_status(status)
        summary = (
            f"[{'READY' if ready else 'WAIT'}] "
            f"episode={episode_dir.name if episode_dir is not None else '-'} "
            f"quest={'ok' if quest_connected else 'missing'} "
            f"controller_age={state_snapshot['age_sec'] if state_snapshot['age_sec'] is not None else 'n/a'} "
            f"orbbec={'on' if self.use_orbbec else 'off'} "
            f"gelsight={'on' if self.use_gelsight else 'off'}"
        )
        if reasons:
            summary = f"{summary} reasons={'; '.join(reasons)}"
        print(summary, flush=True)

    def _print_banner(self, run_dir: Path) -> None:
        print(f"Run started: {run_dir}", flush=True)
        print("Hotkeys: R=start episode  E=end/save  D=discard  H=ready pose  Q=quit", flush=True)

    def _shutdown(self) -> None:
        active_episode = self.sessions.get_active_episode_dir()
        if active_episode is not None:
            self.sessions.stop_episode(outcome="interrupted")
        self.sessions.record_operator_event("run_stopped")

        for worker in self.workers.values():
            worker.stop_event.set()
        for worker in self.workers.values():
            worker.thread.join(timeout=2.0)

        if self.state_bridge is not None:
            self.state_bridge.stop()
        if self.teleop_server is not None:
            self.teleop_server.stop()
        self.state_monitor.stop()
        self.sessions.stop_run()
