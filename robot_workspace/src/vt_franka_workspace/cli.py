from __future__ import annotations

import argparse
import logging
from pathlib import Path
from threading import Event

import uvicorn

from vt_franka_shared.config import load_yaml_model
from vt_franka_shared.transforms import SingleArmCalibration

from .controller.client import ControllerClient
from .publishers.quest_udp import QuestUdpPublisher
from .publishers.state_bridge import StateBridge
from .recording import EpisodeSessionManager, JsonlStreamRecorder, align_episode
from .rollout.real_env import RealWorldEnv
from .rollout.real_runner import RealRunner
from .sensors.gelsight.publisher import GelsightPublisher
from .settings import WorkspaceSettings
from .teleop.quest_server import QuestTeleopService, create_teleop_app


def main() -> None:
    parser = argparse.ArgumentParser(description="VT Franka workspace CLI")
    parser.add_argument("command", choices=["teleop", "state-bridge", "gelsight", "episode-start", "episode-stop", "postprocess", "rollout"])
    parser.add_argument("--config", default="config/workspace.yaml", help="Path to workspace config YAML")
    parser.add_argument("--name", default=None, help="Optional episode name")
    parser.add_argument("--episode-dir", default=None, help="Episode directory for postprocess")
    parser.add_argument("--policy", default=None, help="Policy callable spec module:function for rollout")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    settings = _load_and_resolve_settings(args.config)
    sessions = EpisodeSessionManager(settings.recording.root_dir)

    if args.command == "episode-start":
        episode_dir = sessions.start_episode(args.name)
        print(episode_dir)
        return
    if args.command == "episode-stop":
        sessions.stop_episode()
        return
    if args.command == "postprocess":
        if args.episode_dir is None:
            raise SystemExit("--episode-dir is required for postprocess")
        output = align_episode(args.episode_dir)
        print(output)
        return

    controller = ControllerClient(
        host=settings.controller.host,
        port=settings.controller.port,
        request_timeout_sec=settings.controller.request_timeout_sec,
    )
    calibration = SingleArmCalibration.from_dir(settings.calibration.calibration_dir)
    quest_publisher = QuestUdpPublisher(
        quest_ip=settings.quest_feedback.quest_ip,
        robot_state_udp_port=settings.quest_feedback.robot_state_udp_port,
        tactile_udp_port=settings.quest_feedback.tactile_udp_port,
        force_udp_port=settings.quest_feedback.force_udp_port,
        calibration=calibration,
        force_scale_factor=settings.quest_feedback.force_scale_factor,
    )

    if args.command == "teleop":
        quest_recorder = JsonlStreamRecorder(sessions, "quest_messages") if settings.recording.enabled else None
        command_recorder = JsonlStreamRecorder(sessions, "teleop_commands") if settings.recording.enabled else None
        service = QuestTeleopService(
            settings.teleop,
            controller,
            calibration,
            quest_message_recorder=quest_recorder,
            command_recorder=command_recorder,
        )
        app = create_teleop_app(service)
        uvicorn.run(app, host=settings.teleop.host, port=settings.teleop.port)
        return

    if args.command == "state-bridge":
        recorder = JsonlStreamRecorder(sessions, "controller_state") if settings.recording.enabled else None
        bridge = StateBridge(controller, quest_publisher, settings.quest_feedback, recorder=recorder)
        bridge.start()
        try:
            Event().wait()
        except KeyboardInterrupt:
            bridge.stop()
        return

    if args.command == "gelsight":
        if not settings.gelsight.enabled:
            raise SystemExit("GelSight is disabled in the workspace config")
        marker_recorder = JsonlStreamRecorder(sessions, "gelsight_markers") if settings.recording.enabled else None
        frame_recorder = JsonlStreamRecorder(sessions, "gelsight_frames") if settings.recording.enabled else None
        publisher = GelsightPublisher(
            settings.gelsight,
            quest_publisher,
            marker_recorder=marker_recorder,
            frame_recorder=frame_recorder,
            image_format=settings.recording.image_format,
        )
        try:
            publisher.run()
        except KeyboardInterrupt:
            return
        return

    if args.command == "rollout":
        if args.policy is None:
            raise SystemExit("--policy is required for rollout")
        env = RealWorldEnv(controller)
        runner = RealRunner(env, settings.rollout.control_hz, settings.rollout.max_duration_sec)
        policy = RealRunner.load_policy(args.policy)
        runner.run(policy)
        return


def _load_and_resolve_settings(config_path: str | Path) -> WorkspaceSettings:
    config_path = Path(config_path).resolve()
    settings = load_yaml_model(config_path, WorkspaceSettings)
    if not settings.calibration.calibration_dir.is_absolute():
        settings.calibration.calibration_dir = (config_path.parent / settings.calibration.calibration_dir).resolve()
    if not settings.recording.root_dir.is_absolute():
        settings.recording.root_dir = (config_path.parent / settings.recording.root_dir).resolve()
    return settings

