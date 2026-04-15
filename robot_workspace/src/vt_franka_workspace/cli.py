from __future__ import annotations

import argparse
import logging
import time
from pathlib import Path
from threading import Event

import uvicorn

from vt_franka_shared.config import load_yaml_model
from vt_franka_shared.transforms import SingleArmCalibration

from .collect import CollectSupervisor
from .controller.client import ControllerClient
from .publishers.quest_udp import QuestUdpPublisher
from .publishers.state_bridge import StateBridge
from .recording import JsonlStreamRecorder, align_episode
from .rollout.real_env import RealWorldEnv
from .rollout.real_runner import RealRunner
from .sensors.gelsight.publisher import GelsightPublisher
from .sensors.orbbec import OrbbecRgbRecorder
from .settings import WorkspaceSettings
from .teleop.quest_server import QuestTeleopService, create_teleop_app


def main() -> None:
    parser = argparse.ArgumentParser(description="VT Franka workspace CLI")
    parser.add_argument(
        "command",
        choices=["teleop", "state-bridge", "gelsight", "orbbec", "postprocess", "rollout", "collect"],
    )
    parser.add_argument("--config", default="config/workspace.yaml", help="Path to workspace config YAML")
    parser.add_argument("--run", default=None, help="Run name for collect mode")
    parser.add_argument("--episode-dir", default=None, help="Episode directory for postprocess or replay-policy rollout")
    parser.add_argument("--policy", default=None, help="Policy callable spec module:function for rollout")
    parser.add_argument("--hz", type=float, default=None, help="Rollout control override, replay-policy rate override, or postprocess target rate override")
    parser.add_argument("--speed-scale", type=float, default=1.0, help="Replay-policy speed multiplier")
    parser.add_argument("--skip-gripper", action="store_true", help="Replay-policy TCP only")
    parser.add_argument("--go-ready", action="store_true", help="Move robot to ready pose before rollout")
    parser.add_argument("--go-home", action="store_true", help="Move robot to home pose before rollout")
    parser.add_argument("--with-orbbec", action="store_true", help="Force-enable Orbbec for collect mode")
    parser.add_argument("--without-orbbec", action="store_true", help="Force-disable Orbbec for collect mode")
    parser.add_argument("--with-gelsight", action="store_true", help="Force-enable GelSight for collect mode")
    parser.add_argument("--without-gelsight", action="store_true", help="Force-disable GelSight for collect mode")
    parser.add_argument(
        "--record-quest-messages",
        action="store_true",
        help="Record raw Quest messages in collect mode",
    )
    parser.add_argument(
        "--no-record-quest-messages",
        action="store_true",
        help="Disable raw Quest message recording in collect mode",
    )
    parser.add_argument("--orbbec-record-hz", type=float, default=None, help="Per-episode Orbbec recording rate override")
    parser.add_argument("--gelsight-record-hz", type=float, default=None, help="Per-episode GelSight recording rate override")
    parser.add_argument(
        "--controller-record-hz",
        type=float,
        default=None,
        help="Per-episode controller-state recording rate override",
    )
    parser.add_argument(
        "--teleop-command-record-hz",
        type=float,
        default=None,
        help="Per-episode teleop-command recording rate override",
    )
    parser.add_argument(
        "--quest-message-record-hz",
        type=float,
        default=None,
        help="Per-episode raw Quest message recording rate override",
    )
    parser.add_argument(
        "--allow-without-quest",
        action="store_true",
        help="Allow collect mode to start episodes before a Quest message heartbeat is present",
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    settings = _load_and_resolve_settings(args.config)
    _apply_collect_overrides(settings, args)

    if args.command == "postprocess":
        if args.episode_dir is None:
            raise SystemExit("--episode-dir is required for postprocess")
        target_hz = args.hz if args.hz is not None else settings.recording.postprocess_target_hz
        output = align_episode(args.episode_dir, target_hz=target_hz)
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
        service = QuestTeleopService(
            settings.teleop,
            controller,
            calibration,
        )
        app = create_teleop_app(service)
        uvicorn.run(app, host=settings.teleop.host, port=settings.teleop.port)
        return

    if args.command == "state-bridge":
        bridge = StateBridge(controller, quest_publisher, settings.quest_feedback)
        bridge.start()
        try:
            Event().wait()
        except KeyboardInterrupt:
            bridge.stop()
        return

    if args.command == "gelsight":
        if not settings.gelsight.enabled:
            raise SystemExit("GelSight is disabled in the workspace config")
        publisher = GelsightPublisher(
            settings.gelsight,
            quest_publisher,
            image_format=settings.recording.image_format,
        )
        try:
            publisher.run()
        except KeyboardInterrupt:
            return
        return

    if args.command == "orbbec":
        if not settings.orbbec.enabled:
            raise SystemExit("Orbbec is disabled in the workspace config")
        service = OrbbecRgbRecorder(settings.orbbec, image_format=settings.recording.image_format)
        try:
            service.run()
        except KeyboardInterrupt:
            return
        return

    if args.command == "collect":
        if args.run is None:
            raise SystemExit("--run is required for collect")
        if not settings.recording.enabled:
            raise SystemExit("Collect mode requires recording.enabled=true")
        supervisor = CollectSupervisor(
            settings,
            controller,
            calibration,
            run_name=args.run,
            use_orbbec=settings.orbbec.enabled,
            use_gelsight=settings.gelsight.enabled,
        )
        supervisor.run()
        return

    if args.command == "rollout":
        if args.policy is None:
            raise SystemExit("--policy is required for rollout")
        if args.go_ready and args.go_home:
            raise SystemExit("Cannot use --go-ready and --go-home together")
        if args.go_ready:
            controller.ready()
            time.sleep(1.0)
        if args.go_home:
            controller.home()
            time.sleep(1.0)
        env = RealWorldEnv(controller)
        policy = RealRunner.load_policy(
            args.policy,
            policy_kwargs={
                "episode_dir": args.episode_dir,
                "hz": args.hz,
                "speed_scale": args.speed_scale,
                "skip_gripper": args.skip_gripper,
                "teleop_settings": settings.teleop,
                "workspace_settings": settings,
            },
        )
        control_hz = args.hz if args.hz is not None else getattr(policy, "__vt_franka_control_hz__", settings.rollout.control_hz)
        max_duration_sec = max(
            settings.rollout.max_duration_sec,
            float(getattr(policy, "__vt_franka_max_duration_sec__", settings.rollout.max_duration_sec)),
        )
        runner = RealRunner(env, control_hz, max_duration_sec)
        runner.run(policy)
        return


def _load_and_resolve_settings(config_path: str | Path) -> WorkspaceSettings:
    config_path = Path(config_path).resolve()
    settings = load_yaml_model(config_path, WorkspaceSettings)
    if not settings.calibration.calibration_dir.is_absolute():
        settings.calibration.calibration_dir = (config_path.parent / settings.calibration.calibration_dir).resolve()
    if not settings.recording.run_root.is_absolute():
        settings.recording.run_root = (config_path.parent / settings.recording.run_root).resolve()
    return settings


def _apply_collect_overrides(settings: WorkspaceSettings, args: argparse.Namespace) -> None:
    if args.with_orbbec and args.without_orbbec:
        raise SystemExit("Cannot use --with-orbbec and --without-orbbec together")
    if args.with_gelsight and args.without_gelsight:
        raise SystemExit("Cannot use --with-gelsight and --without-gelsight together")
    if args.record_quest_messages and args.no_record_quest_messages:
        raise SystemExit("Cannot use --record-quest-messages and --no-record-quest-messages together")

    if args.with_orbbec:
        settings.orbbec.enabled = True
    if args.without_orbbec:
        settings.orbbec.enabled = False
    if args.with_gelsight:
        settings.gelsight.enabled = True
    if args.without_gelsight:
        settings.gelsight.enabled = False
    if args.record_quest_messages:
        settings.collect.record_raw_quest_messages = True
    if args.no_record_quest_messages:
        settings.collect.record_raw_quest_messages = False
    if args.allow_without_quest:
        settings.collect.require_quest_connection = False

    for value, setter in (
        (args.orbbec_record_hz, lambda x: setattr(settings.orbbec, "record_hz", x)),
        (args.gelsight_record_hz, lambda x: setattr(settings.gelsight, "record_hz", x)),
        (args.controller_record_hz, lambda x: setattr(settings.quest_feedback, "record_hz", x)),
        (args.teleop_command_record_hz, lambda x: setattr(settings.teleop, "command_record_hz", x)),
        (args.quest_message_record_hz, lambda x: setattr(settings.teleop, "quest_message_record_hz", x)),
    ):
        if value is not None:
            setter(value)
