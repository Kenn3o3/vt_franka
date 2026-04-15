from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from vt_franka_shared.timing import precise_sleep

from .controller.client import ControllerClient
from .settings import TeleopSettings


@dataclass
class ReplayEpisode:
    timestamps: np.ndarray
    target_tcp: np.ndarray
    gripper_closed: np.ndarray
    hz: float


def load_replay_episode(path: str | Path) -> ReplayEpisode:
    path = Path(path)
    if path.is_dir():
        path = path / "aligned_episode.npz"
    if not path.exists():
        raise FileNotFoundError(f"Replay source does not exist: {path}")

    data = np.load(path, allow_pickle=True)
    timestamps = np.asarray(data["timestamps"], dtype=np.float64)
    target_tcp = np.asarray(data["teleop_target_tcp"], dtype=np.float64)
    if target_tcp.ndim != 2 or target_tcp.shape[1] != 7:
        raise RuntimeError(f"Expected teleop_target_tcp to have shape (N, 7), got {target_tcp.shape}")

    if "teleop_gripper_closed" in data:
        gripper_closed = np.asarray(data["teleop_gripper_closed"], dtype=bool)
    else:
        gripper_closed = np.zeros((len(timestamps),), dtype=bool)

    hz = infer_sync_hz(timestamps)
    return ReplayEpisode(
        timestamps=timestamps,
        target_tcp=target_tcp,
        gripper_closed=gripper_closed,
        hz=hz,
    )


def infer_sync_hz(timestamps: np.ndarray) -> float:
    if len(timestamps) < 2:
        return 0.0
    deltas = np.diff(timestamps)
    positive = deltas[deltas > 1e-6]
    if len(positive) == 0:
        return 0.0
    return float(1.0 / np.median(positive))


def replay_episode(
    controller: ControllerClient,
    episode: ReplayEpisode,
    *,
    teleop_settings: TeleopSettings,
    hz: float | None = None,
    speed_scale: float = 1.0,
    skip_gripper: bool = False,
) -> None:
    if speed_scale <= 0.0:
        raise ValueError("speed_scale must be positive")
    effective_hz = hz if hz is not None and hz > 0.0 else episode.hz
    if effective_hz <= 0.0:
        raise ValueError("Replay frequency could not be inferred; pass --hz explicitly")

    period = 1.0 / effective_hz / speed_scale
    last_gripper_closed: bool | None = None

    for target_tcp, gripper_closed in zip(episode.target_tcp, episode.gripper_closed):
        loop_start = time.time()
        controller.queue_tcp(list(map(float, target_tcp)), source="replay")
        if not skip_gripper:
            gripper_closed = bool(gripper_closed)
            if last_gripper_closed is None or gripper_closed != last_gripper_closed:
                if gripper_closed:
                    controller.grasp_gripper(
                        velocity=teleop_settings.gripper_velocity,
                        force_limit=teleop_settings.grasp_force,
                        source="replay",
                    )
                else:
                    controller.move_gripper(
                        width=teleop_settings.max_gripper_width,
                        velocity=teleop_settings.gripper_velocity,
                        force_limit=teleop_settings.grasp_force,
                        source="replay",
                    )
                last_gripper_closed = gripper_closed
        precise_sleep(max(0.0, period - (time.time() - loop_start)))


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Replay a synchronized aligned episode on the robot")
    parser.add_argument("--episode-dir", required=True, help="Episode directory or aligned_episode.npz path")
    parser.add_argument("--controller-host", default="127.0.0.1", help="Controller API host")
    parser.add_argument("--controller-port", type=int, default=8092, help="Controller API port")
    parser.add_argument("--request-timeout-sec", type=float, default=2.0, help="Controller request timeout")
    parser.add_argument("--hz", type=float, default=None, help="Replay rate override. Defaults to aligned timestamps.")
    parser.add_argument("--speed-scale", type=float, default=1.0, help="Replay speed multiplier")
    parser.add_argument("--skip-gripper", action="store_true", help="Replay TCP only")
    parser.add_argument("--go-ready", action="store_true", help="Move robot to ready pose before replay")
    parser.add_argument("--go-home", action="store_true", help="Move robot to home pose before replay")
    return parser


def main(argv: list[str] | None = None) -> None:
    parser = build_argument_parser()
    args = parser.parse_args(argv)

    controller = ControllerClient(
        host=args.controller_host,
        port=args.controller_port,
        request_timeout_sec=args.request_timeout_sec,
    )
    episode = load_replay_episode(args.episode_dir)
    if args.go_ready and args.go_home:
        raise SystemExit("Cannot use --go-ready and --go-home together")
    if args.go_ready:
        controller.ready()
        time.sleep(1.0)
    if args.go_home:
        controller.home()
        time.sleep(1.0)
    replay_episode(
        controller,
        episode,
        teleop_settings=TeleopSettings(),
        hz=args.hz,
        speed_scale=args.speed_scale,
        skip_gripper=args.skip_gripper,
    )


if __name__ == "__main__":
    main()
