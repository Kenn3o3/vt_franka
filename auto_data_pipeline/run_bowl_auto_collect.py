#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the VT Franka bowl auto-collection pipeline")
    parser.add_argument("--config", default="/home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml")
    parser.add_argument("--run", required=True, help="Run name recorded under robot_workspace/data/runs")
    parser.add_argument("--episodes", type=int, required=True, help="Number of auto-collected episodes to record")
    parser.add_argument("--seed", type=int, default=None, help="Optional random seed")
    parser.add_argument("--auto-continue", action="store_true", help="Skip Enter prompts between episodes")
    args = parser.parse_args()

    repo_root = Path("/home/zhenya/kenny/visuotact/vt_franka")
    command = [
        "vt-franka-workspace",
        "auto-collect-bowl",
        "--config",
        str(args.config),
        "--run",
        args.run,
        "--episodes",
        str(args.episodes),
    ]
    if args.seed is not None:
        command.extend(["--seed", str(args.seed)])
    if args.auto_continue:
        command.append("--auto-continue")

    subprocess.run(command, check=True, cwd=repo_root / "robot_workspace")


if __name__ == "__main__":
    main()
