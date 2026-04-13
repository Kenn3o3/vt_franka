from __future__ import annotations

import argparse
import logging

import uvicorn

from vt_franka_shared.config import load_yaml_model

from .api.app import create_app
from .backends.mock import MockFrankaBackend
from .backends.polymetis import PolymetisFrankaBackend
from .control.service import ControllerService
from .settings import ControllerSettings


def build_backend(settings: ControllerSettings):
    if settings.backend.kind == "mock":
        return MockFrankaBackend()
    if settings.backend.kind == "polymetis":
        return PolymetisFrankaBackend(
            robot_ip=settings.backend.robot_ip,
            robot_port=settings.backend.robot_port,
            gripper_ip=settings.backend.gripper_ip,
            gripper_port=settings.backend.gripper_port,
        )
    raise ValueError(f"Unsupported backend kind: {settings.backend.kind}")


def main() -> None:
    parser = argparse.ArgumentParser(description="VT Franka controller CLI")
    parser.add_argument("command", choices=["run", "home"], help="Command to execute")
    parser.add_argument("--config", default="config/controller.yaml", help="Path to YAML config")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    settings = load_yaml_model(args.config, ControllerSettings)
    backend = build_backend(settings)

    if args.command == "home":
        backend.go_home(settings.control.home_joint_positions, settings.control.home_duration_sec)
        backend.shutdown()
        return

    service = ControllerService(settings, backend)
    app = create_app(service)
    uvicorn.run(app, host=settings.server.host, port=settings.server.port)

