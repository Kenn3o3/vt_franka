from __future__ import annotations

import time
from typing import Any

import requests

from vt_franka_shared.models import ControllerState, GripperGraspCommand, GripperWidthCommand, TcpTargetCommand


class ControllerClient:
    def __init__(self, host: str, port: int, request_timeout_sec: float = 0.2) -> None:
        self.base_url = f"http://{host}:{port}"
        self.request_timeout_sec = request_timeout_sec
        self.session = requests.Session()

    def health(self) -> dict[str, Any]:
        return self._get_json("/api/v1/health")

    def get_state(self) -> ControllerState:
        data = self._get_json("/api/v1/state")
        return ControllerState.model_validate(data)

    def queue_tcp(self, target_tcp: list[float], source: str = "workspace") -> None:
        command = TcpTargetCommand(target_tcp=target_tcp, source=source)
        self._post_json("/api/v1/commands/tcp", command.model_dump(mode="json"))

    def move_gripper(self, width: float, velocity: float, force_limit: float, source: str = "workspace") -> None:
        command = GripperWidthCommand(width=width, velocity=velocity, force_limit=force_limit, source=source)
        self._post_json("/api/v1/commands/gripper/width", command.model_dump(mode="json"))

    def grasp_gripper(self, velocity: float, force_limit: float, source: str = "workspace") -> None:
        command = GripperGraspCommand(velocity=velocity, force_limit=force_limit, source=source)
        self._post_json("/api/v1/commands/gripper/grasp", command.model_dump(mode="json"))

    def stop_gripper(self) -> None:
        self._post_json("/stop_gripper/left", {})

    def home(self) -> None:
        self._post_json("/api/v1/actions/home", {})

    def _get_json(self, path: str) -> dict[str, Any]:
        response = self.session.get(f"{self.base_url}{path}", timeout=self.request_timeout_sec)
        response.raise_for_status()
        return response.json()

    def _post_json(self, path: str, payload: dict[str, Any]) -> dict[str, Any]:
        response = self.session.post(f"{self.base_url}{path}", json=payload, timeout=self.request_timeout_sec)
        response.raise_for_status()
        return response.json()

