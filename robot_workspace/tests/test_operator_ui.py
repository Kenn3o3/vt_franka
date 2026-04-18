from __future__ import annotations

import numpy as np
from fastapi.testclient import TestClient

from vt_franka_workspace.operator import OperatorActionError, OperatorLogBuffer, create_operator_app
from vt_franka_workspace.operator.control import OperatorSnapshot


class FakeOperatorController:
    def __init__(self):
        self.actions: list[str] = []
        self.allow_start = True
        self.snapshot = OperatorSnapshot(
            name="third_person",
            image=np.zeros((8, 8, 3), dtype=np.uint8),
            captured_wall_time=1.23,
            label="Frozen pre-episode view",
        )

    def get_operator_status(self) -> dict:
        return {
            "mode": "collect",
            "ready": True,
            "reasons": [],
            "active_episode_name": None,
            "next_episode_name": "episode_0001",
            "allowed_actions": {"reset": True, "start": self.allow_start, "stop": False, "discard": False, "quit": True},
            "controller_state": {"age_sec": 0.1},
            "workers": {},
            "snapshots": {"third_person": {"available": True, "token": self.snapshot.token, "label": self.snapshot.label}},
        }

    def get_operator_snapshot(self, name: str):
        return self.snapshot if name == "third_person" else None

    def operator_reset_ready_pose(self) -> None:
        self.actions.append("reset")

    def operator_start_episode(self) -> None:
        if not self.allow_start:
            raise OperatorActionError("start blocked")
        self.actions.append("start")

    def operator_stop_episode(self) -> None:
        self.actions.append("stop")

    def operator_discard_latest_episode(self) -> None:
        self.actions.append("discard")

    def operator_quit(self) -> None:
        self.actions.append("quit")


def test_operator_status_and_actions():
    controller = FakeOperatorController()
    logs = OperatorLogBuffer()
    app = create_operator_app(controller, logs)

    with TestClient(app) as client:
        status = client.get("/operator/api/status")
        assert status.status_code == 200
        assert status.json()["mode"] == "collect"

        action = client.post("/operator/api/actions/start")
        assert action.status_code == 200
        assert controller.actions == ["start"]

        snapshot = client.get("/operator/api/snapshot/third_person")
        assert snapshot.status_code == 200
        assert snapshot.headers["content-type"].startswith("image/")


def test_operator_action_conflict_returns_409():
    controller = FakeOperatorController()
    controller.allow_start = False
    app = create_operator_app(controller, OperatorLogBuffer())

    with TestClient(app) as client:
        response = client.post("/operator/api/actions/start")

    assert response.status_code == 409
    assert "start blocked" in response.json()["detail"]
