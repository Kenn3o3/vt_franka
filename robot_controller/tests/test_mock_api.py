from fastapi.testclient import TestClient

from vt_franka_controller.api.app import create_app
from vt_franka_controller.backends.mock import MockFrankaBackend
from vt_franka_controller.control.service import ControllerService
from vt_franka_controller.settings import BackendSettings, ControlSettings, ControllerSettings, ServerSettings


def test_mock_controller_api_accepts_waypoint_and_reports_state():
    settings = ControllerSettings(
        server=ServerSettings(host="127.0.0.1", port=18092),
        backend=BackendSettings(kind="mock"),
        control=ControlSettings(
            control_frequency_hz=50.0,
            teleop_command_hz=10.0,
            ready_joint_positions=[-0.1, -0.8, 0.0, -2.3, 0.0, 1.9, 0.7],
        ),
    )
    service = ControllerService(settings, MockFrankaBackend())
    app = create_app(service)

    with TestClient(app) as client:
        health = client.get("/api/v1/health")
        assert health.status_code == 200

        response = client.post(
            "/api/v1/commands/tcp",
            json={"target_tcp": [0.4, 0.1, 0.3, 1.0, 0.0, 0.0, 0.0], "source": "test"},
        )
        assert response.status_code == 200

        state = client.get("/api/v1/state")
        assert state.status_code == 200
        assert len(state.json()["tcp_pose"]) == 7

        ready = client.post("/api/v1/actions/ready")
        assert ready.status_code == 200
