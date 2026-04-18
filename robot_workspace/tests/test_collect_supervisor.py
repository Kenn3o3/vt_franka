from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import pytest

from vt_franka_shared.models import ControllerState
from vt_franka_workspace.collect.supervisor import CollectSupervisor
from vt_franka_workspace.operator import OperatorActionError
from vt_franka_workspace.recording.session import RunSessionManager
from vt_franka_workspace.rollout.live_buffer import LiveSampleBuffer
from vt_franka_workspace.settings import CollectSettings, RgbCameraSettings, WorkspaceSettings


class FakeController:
    def __init__(self):
        self.ready_calls = 0

    def ready(self):
        self.ready_calls += 1


class FakeStateMonitor:
    def __init__(self):
        self.state = ControllerState(
            tcp_pose=[0.4, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0],
            tcp_velocity=[0.0] * 6,
            tcp_wrench=[0.0] * 6,
            joint_positions=[0.0] * 7,
            joint_velocities=[0.0] * 7,
            gripper_width=0.078,
            gripper_force=0.0,
        )

    def is_healthy(self, max_age_sec=2.0):
        del max_age_sec
        return True

    def snapshot(self):
        return {"healthy": True, "age_sec": 0.0, "sample_count": 1, "failure_count": 0, "max_gap_sec": 0.0, "last_error": None}


class FakeTeleopService:
    def __init__(self):
        self.enabled = False

    def set_teleop_enabled(self, enabled: bool):
        self.enabled = enabled

    def has_recent_message(self, timeout_sec: float):
        del timeout_sec
        return True

    def is_teleop_enabled(self) -> bool:
        return self.enabled

    def get_gripper_status(self):
        return {}


class FakeServer:
    def is_alive(self) -> bool:
        return True


def make_supervisor(tmp_path: Path) -> CollectSupervisor:
    settings = WorkspaceSettings(
        recording={"run_root": tmp_path / "runs", "image_format": "jpg"},
        collect=CollectSettings(
            start_countdown_sec=0.0,
            status_print_hz=1000.0,
            require_quest_connection=True,
        ),
        rgb_cameras={"third_person": RgbCameraSettings(stream_name="rgb_third_person")},
        operator_ui={"enabled": False},
    )
    supervisor = CollectSupervisor(
        settings,
        FakeController(),
        calibration=None,
        run_name="fold",
        use_gelsight=False,
    )
    supervisor.sessions = RunSessionManager(tmp_path / "runs")
    supervisor.sessions.start_run("fold")
    supervisor.state_monitor = FakeStateMonitor()
    supervisor.teleop_service = FakeTeleopService()
    supervisor.teleop_server = FakeServer()
    supervisor._rgb_camera_buffers["third_person"] = LiveSampleBuffer("rgb_third_person")
    return supervisor


def test_collect_supervisor_requires_reset_before_start(tmp_path: Path):
    supervisor = make_supervisor(tmp_path)

    with pytest.raises(OperatorActionError):
        supervisor.operator_start_episode()

    supervisor.operator_reset_ready_pose()
    supervisor.operator_start_episode()

    assert supervisor._current_episode_dir is not None
    assert supervisor.teleop_service.is_teleop_enabled() is True


def test_collect_supervisor_freezes_idle_snapshot_when_ready(tmp_path: Path):
    supervisor = make_supervisor(tmp_path)
    supervisor._rgb_camera_buffers["third_person"].update(
        np.zeros((6, 7, 3), dtype=np.uint8),
        metadata={"camera_name": "third_person"},
        captured_wall_time=time.time(),
    )

    supervisor.operator_reset_ready_pose()
    status = supervisor.get_operator_status()

    assert status["ready"] is True
    assert status["snapshots"]["third_person"]["available"] is True
    snapshot = supervisor.get_operator_snapshot("third_person")
    assert snapshot is not None
    assert snapshot.image.shape == (6, 7, 3)
