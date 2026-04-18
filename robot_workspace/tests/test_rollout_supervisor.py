from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from vt_franka_shared.models import ControllerState
from vt_franka_workspace.operator import OperatorActionError
from vt_franka_workspace.recording.session import RunSessionManager
from vt_franka_workspace.rollout.live_buffer import LiveSampleBuffer
from vt_franka_workspace.rollout.observation import ObservationAssembler
from vt_franka_workspace.rollout.supervisor import GripperStatusEstimator, RolloutSupervisor
from vt_franka_workspace.settings import RgbCameraSettings, RolloutPolicyInputSettings, RolloutPolicySettings, RolloutSettings, TeleopSettings, WorkspaceSettings


class FakeController:
    def __init__(self):
        self.ready_calls = 0
        self.tcp_targets = []
        self.gripper_moves = []
        self.gripper_grasps = []
        self.state = ControllerState(
            tcp_pose=[0.4, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0],
            tcp_velocity=[0.0] * 6,
            tcp_wrench=[0.0] * 6,
            joint_positions=[0.0] * 7,
            joint_velocities=[0.0] * 7,
            gripper_width=0.078,
            gripper_force=0.0,
        )

    def get_state(self):
        return self.state

    def ready(self):
        self.ready_calls += 1

    def queue_tcp(self, target_tcp, source="rollout"):
        self.tcp_targets.append((list(target_tcp), source))

    def move_gripper(self, width, velocity, force_limit, source="rollout"):
        self.gripper_moves.append((width, velocity, force_limit, source))

    def grasp_gripper(self, velocity, force_limit, source="rollout"):
        self.gripper_grasps.append((velocity, force_limit, source))


class FakeStateMonitor:
    def __init__(self, state: ControllerState):
        self.state = state
        self.running = False

    def start(self) -> None:
        self.running = True

    def stop(self) -> None:
        self.running = False

    def get_state(self, max_age_sec=None):
        del max_age_sec
        return self.state

    def is_healthy(self, max_age_sec=2.0):
        del max_age_sec
        return True

    def snapshot(self):
        return {"healthy": True, "age_sec": 0.0, "sample_count": 1, "failure_count": 0, "max_gap_sec": 0.0, "last_error": None}


class CountingPolicy:
    def __init__(self):
        self.reset_calls = 0
        self.call_count = 0

    def reset(self):
        self.reset_calls += 1

    def __call__(self, observation):
        self.call_count += 1
        target = observation["controller_state"]["tcp_pose"]
        return {"target_tcp": target, "gripper_width": observation["controller_state"]["gripper_width"], "terminate": self.call_count >= 2}


def make_settings(tmp_path: Path) -> WorkspaceSettings:
    return WorkspaceSettings(
        recording={"run_root": tmp_path / "runs", "image_format": "jpg"},
        rollout=RolloutSettings(
            control_hz=20.0,
            max_duration_sec=1.0,
            start_countdown_sec=0.0,
            status_print_hz=1000.0,
            policy=RolloutPolicySettings(
                entrypoint="unused",
                inputs=RolloutPolicyInputSettings(controller_state=True),
            ),
        ),
        rgb_cameras={"third_person": RgbCameraSettings(stream_name="rgb_third_person")},
        gelsight={"enabled": True},
    )


def test_observation_assembler_records_exact_step_inputs(tmp_path: Path):
    pytest.importorskip("cv2")
    state = ControllerState(
        tcp_pose=[0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0],
        tcp_velocity=[0.0] * 6,
        tcp_wrench=[0.0] * 6,
        joint_positions=[0.1] * 7,
        joint_velocities=[0.0] * 7,
        gripper_width=0.05,
        gripper_force=1.0,
    )
    rgb_camera = LiveSampleBuffer("rgb_third_person")
    gelsight_markers = LiveSampleBuffer("gelsight_markers")
    gelsight_frame = LiveSampleBuffer("gelsight_frame")
    rgb_camera.update(np.zeros((4, 5, 3), dtype=np.uint8), metadata={"camera_name": "third_person"}, captured_wall_time=1.0)
    gelsight_markers.update(
        {"marker_locations": np.array([[0.1, 0.2]], dtype=np.float64), "marker_offsets": np.array([[0.0, 0.1]], dtype=np.float64)},
        metadata={"camera_name": "gel"},
        captured_wall_time=1.1,
    )
    gelsight_frame.update(np.zeros((3, 2, 3), dtype=np.uint8), metadata={"camera_name": "gel"}, captured_wall_time=1.2)
    assembler = ObservationAssembler(
        input_settings=RolloutPolicyInputSettings(
            controller_state=True,
            rgb_cameras=["third_person"],
            gelsight_markers=True,
            gelsight_frame=True,
            controller_state_max_age_sec=5.0,
            rgb_camera_max_age_sec=5.0,
            gelsight_max_age_sec=5.0,
        ),
        state_provider=lambda max_age_sec=None: state,
        rgb_camera_buffers={"third_person": rgb_camera},
        gelsight_marker_buffer=gelsight_markers,
        gelsight_frame_buffer=gelsight_frame,
        image_format="jpg",
    )
    episode_dir = tmp_path / "episode_0000"
    episode_dir.mkdir(parents=True)

    observation, recorded = assembler.assemble(episode_dir, 0)

    assert observation["controller_state"]["tcp_pose"] == state.tcp_pose
    assert "image" in observation["third_person"]
    assert recorded["third_person"]["frame_path"].startswith("streams/rgb_third_person/")
    assert recorded["gelsight_frame"]["frame_path"].startswith("streams/gelsight_frame/")
    assert recorded["gelsight_markers"]["marker_locations"] == [[0.1, 0.2]]


def test_gripper_status_estimator_derives_stability_from_controller_state():
    estimator = GripperStatusEstimator(
        TeleopSettings(
            gripper_stability_window=3,
            gripper_force_close_threshold=15.0,
            gripper_force_open_threshold=5.0,
            gripper_width_vis_precision=0.001,
        )
    )
    for width in (0.078, 0.0782, 0.0781):
        estimator.update(ControllerState(gripper_width=width, gripper_force=1.0))
    status = estimator.get_status()
    assert status["left_gripper_stable_open"] is True
    assert status["left_gripper_stable_closed"] is False


def test_rollout_supervisor_blocks_start_before_reset(tmp_path: Path, monkeypatch):
    settings = make_settings(tmp_path)
    controller = FakeController()
    policy = CountingPolicy()
    monkeypatch.setattr("vt_franka_workspace.rollout.supervisor.RealRunner.load_policy", lambda *args, **kwargs: policy)
    supervisor = RolloutSupervisor(
        settings,
        controller,
        calibration=None,
        run_name="fold",
        policy_spec="local:policy",
        policy_kwargs={},
    )
    supervisor.sessions = RunSessionManager(tmp_path / "runs")
    supervisor.sessions.start_run("fold")
    supervisor.state_monitor = FakeStateMonitor(controller.state)
    supervisor._start_workers = lambda: None
    supervisor.assembler = ObservationAssembler(
        input_settings=settings.rollout.policy.inputs,
        state_provider=lambda max_age_sec=None: controller.state,
        image_format="jpg",
    )

    with pytest.raises(OperatorActionError):
        supervisor.operator_start_episode()
    assert supervisor._current_episode_dir is None

    supervisor.operator_reset_ready_pose()
    supervisor.operator_start_episode()
    assert supervisor._current_episode_dir is not None
    supervisor._wait_for_episode_finish_locked()
    assert policy.reset_calls == 1


def test_rollout_supervisor_records_policy_steps_and_timeout_reason(tmp_path: Path, monkeypatch):
    settings = make_settings(tmp_path)
    controller = FakeController()

    class TimeoutPolicy:
        def __call__(self, observation):
            del observation
            return {"target_tcp": controller.state.tcp_pose, "gripper_width": controller.state.gripper_width}

    monkeypatch.setattr("vt_franka_workspace.rollout.supervisor.RealRunner.load_policy", lambda *args, **kwargs: TimeoutPolicy())
    supervisor = RolloutSupervisor(
        settings.model_copy(update={"rollout": settings.rollout.model_copy(update={"max_duration_sec": 0.05})}),
        controller,
        calibration=None,
        run_name="fold",
        policy_spec="local:policy",
        policy_kwargs={},
    )
    supervisor.sessions = RunSessionManager(tmp_path / "runs")
    run_dir = supervisor.sessions.start_run("fold")
    supervisor.state_monitor = FakeStateMonitor(controller.state)
    supervisor.assembler = ObservationAssembler(
        input_settings=settings.rollout.policy.inputs,
        state_provider=lambda max_age_sec=None: controller.state,
        image_format="jpg",
    )
    supervisor._reset_completed = True

    supervisor.operator_start_episode()
    supervisor._wait_for_episode_finish_locked()

    episode_dir = run_dir / "episodes" / "episode_0000"
    lines = (episode_dir / "streams" / "policy_steps.jsonl").read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) >= 1
    first = json.loads(lines[0])
    assert first["observation"]["controller_state"]["tcp_pose"] == controller.state.tcp_pose
    manifest = json.loads((episode_dir / "episode_manifest.json").read_text(encoding="utf-8"))
    assert manifest["metadata"]["termination_reason"] == "timeout"


def test_rollout_supervisor_policy_terminate_marks_saved(tmp_path: Path, monkeypatch):
    settings = make_settings(tmp_path)
    controller = FakeController()

    class TerminatingPolicy:
        def __call__(self, observation):
            del observation
            return {"target_tcp": controller.state.tcp_pose, "gripper_width": controller.state.gripper_width, "terminate": True}

    monkeypatch.setattr("vt_franka_workspace.rollout.supervisor.RealRunner.load_policy", lambda *args, **kwargs: TerminatingPolicy())
    supervisor = RolloutSupervisor(
        settings,
        controller,
        calibration=None,
        run_name="fold",
        policy_spec="local:policy",
        policy_kwargs={},
    )
    supervisor.sessions = RunSessionManager(tmp_path / "runs")
    run_dir = supervisor.sessions.start_run("fold")
    supervisor.state_monitor = FakeStateMonitor(controller.state)
    supervisor.assembler = ObservationAssembler(
        input_settings=settings.rollout.policy.inputs,
        state_provider=lambda max_age_sec=None: controller.state,
        image_format="jpg",
    )
    supervisor._reset_completed = True

    supervisor.operator_start_episode()
    supervisor._wait_for_episode_finish_locked()

    manifest = json.loads((run_dir / "episodes" / "episode_0000" / "episode_manifest.json").read_text(encoding="utf-8"))
    assert manifest["metadata"]["termination_reason"] == "policy_terminate"
