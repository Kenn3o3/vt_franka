from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
import pytest

from vt_franka_shared.models import ControllerState
from vt_franka_workspace.auto_collect.bowl_task import (
    AutoCollectBowlRunner,
    BowlState,
    apply_rotation,
    apply_translation,
    build_dense_bowl_trajectory,
    canonical_bowl_seed_state,
    enumerate_bowl_episode_plans,
    sample_bowl_episode_plan,
    xyz_rpy_deg_to_pose7,
)
from vt_franka_workspace.settings import AutoCollectSettings, CollectSettings, RecordingSettings, TeleopSettings, WorkspaceSettings


def _make_state_from_pose(pose7: list[float]) -> ControllerState:
    return ControllerState(
        tcp_pose=list(pose7),
        tcp_velocity=[0.0] * 6,
        tcp_wrench=[0.0] * 6,
        joint_positions=[0.0] * 7,
        joint_velocities=[0.0] * 7,
        gripper_width=0.078,
        gripper_force=0.0,
    )


class FakeController:
    def __init__(self, initial_pose: list[float], *, startup_failures: int = 0):
        self.state = _make_state_from_pose(initial_pose)
        self.tcp_commands: list[tuple[list[float], float | None]] = []
        self.gripper_events: list[tuple[str, float, float]] = []
        self.startup_failures = startup_failures
        self.pending_gripper_action: str | None = None
        self.pending_force_limit = 0.0

    def get_state(self) -> ControllerState:
        if self.startup_failures > 0:
            self.startup_failures -= 1
            raise RuntimeError("Controller state is not available yet")
        if self.pending_gripper_action == "close":
            self.state = self.state.model_copy(update={"gripper_width": 0.0, "gripper_force": self.pending_force_limit})
            self.pending_gripper_action = None
        elif self.pending_gripper_action == "open":
            self.state = self.state.model_copy(update={"gripper_width": 0.078, "gripper_force": 0.0})
            self.pending_gripper_action = None
        self.state = self.state.model_copy(update={"wall_time": time.time(), "monotonic_time": time.monotonic()})
        return self.state

    def queue_tcp(self, target_tcp: list[float], source: str = "workspace", target_duration_sec: float | None = None) -> None:
        del source
        self.tcp_commands.append((list(target_tcp), target_duration_sec))
        self.state = _make_state_from_pose(target_tcp)

    def move_gripper(self, width: float, velocity: float, force_limit: float, source: str = "workspace") -> None:
        del source
        self.gripper_events.append(("open", width, force_limit))
        self.pending_gripper_action = "open"
        self.pending_force_limit = force_limit

    def grasp_gripper(self, velocity: float, force_limit: float, source: str = "workspace") -> None:
        del source, velocity
        self.gripper_events.append(("close", 0.0, force_limit))
        self.pending_gripper_action = "close"
        self.pending_force_limit = force_limit


def make_settings(tmp_path: Path) -> WorkspaceSettings:
    return WorkspaceSettings(
        recording=RecordingSettings(run_root=tmp_path / "runs", image_format="jpg", postprocess_target_hz=10.0),
        collect=CollectSettings(
            sync_hz=10.0,
            controller_state_poll_hz=30.0,
            controller_state_max_age_sec=1.0,
            controller_gap_warn_sec=1.0,
            controller_gap_fail_sec=5.0,
        ),
        rgb_cameras={},
        teleop=TeleopSettings(command_record_hz=0.0, grasp_force=5.0, gripper_velocity=0.1),
        auto_collect=AutoCollectSettings(
            command_hz=10.0,
            controller_state_poll_hz=10.0,
            init_pose_xyz_rpy_deg=[0.03, 0.4, 0.4, -180.0, 0.0, 45.0],
            hover_z_m=0.35,
            lift_z_m=0.24,
            table_z_m=0.14,
            grasp_force=5.0,
            gripper_close_wait_sec=1.8,
            gripper_open_tolerance_m=0.015,
            pose_tolerance_m=0.02,
            yaw_tolerance_deg=15.0,
            settle_dwell_sec=0.1,
            post_grasp_dwell_sec=0.1,
            post_release_dwell_sec=0.1,
        ),
    )


def test_translation_and_rotation_lattice_validity():
    start = canonical_bowl_seed_state()
    assert apply_translation(start, "right") == BowlState(0.13, 0.4, 45.0)
    assert apply_translation(start, "left") is None
    assert apply_rotation(start, "clockwise_90") == BowlState(0.03, 0.4, 135.0)
    assert apply_rotation(start, "anticlockwise_90") == BowlState(0.03, 0.4, -45.0)
    assert apply_rotation(start, "clockwise_180") is None


def test_enumerate_bowl_episode_plans_respects_valid_goals():
    rng = __import__("random").Random(0)
    start = canonical_bowl_seed_state()
    plans = enumerate_bowl_episode_plans(start, rng)
    assert plans
    assert all(plan.start_state == start for plan in plans)
    valid_states = set(iter_valid_state_tuples())
    assert all((plan.goal_state.x, plan.goal_state.y, plan.goal_state.yaw_deg) in valid_states for plan in plans)
    assert any(plan.instruction.family == "translate_rotate" for plan in plans)


def iter_valid_state_tuples():
    for x in (0.03, 0.13, 0.23):
        for y in (0.4, 0.5):
            for yaw_deg in (45.0, 135.0, -45.0):
                yield (x, y, yaw_deg)


def test_build_dense_bowl_trajectory_has_expected_phases(tmp_path: Path):
    settings = make_settings(tmp_path)
    rng = __import__("random").Random(0)
    plan = sample_bowl_episode_plan(rng, canonical_bowl_seed_state())
    commands = build_dense_bowl_trajectory(plan, settings)
    phases = [command.phase for command in commands]
    assert phases[0] == "init_hold"
    assert "grasp_hold" in phases
    assert "release_hold" in phases
    assert phases[-1] == "retreat_hover"
    assert any(command.gripper_closed for command in commands)
    assert any(not command.gripper_closed for command in commands)
    lift_z_values = [command.target_tcp[2] for command in commands if command.phase == "lift_hover"]
    transfer_z_values = [command.target_tcp[2] for command in commands if command.phase == "transfer_hover"]
    assert max(lift_z_values) == pytest.approx(settings.auto_collect.lift_z_m)
    assert min(transfer_z_values) >= min(settings.auto_collect.lift_z_m, settings.auto_collect.hover_z_m)
    assert max(transfer_z_values) <= max(settings.auto_collect.lift_z_m, settings.auto_collect.hover_z_m)


@pytest.mark.parametrize("seed", [0, 7])
def test_auto_collect_runner_saves_one_episode(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, seed: int):
    settings = make_settings(tmp_path)
    controller = FakeController(xyz_rpy_deg_to_pose7(settings.auto_collect.init_pose_xyz_rpy_deg))
    runner = AutoCollectBowlRunner(
        settings,
        controller,
        run_name="bowl_test",
        num_episodes=1,
        seed=seed,
        auto_continue=True,
    )

    monkeypatch.setattr("builtins.input", lambda *args, **kwargs: "")
    runner.run()

    run_dirs = sorted((tmp_path / "runs").glob("bowl_test_*"))
    assert run_dirs
    episode_dir = run_dirs[-1] / "episodes" / "episode_0000"
    assert episode_dir.exists()

    manifest = json.loads((episode_dir / "episode_manifest.json").read_text(encoding="utf-8"))
    assert manifest["metadata"]["collection_mode"] == "auto_expert_bowl"
    assert isinstance(manifest["metadata"]["instruction"], str)
    assert "start_state" in manifest["metadata"]
    assert "goal_state" in manifest["metadata"]
    assert manifest["metadata"]["start_state"]["z"] == pytest.approx(0.14)

    aligned_path = episode_dir / "aligned_episode.npz"
    qc_path = episode_dir / "qc.json"
    teleop_path = episode_dir / "streams" / "teleop_commands.jsonl"
    controller_path = episode_dir / "streams" / "controller_state.jsonl"
    assert aligned_path.exists()
    assert qc_path.exists()
    assert teleop_path.exists()
    assert controller_path.exists()

    aligned = np.load(aligned_path, allow_pickle=True)
    assert aligned["teleop_target_tcp"].shape[1] == 7
    assert aligned["teleop_gripper_closed"].dtype == np.bool_
    assert aligned["timestamps"].size > 0


def test_auto_collect_runner_waits_for_initial_controller_state(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    settings = make_settings(tmp_path)
    controller = FakeController(
        xyz_rpy_deg_to_pose7(settings.auto_collect.init_pose_xyz_rpy_deg),
        startup_failures=2,
    )
    runner = AutoCollectBowlRunner(
        settings,
        controller,
        run_name="bowl_wait",
        num_episodes=1,
        seed=0,
        auto_continue=True,
    )

    monkeypatch.setattr("builtins.input", lambda *args, **kwargs: "")
    runner.run()

    run_dirs = sorted((tmp_path / "runs").glob("bowl_wait_*"))
    assert run_dirs
    assert (run_dirs[-1] / "episodes" / "episode_0000" / "aligned_episode.npz").exists()


def test_auto_collect_runner_waits_for_gripper_close_before_lift(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    settings = make_settings(tmp_path)
    controller = FakeController(xyz_rpy_deg_to_pose7(settings.auto_collect.init_pose_xyz_rpy_deg))
    runner = AutoCollectBowlRunner(
        settings,
        controller,
        run_name="bowl_grasp_wait",
        num_episodes=1,
        seed=0,
        auto_continue=True,
    )

    monkeypatch.setattr("builtins.input", lambda *args, **kwargs: "")
    runner.run()

    assert any(event[0] == "close" for event in controller.gripper_events)
    assert len(controller.tcp_commands) > 0
    assert any(duration == pytest.approx(1.0 / settings.auto_collect.command_hz) for _, duration in controller.tcp_commands)
    assert controller.pending_gripper_action is None
    assert controller.state.gripper_width in (0.0, 0.078)
