from __future__ import annotations

import logging
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from vt_franka_shared.models import ControllerState
from vt_franka_shared.pose_math import xyzw_to_wxyz

from .base import FrankaBackend

LOGGER = logging.getLogger(__name__)


class PolymetisFrankaBackend(FrankaBackend):
    name = "polymetis"

    def __init__(self, robot_ip: str, robot_port: int, gripper_ip: str, gripper_port: int) -> None:
        try:
            import torch
            from polymetis import GripperInterface, RobotInterface
        except ImportError as exc:
            raise RuntimeError("Polymetis backend requires the polymetis Python package") from exc

        self._torch = torch
        self._robot = RobotInterface(ip_address=robot_ip, port=robot_port)
        self._gripper = GripperInterface(ip_address=gripper_ip, port=gripper_port)

    def get_tcp_pose(self) -> np.ndarray:
        position, quaternion_xyzw = self._robot.get_ee_pose()
        pos = position.numpy()
        quat = xyzw_to_wxyz(quaternion_xyzw.numpy())
        return np.concatenate([pos, quat])

    def _get_base_to_flange_rotation_matrix(self):
        joint_positions = self._robot.get_joint_positions()
        _, quat = self._robot.robot_model.forward_kinematics(joint_positions)
        return self._torch.from_numpy(Rotation.from_quat(quat.numpy()).as_matrix()).to(joint_positions.dtype)

    def _get_tcp_velocity_flange(self):
        robot_state = self._robot.get_robot_state()
        joint_positions = self._torch.Tensor(robot_state.joint_positions)
        joint_velocities = self._torch.Tensor(robot_state.joint_velocities)
        jacobian = self._robot.robot_model.compute_jacobian(joint_positions)
        tcp_velocity_base = jacobian @ joint_velocities
        rotation = self._get_base_to_flange_rotation_matrix().T
        linear = rotation @ tcp_velocity_base[0:3]
        angular = rotation @ tcp_velocity_base[3:6]
        return self._torch.cat([linear, angular])

    def _get_tcp_wrench_flange(self):
        robot_state = self._robot.get_robot_state()
        joint_positions = self._torch.Tensor(robot_state.joint_positions)
        tau_external = self._torch.Tensor(robot_state.motor_torques_external)
        jacobian = self._robot.robot_model.compute_jacobian(joint_positions)
        wrench_base, _, _, _ = self._torch.linalg.lstsq(jacobian.T, tau_external)
        rotation = self._get_base_to_flange_rotation_matrix().T
        force = rotation @ wrench_base[0:3]
        torque = rotation @ wrench_base[3:6]
        return self._torch.cat([force, torque])

    def get_controller_state(self, control_frequency_hz: float) -> ControllerState:
        robot_state = self._robot.get_robot_state()
        gripper_state = self._gripper.get_state()
        try:
            gripper_force = float(gripper_state.force)
        except (AttributeError, TypeError):
            gripper_force = 0.0
        return ControllerState(
            tcp_pose=self.get_tcp_pose().tolist(),
            tcp_velocity=self._get_tcp_velocity_flange().tolist(),
            tcp_wrench=self._get_tcp_wrench_flange().tolist(),
            joint_positions=list(robot_state.joint_positions),
            joint_velocities=list(robot_state.joint_velocities),
            gripper_width=float(gripper_state.width),
            gripper_force=gripper_force,
            control_frequency_hz=control_frequency_hz,
            backend=self.name,
        )

    def start_cartesian_impedance(self, stiffness: Sequence[float], damping: Sequence[float]) -> None:
        self._robot.start_cartesian_impedance(
            Kx=self._torch.Tensor(np.asarray(stiffness, dtype=np.float32)),
            Kxd=self._torch.Tensor(np.asarray(damping, dtype=np.float32)),
        )

    def update_desired_tcp(self, target_pose6d: np.ndarray) -> None:
        target_pose6d = np.asarray(target_pose6d, dtype=np.float64)
        quaternion_xyzw = Rotation.from_rotvec(target_pose6d[3:]).as_quat()
        self._robot.update_desired_ee_pose(
            position=self._torch.Tensor(target_pose6d[:3]),
            orientation=self._torch.Tensor(quaternion_xyzw),
        )

    def move_gripper(self, width: float, velocity: float, force_limit: float) -> None:
        self._gripper.goto(width=width, speed=velocity, force=force_limit)

    def grasp(self, velocity: float, force_limit: float) -> None:
        self._gripper.grasp(speed=velocity, force=force_limit)

    def stop_gripper(self) -> None:
        LOGGER.info("Polymetis gripper stop is not exposed; keeping current state")

    def go_home(self, joint_positions: Sequence[float], duration_sec: float) -> None:
        self._robot.move_to_joint_positions(
            positions=self._torch.Tensor(np.asarray(joint_positions, dtype=np.float32)),
            time_to_go=duration_sec,
        )

    def shutdown(self) -> None:
        try:
            self._robot.terminate_current_policy()
        except Exception:  # pragma: no cover - best effort on hardware shutdown
            LOGGER.exception("Failed to terminate current Polymetis policy cleanly")

