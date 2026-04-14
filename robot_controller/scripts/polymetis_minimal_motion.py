#!/usr/bin/env python3

from __future__ import annotations

import numpy as np
import torch
from polymetis import RobotInterface
from scipy.spatial.transform import Rotation


# Edit these values directly for your test.
ROBOT_IP = "127.0.0.1"
ROBOT_PORT = 50051

TARGET_POSITION_M = [0.15, 0.39, 0.21]
TARGET_RPY_DEG = [180, 0, 20]
POSE_MOVE_TIME_S = 4.0

# Target joint positions in radians.
TARGET_JOINTS_RAD = [0.9267, 0.0902, 0.2636, -2.6279, -0.0526, 2.6525, 0.8867]
JOINT_MOVE_TIME_S = 5.0

# Keep this on so the robot does not move until you explicitly confirm.
ASK_FOR_CONFIRMATION = True


def tensor_to_numpy(value) -> np.ndarray:
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().numpy()
    return np.asarray(value, dtype=np.float64)


def format_array(values) -> str:
    return np.array2string(np.asarray(values, dtype=np.float64), precision=4, suppress_small=True)


def quaternion_xyzw_to_rpy_deg(quaternion_xyzw: np.ndarray) -> np.ndarray:
    return Rotation.from_quat(quaternion_xyzw).as_euler("xyz", degrees=True)


def print_pose_and_joints(robot: RobotInterface, label: str) -> None:
    joint_positions = tensor_to_numpy(robot.get_joint_positions())
    ee_position, ee_quaternion_xyzw = robot.get_ee_pose()
    ee_position = tensor_to_numpy(ee_position)
    ee_quaternion_xyzw = tensor_to_numpy(ee_quaternion_xyzw)
    ee_rpy_deg = quaternion_xyzw_to_rpy_deg(ee_quaternion_xyzw)

    print(f"{label} position xyz (m): {format_array(ee_position)}")
    print(f"{label} orientation quaternion xyzw: {format_array(ee_quaternion_xyzw)}")
    print(f"{label} orientation roll/pitch/yaw (deg): {format_array(ee_rpy_deg)}")
    print(f"{label} joints (rad): {format_array(joint_positions)}")
    print()


def confirm_or_abort(message: str) -> None:
    if not ASK_FOR_CONFIRMATION:
        return
    response = input(f"{message} Type 'yes' to continue: ").strip().lower()
    if response != "yes":
        raise SystemExit("Aborted.")


def main() -> None:
    target_position = np.asarray(TARGET_POSITION_M, dtype=np.float32)
    target_rpy_deg = np.asarray(TARGET_RPY_DEG, dtype=np.float64)
    target_quaternion_xyzw = Rotation.from_euler("xyz", target_rpy_deg, degrees=True).as_quat().astype(np.float32)
    target_joints = np.asarray(TARGET_JOINTS_RAD, dtype=np.float32)

    robot = RobotInterface(ip_address=ROBOT_IP, port=ROBOT_PORT)
    try:
        print_pose_and_joints(robot, "Current")

        print("Target pose command:")
        print(f"  position xyz (m): {format_array(target_position)}")
        print(f"  roll/pitch/yaw (deg): {format_array(target_rpy_deg)}")
        print(f"  quaternion xyzw: {format_array(target_quaternion_xyzw)}")
        print()
        confirm_or_abort("Move robot to the target pose?")
        robot.move_to_ee_pose(
            position=torch.tensor(target_position),
            orientation=torch.tensor(target_quaternion_xyzw),
            time_to_go=POSE_MOVE_TIME_S,
        )
        print()
        print_pose_and_joints(robot, "After pose move")

        print("Target joint command:")
        print(f"  joints (rad): {format_array(target_joints)}")
        print()
        confirm_or_abort("Move robot to the target joints?")
        robot.move_to_joint_positions(
            positions=torch.tensor(target_joints),
            time_to_go=JOINT_MOVE_TIME_S,
        )
        print()
        print_pose_and_joints(robot, "After joint move")
    finally:
        try:
            robot.terminate_current_policy()
        except Exception:
            pass


if __name__ == "__main__":
    main()
