#!/usr/bin/env python3
from __future__ import annotations

import time
from contextlib import nullcontext
from pathlib import Path
from threading import Event, RLock, Thread

import numpy as np
import torch
from polymetis import GripperInterface, RobotInterface
from scipy.spatial.transform import Rotation, Slerp
import uvicorn

from vt_franka_controller.api.demo_state_app import create_demo_state_app
from vt_franka_controller.settings import BackendSettings, ControlSettings, ControllerSettings, ServerSettings
from vt_franka_shared.models import ControllerState, HealthStatus
from vt_franka_shared.pose_math import xyzw_to_wxyz


# Edit these values directly for your test.
ROBOT_IP = "127.0.0.1"
ROBOT_PORT = 50051
GRIPPER_IP = "127.0.0.1"
GRIPPER_PORT = 50052

# Serve read-only robot state to the workspace PC while this demo is running.
STATE_SERVER_HOST = "0.0.0.0"
STATE_SERVER_PORT = 8092
STATE_CACHE_HZ = 60.0

# Initial pose before starting the waypoint sequence.
INITIAL_POSITION_M = [-0.1, 0.53, 0.45]
INITIAL_RPY_DEG = [-180, 0, 45]

# Motion timing
POSE_MOVE_TIME_S = 4.0
CONTROL_HZ = 100.0

# Keep one Cartesian impedance policy active for the whole trajectory.
CARTESIAN_STIFFNESS = [750.0, 750.0, 750.0, 15.0, 15.0, 15.0]
CARTESIAN_DAMPING = [37.0, 37.0, 37.0, 2.0, 2.0, 2.0]
HOLD_FINAL_POSE_S = 0.5

# Gripper commands use meters/second and Newtons.
CLOSE_SPEED_MPS = 0.03
CLOSE_FORCE_N = 20.0

# Safety: keep this on so the robot does not move until you explicitly confirm.
ASK_FOR_CONFIRMATION = True

# Optional: ask before *each* waypoint (True) or only once before the whole sequence (False).
CONFIRM_EACH_WAYPOINT = False

# Waypoints after the initial pose: (position [m], rpy [deg])
WAYPOINTS = [
    ([-0.1, 0.53, 0.345], [-180, 0, 45]), # C
    ([-0.14, 0.53, 0.35], [-180, 0, 45]),
    ([-0.14, 0.45, 0.35], [-180, 0, 45]),
    ([-0.09, 0.45, 0.35], [-180, 0, 45]),
    ([-0.1, 0.45, 0.4], [-180, 0, 45]), # Finished
    ([-0.08, 0.53, 0.35], [-180, 0, 45]), # U
    ([-0.08, 0.45, 0.35], [-180, 0, 45]),
    ([-0.04, 0.45, 0.35], [-180, 0, 45]),
    ([-0.04, 0.53, 0.35], [-180, 0, 45]),
    ([-0.04, 0.45, 0.36], [-180, 0, 45]),
    ([-0.04, 0.53, 0.4], [-180, 0, 45]), # Finished
    ([-0.02, 0.53, 0.35], [-180, 0, 45]), # H
    ([-0.02, 0.45, 0.35], [-180, 0, 45]),
    ([-0.02, 0.45, 0.4], [-180, 0, 45]),
    ([0.02, 0.53, 0.35], [-180, 0, 45]),
    ([0.02, 0.45, 0.35], [-180, 0, 45]),
    ([0.02, 0.45, 0.4], [-180, 0, 45]),
    ([-0.02, 0.49, 0.35], [-180, 0, 45]),
    ([0.02, 0.49, 0.35], [-180, 0, 45]),
    ([-0.02, 0.49, 0.355], [-180, 0, 45]),
    ([-0.02, 0.49, 0.4], [-180, 0, 45]), # Finished
    ([0.04, 0.53, 0.35], [-180, 0, 45]), # K
    ([0.04, 0.45, 0.35], [-180, 0, 45]), # vertical line
    ([0.08, 0.53, 0.4], [-180, 0, 45]),  # lift and reposition
    ([0.08, 0.53, 0.35], [-180, 0, 45]), # K: rotated V
    ([0.04, 0.49, 0.35], [-180, 0, 45]),
    ([0.08, 0.45, 0.35], [-180, 0, 45]),
    ([0.08, 0.45, 0.45], [-180, 0, 45]),
]


def tensor_to_numpy(value) -> np.ndarray:
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().numpy()
    return np.asarray(value, dtype=np.float64)


def format_array(values) -> str:
    return np.array2string(np.asarray(values, dtype=np.float64), precision=4, suppress_small=True)


def quaternion_xyzw_to_rpy_deg(quaternion_xyzw: np.ndarray) -> np.ndarray:
    return Rotation.from_quat(quaternion_xyzw).as_euler("xyz", degrees=True)


def maybe_lock(lock):
    return lock if lock is not None else nullcontext()


def confirm_or_abort(message: str) -> None:
    if not ASK_FOR_CONFIRMATION:
        return
    response = input(f"{message} Type 'yes' to continue: ").strip().lower()
    if response != "yes":
        raise SystemExit("Aborted.")


def rpy_deg_to_quat_xyzw(rpy_deg: np.ndarray) -> np.ndarray:
    return Rotation.from_euler("xyz", rpy_deg, degrees=True).as_quat()


class DemoStateService:
    def __init__(
        self,
        robot: RobotInterface,
        gripper: GripperInterface,
        settings: ControllerSettings,
        *,
        robot_lock=None,
        gripper_lock=None,
    ) -> None:
        self.robot = robot
        self.gripper = gripper
        self.settings = settings
        self.robot_lock = robot_lock
        self.gripper_lock = gripper_lock
        self._running = Event()
        self._thread: Thread | None = None
        self._state_lock = RLock()
        self._cached_state: ControllerState | None = None
        self._last_error: str | None = None

    def start(self) -> None:
        if self._running.is_set():
            return
        self._running.set()
        self._thread = Thread(target=self._loop, name="demo-state-cache", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def get_state(self) -> ControllerState:
        with self._state_lock:
            if self._cached_state is not None:
                return self._cached_state
            last_error = self._last_error
        state = self._compute_state()
        with self._state_lock:
            self._cached_state = state
            self._last_error = None
        if last_error is not None:
            return state
        return state

    def get_health(self) -> HealthStatus:
        with self._state_lock:
            last_state = self._cached_state
            last_error = self._last_error
        return HealthStatus(
            ok=self._running.is_set() and last_error is None,
            backend="polymetis-demo",
            message="running" if last_error is None else last_error,
            queue_depth=0,
            control_loop_running=self._running.is_set(),
            last_state_monotonic_time=last_state.monotonic_time if last_state is not None else None,
        )

    def _loop(self) -> None:
        period = 1.0 / max(self.settings.control.state_cache_hz, 1e-6)
        while self._running.is_set():
            try:
                state = self._compute_state()
                with self._state_lock:
                    self._cached_state = state
                    self._last_error = None
            except Exception as exc:
                with self._state_lock:
                    self._last_error = str(exc)
            time.sleep(period)

    def _compute_state(self) -> ControllerState:
        with maybe_lock(self.robot_lock):
            robot_state = self.robot.get_robot_state()
            joint_positions = torch.as_tensor(robot_state.joint_positions)
            joint_velocities = torch.as_tensor(robot_state.joint_velocities)
            tau_external = torch.as_tensor(robot_state.motor_torques_external)

            position, quaternion_xyzw = self.robot.robot_model.forward_kinematics(joint_positions)
            pose = np.concatenate([tensor_to_numpy(position), xyzw_to_wxyz(tensor_to_numpy(quaternion_xyzw))])

            rotation_base_to_flange = torch.as_tensor(
                Rotation.from_quat(tensor_to_numpy(quaternion_xyzw)).as_matrix(),
                dtype=joint_positions.dtype,
            )
            rotation_flange_to_base = rotation_base_to_flange.T

            jacobian = self.robot.robot_model.compute_jacobian(joint_positions)
            tcp_velocity_base = jacobian @ joint_velocities
            linear_velocity = tensor_to_numpy(rotation_flange_to_base @ tcp_velocity_base[0:3])
            angular_velocity = tensor_to_numpy(rotation_flange_to_base @ tcp_velocity_base[3:6])
            tcp_velocity = np.concatenate([linear_velocity, angular_velocity])

            wrench_base, _, _, _ = torch.linalg.lstsq(jacobian.T, tau_external)
            force = tensor_to_numpy(rotation_flange_to_base @ wrench_base[0:3])
            torque = tensor_to_numpy(rotation_flange_to_base @ wrench_base[3:6])
            tcp_wrench = np.concatenate([force, torque])

        with maybe_lock(self.gripper_lock):
            gripper_state = self.gripper.get_state()

        try:
            gripper_force = float(gripper_state.force)
        except (AttributeError, TypeError):
            gripper_force = 0.0

        return ControllerState(
            tcp_pose=pose.tolist(),
            tcp_velocity=tcp_velocity.tolist(),
            tcp_wrench=tcp_wrench.tolist(),
            joint_positions=list(robot_state.joint_positions),
            joint_velocities=list(robot_state.joint_velocities),
            gripper_width=float(gripper_state.width),
            gripper_force=gripper_force,
            control_frequency_hz=self.settings.control.control_frequency_hz,
            backend="polymetis-demo",
        )


class ManagedUvicornServer:
    def __init__(self, app, host: str, port: int) -> None:
        self.app = app
        self.host = host
        self.port = port
        self._server: uvicorn.Server | None = None
        self._thread: Thread | None = None

    def start(self) -> None:
        config = uvicorn.Config(self.app, host=self.host, port=self.port, log_level="info")
        self._server = uvicorn.Server(config)
        self._thread = Thread(target=self._server.run, name="demo-state-api", daemon=True)
        self._thread.start()
        time.sleep(0.2)

    def stop(self) -> None:
        if self._server is not None:
            self._server.should_exit = True
        if self._thread is not None:
            self._thread.join(timeout=2.0)

def print_pose_and_joints(robot: RobotInterface, label: str, *, robot_lock=None) -> None:
    with maybe_lock(robot_lock):
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


def get_ee_pose_numpy(robot: RobotInterface, *, robot_lock=None) -> tuple[np.ndarray, np.ndarray]:
    with maybe_lock(robot_lock):
        ee_position, ee_quaternion_xyzw = robot.get_ee_pose()
    return tensor_to_numpy(ee_position), tensor_to_numpy(ee_quaternion_xyzw)


def print_gripper_state(gripper: GripperInterface, label: str, *, gripper_lock=None) -> None:
    with maybe_lock(gripper_lock):
        gripper_state = gripper.get_state()
    width = float(gripper_state.width)
    force = float(getattr(gripper_state, "force", 0.0))
    print(f"{label} gripper width (m): {width:.4f}")
    print(f"{label} gripper force (N): {force:.2f}")
    print()


def start_cartesian_hold(robot: RobotInterface, *, robot_lock=None) -> None:
    with maybe_lock(robot_lock):
        robot.start_cartesian_impedance(
            Kx=torch.tensor(np.asarray(CARTESIAN_STIFFNESS, dtype=np.float32)),
            Kxd=torch.tensor(np.asarray(CARTESIAN_DAMPING, dtype=np.float32)),
        )
    current_position, current_quaternion_xyzw = get_ee_pose_numpy(robot, robot_lock=robot_lock)
    with maybe_lock(robot_lock):
        robot.update_desired_ee_pose(
            position=torch.tensor(current_position.astype(np.float32)),
            orientation=torch.tensor(current_quaternion_xyzw.astype(np.float32)),
        )


def stream_pose_segment(
    robot: RobotInterface,
    start_position_m: np.ndarray,
    start_quaternion_xyzw: np.ndarray,
    target_position_m: np.ndarray,
    target_quaternion_xyzw: np.ndarray,
    duration_s: float,
    *,
    robot_lock=None,
) -> None:
    if duration_s <= 0.0:
        with maybe_lock(robot_lock):
            robot.update_desired_ee_pose(
                position=torch.tensor(target_position_m.astype(np.float32)),
                orientation=torch.tensor(target_quaternion_xyzw.astype(np.float32)),
            )
        return

    steps = max(1, int(np.ceil(duration_s * CONTROL_HZ)))
    slerp = Slerp(
        [0.0, duration_s],
        Rotation.from_quat(np.vstack([start_quaternion_xyzw, target_quaternion_xyzw])),
    )
    start_time = time.monotonic()

    for step in range(1, steps + 1):
        alpha = step / steps
        segment_time = alpha * duration_s
        position = (1.0 - alpha) * start_position_m + alpha * target_position_m
        quaternion_xyzw = slerp([segment_time]).as_quat()[0]
        with maybe_lock(robot_lock):
            robot.update_desired_ee_pose(
                position=torch.tensor(position.astype(np.float32)),
                orientation=torch.tensor(quaternion_xyzw.astype(np.float32)),
            )

        if step < steps:
            deadline = start_time + segment_time
            remaining = deadline - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)


def move_to_pose(
    robot: RobotInterface,
    label: str,
    position_m: np.ndarray,
    rpy_deg: np.ndarray,
    *,
    robot_lock=None,
) -> None:
    quat_xyzw = rpy_deg_to_quat_xyzw(rpy_deg).astype(np.float32)
    current_position, current_quaternion_xyzw = get_ee_pose_numpy(robot, robot_lock=robot_lock)
    stream_pose_segment(
        robot=robot,
        start_position_m=current_position.astype(np.float64),
        start_quaternion_xyzw=current_quaternion_xyzw.astype(np.float64),
        target_position_m=position_m.astype(np.float64),
        target_quaternion_xyzw=quat_xyzw.astype(np.float64),
        duration_s=POSE_MOVE_TIME_S,
        robot_lock=robot_lock,
    )


def main() -> None:
    robot = RobotInterface(ip_address=ROBOT_IP, port=ROBOT_PORT)
    gripper = GripperInterface(ip_address=GRIPPER_IP, port=GRIPPER_PORT)
    robot_lock = RLock()
    gripper_lock = RLock()
    state_service = DemoStateService(
        robot,
        gripper,
        ControllerSettings(
            server=ServerSettings(host=STATE_SERVER_HOST, port=STATE_SERVER_PORT),
            backend=BackendSettings(
                kind="polymetis",
                robot_ip=ROBOT_IP,
                robot_port=ROBOT_PORT,
                gripper_ip=GRIPPER_IP,
                gripper_port=GRIPPER_PORT,
            ),
            control=ControlSettings(
                control_frequency_hz=CONTROL_HZ,
                state_cache_hz=STATE_CACHE_HZ,
                teleop_command_hz=CONTROL_HZ,
            ),
        ),
        robot_lock=robot_lock,
        gripper_lock=gripper_lock,
    )
    server = ManagedUvicornServer(create_demo_state_app(state_service), STATE_SERVER_HOST, STATE_SERVER_PORT)
    try:
        state_service.start()
        server.start()
        print("Demo state server:")
        print(f"  host: {STATE_SERVER_HOST}")
        print(f"  port: {STATE_SERVER_PORT}")
        print()

        print_pose_and_joints(robot, "Current", robot_lock=robot_lock)
        print_gripper_state(gripper, "Current", gripper_lock=gripper_lock)
        start_cartesian_hold(robot, robot_lock=robot_lock)

        initial_position = np.asarray(INITIAL_POSITION_M, dtype=np.float32)
        initial_rpy = np.asarray(INITIAL_RPY_DEG, dtype=np.float64)

        confirm_or_abort("Move robot to the initial pose?")
        move_to_pose(robot, "Initial pose", initial_position, initial_rpy, robot_lock=robot_lock)

        print("Initial gripper command:")
        print("  action: grasp until contact")
        print(f"  speed (m/s): {CLOSE_SPEED_MPS:.4f}")
        print(f"  force (N): {CLOSE_FORCE_N:.2f}")
        print()
        confirm_or_abort("Close gripper at the initial pose?")
        with maybe_lock(gripper_lock):
            gripper.grasp(speed=CLOSE_SPEED_MPS, force=CLOSE_FORCE_N)
        print_gripper_state(gripper, "After initial close", gripper_lock=gripper_lock)

        if not CONFIRM_EACH_WAYPOINT:
            confirm_or_abort("Run inference automatically?")

        for i, (pos_list, rpy_list) in enumerate(WAYPOINTS, start=1):
            pos = np.asarray(pos_list, dtype=np.float32)
            rpy = np.asarray(rpy_list, dtype=np.float64)
            if CONFIRM_EACH_WAYPOINT:
                confirm_or_abort(f"Move robot to waypoint {i}?")
            move_to_pose(robot, f"Waypoint {i}", pos, rpy, robot_lock=robot_lock)

        if HOLD_FINAL_POSE_S > 0.0:
            print(f"Holding final pose for {HOLD_FINAL_POSE_S:.2f} s before releasing policy.")
            time.sleep(HOLD_FINAL_POSE_S)

        print("Done.")
    finally:
        server.stop()
        state_service.stop()
        try:
            with maybe_lock(robot_lock):
                robot.terminate_current_policy()
        except Exception:
            pass


if __name__ == "__main__":
    main()
