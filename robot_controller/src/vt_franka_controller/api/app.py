from __future__ import annotations

from contextlib import asynccontextmanager
from typing import Any, Dict

from fastapi import FastAPI, HTTPException

from vt_franka_shared.models import GripperGraspCommand, GripperWidthCommand, TcpTargetCommand

from ..control.service import ControllerService


def create_app(service: ControllerService) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app: FastAPI):
        service.start()
        try:
            yield
        finally:
            service.shutdown()

    app = FastAPI(title="VT Franka Controller", version="0.1.0", lifespan=lifespan)

    @app.get("/api/v1/health")
    def health():
        return service.get_health()

    @app.get("/api/v1/state")
    def state():
        return service.get_state()

    @app.get("/api/v1/tcp")
    def tcp():
        return service.get_state().tcp_pose

    @app.post("/api/v1/commands/tcp")
    def move_tcp(command: TcpTargetCommand):
        service.queue_tcp_command(command)
        return {"status": "queued"}

    @app.post("/api/v1/commands/gripper/width")
    def move_gripper(command: GripperWidthCommand):
        service.queue_gripper_width_command(command)
        return {"status": "queued"}

    @app.post("/api/v1/commands/gripper/grasp")
    def grasp_gripper(command: GripperGraspCommand):
        service.queue_gripper_grasp_command(command)
        return {"status": "queued"}

    @app.post("/api/v1/actions/home")
    def go_home():
        service.go_home()
        return {"status": "ok"}

    @app.post("/api/v1/actions/ready")
    def go_ready():
        try:
            service.go_ready()
        except RuntimeError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return {"status": "ok"}

    @app.get("/get_current_tcp/{robot_side}")
    def legacy_get_current_tcp(robot_side: str):
        _ensure_left(robot_side)
        return service.get_state().tcp_pose

    @app.get("/get_current_robot_states")
    def legacy_get_current_robot_states():
        state = service.get_state()
        return _legacy_state_payload(state)

    @app.post("/move_tcp/{robot_side}")
    def legacy_move_tcp(robot_side: str, command: TcpTargetCommand):
        _ensure_left(robot_side)
        service.queue_tcp_command(command)
        return {"message": "Waypoint added for franka robot"}

    @app.post("/move_gripper/{robot_side}")
    def legacy_move_gripper(robot_side: str, command: GripperWidthCommand):
        _ensure_left(robot_side)
        service.queue_gripper_width_command(command)
        return {"message": f"Gripper moving to width {command.width}"}

    @app.post("/move_gripper_force/{robot_side}")
    def legacy_grasp_gripper(robot_side: str, command: GripperWidthCommand):
        _ensure_left(robot_side)
        grasp_command = GripperGraspCommand(
            velocity=command.velocity,
            force_limit=command.force_limit,
            source=command.source,
        )
        service.queue_gripper_grasp_command(grasp_command)
        return {"message": f"Gripper grasping with force {command.force_limit}"}

    @app.post("/stop_gripper/{robot_side}")
    def legacy_stop_gripper(robot_side: str):
        _ensure_left(robot_side)
        service.stop_gripper()
        return {"message": "Gripper stopped"}

    @app.post("/birobot_go_home")
    def legacy_home():
        service.go_home()
        return {"message": "Robot moved to home position"}

    return app


def _ensure_left(robot_side: str) -> None:
    if robot_side != "left":
        raise HTTPException(status_code=400, detail="Only the left arm is supported in VT Franka v1")


def _legacy_state_payload(state) -> Dict[str, Any]:
    return {
        "leftRobotTCP": state.tcp_pose,
        "leftRobotTCPVel": state.tcp_velocity,
        "leftRobotTCPWrench": state.tcp_wrench,
        "leftGripperState": [state.gripper_width, state.gripper_force],
        "rightRobotTCP": [0.0] * 7,
        "rightRobotTCPVel": [0.0] * 6,
        "rightRobotTCPWrench": [0.0] * 6,
        "rightGripperState": [0.0, 0.0],
    }
