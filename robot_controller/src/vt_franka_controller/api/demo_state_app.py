from __future__ import annotations

from fastapi import FastAPI, HTTPException


def create_demo_state_app(service) -> FastAPI:
    app = FastAPI(title="VT Franka Demo State", version="0.1.0")

    @app.get("/api/v1/health")
    def health():
        return service.get_health()

    @app.get("/api/v1/state")
    def state():
        return service.get_state()

    @app.get("/api/v1/tcp")
    def tcp():
        return service.get_state().tcp_pose

    @app.get("/get_current_tcp/{robot_side}")
    def legacy_get_current_tcp(robot_side: str):
        _ensure_left(robot_side)
        return service.get_state().tcp_pose

    @app.get("/get_current_robot_states")
    def legacy_get_current_robot_states():
        state = service.get_state()
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

    return app


def _ensure_left(robot_side: str) -> None:
    if robot_side != "left":
        raise HTTPException(status_code=400, detail="Only the left arm is supported in VT Franka v1")
