from __future__ import annotations

import time
from typing import List, Optional

from pydantic import BaseModel, Field, field_validator


def _fixed_length(values: List[float], expected: int, field_name: str) -> List[float]:
    if len(values) != expected:
        raise ValueError(f"{field_name} must contain exactly {expected} elements")
    return values


class TcpTargetCommand(BaseModel):
    target_tcp: List[float] = Field(default_factory=lambda: [0.0] * 7)
    issued_at_wall_time: float = Field(default_factory=time.time)
    issued_at_monotonic_time: float = Field(default_factory=time.monotonic)
    source: str = "unknown"

    @field_validator("target_tcp")
    @classmethod
    def _validate_pose(cls, value: List[float]) -> List[float]:
        return _fixed_length(value, 7, "target_tcp")


class GripperWidthCommand(BaseModel):
    width: float
    velocity: float = 0.1
    force_limit: float = 5.0
    issued_at_wall_time: float = Field(default_factory=time.time)
    issued_at_monotonic_time: float = Field(default_factory=time.monotonic)
    source: str = "unknown"


class GripperGraspCommand(BaseModel):
    velocity: float = 0.1
    force_limit: float = 5.0
    issued_at_wall_time: float = Field(default_factory=time.time)
    issued_at_monotonic_time: float = Field(default_factory=time.monotonic)
    source: str = "unknown"


class ControllerState(BaseModel):
    tcp_pose: List[float] = Field(default_factory=lambda: [0.0] * 7)
    tcp_velocity: List[float] = Field(default_factory=lambda: [0.0] * 6)
    tcp_wrench: List[float] = Field(default_factory=lambda: [0.0] * 6)
    joint_positions: List[float] = Field(default_factory=lambda: [0.0] * 7)
    joint_velocities: List[float] = Field(default_factory=lambda: [0.0] * 7)
    gripper_width: float = 0.0
    gripper_force: float = 0.0
    wall_time: float = Field(default_factory=time.time)
    monotonic_time: float = Field(default_factory=time.monotonic)
    control_frequency_hz: float = 0.0
    backend: str = "unknown"

    @field_validator("tcp_pose")
    @classmethod
    def _validate_tcp_pose(cls, value: List[float]) -> List[float]:
        return _fixed_length(value, 7, "tcp_pose")

    @field_validator("tcp_velocity", "tcp_wrench")
    @classmethod
    def _validate_six_dof(cls, value: List[float], info) -> List[float]:
        return _fixed_length(value, 6, info.field_name)

    @field_validator("joint_positions", "joint_velocities")
    @classmethod
    def _validate_joints(cls, value: List[float], info) -> List[float]:
        return _fixed_length(value, 7, info.field_name)


class HealthStatus(BaseModel):
    ok: bool = True
    backend: str = "unknown"
    message: str = "running"
    queue_depth: int = 0
    control_loop_running: bool = False
    last_state_monotonic_time: Optional[float] = None


class QuestHandState(BaseModel):
    wristPos: List[float]
    wristQuat: List[float]
    triggerState: float = 0.0
    buttonState: List[bool] = Field(default_factory=list)

    @field_validator("wristPos")
    @classmethod
    def _validate_pos(cls, value: List[float]) -> List[float]:
        return _fixed_length(value, 3, "wristPos")

    @field_validator("wristQuat")
    @classmethod
    def _validate_quat(cls, value: List[float]) -> List[float]:
        return _fixed_length(value, 4, "wristQuat")


class UnityTeleopMessage(BaseModel):
    timestamp: float
    leftHand: QuestHandState
    rightHand: QuestHandState


class Arrow(BaseModel):
    start: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0])
    end: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0])

    @field_validator("start", "end")
    @classmethod
    def _validate_points(cls, value: List[float], info) -> List[float]:
        return _fixed_length(value, 3, info.field_name)


class TactileSensorMessage(BaseModel):
    device_id: str
    arrows: List[Arrow]
    scale: List[float] = Field(default_factory=lambda: [0.01, 0.005, 0.005])

    @field_validator("scale")
    @classmethod
    def _validate_scale(cls, value: List[float]) -> List[float]:
        return _fixed_length(value, 3, "scale")


class ForceSensorMessage(BaseModel):
    device_id: str
    arrow: Arrow
    scale: List[float] = Field(default_factory=lambda: [0.01, 0.005, 0.005])

    @field_validator("scale")
    @classmethod
    def _validate_scale(cls, value: List[float]) -> List[float]:
        return _fixed_length(value, 3, "scale")
