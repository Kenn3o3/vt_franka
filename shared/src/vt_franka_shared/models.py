from __future__ import annotations

import time
import re
from collections.abc import Mapping, Sequence
from typing import Any, List, Optional

from pydantic import BaseModel, Field, field_validator


def _fixed_length(values: List[float], expected: int, field_name: str) -> List[float]:
    if len(values) != expected:
        raise ValueError(f"{field_name} must contain exactly {expected} elements")
    return values


class TcpTargetCommand(BaseModel):
    target_tcp: List[float] = Field(default_factory=lambda: [0.0] * 7)
    target_duration_sec: Optional[float] = None
    issued_at_wall_time: float = Field(default_factory=time.time)
    issued_at_monotonic_time: float = Field(default_factory=time.monotonic)
    source: str = "unknown"

    @field_validator("target_tcp")
    @classmethod
    def _validate_pose(cls, value: List[float]) -> List[float]:
        return _fixed_length(value, 7, "target_tcp")

    @field_validator("target_duration_sec")
    @classmethod
    def _validate_target_duration(cls, value: Optional[float]) -> Optional[float]:
        if value is not None and value <= 0.0:
            raise ValueError("target_duration_sec must be positive when provided")
        return value


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


def parse_unity_teleop_message(payload: Any) -> UnityTeleopMessage:
    if isinstance(payload, UnityTeleopMessage):
        return payload
    normalized = _normalize_unity_teleop_payload(payload)
    return UnityTeleopMessage.model_validate(normalized)


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


def _normalize_unity_teleop_payload(payload: Any) -> dict[str, Any]:
    if not isinstance(payload, Mapping):
        raise ValueError("Quest teleop payload must be a JSON object")

    button_states = payload.get("buttonStates")
    return {
        "timestamp": float(payload.get("timestamp", time.time())),
        "leftHand": _normalize_hand_payload(
            payload.get("leftHand"),
            pose_value=payload.get("leftHandPose"),
            trigger_value=payload.get("leftGripperState", payload.get("leftTriggerState")),
            button_value=payload.get("leftButtonStates", _extract_side_button_states(button_states, "left")),
            required=True,
        ),
        "rightHand": _normalize_hand_payload(
            payload.get("rightHand"),
            pose_value=payload.get("rightHandPose"),
            trigger_value=payload.get("rightGripperState", payload.get("rightTriggerState")),
            button_value=payload.get("rightButtonStates", _extract_side_button_states(button_states, "right")),
            required=False,
        ),
    }


def _normalize_hand_payload(
    raw_hand: Any,
    *,
    pose_value: Any,
    trigger_value: Any,
    button_value: Any,
    required: bool,
) -> dict[str, Any]:
    hand = raw_hand if isinstance(raw_hand, Mapping) else {}
    position = hand.get("wristPos", hand.get("position"))
    quaternion = hand.get("wristQuat", hand.get("rotation"))

    if position is None or quaternion is None:
        position, quaternion = _extract_pose(pose_value if pose_value is not None else hand.get("handPose"), required=required)

    if position is None or quaternion is None:
        position, quaternion = [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]

    return {
        "wristPos": [float(value) for value in position],
        "wristQuat": [float(value) for value in quaternion],
        "triggerState": _coerce_trigger(
            hand.get("triggerState", hand.get("gripState", hand.get("gripperState", trigger_value)))
        ),
        "buttonState": _coerce_button_states(hand.get("buttonState", hand.get("buttons", button_value))),
    }


def _extract_pose(raw_pose: Any, *, required: bool) -> tuple[list[float] | None, list[float] | None]:
    if raw_pose is None:
        if required:
            raise ValueError("Quest teleop payload is missing a hand pose")
        return None, None

    if isinstance(raw_pose, Mapping):
        position = raw_pose.get("wristPos", raw_pose.get("position"))
        quaternion = raw_pose.get("wristQuat", raw_pose.get("rotation"))
        if position is not None and quaternion is not None:
            return list(position), list(quaternion)

    if isinstance(raw_pose, Sequence) and not isinstance(raw_pose, (str, bytes)):
        values = list(raw_pose)
        if len(values) == 7:
            return values[:3], values[3:]

    raise ValueError("Quest teleop hand pose must be [x, y, z, qw, qx, qy, qz]")


def _coerce_trigger(raw_value: Any) -> float:
    if raw_value is None:
        return 0.0
    if isinstance(raw_value, Sequence) and not isinstance(raw_value, (str, bytes)):
        values = list(raw_value)
        if not values:
            return 0.0
        raw_value = values[0]
    return float(raw_value)


def _extract_side_button_states(raw_value: Any, side: str) -> Any:
    if not isinstance(raw_value, Mapping):
        return raw_value
    for key in (side, f"{side}Hand", f"{side}_hand"):
        if key in raw_value:
            return raw_value[key]
    return raw_value if side == "left" else []


def _coerce_button_states(raw_value: Any) -> list[bool]:
    if raw_value is None:
        return []

    if isinstance(raw_value, Mapping):
        parsed = {}
        for key, value in raw_value.items():
            index = _parse_button_index(key)
            if index is not None:
                parsed[index] = bool(value)
        if not parsed:
            return []
        button_states = [False] * (max(parsed) + 1)
        for index, value in parsed.items():
            button_states[index] = value
        return button_states

    if isinstance(raw_value, Sequence) and not isinstance(raw_value, (str, bytes)):
        return [bool(value) for value in raw_value]

    raise ValueError("Quest teleop buttonState must be an array or button-index map")


def _parse_button_index(raw_key: Any) -> int | None:
    if isinstance(raw_key, int):
        return raw_key if raw_key >= 0 else None
    if not isinstance(raw_key, str):
        return None

    match = re.search(r"(\d+)$", raw_key)
    if match is None:
        return None
    return int(match.group(1))
