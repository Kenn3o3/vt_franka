from __future__ import annotations

import logging
import socket
from typing import Iterable

import bson
import numpy as np

from vt_franka_shared.models import Arrow, ControllerState, ForceSensorMessage, TactileSensorMessage
from vt_franka_shared.pose_math import pose7d_to_matrix
from vt_franka_shared.transforms import SingleArmCalibration

LOGGER = logging.getLogger(__name__)


class QuestUdpPublisher:
    def __init__(
        self,
        quest_ip: str,
        robot_state_udp_port: int,
        tactile_udp_port: int,
        force_udp_port: int,
        calibration: SingleArmCalibration,
        force_scale_factor: float = 0.025,
    ) -> None:
        self.quest_ip = quest_ip
        self.robot_state_udp_port = robot_state_udp_port
        self.tactile_udp_port = tactile_udp_port
        self.force_udp_port = force_udp_port
        self.calibration = calibration
        self.force_scale_factor = force_scale_factor
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def publish_robot_state(self, state: ControllerState) -> None:
        unity_pose = self.calibration.robot_to_unity_pose(np.asarray(state.tcp_pose, dtype=np.float64))
        payload = {
            "leftGripperState": [state.gripper_width, state.gripper_force],
            "rightGripperState": [0.0, 0.0],
            "leftRobotTCP": unity_pose.tolist(),
            "rightRobotTCP": [0.0] * 7,
        }
        self._send(payload, self.robot_state_udp_port)
        self.publish_force_feedback(state)

    def publish_force_feedback(self, state: ControllerState) -> None:
        tcp_pose = np.asarray(state.tcp_pose, dtype=np.float64)
        tcp_transform = pose7d_to_matrix(tcp_pose)
        force_vector_tcp = np.asarray(state.tcp_wrench[:3], dtype=np.float64) * self.force_scale_factor
        force_vector_robot = tcp_transform[:3, :3] @ force_vector_tcp
        start_pose = np.concatenate([tcp_pose[:3], [1.0, 0.0, 0.0, 0.0]])
        end_pose = np.concatenate([tcp_pose[:3] + force_vector_robot, [1.0, 0.0, 0.0, 0.0]])
        arrow = Arrow(
            start=self.calibration.robot_to_unity_pose(start_pose)[:3].tolist(),
            end=self.calibration.robot_to_unity_pose(end_pose)[:3].tolist(),
        )
        payload = ForceSensorMessage(device_id="left", arrow=arrow).model_dump(mode="json")
        self._send(payload, self.force_udp_port)

    def publish_tactile(self, tactile: TactileSensorMessage) -> None:
        self._send(tactile.model_dump(mode="json"), self.tactile_udp_port)

    def _send(self, payload: dict, port: int) -> None:
        packet = _encode_bson(payload)
        self.socket.sendto(packet, (self.quest_ip, port))


def _encode_bson(payload: dict) -> bytes:
    dumps = getattr(bson, "dumps", None)
    if callable(dumps):
        return dumps(payload)

    bson_class = getattr(bson, "BSON", None)
    if bson_class is not None and callable(getattr(bson_class, "encode", None)):
        return bson_class.encode(payload)

    raise RuntimeError(
        "Installed bson module does not provide bson.dumps or bson.BSON.encode; "
        "install pymongo or a compatible bson package."
    )
