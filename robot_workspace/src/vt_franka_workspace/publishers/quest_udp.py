from __future__ import annotations

import logging
import socket
import math
import time
from typing import Iterable

import bson
import numpy as np

from vt_franka_shared.models import Arrow, ControllerState, ForceSensorMessage, TactileSensorMessage
from vt_franka_shared.pose_math import pose7d_to_matrix
from vt_franka_shared.transforms import SingleArmCalibration

from ..settings import QuestImageStreamSettings

LOGGER = logging.getLogger(__name__)


class QuestUdpPublisher:
    def __init__(
        self,
        quest_ip: str,
        robot_state_udp_port: int,
        tactile_udp_port: int,
        image_udp_port: int,
        force_udp_port: int,
        calibration: SingleArmCalibration,
        force_scale_factor: float = 0.025,
    ) -> None:
        self.quest_ip = quest_ip
        self.robot_state_udp_port = robot_state_udp_port
        self.tactile_udp_port = tactile_udp_port
        self.image_udp_port = image_udp_port
        self.force_udp_port = force_udp_port
        self.calibration = calibration
        self.force_scale_factor = force_scale_factor
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._last_image_publish_monotonic: dict[str, float] = {}

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

    def publish_image(self, image: np.ndarray, settings: QuestImageStreamSettings) -> None:
        if not settings.enabled:
            return
        if not settings.image_id:
            raise ValueError("Quest image stream settings require a non-empty image_id")

        cv2 = _require_cv2()
        now = time.monotonic()
        last_publish = self._last_image_publish_monotonic.get(settings.image_id)
        if settings.max_publish_hz > 0.0 and last_publish is not None:
            min_period = 1.0 / settings.max_publish_hz
            if now - last_publish < min_period:
                return

        prepared = _prepare_image_for_quest(
            image,
            max_width=settings.max_width,
            max_height=settings.max_height,
        )
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), int(np.clip(settings.quality, 1, 100))]
        ok, encoded = cv2.imencode(".jpg", prepared, encode_param)
        if not ok:
            raise RuntimeError(f"Failed to JPEG-encode Quest image stream {settings.image_id}")

        payload = {
            "images": [
                {
                    "id": settings.image_id,
                    "inHeadSpace": settings.in_head_space,
                    "leftOrRight": settings.left_or_right,
                    "position": [float(value) for value in settings.position],
                    "rotation": [float(value) for value in settings.rotation],
                    "scale": [float(value) for value in settings.scale],
                    "image": encoded.tobytes(),
                }
            ]
        }
        packed = _encode_bson(payload)
        chunk_size = max(1, int(settings.chunk_size))
        address = (self.quest_ip, self.image_udp_port)
        self.socket.sendto(len(packed).to_bytes(length=4, byteorder="little", signed=False), address)
        self.socket.sendto(chunk_size.to_bytes(length=4, byteorder="little", signed=False), address)
        count = math.ceil(len(packed) / chunk_size)
        for index in range(count):
            start = index * chunk_size
            end = min((index + 1) * chunk_size, len(packed))
            self.socket.sendto(packed[start:end], address)
        self._last_image_publish_monotonic[settings.image_id] = now

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


def _prepare_image_for_quest(image: np.ndarray, *, max_width: int, max_height: int) -> np.ndarray:
    cv2 = _require_cv2()
    array = np.asarray(image)
    if array.ndim != 3 or array.shape[2] != 3:
        raise ValueError("Quest image streaming expects an HxWx3 image array")
    if max_width <= 0 or max_height <= 0:
        return array
    height, width = array.shape[:2]
    scale = min(max_width / max(width, 1), max_height / max(height, 1), 1.0)
    if scale >= 1.0:
        return array
    resized_width = max(1, int(round(width * scale)))
    resized_height = max(1, int(round(height * scale)))
    return cv2.resize(array, (resized_width, resized_height), interpolation=cv2.INTER_AREA)


def _require_cv2():
    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - runtime dependency
        raise RuntimeError("OpenCV is required for Quest image streaming") from exc
    return cv2
