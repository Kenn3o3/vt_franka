import types

import numpy as np

from vt_franka_workspace.publishers import quest_udp
from vt_franka_workspace.settings import QuestImageStreamSettings


def test_encode_bson_uses_bson_class_when_dumps_missing(monkeypatch):
    class FakeBsonClass:
        @staticmethod
        def encode(payload):
            return f"encoded:{payload['value']}".encode()

    monkeypatch.setattr(quest_udp, "bson", types.SimpleNamespace(BSON=FakeBsonClass))

    assert quest_udp._encode_bson({"value": 7}) == b"encoded:7"


def test_publish_image_sends_length_chunk_and_payload(monkeypatch):
    sent_packets = []

    class FakeSocket:
        def sendto(self, payload, address):
            sent_packets.append((payload, address))

    class FakeCv2:
        IMWRITE_JPEG_QUALITY = 1
        INTER_AREA = 2

        @staticmethod
        def imencode(ext, image, params=None):
            del ext, image, params
            return True, np.frombuffer(b"jpeg-bytes", dtype=np.uint8)

        @staticmethod
        def resize(image, size, interpolation=None):
            del size, interpolation
            return image

    monkeypatch.setattr(quest_udp, "_require_cv2", lambda: FakeCv2)
    monkeypatch.setattr(quest_udp, "_encode_bson", lambda payload: b"encoded-image-payload")
    publisher = quest_udp.QuestUdpPublisher(
        quest_ip="127.0.0.1",
        robot_state_udp_port=10001,
        tactile_udp_port=10002,
        image_udp_port=10004,
        force_udp_port=10005,
        calibration=None,
    )
    publisher.socket = FakeSocket()
    settings = QuestImageStreamSettings(enabled=True, image_id="rgb", chunk_size=8, max_publish_hz=0.0)

    publisher.publish_image(np.zeros((4, 5, 3), dtype=np.uint8), settings)

    assert len(sent_packets) == 5
    assert sent_packets[0][0] == len(b"encoded-image-payload").to_bytes(4, "little", signed=False)
    assert sent_packets[1][0] == (8).to_bytes(4, "little", signed=False)
    assert b"".join(packet for packet, _ in sent_packets[2:]) == b"encoded-image-payload"
