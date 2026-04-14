import types

from vt_franka_workspace.publishers import quest_udp


def test_encode_bson_uses_bson_class_when_dumps_missing(monkeypatch):
    class FakeBsonClass:
        @staticmethod
        def encode(payload):
            return f"encoded:{payload['value']}".encode()

    monkeypatch.setattr(quest_udp, "bson", types.SimpleNamespace(BSON=FakeBsonClass))

    assert quest_udp._encode_bson({"value": 7}) == b"encoded:7"
