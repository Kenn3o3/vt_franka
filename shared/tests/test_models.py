from vt_franka_shared.models import parse_unity_teleop_message


def test_parse_unity_teleop_message_accepts_nested_payload():
    message = parse_unity_teleop_message(
        {
            "timestamp": 1.0,
            "leftHand": {
                "wristPos": [0.1, 0.2, 0.3],
                "wristQuat": [1.0, 0.0, 0.0, 0.0],
                "triggerState": 0.4,
                "buttonState": [False, False, False, False, True],
            },
            "rightHand": {
                "wristPos": [0.0, 0.0, 0.0],
                "wristQuat": [1.0, 0.0, 0.0, 0.0],
                "triggerState": 0.0,
                "buttonState": [False, False, False, False, False],
            },
        }
    )

    assert message.leftHand.wristPos == [0.1, 0.2, 0.3]
    assert message.leftHand.buttonState[4] is True


def test_parse_unity_teleop_message_accepts_flat_payload():
    message = parse_unity_teleop_message(
        {
            "timestamp": 2.0,
            "leftHandPose": [0.4, 0.5, 0.6, 1.0, 0.0, 0.0, 0.0],
            "leftGripperState": 0.9,
            "buttonStates": {"button_0": True, "button_4": True},
        }
    )

    assert message.leftHand.wristPos == [0.4, 0.5, 0.6]
    assert message.leftHand.triggerState == 0.9
    assert message.leftHand.buttonState == [True, False, False, False, True]
    assert message.rightHand.wristQuat == [1.0, 0.0, 0.0, 0.0]
