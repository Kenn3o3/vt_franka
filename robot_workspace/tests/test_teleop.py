import numpy as np

from vt_franka_shared.models import ControllerState
from vt_franka_shared.transforms import SingleArmCalibration
from vt_franka_workspace.settings import TeleopSettings
from vt_franka_workspace.teleop.quest_server import QuestTeleopService


class FakeController:
    def __init__(self):
        self.tcp_targets = []

    def get_state(self):
        return ControllerState(tcp_pose=[0.4, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0])

    def queue_tcp(self, target_tcp, source="test"):
        self.tcp_targets.append(target_tcp)

    def move_gripper(self, width, velocity, force_limit, source="test"):
        return None

    def grasp_gripper(self, velocity, force_limit, source="test"):
        return None

    def stop_gripper(self):
        return None


def test_relative_target_uses_robot_frame_delta():
    calibration = SingleArmCalibration.from_dir(
        "/home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/calibration/v6"
    )
    controller = FakeController()
    service = QuestTeleopService(TeleopSettings(relative_translation_scale=1.0), controller, calibration)
    service._start_real_tcp = np.array([0.4, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0])
    service._start_hand_tcp = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    target = service._calculate_relative_target(np.array([0.1, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]))
    assert np.allclose(target[:3], [0.5, 0.0, 0.3])
