from __future__ import annotations

from vt_franka_shared.models import ControllerState


class Ros2StatePublisher:
    def __init__(self) -> None:
        try:
            import rclpy  # noqa: F401
        except ImportError as exc:
            raise RuntimeError("ROS2 publishing requires rclpy and ROS2 message packages") from exc
        self._enabled = True

    def publish_controller_state(self, state: ControllerState) -> None:
        # The clean repo keeps ROS2 optional. The main workspace stack talks directly through
        # controller HTTP + Quest UDP + raw stream recording. A fuller ROS2 publisher can be
        # added later without changing the runtime contracts.
        return None

    def shutdown(self) -> None:
        return None

