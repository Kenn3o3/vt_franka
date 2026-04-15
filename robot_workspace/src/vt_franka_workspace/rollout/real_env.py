from __future__ import annotations

from typing import Callable

from ..controller.client import ControllerClient


class RealWorldEnv:
    def __init__(
        self,
        controller: ControllerClient,
        tactile_provider: Callable[[], dict] | None = None,
    ) -> None:
        self.controller = controller
        self.tactile_provider = tactile_provider

    def get_observation(self) -> dict:
        state = self.controller.get_state()
        observation = {"controller_state": state.model_dump(mode="json")}
        if self.tactile_provider is not None:
            observation["tactile"] = self.tactile_provider()
        return observation

    def execute_action(self, action: dict) -> None:
        target_tcp = action.get("target_tcp")
        if target_tcp is not None:
            self.controller.queue_tcp(list(target_tcp), source="rollout")

        gripper_velocity = float(action.get("gripper_velocity", 0.1))
        gripper_force_limit = float(action.get("gripper_force_limit", 5.0))
        if action.get("gripper_closed") is True:
            self.controller.grasp_gripper(
                velocity=gripper_velocity,
                force_limit=gripper_force_limit,
                source="rollout",
            )
        elif "gripper_width" in action:
            self.controller.move_gripper(
                width=float(action["gripper_width"]),
                velocity=gripper_velocity,
                force_limit=gripper_force_limit,
                source="rollout",
            )
