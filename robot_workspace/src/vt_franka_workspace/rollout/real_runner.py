from __future__ import annotations

import importlib
import time
from collections.abc import Callable

from vt_franka_shared.timing import precise_sleep

from .real_env import RealWorldEnv


class RealRunner:
    def __init__(self, env: RealWorldEnv, control_hz: float, max_duration_sec: float) -> None:
        self.env = env
        self.control_hz = control_hz
        self.max_duration_sec = max_duration_sec

    def run(self, policy: Callable[[dict], dict]) -> None:
        period = 1.0 / self.control_hz
        start_time = time.time()
        while time.time() - start_time < self.max_duration_sec:
            loop_start = time.time()
            observation = self.env.get_observation()
            action = policy(observation)
            self.env.execute_action(action)
            if action.get("terminate") is True:
                break
            precise_sleep(max(0.0, period - (time.time() - loop_start)))

    @staticmethod
    def load_policy(policy_spec: str, policy_kwargs: dict | None = None) -> Callable[[dict], dict]:
        module_name, function_name = policy_spec.split(":", maxsplit=1)
        module = importlib.import_module(module_name)
        policy = getattr(module, function_name)
        if getattr(policy, "__vt_franka_policy_factory__", False):
            policy = policy(**(policy_kwargs or {}))
        if not callable(policy):
            raise TypeError(f"{policy_spec} does not resolve to a callable policy")
        return policy
