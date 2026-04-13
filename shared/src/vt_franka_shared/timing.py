from __future__ import annotations

import time
from typing import Callable


def precise_sleep(dt: float, slack_time: float = 0.001, time_func: Callable[[], float] = time.monotonic) -> None:
    start = time_func()
    if dt > slack_time:
        time.sleep(dt - slack_time)
    deadline = start + dt
    while time_func() < deadline:
        pass


def precise_wait(deadline: float, slack_time: float = 0.001, time_func: Callable[[], float] = time.monotonic) -> None:
    remaining = deadline - time_func()
    if remaining <= 0:
        return
    if remaining > slack_time:
        time.sleep(remaining - slack_time)
    while time_func() < deadline:
        pass

