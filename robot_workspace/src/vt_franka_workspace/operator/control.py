from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol

import numpy as np


class OperatorActionError(RuntimeError):
    """Raised when an operator action is not currently allowed."""


@dataclass(frozen=True)
class OperatorSnapshot:
    name: str
    image: np.ndarray
    captured_wall_time: float
    label: str | None = None
    image_format: str = "jpg"

    @property
    def token(self) -> str:
        return f"{self.name}:{self.captured_wall_time:.6f}"


class SupportsOperatorUi(Protocol):
    def get_operator_status(self) -> dict[str, Any]:
        ...

    def get_operator_snapshot(self, name: str) -> OperatorSnapshot | None:
        ...

    def operator_reset_ready_pose(self) -> None:
        ...

    def operator_start_episode(self) -> None:
        ...

    def operator_stop_episode(self) -> None:
        ...

    def operator_discard_latest_episode(self) -> None:
        ...

    def operator_quit(self) -> None:
        ...
