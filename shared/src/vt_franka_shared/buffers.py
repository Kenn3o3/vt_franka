from __future__ import annotations

from collections import deque
from threading import Lock
from typing import Deque, Generic, List, Optional, TypeVar

T = TypeVar("T")


class ThreadSafeRingBuffer(Generic[T]):
    def __init__(self, capacity: int = 1024) -> None:
        if capacity <= 0:
            raise ValueError("capacity must be positive")
        self._capacity = capacity
        self._items: Deque[T] = deque(maxlen=capacity)
        self._lock = Lock()

    @property
    def capacity(self) -> int:
        return self._capacity

    def clear(self) -> None:
        with self._lock:
            self._items.clear()

    def push(self, item: T) -> None:
        with self._lock:
            self._items.append(item)

    def latest(self) -> Optional[T]:
        with self._lock:
            if not self._items:
                return None
            return self._items[-1]

    def last_n(self, count: int) -> List[T]:
        if count <= 0:
            return []
        with self._lock:
            items = list(self._items)
        return items[-count:]

    def __len__(self) -> int:
        with self._lock:
            return len(self._items)

