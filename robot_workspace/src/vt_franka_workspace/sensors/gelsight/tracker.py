from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np

from .utility import GelsightUtility


def _load_find_marker_module():
    library_path = Path(__file__).resolve().parent / "lib" / "find_marker.so"
    spec = importlib.util.spec_from_file_location("find_marker", library_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load marker tracker extension from {library_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class GelsightMarkerTracker:
    def __init__(self, utility: GelsightUtility | None = None) -> None:
        self.utility = utility or GelsightUtility()
        self._find_marker = _load_find_marker_module()
        self._matcher = self._find_marker.Matching(
            N_=self.utility.n_markers,
            M_=self.utility.m_markers,
            fps_=self.utility.fps,
            x0_=self.utility.x0,
            y0_=self.utility.y0,
            dx_=self.utility.dx,
            dy_=self.utility.dy,
        )

    def process_frame(self, frame: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        prepared = self.utility.prepare_frame(frame)
        mask = self.utility.find_marker_mask(prepared)
        marker_centers = self.utility.marker_centers(mask)
        if len(marker_centers) < 8:
            raise RuntimeError("Too few GelSight markers detected to initialize tracking")

        self._matcher.init(marker_centers)
        self._matcher.run()
        ox, oy, cx, cy, _ = self._matcher.get_flow()
        rows = len(ox)
        cols = len(ox[0])
        initial_markers = np.zeros((rows * cols, 3), dtype=np.float32)
        marker_offsets = np.zeros((rows * cols, 2), dtype=np.float32)
        index = 0
        for row in range(rows):
            for col in range(cols):
                initial_markers[index] = [ox[row][col], oy[row][col], 0.0]
                marker_offsets[index] = [cx[row][col] - ox[row][col], cy[row][col] - oy[row][col]]
                index += 1
        return initial_markers, marker_offsets

    @staticmethod
    def normalize_markers(
        marker_locations: np.ndarray,
        marker_offsets: np.ndarray,
        width: int,
        height: int,
    ) -> tuple[np.ndarray, np.ndarray]:
        normalized_locations = marker_locations.copy()
        normalized_offsets = marker_offsets.copy()
        normalized_locations[:, 0] /= width
        normalized_locations[:, 1] /= height
        normalized_offsets[:, 0] /= width
        normalized_offsets[:, 1] /= height
        return normalized_locations, normalized_offsets
