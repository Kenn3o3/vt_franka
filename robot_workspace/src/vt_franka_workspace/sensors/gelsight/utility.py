from __future__ import annotations

import cv2
import numpy as np


class GelsightUtility:
    def __init__(
        self,
        rescale: int = 1,
        n_markers: int = 7,
        m_markers: int = 9,
        fps: int = 30,
        x0: int = 143,
        y0: int = 108,
        dx: int = 42,
        dy: int = 46,
    ) -> None:
        self.rescale = rescale
        self.n_markers = n_markers
        self.m_markers = m_markers
        self.fps = fps
        self.x0 = x0 / rescale
        self.y0 = y0 / rescale
        self.dx = dx / rescale
        self.dy = dy / rescale

    def prepare_frame(self, frame: np.ndarray) -> np.ndarray:
        return cv2.resize(frame, (0, 0), fx=1.0 / self.rescale, fy=1.0 / self.rescale)

    def find_marker_mask(self, frame: np.ndarray) -> np.ndarray:
        marker_thresh = -5
        gaussian = np.int16(cv2.GaussianBlur(frame, (int(63 / self.rescale), int(63 / self.rescale)), 0))
        delta = frame.astype(np.float64) - gaussian.astype(np.float64)
        marker_mask = ((np.max(delta, axis=2)) < marker_thresh).astype(np.uint8)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (int(15 / self.rescale), int(15 / self.rescale)))
        marker_mask = cv2.morphologyEx(marker_mask, cv2.MORPH_CLOSE, kernel)

        height, width = marker_mask.shape
        w_margin = int(width // 2 - width // 2.5)
        h_margin = int(height // 2 - height // 2.5)
        marker_mask[:h_margin, :] = 0
        marker_mask[height - h_margin :, :] = 0
        marker_mask[:, :w_margin] = 0
        marker_mask[:, width - w_margin :] = 0
        return marker_mask * 255

    def marker_centers(self, mask: np.ndarray) -> list[list[float]]:
        area_thresh_min = 50 / self.rescale**2
        area_thresh_max = 1920 / self.rescale**2
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centers: list[list[float]] = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            aspect = abs(max(w, h) / max(min(w, h), 1) - 1)
            if not (area_thresh_min < area < area_thresh_max) or aspect >= 2:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue
            centers.append([moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]])
        return centers

