from __future__ import annotations

from typing import Any

import numpy as np


def decode_color_frame(frame: Any) -> np.ndarray:
    return decode_color_buffer(
        frame.get_data(),
        width=int(frame.get_width()),
        height=int(frame.get_height()),
        color_format=_normalize_color_format(frame.get_format()),
    )


def decode_color_buffer(data: Any, *, width: int, height: int, color_format: str) -> np.ndarray:
    cv2 = _require_cv2()
    buffer = _as_uint8_array(data)
    format_name = _normalize_color_format(color_format)

    if format_name == "RGB":
        return cv2.cvtColor(buffer.reshape((height, width, 3)), cv2.COLOR_RGB2BGR)
    if format_name == "BGR":
        return buffer.reshape((height, width, 3)).copy()
    if format_name == "MJPG":
        image = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("Failed to decode MJPG Orbbec frame")
        return image
    if format_name == "YUYV":
        return cv2.cvtColor(buffer.reshape((height, width, 2)), cv2.COLOR_YUV2BGR_YUY2)
    if format_name == "UYVY":
        return cv2.cvtColor(buffer.reshape((height, width, 2)), cv2.COLOR_YUV2BGR_UYVY)
    if format_name == "NV12":
        return cv2.cvtColor(buffer.reshape((height * 3 // 2, width)), cv2.COLOR_YUV2BGR_NV12)
    if format_name == "NV21":
        return cv2.cvtColor(buffer.reshape((height * 3 // 2, width)), cv2.COLOR_YUV2BGR_NV21)
    if format_name == "I420":
        return cv2.cvtColor(buffer.reshape((height * 3 // 2, width)), cv2.COLOR_YUV2BGR_I420)
    raise ValueError(f"Unsupported Orbbec color format: {format_name}")


def _as_uint8_array(data: Any) -> np.ndarray:
    if isinstance(data, np.ndarray):
        return data.astype(np.uint8, copy=False).reshape(-1)
    return np.frombuffer(data, dtype=np.uint8)


def _normalize_color_format(color_format: Any) -> str:
    if hasattr(color_format, "name"):
        return str(color_format.name).upper()
    return str(color_format).split(".")[-1].upper()


def _require_cv2():
    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - depends on local runtime setup
        raise RuntimeError("OpenCV is required for Orbbec frame decoding") from exc
    return cv2
