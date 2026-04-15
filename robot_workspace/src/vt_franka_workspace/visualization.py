from __future__ import annotations

from pathlib import Path

import numpy as np


def export_episode_composite_video(
    episode_dir_or_npz: str | Path,
    output_path: str | Path | None = None,
    *,
    fps: float | None = None,
) -> Path:
    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - runtime dependency
        raise RuntimeError("OpenCV is required for composite episode visualization") from exc

    episode_dir, npz_path = _resolve_episode_paths(episode_dir_or_npz)
    data = np.load(npz_path, allow_pickle=True)

    timestamps = np.asarray(data["timestamps"], dtype=np.float64)
    action_pose = _pose7_array_to_xyz_rpy(np.asarray(data["teleop_target_tcp"], dtype=np.float64))
    proprio_pose = _pose7_array_to_xyz_rpy(np.asarray(data["robot_tcp_pose"], dtype=np.float64))
    rgb_paths = np.asarray(data["orbbec_rgb_frame_paths"], dtype=object)
    if timestamps.size == 0:
        raise RuntimeError(f"No aligned timestamps found in {npz_path}")

    if output_path is None:
        output_path = episode_dir / "episode_composite.mp4"
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    export_fps = _infer_video_fps(timestamps, fps)
    video_size = (1920, 1080)
    writer = cv2.VideoWriter(str(output_path), cv2.VideoWriter_fourcc(*"mp4v"), export_fps, video_size)
    if not writer.isOpened():
        raise RuntimeError(f"Failed to open video writer for {output_path}")

    try:
        for index in range(len(timestamps)):
            frame = _render_composite_frame(
                index=index,
                episode_dir=episode_dir,
                timestamps=timestamps,
                rgb_paths=rgb_paths,
                action_pose=action_pose,
                proprio_pose=proprio_pose,
                canvas_width=video_size[0],
                canvas_height=video_size[1],
                cv2=cv2,
            )
            writer.write(frame)
    finally:
        writer.release()

    return output_path


def _resolve_episode_paths(episode_dir_or_npz: str | Path) -> tuple[Path, Path]:
    path = Path(episode_dir_or_npz)
    if path.is_dir():
        npz_path = path / "aligned_episode.npz"
        return path, npz_path
    if path.name != "aligned_episode.npz":
        raise ValueError("Expected an episode directory or aligned_episode.npz path")
    return path.parent, path


def _pose7_array_to_xyz_rpy(pose7: np.ndarray) -> np.ndarray:
    from scipy.spatial.transform import Rotation

    pose7 = np.asarray(pose7, dtype=np.float64)
    out = np.zeros((pose7.shape[0], 6), dtype=np.float64)
    out[:, :3] = pose7[:, :3]
    quat_xyzw = np.column_stack([pose7[:, 4], pose7[:, 5], pose7[:, 6], pose7[:, 3]])
    out[:, 3:] = Rotation.from_quat(quat_xyzw).as_euler("xyz", degrees=True)
    return out


def _infer_video_fps(timestamps: np.ndarray, fps: float | None) -> float:
    if fps is not None:
        return float(fps)
    timestamps = np.asarray(timestamps, dtype=np.float64)
    if timestamps.size < 2:
        return 10.0
    dt = np.diff(timestamps)
    dt = dt[dt > 0.0]
    if dt.size == 0:
        return 10.0
    return float(1.0 / np.median(dt))


def _render_composite_frame(
    *,
    index: int,
    episode_dir: Path,
    timestamps: np.ndarray,
    rgb_paths: np.ndarray,
    action_pose: np.ndarray,
    proprio_pose: np.ndarray,
    canvas_width: int,
    canvas_height: int,
    cv2,
) -> np.ndarray:
    canvas = np.full((canvas_height, canvas_width, 3), 246, dtype=np.uint8)
    _draw_header(canvas, episode_dir.name, index, timestamps, cv2)

    left_x0 = 36
    left_y0 = 92
    left_width = 760
    left_height = 952
    right_x0 = 830
    right_y0 = 92
    right_width = canvas_width - right_x0 - 36
    right_height = 952

    _draw_rgb_panel(
        canvas,
        x=left_x0,
        y=left_y0,
        width=left_width,
        height=left_height,
        episode_dir=episode_dir,
        rgb_paths=rgb_paths,
        index=index,
        timestamps=timestamps,
        cv2=cv2,
    )

    labels = ["x", "y", "z", "roll", "pitch", "yaw"]
    colors = [
        (74, 73, 209),
        (73, 174, 237),
        (140, 121, 0),
        (142, 99, 48),
        (64, 15, 95),
        (143, 157, 42),
    ]
    _draw_pose_grid(
        canvas,
        x=right_x0,
        y=right_y0,
        width=right_width,
        height=right_height // 2 - 12,
        section_title="Action / Policy Label",
        labels=labels,
        colors=colors,
        times=timestamps - timestamps[0],
        values=action_pose,
        current_index=index,
        cv2=cv2,
    )
    _draw_pose_grid(
        canvas,
        x=right_x0,
        y=right_y0 + right_height // 2 + 12,
        width=right_width,
        height=right_height // 2 - 12,
        section_title="Proprioception",
        labels=labels,
        colors=colors,
        times=timestamps - timestamps[0],
        values=proprio_pose,
        current_index=index,
        cv2=cv2,
    )
    return canvas


def _draw_header(canvas: np.ndarray, episode_name: str, index: int, timestamps: np.ndarray, cv2) -> None:
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 74), (231, 236, 243), thickness=-1)
    cv2.putText(
        canvas,
        f"Episode Composite: {episode_name}",
        (32, 42),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (34, 40, 49),
        2,
        cv2.LINE_AA,
    )
    elapsed = float(timestamps[index] - timestamps[0])
    cv2.putText(
        canvas,
        f"frame={index:04d}   t={elapsed:6.2f}s",
        (32, 66),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (70, 78, 89),
        2,
        cv2.LINE_AA,
    )


def _draw_rgb_panel(
    canvas: np.ndarray,
    *,
    x: int,
    y: int,
    width: int,
    height: int,
    episode_dir: Path,
    rgb_paths: np.ndarray,
    index: int,
    timestamps: np.ndarray,
    cv2,
) -> None:
    _draw_panel_card(canvas, x, y, width, height, title="RGB")
    image = _load_rgb_frame(episode_dir, rgb_paths, index, cv2)
    content_x = x + 18
    content_y = y + 54
    content_w = width - 36
    content_h = height - 122

    if image is None:
        cv2.putText(
            canvas,
            "No RGB frame for this aligned step",
            (content_x + 20, content_y + 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (90, 90, 90),
            2,
            cv2.LINE_AA,
        )
    else:
        fitted = _fit_image(image, content_w, content_h, cv2)
        ih, iw = fitted.shape[:2]
        offset_x = content_x + (content_w - iw) // 2
        offset_y = content_y + (content_h - ih) // 2
        canvas[offset_y : offset_y + ih, offset_x : offset_x + iw] = fitted

    strip_y = y + height - 78
    _draw_episode_thumbnail_strip(
        canvas,
        x=content_x,
        y=strip_y,
        width=content_w,
        height=46,
        episode_dir=episode_dir,
        rgb_paths=rgb_paths,
        current_index=index,
        timestamps=timestamps,
        cv2=cv2,
    )


def _draw_episode_thumbnail_strip(
    canvas: np.ndarray,
    *,
    x: int,
    y: int,
    width: int,
    height: int,
    episode_dir: Path,
    rgb_paths: np.ndarray,
    current_index: int,
    timestamps: np.ndarray,
    cv2,
) -> None:
    valid = [(idx, str(path)) for idx, path in enumerate(rgb_paths) if str(path)]
    if not valid:
        return
    count = min(10, len(valid))
    selected = [valid[i] for i in np.linspace(0, len(valid) - 1, count, dtype=int)]
    thumb_gap = 8
    thumb_w = max(24, (width - thumb_gap * (count - 1)) // count)
    for thumb_idx, (frame_idx, rel_path) in enumerate(selected):
        frame = _load_rgb_frame(episode_dir, rgb_paths, frame_idx, cv2)
        thumb_x = x + thumb_idx * (thumb_w + thumb_gap)
        thumb_rect = (thumb_x, y, thumb_w, height)
        if frame is not None:
            fitted = _fit_image(frame, thumb_w, height, cv2)
            ih, iw = fitted.shape[:2]
            ox = thumb_x + (thumb_w - iw) // 2
            oy = y + (height - ih) // 2
            canvas[oy : oy + ih, ox : ox + iw] = fitted
        border_color = (30, 30, 30)
        border_thickness = 1
        if frame_idx == current_index:
            border_color = (0, 120, 255)
            border_thickness = 3
        cv2.rectangle(
            canvas,
            (thumb_rect[0], thumb_rect[1]),
            (thumb_rect[0] + thumb_rect[2], thumb_rect[1] + thumb_rect[3]),
            border_color,
            border_thickness,
        )
    cv2.putText(
        canvas,
        f"{timestamps[current_index] - timestamps[0]:.2f}s",
        (x, y + height + 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (80, 80, 80),
        1,
        cv2.LINE_AA,
    )


def _draw_pose_grid(
    canvas: np.ndarray,
    *,
    x: int,
    y: int,
    width: int,
    height: int,
    section_title: str,
    labels: list[str],
    colors: list[tuple[int, int, int]],
    times: np.ndarray,
    values: np.ndarray,
    current_index: int,
    cv2,
) -> None:
    _draw_panel_card(canvas, x, y, width, height, title=section_title)
    cols = 3
    rows = 2
    inner_x = x + 18
    inner_y = y + 54
    inner_w = width - 36
    inner_h = height - 72
    gap_x = 14
    gap_y = 14
    panel_w = (inner_w - gap_x * (cols - 1)) // cols
    panel_h = (inner_h - gap_y * (rows - 1)) // rows
    for idx, (label, color) in enumerate(zip(labels, colors)):
        col = idx % cols
        row = idx // cols
        px = inner_x + col * (panel_w + gap_x)
        py = inner_y + row * (panel_h + gap_y)
        _draw_series_panel(
            canvas,
            x=px,
            y=py,
            width=panel_w,
            height=panel_h,
            title=label,
            color=color,
            times=times,
            series=np.asarray(values[:, idx], dtype=np.float64),
            current_index=current_index,
            cv2=cv2,
        )


def _draw_series_panel(
    canvas: np.ndarray,
    *,
    x: int,
    y: int,
    width: int,
    height: int,
    title: str,
    color: tuple[int, int, int],
    times: np.ndarray,
    series: np.ndarray,
    current_index: int,
    cv2,
) -> None:
    cv2.rectangle(canvas, (x, y), (x + width, y + height), (226, 226, 226), thickness=-1)
    cv2.rectangle(canvas, (x, y), (x + width, y + height), (210, 210, 210), thickness=1)
    cv2.putText(canvas, title, (x + 10, y + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (36, 36, 36), 2, cv2.LINE_AA)

    plot_x = x + 10
    plot_y = y + 34
    plot_w = width - 108
    plot_h = height - 48
    indicator_x = x + width - 92
    indicator_y = y + 42
    indicator_w = 78
    indicator_h = height - 56

    _draw_line_plot(
        canvas,
        x=plot_x,
        y=plot_y,
        width=plot_w,
        height=plot_h,
        times=times,
        series=series,
        current_index=current_index,
        color=color,
        cv2=cv2,
    )
    _draw_arrow_indicator(
        canvas,
        x=indicator_x,
        y=indicator_y,
        width=indicator_w,
        height=indicator_h,
        current_value=float(series[current_index]),
        series=series,
        color=color,
        cv2=cv2,
    )


def _draw_line_plot(
    canvas: np.ndarray,
    *,
    x: int,
    y: int,
    width: int,
    height: int,
    times: np.ndarray,
    series: np.ndarray,
    current_index: int,
    color: tuple[int, int, int],
    cv2,
) -> None:
    cv2.rectangle(canvas, (x, y), (x + width, y + height), (251, 251, 248), thickness=-1)
    cv2.rectangle(canvas, (x, y), (x + width, y + height), (210, 210, 210), thickness=1)

    ymin = float(np.min(series))
    ymax = float(np.max(series))
    if np.isclose(ymin, ymax):
        pad = 1.0 if np.isclose(ymin, 0.0) else abs(ymin) * 0.1
        ymin -= pad
        ymax += pad
    pad = 0.12 * (ymax - ymin)
    ymin -= pad
    ymax += pad

    times = np.asarray(times, dtype=np.float64)
    x_vals = np.linspace(x + 8, x + width - 8, len(series))
    pts = []
    for idx, value in enumerate(series):
        ratio = 0.5 if np.isclose(ymax, ymin) else (float(value) - ymin) / (ymax - ymin)
        py = int(round(y + height - 8 - ratio * (height - 16)))
        px = int(round(x_vals[idx]))
        pts.append([px, py])
    pts_np = np.asarray(pts, dtype=np.int32).reshape(-1, 1, 2)
    cv2.polylines(canvas, [pts_np], isClosed=False, color=color, thickness=2, lineType=cv2.LINE_AA)

    current_pt = tuple(int(v) for v in pts[current_index])
    cv2.circle(canvas, current_pt, 4, color, thickness=-1, lineType=cv2.LINE_AA)
    cv2.line(canvas, (current_pt[0], y + 4), (current_pt[0], y + height - 4), (190, 190, 190), 1, cv2.LINE_AA)

    cv2.putText(
        canvas,
        f"{series[current_index]: .3f}",
        (x + 10, y + height - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.46,
        (85, 85, 85),
        1,
        cv2.LINE_AA,
    )


def _draw_arrow_indicator(
    canvas: np.ndarray,
    *,
    x: int,
    y: int,
    width: int,
    height: int,
    current_value: float,
    series: np.ndarray,
    color: tuple[int, int, int],
    cv2,
) -> None:
    cv2.rectangle(canvas, (x, y), (x + width, y + height), (245, 242, 234), thickness=-1)
    cv2.rectangle(canvas, (x, y), (x + width, y + height), (190, 190, 190), thickness=1)

    base_y = y + height - 14
    center_x = x + width // 2
    half_base = width // 2 - 12
    cv2.line(canvas, (center_x - half_base, base_y), (center_x + half_base, base_y), (70, 70, 70), 3, cv2.LINE_AA)

    max_abs = max(abs(float(np.min(series))), abs(float(np.max(series))), 1e-6)
    angle_deg = 80.0 * float(np.clip(current_value / max_abs, -1.0, 1.0))
    length = min(width, height) * 0.34
    theta = np.deg2rad(90.0 - angle_deg)
    end_x = int(round(center_x + length * np.cos(theta)))
    end_y = int(round(base_y - length * np.sin(theta)))
    cv2.arrowedLine(
        canvas,
        (center_x, base_y),
        (end_x, end_y),
        color,
        thickness=3,
        line_type=cv2.LINE_AA,
        tipLength=0.18,
    )
    cv2.putText(
        canvas,
        f"{current_value:.3f}",
        (x + 8, y + 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (50, 50, 50),
        1,
        cv2.LINE_AA,
    )


def _draw_panel_card(canvas: np.ndarray, x: int, y: int, width: int, height: int, *, title: str) -> None:
    cv2 = __import__("cv2")
    cv2.rectangle(canvas, (x, y), (x + width, y + height), (236, 240, 245), thickness=-1)
    cv2.rectangle(canvas, (x, y), (x + width, y + height), (205, 212, 220), thickness=2)
    cv2.putText(canvas, title, (x + 14, y + 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 42, 49), 2, cv2.LINE_AA)


def _load_rgb_frame(episode_dir: Path, rgb_paths: np.ndarray, index: int, cv2):
    rel_path = str(rgb_paths[index])
    if not rel_path:
        return None
    frame_path = episode_dir / rel_path
    image = cv2.imread(str(frame_path), cv2.IMREAD_COLOR)
    return image


def _fit_image(image: np.ndarray, width: int, height: int, cv2) -> np.ndarray:
    ih, iw = image.shape[:2]
    scale = min(width / max(iw, 1), height / max(ih, 1))
    target_w = max(1, int(round(iw * scale)))
    target_h = max(1, int(round(ih * scale)))
    return cv2.resize(image, (target_w, target_h), interpolation=cv2.INTER_AREA)
