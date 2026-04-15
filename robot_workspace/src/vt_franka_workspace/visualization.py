from __future__ import annotations

from pathlib import Path

import numpy as np


def visualize_aligned_episode(
    episode_dir_or_npz: str | Path,
    output_path: str | Path | None = None,
    *,
    max_rgb_frames: int = 12,
) -> Path:
    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - runtime dependency for visualization
        raise RuntimeError("OpenCV is required for episode visualization") from exc

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib import gridspec
        from matplotlib.patches import FancyArrowPatch, Rectangle
    except ImportError as exc:  # pragma: no cover - runtime dependency for visualization
        raise RuntimeError("matplotlib is required for episode visualization") from exc

    episode_dir, npz_path = _resolve_episode_paths(episode_dir_or_npz)
    data = np.load(npz_path, allow_pickle=True)

    timestamps = np.asarray(data["timestamps"], dtype=np.float64)
    action_pose = _pose7_array_to_xyz_rpy(np.asarray(data["teleop_target_tcp"], dtype=np.float64))
    proprio_pose = _pose7_array_to_xyz_rpy(np.asarray(data["robot_tcp_pose"], dtype=np.float64))
    rgb_paths = np.asarray(data["orbbec_rgb_frame_paths"], dtype=object)

    if output_path is None:
        output_path = episode_dir / "episode_visualization.png"
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    fig = plt.figure(figsize=(24, 18), constrained_layout=True)
    outer = gridspec.GridSpec(3, 1, figure=fig, height_ratios=[1.2, 1.5, 1.5])
    fig.suptitle(f"Episode Overview: {episode_dir.name}", fontsize=22, fontweight="bold")

    ax_rgb = fig.add_subplot(outer[0])
    _draw_rgb_strip(ax_rgb, episode_dir, rgb_paths, timestamps, cv2, max_frames=max_rgb_frames)

    action_grid = outer[1].subgridspec(2, 3, wspace=0.18, hspace=0.26)
    proprio_grid = outer[2].subgridspec(2, 3, wspace=0.18, hspace=0.26)
    labels = ["x", "y", "z", "roll", "pitch", "yaw"]
    colors = ["#d1495b", "#edae49", "#00798c", "#30638e", "#5f0f40", "#2a9d8f"]

    for idx, (label, color) in enumerate(zip(labels, colors)):
        ax = fig.add_subplot(action_grid[idx // 3, idx % 3])
        _plot_pose_series(
            ax,
            timestamps,
            action_pose[:, idx],
            title=f"Action {label}",
            color=color,
            FancyArrowPatch=FancyArrowPatch,
            Rectangle=Rectangle,
        )

    for idx, (label, color) in enumerate(zip(labels, colors)):
        ax = fig.add_subplot(proprio_grid[idx // 3, idx % 3])
        _plot_pose_series(
            ax,
            timestamps,
            proprio_pose[:, idx],
            title=f"Proprioception {label}",
            color=color,
            FancyArrowPatch=FancyArrowPatch,
            Rectangle=Rectangle,
        )

    fig.savefig(output_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
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


def _draw_rgb_strip(ax, episode_dir: Path, rgb_paths, timestamps, cv2, *, max_frames: int) -> None:
    ax.set_title("RGB Across Episode", fontsize=16, loc="left", fontweight="bold")
    ax.axis("off")
    valid = [(idx, str(path)) for idx, path in enumerate(rgb_paths) if str(path)]
    if not valid:
        ax.text(0.5, 0.5, "No RGB frames recorded", ha="center", va="center", fontsize=16)
        return

    frame_indices = np.linspace(0, len(valid) - 1, min(max_frames, len(valid)), dtype=int)
    chosen = [valid[i] for i in frame_indices]
    tiles = []
    captions = []
    for idx, rel_path in chosen:
        frame_path = episode_dir / rel_path
        image = cv2.imread(str(frame_path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        tiles.append(_resize_to_tile(image, target_height=240))
        captions.append(f"{idx}\n{timestamps[idx] - timestamps[0]:.1f}s")

    if not tiles:
        ax.text(0.5, 0.5, "RGB frames could not be loaded", ha="center", va="center", fontsize=16)
        return

    strip = np.concatenate(tiles, axis=1)
    ax.imshow(strip)
    tile_width = tiles[0].shape[1]
    for i, caption in enumerate(captions):
        x = i * tile_width + tile_width / 2
        ax.text(
            x,
            strip.shape[0] + 12,
            caption,
            ha="center",
            va="top",
            fontsize=10,
            color="#222222",
        )
    ax.set_xlim(0, strip.shape[1])
    ax.set_ylim(strip.shape[0] + 42, 0)


def _resize_to_tile(image: np.ndarray, target_height: int) -> np.ndarray:
    h, w = image.shape[:2]
    scale = target_height / max(h, 1)
    target_width = max(1, int(round(w * scale)))
    try:
        import cv2
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError("OpenCV is required for episode visualization") from exc
    return cv2.resize(image, (target_width, target_height), interpolation=cv2.INTER_AREA)


def _plot_pose_series(ax, timestamps, values, *, title: str, color: str, FancyArrowPatch, Rectangle) -> None:
    times = np.asarray(timestamps, dtype=np.float64) - float(timestamps[0])
    values = np.asarray(values, dtype=np.float64)
    ax.plot(times, values, color=color, linewidth=2.3)
    ax.set_title(title, fontsize=14, loc="left", fontweight="bold")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Value")
    ax.grid(True, alpha=0.25, linewidth=0.8)
    ax.set_facecolor("#fbfbf8")

    ymin = float(np.min(values))
    ymax = float(np.max(values))
    if np.isclose(ymin, ymax):
        pad = 1.0 if np.isclose(ymin, 0.0) else abs(ymin) * 0.1
        ymin -= pad
        ymax += pad
    pad = 0.12 * (ymax - ymin)
    ymin -= pad
    ymax += pad
    ax.set_ylim(ymin, ymax)

    indicator = ax.inset_axes([0.69, 0.08, 0.27, 0.32])
    indicator.set_xlim(-1.2, 1.2)
    indicator.set_ylim(-0.1, 1.1)
    indicator.axis("off")
    indicator.add_patch(Rectangle((-0.9, 0.0), 1.8, 0.9, linewidth=1.2, edgecolor="#aaaaaa", facecolor="#f5f2ea"))
    indicator.plot([-0.6, 0.6], [0.08, 0.08], color="#555555", linewidth=3, solid_capstyle="round")

    current = float(values[-1])
    max_abs = max(abs(float(np.min(values))), abs(float(np.max(values))), 1e-6)
    angle_deg = 80.0 * np.clip(current / max_abs, -1.0, 1.0)
    length = 0.62
    theta = np.deg2rad(90.0 - angle_deg)
    x_end = length * np.cos(theta)
    y_end = 0.08 + length * np.sin(theta)
    arrow = FancyArrowPatch(
        (0.0, 0.08),
        (x_end, y_end),
        arrowstyle="-|>",
        mutation_scale=18,
        linewidth=3,
        color=color,
    )
    indicator.add_patch(arrow)
    indicator.text(0.0, -0.02, f"{current:.3f}", ha="center", va="top", fontsize=10, color="#222222")
