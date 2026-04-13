from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any


class EpisodeSessionManager:
    def __init__(self, root_dir: str | Path) -> None:
        self.root_dir = Path(root_dir)
        self.root_dir.mkdir(parents=True, exist_ok=True)
        self.active_path = self.root_dir / "active_episode.json"

    def start_episode(self, name: str | None = None, metadata: dict[str, Any] | None = None) -> Path:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        suffix = f"_{name}" if name else ""
        episode_dir = self.root_dir / f"{timestamp}{suffix}"
        episode_dir.mkdir(parents=True, exist_ok=False)
        manifest = {
            "episode_dir": str(episode_dir),
            "started_at_wall_time": time.time(),
            "metadata": metadata or {},
        }
        (episode_dir / "episode_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        self.active_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        return episode_dir

    def stop_episode(self) -> None:
        if self.active_path.exists():
            self.active_path.unlink()

    def get_active_episode_dir(self) -> Path | None:
        if not self.active_path.exists():
            return None
        data = json.loads(self.active_path.read_text(encoding="utf-8"))
        return Path(data["episode_dir"])

