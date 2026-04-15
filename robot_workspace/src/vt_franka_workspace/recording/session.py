from __future__ import annotations

import json
import re
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


class RunSessionManager:
    def __init__(self, root_dir: str | Path) -> None:
        self.root_dir = Path(root_dir)
        self.root_dir.mkdir(parents=True, exist_ok=True)
        self._run_dir: Path | None = None
        self._run_manifest: dict[str, Any] | None = None
        self._active_episode_dir: Path | None = None

    def start_run(self, name: str, metadata: dict[str, Any] | None = None, *, resume: bool = True) -> Path:
        if self._run_dir is not None:
            return self._run_dir
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        slug = _slugify(name) or "run"
        if resume:
            latest_run_dir = self._find_latest_run_dir(slug)
            if latest_run_dir is not None:
                self._ensure_latest_episode_finished(latest_run_dir)
                manifest = self._read_json(latest_run_dir / "run_manifest.json")
                manifest.setdefault("metadata", {})
                if metadata:
                    manifest["metadata"].update(metadata)
                manifest["resumed_at_wall_time"] = time.time()
                self._write_json(latest_run_dir / "run_manifest.json", manifest)
                self._run_dir = latest_run_dir
                self._run_manifest = manifest
                return latest_run_dir

        run_id = f"{slug}_{timestamp}"
        self._run_dir = self.root_dir / run_id
        episodes_dir = self._run_dir / "episodes"
        episodes_dir.mkdir(parents=True, exist_ok=False)

        self._run_manifest = {
            "run_id": run_id,
            "run_name": name,
            "run_dir": str(self._run_dir),
            "episodes_dir": str(episodes_dir),
            "started_at_wall_time": time.time(),
            "metadata": metadata or {},
        }
        self._write_json(self._run_dir / "run_manifest.json", self._run_manifest)
        return self._run_dir

    def stop_run(self, metadata_updates: dict[str, Any] | None = None) -> None:
        if self._run_dir is None or self._run_manifest is None:
            return
        run_manifest_path = self._run_dir / "run_manifest.json"
        manifest = self._read_json(run_manifest_path)
        manifest["stopped_at_wall_time"] = time.time()
        if metadata_updates:
            manifest.setdefault("metadata", {}).update(metadata_updates)
        self._write_json(run_manifest_path, manifest)
        self._run_manifest = manifest

    def start_episode(self, name: str | None = None, metadata: dict[str, Any] | None = None) -> Path:
        if self._run_dir is None or self._run_manifest is None:
            raise RuntimeError("No active run. Start a run before starting an episode.")
        if self._active_episode_dir is not None:
            raise RuntimeError(f"An episode is already active: {self._active_episode_dir}")

        run_manifest_path = self._run_dir / "run_manifest.json"
        run_manifest = self._read_json(run_manifest_path)
        episode_index = self.get_next_episode_index()
        episode_id = f"episode_{episode_index:04d}"
        episode_dir = self._run_dir / "episodes" / episode_id
        episode_dir.mkdir(parents=True, exist_ok=False)

        manifest = {
            "run_id": run_manifest["run_id"],
            "episode_id": episode_id,
            "episode_index": episode_index,
            "episode_name": name,
            "episode_dir": str(episode_dir),
            "started_at_wall_time": time.time(),
            "metadata": metadata or {},
            "outcome": "recording",
        }
        self._write_json(episode_dir / "episode_manifest.json", manifest)
        self._active_episode_dir = episode_dir
        return episode_dir

    def stop_episode(self, outcome: str = "saved", metadata_updates: dict[str, Any] | None = None) -> Path | None:
        if self._active_episode_dir is None:
            return None
        episode_dir = self._active_episode_dir
        manifest_path = self._active_episode_dir / "episode_manifest.json"
        manifest = self._read_json(manifest_path)
        manifest["stopped_at_wall_time"] = time.time()
        manifest["outcome"] = outcome
        if metadata_updates:
            manifest.setdefault("metadata", {}).update(metadata_updates)
        self._write_json(manifest_path, manifest)
        self._active_episode_dir = None
        return episode_dir

    def get_active_run_dir(self) -> Path | None:
        return self._run_dir

    def get_active_episode_dir(self) -> Path | None:
        return self._active_episode_dir

    def get_next_episode_index(self) -> int:
        if self._run_dir is None:
            return 0
        episode_dirs = self._list_episode_dirs(self._run_dir)
        if not episode_dirs:
            return 0
        latest_index = max(int(path.name.split("_", maxsplit=1)[1]) for path in episode_dirs)
        return latest_index + 1

    def get_latest_saved_episode_dir(self) -> Path | None:
        if self._run_dir is None:
            return None
        for episode_dir in reversed(self._list_episode_dirs(self._run_dir)):
            manifest_path = episode_dir / "episode_manifest.json"
            if not manifest_path.exists():
                continue
            manifest = self._read_json(manifest_path)
            if manifest.get("outcome") == "saved":
                return episode_dir
        return None

    def discard_episode(self, episode_dir: str | Path) -> None:
        episode_dir = Path(episode_dir)
        if self._active_episode_dir is not None and episode_dir == self._active_episode_dir:
            raise RuntimeError("Cannot discard an active episode")
        if not episode_dir.exists():
            raise FileNotFoundError(f"Episode does not exist: {episode_dir}")
        import shutil

        shutil.rmtree(episode_dir)

    def record_operator_event(self, event_type: str, payload: dict[str, Any] | None = None) -> None:
        if self._run_dir is None:
            return
        path = self._run_dir / "operator_events.jsonl"
        record = {
            "event_type": event_type,
            "recorded_at_wall_time": time.time(),
            "payload": payload or {},
        }
        with path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record))
            handle.write("\n")

    def write_latest_status(self, payload: dict[str, Any]) -> None:
        if self._run_dir is None:
            return
        self._write_json(self._run_dir / "latest_status.json", payload)

    @staticmethod
    def _read_json(path: Path) -> dict[str, Any]:
        return json.loads(path.read_text(encoding="utf-8"))

    @staticmethod
    def _write_json(path: Path, payload: dict[str, Any]) -> None:
        path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    def _find_latest_run_dir(self, slug: str) -> Path | None:
        candidates = sorted(path for path in self.root_dir.glob(f"{slug}_*") if path.is_dir())
        if not candidates:
            return None
        return candidates[-1]

    def _ensure_latest_episode_finished(self, run_dir: Path) -> None:
        episode_dirs = self._list_episode_dirs(run_dir)
        if not episode_dirs:
            return
        latest_episode = episode_dirs[-1]
        manifest_path = latest_episode / "episode_manifest.json"
        if not manifest_path.exists():
            raise RuntimeError(f"Latest episode is missing its manifest: {latest_episode}")
        manifest = self._read_json(manifest_path)
        if manifest.get("outcome") == "recording" or "stopped_at_wall_time" not in manifest:
            raise RuntimeError(
                f"Latest episode appears unfinished: {latest_episode}. Handle it manually before resuming this run."
            )

    @staticmethod
    def _list_episode_dirs(run_dir: Path) -> list[Path]:
        episodes_dir = run_dir / "episodes"
        if not episodes_dir.exists():
            return []
        return sorted(path for path in episodes_dir.glob("episode_*") if path.is_dir())


def _slugify(value: str) -> str:
    value = value.strip().lower()
    value = re.sub(r"[^a-z0-9]+", "_", value)
    return value.strip("_")
