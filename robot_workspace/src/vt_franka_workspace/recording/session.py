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
        self.active_run_path = self.root_dir / "active_run.json"
        self.active_episode_path = self.root_dir / "active_episode.json"

    def start_run(self, name: str, metadata: dict[str, Any] | None = None) -> Path:
        if self.active_run_path.exists():
            data = json.loads(self.active_run_path.read_text(encoding="utf-8"))
            raise RuntimeError(f"A run is already active: {data['run_dir']}")

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        slug = _slugify(name) or "run"
        run_id = f"{slug}_{timestamp}"
        run_dir = self.root_dir / run_id
        episodes_dir = run_dir / "episodes"
        episodes_dir.mkdir(parents=True, exist_ok=False)

        manifest = {
            "run_id": run_id,
            "run_name": name,
            "run_dir": str(run_dir),
            "episodes_dir": str(episodes_dir),
            "started_at_wall_time": time.time(),
            "metadata": metadata or {},
            "next_episode_index": 0,
        }
        self._write_json(run_dir / "run_manifest.json", manifest)
        self._write_json(self.active_run_path, manifest)
        return run_dir

    def stop_run(self, metadata_updates: dict[str, Any] | None = None) -> None:
        run_data = self._read_active_run()
        if run_data is None:
            return
        run_dir = Path(run_data["run_dir"])
        run_manifest_path = run_dir / "run_manifest.json"
        manifest = self._read_json(run_manifest_path)
        manifest["stopped_at_wall_time"] = time.time()
        if metadata_updates:
            manifest.setdefault("metadata", {}).update(metadata_updates)
        self._write_json(run_manifest_path, manifest)
        self.active_run_path.unlink(missing_ok=True)

    def start_episode(self, name: str | None = None, metadata: dict[str, Any] | None = None) -> Path:
        run_data = self._read_active_run()
        if run_data is None:
            raise RuntimeError("No active run. Start a run before starting an episode.")
        if self.active_episode_path.exists():
            active_episode = json.loads(self.active_episode_path.read_text(encoding="utf-8"))
            raise RuntimeError(f"An episode is already active: {active_episode['episode_dir']}")

        run_dir = Path(run_data["run_dir"])
        run_manifest_path = run_dir / "run_manifest.json"
        run_manifest = self._read_json(run_manifest_path)
        episode_index = int(run_manifest.get("next_episode_index", 0))

        episode_id = f"episode_{episode_index:04d}"
        episode_dir = run_dir / "episodes" / episode_id
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
        self._write_json(self.active_episode_path, manifest)

        run_manifest["next_episode_index"] = episode_index + 1
        self._write_json(run_manifest_path, run_manifest)
        self._write_json(self.active_run_path, run_manifest)
        return episode_dir

    def stop_episode(self, outcome: str = "saved", metadata_updates: dict[str, Any] | None = None) -> Path | None:
        active_episode = self._read_active_episode()
        if active_episode is None:
            return None
        episode_dir = Path(active_episode["episode_dir"])
        manifest_path = episode_dir / "episode_manifest.json"
        manifest = self._read_json(manifest_path)
        manifest["stopped_at_wall_time"] = time.time()
        manifest["outcome"] = outcome
        if metadata_updates:
            manifest.setdefault("metadata", {}).update(metadata_updates)
        self._write_json(manifest_path, manifest)
        self.active_episode_path.unlink(missing_ok=True)
        return episode_dir

    def get_active_run_dir(self) -> Path | None:
        data = self._read_active_run()
        if data is None:
            return None
        return Path(data["run_dir"])

    def get_active_episode_dir(self) -> Path | None:
        data = self._read_active_episode()
        if data is None:
            return None
        return Path(data["episode_dir"])

    def record_operator_event(self, event_type: str, payload: dict[str, Any] | None = None) -> None:
        run_dir = self.get_active_run_dir()
        if run_dir is None:
            return
        path = run_dir / "operator_events.jsonl"
        record = {
            "event_type": event_type,
            "recorded_at_wall_time": time.time(),
            "payload": payload or {},
        }
        with path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record))
            handle.write("\n")

    def write_latest_status(self, payload: dict[str, Any]) -> None:
        run_dir = self.get_active_run_dir()
        if run_dir is None:
            return
        self._write_json(run_dir / "latest_status.json", payload)

    def _read_active_run(self) -> dict[str, Any] | None:
        if not self.active_run_path.exists():
            return None
        return self._read_json(self.active_run_path)

    def _read_active_episode(self) -> dict[str, Any] | None:
        if not self.active_episode_path.exists():
            return None
        return self._read_json(self.active_episode_path)

    @staticmethod
    def _read_json(path: Path) -> dict[str, Any]:
        return json.loads(path.read_text(encoding="utf-8"))

    @staticmethod
    def _write_json(path: Path, payload: dict[str, Any]) -> None:
        path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def _slugify(value: str) -> str:
    value = value.strip().lower()
    value = re.sub(r"[^a-z0-9]+", "_", value)
    return value.strip("_")
