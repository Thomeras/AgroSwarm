"""
avoidance_logging.py — Simple JSONL logger for obstacle avoidance runs.

Creates per-process logs under:
  <workspace>/logs/avoidance_logs/
"""

from __future__ import annotations

import json
import os
import socket
import threading
import time
from pathlib import Path
from typing import Any


def _workspace_root_from(anchor: Path) -> Path:
    for parent in [anchor] + list(anchor.parents):
        if (parent / "CLAUDE.md").exists() and (parent / "src").is_dir():
            return parent
        if (parent / "src").is_dir() and (parent / "launch_files").is_dir():
            return parent
    return Path.cwd()


class AvoidanceRunLogger:
    def __init__(
        self,
        *,
        source: str,
        drone_id: int,
        run_label: str = "",
    ) -> None:
        root = _workspace_root_from(Path(__file__).resolve())
        logs_dir = root / "logs" / "avoidance_logs"
        logs_dir.mkdir(parents=True, exist_ok=True)

        stamp = time.strftime("%Y%m%d_%H%M%S")
        slug = run_label.strip() or f"{stamp}_{source}_drone{drone_id}"
        slug = "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in slug)
        path = logs_dir / f"{slug}.jsonl"
        if path.exists():
            path = logs_dir / f"{slug}_{os.getpid()}.jsonl"

        self.path = path
        self.logs_dir = logs_dir
        self.assets_dir = logs_dir / f"{path.stem}_assets"
        self.assets_dir.mkdir(parents=True, exist_ok=True)
        self.source = source
        self.drone_id = drone_id
        self._host = socket.gethostname()
        self._pid = os.getpid()
        self._lock = threading.Lock()
        self._fh = path.open("a", encoding="utf-8", buffering=1)

        self.log(
            "session_start",
            level="INFO",
            log_path=str(self.path),
            cwd=os.getcwd(),
        )

    def log(self, event: str, *, level: str = "INFO", **fields: Any) -> None:
        record = {
            "ts": round(time.time(), 3),
            "iso_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "level": level,
            "event": event,
            "source": self.source,
            "drone_id": self.drone_id,
            "pid": self._pid,
            "host": self._host,
        }
        record.update(fields)
        line = json.dumps(record, ensure_ascii=True, sort_keys=True)
        with self._lock:
            self._fh.write(line + "\n")

    def close(self) -> None:
        with self._lock:
            if self._fh.closed:
                return
            self._fh.write(
                json.dumps(
                    {
                        "ts": round(time.time(), 3),
                        "iso_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                        "level": "INFO",
                        "event": "session_end",
                        "source": self.source,
                        "drone_id": self.drone_id,
                        "pid": self._pid,
                        "host": self._host,
                    },
                    ensure_ascii=True,
                    sort_keys=True,
                )
                + "\n"
            )
            self._fh.close()
