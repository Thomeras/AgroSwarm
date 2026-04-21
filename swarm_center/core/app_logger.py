from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime

from PyQt6.QtCore import QObject, pyqtSignal


@dataclass
class AppLogEntry:
    ts: str
    level: str
    source: str
    message: str

    def format_line(self) -> str:
        return f"{self.ts} [{self.level}] [{self.source}] {self.message}"


class AppLogger(QObject):
    entry_added = pyqtSignal(object)   # AppLogEntry

    def __init__(self) -> None:
        super().__init__()

    def debug(self, source: str, message: str) -> None:
        self._append("DEBUG", source, message)

    def info(self, source: str, message: str) -> None:
        self._append("INFO", source, message)

    def warn(self, source: str, message: str) -> None:
        self._append("WARN", source, message)

    def error(self, source: str, message: str) -> None:
        self._append("ERROR", source, message)

    def _append(self, level: str, source: str, message: str) -> None:
        entry = AppLogEntry(
            ts=datetime.now().strftime("%H:%M:%S"),
            level=level,
            source=source,
            message=str(message),
        )
        self.entry_added.emit(entry)
