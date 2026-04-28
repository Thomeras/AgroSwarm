"""
field_model_loader.py — Loads the three field-model overlay JSONs.

Files are resolved relative to the workspace root (walks up to CLAUDE.md marker):
  perimeters/field_model/no_go_zones.json
  perimeters/field_model/static_obstacles.json
  perimeters/field_model/terrain_map.json

All three load functions return empty structures gracefully when files are absent.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from typing import Optional


def _find_ws_root() -> str:
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(12):
        if os.path.exists(os.path.join(d, "CLAUDE.md")):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return os.path.expanduser("~/scout_ws")


_WS_ROOT        = _find_ws_root()
_MODEL_DIR      = os.path.join(_WS_ROOT, "perimeters", "field_model")

NO_GO_FILE      = os.path.join(_MODEL_DIR, "no_go_zones.json")
OBSTACLES_FILE  = os.path.join(_MODEL_DIR, "static_obstacles.json")
TERRAIN_FILE    = os.path.join(_MODEL_DIR, "terrain_map.json")


@dataclass
class FieldModel:
    no_go_zones: list[dict] = field(default_factory=list)
    obstacles:   list[dict] = field(default_factory=list)
    terrain:     Optional[dict] = None


class FieldModelLoader:
    @staticmethod
    def load() -> FieldModel:
        return FieldModel(
            no_go_zones=_load_list(NO_GO_FILE),
            obstacles=_load_list(OBSTACLES_FILE),
            terrain=_load_dict(TERRAIN_FILE),
        )


def _load_list(path: str) -> list[dict]:
    if not os.path.isfile(path):
        return []
    try:
        with open(path) as f:
            data = json.load(f)
        if isinstance(data, list):
            return data
        if isinstance(data, dict):
            for key in ("zones", "obstacles", "items"):
                items = data.get(key)
                if isinstance(items, list):
                    return items
        return []
    except (OSError, json.JSONDecodeError):
        return []


def _load_dict(path: str) -> Optional[dict]:
    if not os.path.isfile(path):
        return None
    try:
        with open(path) as f:
            data = json.load(f)
        return data if isinstance(data, dict) else None
    except (OSError, json.JSONDecodeError):
        return None
