"""
paths.py — canonical file paths for scout_control data files

Import this module instead of constructing paths manually:

    from scout_control.utils.paths import PERIMETER_FILE, GRID_FILE, HOME_POS_FILE

The workspace root is discovered at runtime by walking up from this file
until a directory containing CLAUDE.md is found (the project root marker).
Falls back to ~/scout_ws if the marker is not found.
"""

import os


def _find_ws_root() -> str:
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(12):
        if os.path.exists(os.path.join(d, "CLAUDE.md")):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    # Fallback — keeps compatibility with old ~/scout_ws layout
    return os.path.expanduser("~/scout_ws")


WS_ROOT        = _find_ws_root()
PERIMETERS_DIR = os.path.join(WS_ROOT, "perimeters")

PERIMETER_FILE = os.path.join(PERIMETERS_DIR, "field_perimeter.json")
FIELD_BOUNDARY_FILE = os.path.join(PERIMETERS_DIR, "field_boundary.json")
GRID_FILE      = os.path.join(PERIMETERS_DIR, "field_grid.json")
HOME_POS_FILE  = os.path.join(PERIMETERS_DIR, "home_positions.json")
SPAWN_ORIGINS_FILE = os.path.join(PERIMETERS_DIR, "spawn_origins.json")
SPRAY_LOG_FILE = os.path.join(WS_ROOT, "spray_log.json")
CELL_DATA_DIR  = os.path.join(WS_ROOT, "cell_data")
FIELD_MODEL_DIR = os.path.join(PERIMETERS_DIR, "field_model")
NO_GO_FILE      = os.path.join(FIELD_MODEL_DIR, "no_go_zones.json")
OBSTACLES_FILE  = os.path.join(FIELD_MODEL_DIR, "static_obstacles.json")
TERRAIN_FILE    = os.path.join(FIELD_MODEL_DIR, "terrain_map.json")
REPORTS_DIR     = os.path.join(WS_ROOT, "reports")
