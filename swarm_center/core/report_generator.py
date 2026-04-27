"""
report_generator.py — Post-mission HTML report generator

Pure Python — no ROS2 dependencies. Reads data files directly from disk.

Usage:
    path = ReportGenerator.generate(cells, mission_id)
    # path → reports/<mission_id>/report.html
"""

from __future__ import annotations

import json
import math
import os
import webbrowser
from datetime import datetime, timezone
from typing import Optional


# ── Workspace root discovery ─────────────────────────────────────────────────

def _ws_root() -> str:
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(15):
        if os.path.exists(os.path.join(d, "CLAUDE.md")):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return os.path.expanduser("~/scout_ws")


_WS = _ws_root()
_REPORTS_DIR   = os.path.join(_WS, "reports")
_SPRAY_LOG     = os.path.join(_WS, "spray_log.json")
_CELL_DATA_DIR = os.path.join(_WS, "cell_data")
_HOME_POS      = os.path.join(_WS, "perimeters", "home_positions.json")


# ── Public API ───────────────────────────────────────────────────────────────

class ReportGenerator:
    """
    Generate a self-contained HTML mission report from in-memory grid state
    and on-disk data files.

    `cells` is a list of dicts with keys:
        id, col, row, x (NED-N), y (NED-E), status
    Status values: "visited" | "unvisited" | "hovering" | "blocked" | "skipped"
    """

    @staticmethod
    def generate(
        cells: list[dict],
        mission_id: Optional[str] = None,
        cols: int = 0,
        rows: int = 0,
        cell_size_m: float = 5.0,
        open_browser: bool = True,
    ) -> str:
        if mission_id is None:
            mission_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")

        out_dir = os.path.join(_REPORTS_DIR, mission_id)
        os.makedirs(out_dir, exist_ok=True)

        # Save grid snapshot for future regeneration
        snapshot = {
            "mission_id": mission_id,
            "generated_utc": datetime.now(timezone.utc).isoformat(),
            "cols": cols or (max(c["col"] for c in cells) + 1 if cells else 0),
            "rows": rows or (max(c["row"] for c in cells) + 1 if cells else 0),
            "cell_size_m": cell_size_m,
            "cells": cells,
        }
        with open(os.path.join(out_dir, "grid_snapshot.json"), "w") as f:
            json.dump(snapshot, f)

        html = _build_report(cells, mission_id, snapshot["cols"], snapshot["rows"], cell_size_m)
        report_path = os.path.join(out_dir, "report.html")
        with open(report_path, "w", encoding="utf-8") as f:
            f.write(html)

        if open_browser:
            webbrowser.open(f"file://{report_path}")

        return report_path

    @staticmethod
    def regenerate(mission_id: str, open_browser: bool = True) -> Optional[str]:
        """Re-generate report from saved grid snapshot."""
        snap_path = os.path.join(_REPORTS_DIR, mission_id, "grid_snapshot.json")
        if not os.path.isfile(snap_path):
            return None
        with open(snap_path) as f:
            snap = json.load(f)
        return ReportGenerator.generate(
            cells=snap["cells"],
            mission_id=mission_id,
            cols=snap.get("cols", 0),
            rows=snap.get("rows", 0),
            cell_size_m=snap.get("cell_size_m", 5.0),
            open_browser=open_browser,
        )

    @staticmethod
    def last_mission_id() -> Optional[str]:
        """Return the most recently generated mission_id, or None."""
        if not os.path.isdir(_REPORTS_DIR):
            return None
        entries = [
            e for e in os.listdir(_REPORTS_DIR)
            if os.path.isfile(os.path.join(_REPORTS_DIR, e, "grid_snapshot.json"))
        ]
        return max(entries) if entries else None


# ── Data loading ─────────────────────────────────────────────────────────────

def _load_spray() -> list[dict]:
    if not os.path.isfile(_SPRAY_LOG):
        return []
    try:
        with open(_SPRAY_LOG) as f:
            data = json.load(f)
        return data if isinstance(data, list) else []
    except Exception:
        return []


def _load_cell_meta() -> list[dict]:
    """Return all visit meta.json records from cell_data/."""
    metas = []
    if not os.path.isdir(_CELL_DATA_DIR):
        return metas
    for cell_dir in os.scandir(_CELL_DATA_DIR):
        if not cell_dir.is_dir():
            continue
        for visit_dir in os.scandir(cell_dir.path):
            if not visit_dir.is_dir():
                continue
            meta_path = os.path.join(visit_dir.path, "meta.json")
            if os.path.isfile(meta_path):
                try:
                    with open(meta_path) as f:
                        metas.append(json.load(f))
                except Exception:
                    pass
    return metas


def _load_homes() -> list[dict]:
    if not os.path.isfile(_HOME_POS):
        return []
    try:
        with open(_HOME_POS) as f:
            d = json.load(f)
        return d.get("home_positions", [])
    except Exception:
        return []


# ── Stats computation ─────────────────────────────────────────────────────────

def _coverage_stats(cells: list[dict]) -> dict:
    total = len(cells)
    visited  = sum(1 for c in cells if c.get("status") == "visited")
    blocked  = sum(1 for c in cells if c.get("status") == "blocked")
    skipped  = sum(1 for c in cells if c.get("status") == "skipped")
    missed   = total - visited - blocked - skipped

    return {
        "total":   total,
        "visited": visited,
        "blocked": blocked,
        "skipped": skipped,
        "missed":  missed,
        "pct_visited": round(visited / total * 100, 1) if total else 0.0,
        "pct_missed":  round(missed  / total * 100, 1) if total else 0.0,
    }


def _spray_stats(spray: list[dict], cells: list[dict]) -> dict:
    visited_ids = {c["id"] for c in cells if c.get("status") == "visited"}
    sprayed_events = spray  # one entry per spray event
    cell_doses: dict[str, float] = {}
    for ev in sprayed_events:
        cid = ev.get("cell_id", "")
        cell_doses[cid] = cell_doses.get(cid, 0.0) + float(ev.get("dose_ml", 0.0))

    cells_sprayed  = len(cell_doses)
    total_dose     = sum(cell_doses.values())
    cells_missed   = len(visited_ids - cell_doses.keys())
    avg_dose       = total_dose / cells_sprayed if cells_sprayed else 0.0

    return {
        "total_dose_ml":  round(total_dose, 1),
        "cells_sprayed":  cells_sprayed,
        "cells_missed":   cells_missed,
        "avg_dose_ml":    round(avg_dose, 1),
        "by_cell":        cell_doses,
    }


def _blocked_events(cells: list[dict]) -> dict:
    blocked = [c["id"] for c in cells if c.get("status") == "blocked"]
    skipped = [c["id"] for c in cells if c.get("status") == "skipped"]
    return {
        "count": len(blocked) + len(skipped),
        "blocked_ids": blocked,
        "skipped_ids": skipped,
    }


def _drone_summaries(
    cells: list[dict],
    spray: list[dict],
    cell_meta: list[dict],
) -> dict[str, dict]:
    drones: dict[str, dict] = {}

    # Accumulate visit metadata by drone
    for m in cell_meta:
        did = m.get("drone_id", "unknown")
        if did not in drones:
            drones[did] = {"cells_visited": [], "spray_events": [], "flight_dist_m": 0.0}
        drones[did]["cells_visited"].append(m.get("cell_id", ""))

    # Accumulate spray events by drone
    for ev in spray:
        did = ev.get("drone_id", "unknown")
        if did not in drones:
            drones[did] = {"cells_visited": [], "spray_events": [], "flight_dist_m": 0.0}
        drones[did]["spray_events"].append(ev)

    # Estimate flight distance from cell centre trail
    cell_pos: dict[str, tuple[float, float]] = {
        c["id"]: (float(c.get("x", 0)), float(c.get("y", 0))) for c in cells
    }
    visit_by_drone: dict[str, list[dict]] = {}
    for m in cell_meta:
        did = m.get("drone_id", "unknown")
        visit_by_drone.setdefault(did, []).append(m)

    for did, visits in visit_by_drone.items():
        # Sort by timestamp
        try:
            visits_sorted = sorted(visits, key=lambda v: v.get("timestamp_utc", ""))
        except Exception:
            visits_sorted = visits

        dist = 0.0
        prev: Optional[tuple[float, float]] = None
        for v in visits_sorted:
            pos = cell_pos.get(v.get("cell_id", ""))
            if pos and prev:
                dist += math.hypot(pos[0] - prev[0], pos[1] - prev[1])
            prev = pos
        if did in drones:
            drones[did]["flight_dist_m"] = round(dist, 1)

    return drones


# ── SVG rendering ─────────────────────────────────────────────────────────────

_STATUS_COLOR = {
    "visited":   "#4cbb72",
    "hovering":  "#4cbb72",
    "blocked":   "#e67e22",
    "skipped":   "#f1c40f",
    "unvisited": "#c0392b",
}

_CELL_PX = 40


def _make_grid_svg(
    cells: list[dict],
    cols: int,
    rows: int,
    spray_by_cell: dict[str, float],
    max_dose: float,
) -> str:
    w = cols * _CELL_PX
    h = rows * _CELL_PX
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" '
        f'viewBox="0 0 {w} {h}" style="border:1px solid #444; border-radius:4px;">'
    ]

    for c in cells:
        col = int(c.get("col", 0))
        row = int(c.get("row", 0))
        # Flip row so North (high row index = high x = North) is at top
        svg_x = col * _CELL_PX
        svg_y = (rows - 1 - row) * _CELL_PX
        status = c.get("status", "unvisited")
        fill = _STATUS_COLOR.get(status, "#95a5a6")
        cell_id = c.get("id", "")

        parts.append(
            f'<rect x="{svg_x}" y="{svg_y}" width="{_CELL_PX}" height="{_CELL_PX}" '
            f'fill="{fill}" stroke="#1a1a2e" stroke-width="1">'
            f'<title>{cell_id} — {status}</title>'
            f'</rect>'
        )

        # Spray dose overlay: semi-transparent blue circle
        dose = spray_by_cell.get(cell_id, 0.0)
        if dose > 0 and max_dose > 0:
            alpha = round(0.25 + 0.55 * (dose / max_dose), 2)
            r = _CELL_PX * 0.38
            cx = svg_x + _CELL_PX / 2
            cy = svg_y + _CELL_PX / 2
            parts.append(
                f'<circle cx="{cx}" cy="{cy}" r="{r}" '
                f'fill="rgba(0,100,255,{alpha})" stroke="none">'
                f'<title>{cell_id}: {dose:.0f} ml</title>'
                f'</circle>'
            )

    # Compass rose (N label top-centre)
    cx = w // 2
    parts.append(
        f'<text x="{cx}" y="14" text-anchor="middle" '
        f'font-size="12" fill="#ccc" font-family="sans-serif">N ↑</text>'
    )

    parts.append("</svg>")
    return "\n".join(parts)


# ── HTML rendering ────────────────────────────────────────────────────────────

_CSS = """
* { box-sizing: border-box; margin: 0; padding: 0; }
body { background: #1a1a2e; color: #e0e0e0; font-family: 'Segoe UI', Arial, sans-serif;
       font-size: 14px; padding: 24px; }
h1 { color: #4cbb72; font-size: 22px; margin-bottom: 4px; }
h2 { color: #a0d0ff; font-size: 16px; margin: 20px 0 8px; border-bottom: 1px solid #333; padding-bottom: 4px; }
.meta { color: #888; font-size: 12px; margin-bottom: 20px; }
.grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(200px, 1fr)); gap: 12px; margin-bottom: 16px; }
.card { background: #16213e; border-radius: 8px; padding: 14px; }
.card .val { font-size: 28px; font-weight: bold; color: #fff; }
.card .lbl { font-size: 11px; color: #888; margin-top: 4px; text-transform: uppercase; }
.good  { color: #4cbb72; }
.warn  { color: #f1c40f; }
.bad   { color: #e74c3c; }
.info  { color: #5dade2; }
table { width: 100%; border-collapse: collapse; margin-top: 8px; }
th, td { padding: 8px 12px; text-align: left; border-bottom: 1px solid #2a2a4a; }
th { background: #16213e; color: #a0d0ff; font-weight: 600; font-size: 12px; text-transform: uppercase; }
tr:hover td { background: #1e2d4a; }
.legend { display: flex; gap: 16px; flex-wrap: wrap; margin: 10px 0; }
.legend-item { display: flex; align-items: center; gap: 6px; font-size: 12px; }
.legend-swatch { width: 16px; height: 16px; border-radius: 3px; border: 1px solid #444; }
.vis-wrap { overflow-x: auto; margin: 12px 0; }
.two-col { display: grid; grid-template-columns: 1fr auto; gap: 24px; align-items: start; }
@media (max-width: 700px) { .two-col { grid-template-columns: 1fr; } }
"""


def _stat_card(label: str, value: str, color_class: str = "") -> str:
    val_cls = f' {color_class}' if color_class else ""
    return (
        f'<div class="card">'
        f'<div class="val{val_cls}">{value}</div>'
        f'<div class="lbl">{label}</div>'
        f'</div>'
    )


def _build_report(
    cells: list[dict],
    mission_id: str,
    cols: int,
    rows: int,
    cell_size_m: float,
) -> str:
    spray      = _load_spray()
    cell_meta  = _load_cell_meta()
    homes      = _load_homes()

    cov  = _coverage_stats(cells)
    spr  = _spray_stats(spray, cells)
    blk  = _blocked_events(cells)
    dsum = _drone_summaries(cells, spray, cell_meta)

    max_dose = max(spr["by_cell"].values(), default=0.0)
    svg = _make_grid_svg(cells, cols, rows, spr["by_cell"], max_dose)

    gen_time = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    ts_pretty = (
        mission_id[:4] + "-" + mission_id[4:6] + "-" + mission_id[6:8] + " " +
        mission_id[9:11] + ":" + mission_id[11:13] + " UTC"
        if len(mission_id) >= 15 else mission_id
    )

    # ── Coverage cards
    cov_pct_cls = "good" if cov["pct_visited"] >= 90 else ("warn" if cov["pct_visited"] >= 70 else "bad")
    coverage_cards = (
        _stat_card("Total cells", str(cov["total"])) +
        _stat_card("Completed", f'{cov["visited"]}', "good") +
        _stat_card("Coverage", f'{cov["pct_visited"]}%', cov_pct_cls) +
        _stat_card("Missed", f'{cov["missed"]}', "bad" if cov["missed"] else "") +
        _stat_card("Blocked", f'{cov["blocked"]}', "warn" if cov["blocked"] else "") +
        _stat_card("Skipped", f'{cov["skipped"]}', "warn" if cov["skipped"] else "")
    )

    # ── Spray cards
    spray_cards = (
        _stat_card("Total dose", f'{spr["total_dose_ml"]} ml', "info") +
        _stat_card("Cells sprayed", str(spr["cells_sprayed"])) +
        _stat_card("Cells missed", str(spr["cells_missed"]), "warn" if spr["cells_missed"] else "") +
        _stat_card("Avg dose/cell", f'{spr["avg_dose_ml"]} ml')
    )

    # ── Per-drone table rows
    drone_rows = ""
    for did, info in sorted(dsum.items()):
        n_visited = len(set(info.get("cells_visited", [])))
        n_spray   = len(info.get("spray_events", []))
        dose      = sum(ev.get("dose_ml", 0) for ev in info.get("spray_events", []))
        dist      = info.get("flight_dist_m", 0.0)
        drone_rows += (
            f'<tr><td>{did}</td><td>{n_visited}</td>'
            f'<td>{n_spray}</td><td>{dose:.0f} ml</td>'
            f'<td>{dist:.0f} m</td></tr>'
        )
    if not drone_rows:
        drone_rows = '<tr><td colspan="5" style="color:#666">No per-drone data available</td></tr>'

    # ── Blocked cell list
    blk_detail = ""
    if blk["blocked_ids"]:
        blk_detail += f'<p><strong>Blocked:</strong> {", ".join(blk["blocked_ids"])}</p>'
    if blk["skipped_ids"]:
        blk_detail += f'<p><strong>Skipped:</strong> {", ".join(blk["skipped_ids"])}</p>'
    if not blk_detail:
        blk_detail = '<p style="color:#4cbb72">No blocked or skipped cells.</p>'

    # ── Home pads
    pads_html = ""
    for pad in homes:
        ned = pad.get("ned", {})
        pads_html += (
            f'<tr><td>{pad.get("pad_id","—")}</td>'
            f'<td>{pad.get("drone_id","—")}</td>'
            f'<td>{ned.get("x",0):.1f}, {ned.get("y",0):.1f}</td>'
            f'<td>{pad.get("status","—")}</td></tr>'
        )
    if not pads_html:
        pads_html = '<tr><td colspan="4" style="color:#666">No home position data</td></tr>'

    # ── Legend
    legend = "".join(
        f'<div class="legend-item"><div class="legend-swatch" style="background:{col}"></div>{lbl}</div>'
        for lbl, col in [
            ("Visited", "#4cbb72"),
            ("Missed", "#c0392b"),
            ("Blocked", "#e67e22"),
            ("Skipped", "#f1c40f"),
            ("Spray overlay", "rgba(0,100,255,0.55)"),
        ]
    )

    # ── Field dimensions
    field_w = cols * cell_size_m
    field_h = rows * cell_size_m

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Scout Mission Report — {mission_id}</title>
<style>{_CSS}</style>
</head>
<body>
<h1>Scout Mission Report</h1>
<div class="meta">
  Mission: <strong>{ts_pretty}</strong> &nbsp;|&nbsp;
  Field: {field_w:.0f} × {field_h:.0f} m ({cols}×{rows} cells, {cell_size_m} m each) &nbsp;|&nbsp;
  Generated: {gen_time}
</div>

<div class="two-col">
  <div>
    <h2>Coverage</h2>
    <div class="grid">{coverage_cards}</div>

    <h2>Spray Summary</h2>
    <div class="grid">{spray_cards}</div>

    <h2>Blocked Events</h2>
    <div>{blk_detail}</div>
    <p style="font-size:12px; color:#666; margin-top:6px;">Total: {blk["count"]} cell(s) not completed normally.</p>

    <h2>Per-Drone Summary</h2>
    <table>
      <tr><th>Drone</th><th>Cells visited</th><th>Spray events</th><th>Total dose</th><th>Est. distance</th></tr>
      {drone_rows}
    </table>

    <h2>Home Pads</h2>
    <table>
      <tr><th>Pad</th><th>Drone</th><th>NED (N, E)</th><th>Status</th></tr>
      {pads_html}
    </table>
  </div>

  <div>
    <h2>Field Heatmap</h2>
    <div class="legend">{legend}</div>
    <div class="vis-wrap">{svg}</div>
    <p style="font-size:11px; color:#666; margin-top:6px;">
      Axes: North ↑ / East →. Blue dots = spray dose (opacity ∝ dose).
    </p>
  </div>
</div>

</body>
</html>"""

    return html
