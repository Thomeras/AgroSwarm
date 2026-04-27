#!/usr/bin/env bash
set -u

WS=/home/tj/_Data/_Projekty/TJlabs/scout_ws

echo "=== Isaac Phase 1+2+3 live checks ==="
echo

echo "=== Phase 1: single PX4 trajectory_setpoint owner ==="
ros2 topic info /fmu/in/trajectory_setpoint -v || true
echo

echo "=== Phase 1: route providers should not publish /fmu/in topics ==="
for node in /swarm_agent_0 /mapping_mission; do
  ros2 node info "$node" 2>/dev/null | grep -A30 "Publishers" | grep "fmu/in" \
    && echo "WARNING: $node publishes /fmu/in topics" \
    || echo "OK: no /fmu/in publishers found on $node"
done
echo

echo "=== Phase 1: avoidance status contract ==="
timeout 5 ros2 topic echo --once /drone_0/avoidance/status || true
echo

echo "=== Isaac sensor streams ==="
timeout 5 ros2 topic hz /drone_0/camera/image_raw || true
timeout 5 ros2 topic hz /drone_0/depth/image_raw || true
echo

echo "=== Phase 2: setup state ==="
timeout 3 ros2 topic echo --once /field/setup_status || true
echo

echo "=== Phase 2: home_positions schema ==="
python3 - <<'PY'
import json
from pathlib import Path

p = Path("/home/tj/_Data/_Projekty/TJlabs/scout_ws/perimeters/home_positions.json")
if not p.exists():
    print("NOT YET WRITTEN")
    raise SystemExit(0)
d = json.loads(p.read_text())
if isinstance(d, dict):
    pads = d.get("pads") or d.get("home_positions") or []
elif isinstance(d, list):
    pads = d
else:
    pads = []
required = {"pad_id", "status", "charging_capable", "orientation_deg", "service_priority"}
missing_by_pad = {}
for idx, pad in enumerate(pads):
    if isinstance(pad, dict):
        missing = required - set(pad)
        if missing:
            missing_by_pad[pad.get("pad_id", f"pad_{idx}")] = sorted(missing)
print(f"pads={len(pads)} missing={missing_by_pad or 'none'}")
PY
echo

echo "=== Phase 2: boundary artifact ==="
python3 - <<'PY'
import json
from pathlib import Path

p = Path("/home/tj/_Data/_Projekty/TJlabs/scout_ws/perimeters/field_boundary.json")
if not p.exists():
    print("NOT YET WRITTEN")
    raise SystemExit(0)
d = json.loads(p.read_text())
verts = d.get("vertices_ned") or d.get("waypoints_ned") or []
print(f"capture_mode={d.get('capture_mode')} closed={d.get('closed')} vertices={len(verts)}")
PY
echo

echo "=== Phase 2: grid artifact ==="
python3 - <<'PY'
import json
from pathlib import Path

p = Path("/home/tj/_Data/_Projekty/TJlabs/scout_ws/perimeters/field_grid.json")
if not p.exists():
    print("NOT YET WRITTEN")
    raise SystemExit(0)
d = json.loads(p.read_text())
cells = d.get("cells", [])
classes = sorted({c.get("cell_class", "inside") for c in cells if isinstance(c, dict)})
print(f"capture_mode={d.get('capture_mode')} cells={len(cells)} classes={classes}")
PY
echo

echo "=== Phase 3: mapping progress topic ==="
timeout 5 ros2 topic echo --once /swarm/mapping_progress || true
echo

echo "=== Phase 3: field model artifact ==="
python3 - <<'PY'
import json
from pathlib import Path

root = Path("/home/tj/_Data/_Projekty/TJlabs/scout_ws/perimeters/field_model")
manifest = root / "manifest.json"
if not manifest.exists():
    print("NOT YET WRITTEN")
    raise SystemExit(0)
d = json.loads(manifest.read_text())
latest = d.get("latest") or {}
print(f"entries={len(d.get('entries', []))} latest={latest}")
for key in ("heightmap_json", "heightmap_npy", "obstacles_json"):
    name = latest.get(key)
    print(f"{key}: {name} exists={(root / name).exists() if name else False}")
PY
echo

echo "=== Unit checks: Phase 1+2+3 helper tests ==="
cd "$WS" || exit 1
PYTHONPATH=src/scout_control:${PYTHONPATH:-} pytest -q \
  src/scout_control/test/test_local_planner.py \
  src/scout_control/test/test_avoidance_helpers.py \
  src/scout_control/test/test_home_manager.py \
  src/scout_control/test/test_grid_from_polygon.py \
  src/scout_control/test/test_boundary_capture.py \
  src/scout_control/test/test_e2e_setup_flow.py \
  src/scout_control/test/test_lawnmower.py \
  src/scout_control/test/test_heightmap.py \
  src/scout_control/test/test_obstacle_extractor.py \
  src/scout_control/test/test_mapping_mission.py \
  src/scout_control/test/test_pad_detector.py

echo
echo "=== Done. Run after Phase 2 setup and again after Phase 3 mapping completes. ==="

