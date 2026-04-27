# Project Memory — scout_ws

## Core Architecture Facts
- **Flight Ownership:** `obstacle_avoidance_runtime` is the single source of PX4 setpoints for autonomous flight.
- **Mission Execution:** `swarm_agent` delegates to the runtime via `/{drone}/avoidance/target_cmd`.
- **Topic Contracts:** Centralized in `scout_control.avoidance.telemetry_hub`.
- **Coordinate System:** PX4 uses NED (North-East-Down). Z is down; altitude is negative.
- **Workspace Root:** `/home/tj/_Data/_Projekty/TJlabs/scout_ws`.
- **Phase Status:** Phase 1, 2, 3, 4, 5 — ALL DONE.

## Phase Summary
| Phase | Name | Status |
|-------|------|--------|
| 1 | Stable Onboard Runtime | ✓ DONE |
| 2 | Boundary to Base Grid Workflow | ✓ DONE |
| 3 | Mapping Mission Pipeline | ✓ DONE |
| 4 | Operational Hardening (grid_refiner, mission_package_builder, bridge v1.3) | ✓ DONE |
| 5 | Swarm Center GCS (avoidance panel, field model overlays, report generator) | ✓ DONE |

## Hardware & Simulation Contracts
- **Isaac Sim Camera:** RGB/Depth published via `simulation_cam.py` in-session helper.
- **Gazebo Bridge:** `ros_gz_image` for camera streams.
- **Telemetry Hub:** Handles drone-prefixed namespaces (e.g., `drone_0`, `drone_1`).

## Key CLI Workflows
- **Build:** `colcon build --packages-select scout_control`
- **Tests:** `PYTHONPATH=src/scout_control pytest src/scout_control/test/`
- **Launcher:** `python3 scout_launcher.py`
- **GCS:** `cd swarm_center && python3 main.py`

## Key Output Paths
- `perimeters/` — field_boundary.json, field_grid.json, home_positions.json, field_model/
- `spray_log.json` — per-cell spray events
- `cell_data/<cell_id>/visit_N/meta.json` — visit snapshots with drone_id, ned, timestamp
- `reports/<mission_id>/report.html` — post-mission HTML report (Phase 5)
- `logs/avoidance_logs/*.jsonl` — avoidance runtime logs

## Active Decisions
- [x] Use `navigation_backend=avoidance_runtime` by default in `swarm_agent`.
- [x] Legacy nodes (`offboard_control`, `terrain_follower`) are archived and not installed.
- [x] Runtime gates navigation on EKF/Sensor health.
- [x] ReportGenerator is pure Python (no ROS2), reads files directly, saves grid snapshot for re-generation.
- [x] Bridge protocol v1.3 (MSG_NO_GO_OVERLAY, MSG_REFINED_GRID_EVENT planned but not wired on GCS side yet).

## Open Items
- Remove deprecated `navigation_backend=direct` from `swarm_agent` after next E2E verification.
- `task_allocator.yaml` not registered in `setup.py` — not standalone-runnable.
- Bridge v1.3 overlay payloads not yet consumed by `swarm_center` (GCS side pending).
