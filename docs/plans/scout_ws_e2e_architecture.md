# SCOUT_WS – End-to-End Agrodrone Swarm Architecture

**Status:** Phase 1 [DONE], Phase 2 [DONE], Phase 3 [DONE], Phase 4 [DONE], Phase 5 [DONE]
**Last Update:** 2026-04-27

## 1. Executive Summary
The system is intentionally split into two planning horizons. Before an operational mission, the field is bounded, mapped and converted into a field model. During an operational mission, each drone follows a preplanned task assignment while onboard obstacle avoidance remains active as a local safety and recovery layer.

**The single flight owner for any autonomy-enabled mission is `obstacle_avoidance_runtime`.** Mission logic (`swarm_agent`), swarm allocation and operator tools do not publish competing PX4 setpoints; they delegate high-level targets via `/{drone_ns}/avoidance/target_cmd`.

## 2. Design Principles
- **Mission-first:** Grid / waypoint mission planning remains the main navigation concept.
- **Single flight owner:** Only `obstacle_avoidance_runtime` owns offboard flight setpoints.
- **Telemetry Source of Truth:** All topic contracts are centralized in `TelemetryHub`.
- **Health-Gated Autonomy:** Flight execution is automatically gated by EKF health and sensor readiness.
- **Backwards-compatible JSON:** All persisted JSON (boundary, grid, home positions) must load older payloads with default values for new fields.
- **NED everywhere:** Z is down; altitude is negative.

## 3. Backend Modes

| Backend | Status | Notes |
| :--- | :--- | :--- |
| `navigation_backend=avoidance_runtime` | **DEFAULT / TARGET** | `swarm_agent` delegates; `obstacle_avoidance_runtime` owns PX4 setpoints. |
| `navigation_backend=direct` | DEPRECATED (compat-only) | Legacy direct PX4 path inside `swarm_agent`. Behind backend gate, no new features. To be removed after Phase 3 verification. |

## 4. High-Level Data Flow

```
operator (field_setup_tool / swarm_center)
   │   pad assignments, boundary points, mission_confirm
   ▼
field_setup_coordinator ── home_manager
   │   field_boundary.json, field_grid.json, home_positions.json
   ▼
swarm_coordinator (task_allocator) ──► swarm_agent (per drone)
                                          │  /{drone}/avoidance/target_cmd
                                          ▼
                                obstacle_avoidance_runtime
                                          │  /fmu/in/* (PX4 setpoints)
                                          ▼
                                        PX4
```

## 5. Node Roles and Status (Updated 2026-04-25)

| Node / Module | Current Status | Role | Key Topics |
| :--- | :--- | :--- | :--- |
| **obstacle_avoidance_runtime** | **ACTIVE** | **Single Flight Owner.** Manages PX4 setpoints, local mapping, safety. | `/{drone}/avoidance/target_cmd`, `/{drone}/avoidance/status`, `/fmu/in/*` |
| **precision_landing** | **ACTIVE** | **Advisory Node.** Detects home pad ArUco marker and publishes offset. | `/{drone}/precision_landing/offset`, `/{drone}/camera/image_raw` |
| **swarm_agent** | **ACTIVE** | **Mission Delegator.** No direct PX4 ownership in runtime mode. | `/{drone}/next_cell`, `/{drone}/avoidance/target_cmd`, `/swarm/drone_status` |
| **swarm_coordinator** | ACTIVE | Cell allocation wrapper around `task_allocator`. | `/swarm/task_status`, `/{drone}/next_cell` |
| **task_allocator** | ACTIVE (internal) | Pure-Python allocator with blocked/deferred semantics (`SOFT/HARD`, `CELL_DEFERRED`, `TEMP_BLOCKED`). | n/a (in-process) |
| **telemetry_hub** | **ACTIVE** | Central topic registry. Single source of truth for ROS2 topic contracts. | Contract source for drone/swarm topics; includes `precision_landing_offset` |
| **field_setup_coordinator** | ACTIVE | Setup orchestration; polygon boundary capture, grid generation, RTH gating. | `/swarm/pad_assignment`, `/field/boundary_point`, `/field/boundary_close`, `/field/corner_marked` (legacy), `/field/setup_complete`, `/swarm/rth_request` |
| **field_setup_tool** | ACTIVE | Setup-only operator helper (curses UI). Pad / corner / boundary keys. No PX4 setpoints. | `/swarm/manual_control`, `/swarm/pad_assignment`, `/field/boundary_point`, `/field/boundary_close`, `/field/corner_marked`, `/field/mission_confirm` |
| **home_manager** | ACTIVE | Pad registry with metadata (id, orientation, charging, occupancy, priority) and RTH coordination. | `/swarm/rth_request`, `/swarm/home_positions`, `/swarm/landed_confirmation`, `/swarm/pad_query`, `/swarm/pad_response` |
| **grid_generator** | ACTIVE | Grid generation: `sim_mode`, perimeter mode, **polygon `boundary_mode`** with point-in-polygon classification (`inside/edge/outside`). | n/a (utility) |
| **mission_launcher** | ACTIVE | Mission lifecycle / start triggering. | `/field/mission_confirm`, `/swarm/mission_ready` |
| **gcs_bridge** | ACTIVE | TCP bridge to `swarm_center`. Forwards drone status, avoidance detail, camera/depth frames. | TCP `127.0.0.1:17845`, protocol v1.2 |
| **swarm_center** | ACTIVE (external app) | Standalone PyQt6 GCS. Map, mission progress, RTH all, manual goto. | MAVLink UDP + TCP bridge |
| **cell_data_recorder** | ACTIVE | Per-cell telemetry persistence. | `/swarm/cell_data` |
| **spray_controller** | ACTIVE | Spray actuator management. | `/swarm/spray_cmd` |
| **ml_interface** | ACTIVE | ML / inference plumbing for cell data. | `/swarm/ml_*` |
| **local_mapper / local_planner / scan_manager** | ACTIVE | Onboard safety layer integrated inside runtime. | Runtime-internal map/planning data |
| **obstacle_detector** | LEGACY | Debug / comparison node only. Not in production launches. | Replaced by runtime sensing |
| **offboard_control** | LEGACY | Archived. Replaced by runtime ownership. | — |
| **legacy_manual_controller** | LEGACY | Debug/manual PX4 controller (curses, real TTY). Not in production E2E. | May publish PX4 setpoints; keep out of production launches |

## 6. Persisted Data Model

### `perimeters/field_boundary.json` (Phase 2B)
```json
{
  "vertices_ned": [{"x": 0.0, "y": 0.0, "z": 0.0}, ...],
  "closed": true,
  "inset_buffer_m": 1.0,
  "capture_mode": "polygon"
}
```
Legacy 4-corner capture is still supported and produces a bounding-box equivalent.

### `perimeters/field_grid.json` (Phase 2C)
- Each cell carries `cell_class` ∈ `{inside, edge}` (cells classified `outside` are not emitted).
- Cells from older generations without `cell_class` are implicitly treated as `inside`.

### `perimeters/home_positions.json` (Phase 2A)
Per-pad metadata: `pad_id`, `drone_id`, `ned`, `status` (`available|occupied|charging|maintenance`), `charging_capable`, `orientation_deg`, `service_priority`, `allowed_drone_classes`. Older payloads load with defaults.

### Other persisted artifacts
- `spray_log.json`
- `cell_data/<cell_id>/...`
- `logs/avoidance_logs/*.jsonl`

## 7. Setup State Machine (Phase 2 final)

```
IDLE
  └─► ASSIGN_PADS         (pad assignments published & acked)
        └─► CAPTURE_BOUNDARY      ← polygon points via /field/boundary_point
              └─► GENERATE_GRID    (after /field/boundary_close OR 4-corner legacy fallback)
                    └─► WAITING_FOR_LANDING   (RTH, landed confirmations)
                          └─► READY_FOR_MISSION
```

Legacy 4-corner path enters `GENERATE_GRID` directly via `/field/corner_marked` and uses bounding-box generation for backwards compatibility.

## 8. Boundary → Grid Algorithm

1. Operator marks polygon vertices in flight via `/field/boundary_point`.
2. `/field/boundary_close` finalizes the polygon.
3. `field_setup_coordinator` writes `field_boundary.json`.
4. `grid_generator` (mode `boundary_mode`):
   - Compute polygon AABB.
   - Apply `boundary_inset_m` (default `1.0 m`) — per-segment inward offset (approximation, not full Minkowski).
   - Generate grid over AABB; classify each cell by ray-casting point-in-polygon on cell center:
     - center inside → `inside`
     - center inside but cell extent crosses boundary → `edge`
     - center outside → dropped
5. Persist to `field_grid.json` with `cell_class` per cell.

## 9. Pad State Machine (Phase 2A)

```
available ──► occupied      (RTH request accepted)
occupied  ──► charging      (landed confirm + charging_capable)
occupied  ──► available     (landed confirm + not charging_capable)
charging  ──► available     (charge complete / manual release)
*         ──► maintenance   (manual override)
```

Pad allocation query/response: `/swarm/pad_query` → `/swarm/pad_response`. Low-battery requests prefer the nearest free `charging_capable` pad.

## 10. Avoidance Status Contract (stable subset)

`/{drone}/avoidance/status` includes at minimum:
- `phase`, `state`, `result`
- `planner_mode`, `planner_state`
- `scan_state`, `scan_active`, `last_scan`
- `no_path_streak`, `scan_attempts_for_target`
- `blocked_reason`, `blocked_since_s`, `blocked_severity` (`NONE|SOFT|HARD`)
- `reassign_recommended`
- `last_runtime_event`
- mission feedback: `accepted_target_id`, `active_target_id`, `last_completed_target_id`

`swarm_agent` derives `CELL_COMPLETE` from `last_completed_target_id` (no direct flight loop).

## 11. QoS Conventions

- `QOS_LATCHED` — stateful broadcasts (`/swarm/home_positions`, `/swarm/mission_ready`, `/field/setup_complete`, `/{drone}/avoidance/status`)
- `QOS_VOL` — ephemeral commands and per-tick events
- PX4 topics use the project-standard `_v1` suffix (e.g. `/fmu/out/vehicle_local_position_v1`)
- Bridge / camera streams use volatile QoS sized for image throughput

## 12. GCS Bridge Protocol (v1.2)

Shared between `src/scout_control/scout_control/bridge_protocol.py` and `swarm_center/core/bridge_protocol.py`. Any change must remain synchronized in both files.

Notable messages: `MSG_DRONE_STATUS` (carries `AVOIDANCE_STATUS`, `AVOIDANCE_EVENT` payloads), `MSG_CAMERA_FRAME`, `MSG_DEPTH_FRAME`, `MSG_RTH_ALL`, `MSG_GOTO_CELL`.

## 13. Launch / Scenario Surface

| Launch | Purpose |
| :--- | :--- |
| `full_e2e_mission.launch.py` | Gazebo full E2E swarm mission (tilted_field). |
| `isaac_e2e_mission.launch.py` | Isaac Sim / Pegasus variant. Headless `manual_controller` with `ui:=False`. |
| `obstacle_avoidance_test.launch.py` | Runtime + mission harness + viz. |

All production E2E launches default to `navigation_backend=avoidance_runtime`. `field_setup_tool` is the preferred setup operator node; `legacy_manual_controller` is excluded from production launches.

## 14. Build, Test, Run

```bash
# Build
cd /home/tj/_Data/_Projekty/TJlabs/scout_ws
colcon build --packages-select scout_control
source install/setup.bash

# Targeted unit tests
PYTHONPATH=src/scout_control pytest \
  src/scout_control/test/test_local_planner.py \
  src/scout_control/test/test_avoidance_helpers.py \
  src/scout_control/test/test_home_manager.py \
  src/scout_control/test/test_grid_from_polygon.py \
  src/scout_control/test/test_boundary_capture.py \
  src/scout_control/test/test_e2e_setup_flow.py

# Operator launcher
python3 scout_launcher.py
```

## 15. Implementation Roadmap

### Phase 1 – Stable Onboard Runtime [DONE]
- [x] Finish runtime split into `local_mapper`, `local_planner`, `scan_manager`.
- [x] Make `swarm_agent` a pure delegator (no direct PX4 control in runtime mode).
- [x] Establish `obstacle_avoidance_runtime` as the **Single Flight Owner**.
- [x] Centralize topic contracts into `TelemetryHub`.
- [x] Implement EKF and sensor health gates.

### Phase 2 – Boundary to Base Grid Workflow [DONE]
- [x] **2A** Home pad registration with metadata (pad_id, orientation, charging, occupancy state machine, service priority, allowed_drone_classes). Backwards-compatible `home_positions.json`. Pad query/response topics.
- [x] **2B** Polygon boundary capture replaces 4-corner bounding box. `/field/boundary_point`, `/field/boundary_close`. Inset-buffer safety margin. 4-corner legacy fallback retained.
- [x] **2C** Grid generation from polygon (`boundary_mode`) with ray-cast point-in-polygon and `cell_class` classification. Backwards-compatible `field_grid.json`.
- [x] **2D** Full E2E swarm field mission verification. Launch/config defaults set to `navigation_backend=avoidance_runtime`. Scenario YAMLs cleaned (workspace path, backend defaults). New `test_e2e_setup_flow.py` covers setup → boundary → grid → RTH.

### Phase 3 – Mapping Mission Pipeline [DONE]
- [x] Mapping mission flight pattern (lawnmower over polygon with overlap).
- [x] Field model outputs: terrain map (2.5D heightmap), static obstacle extraction.
- [x] Persist mapping artifacts under `perimeters/field_model/`.
- [x] Precision landing / home pad vision integration (advisory-only).
- [ ] Remove deprecated `navigation_backend=direct` after full Phase 3 verification.
- [ ] Replace ad-hoc `task_allocator.yaml` scenario or register a proper entry point.

### Phase 4 – Operational Hardening [DONE]
- [x] Grid refiner (`mapping/grid_refiner.py`) — marks cells `no_go`/`caution`/`available` from field model.
- [x] Mission package builder (`mapping/mission_package_builder.py`) — bundles mission snapshot for dispatch/archive.
- [x] Bridge protocol v1.3 — added `MSG_NO_GO_OVERLAY`, `MSG_REFINED_GRID_EVENT` (both files kept in sync).
- [x] Home manager improvements — occupancy state machine, pad service_priority.
- [x] Field setup coordinator improvements — boundary RTH gating.
- [ ] Workspace path portability — `WS_DIR` in `scout_launcher.py` still hard-coded (deferred).
- [ ] Charging lifecycle integration with real hardware feedback (deferred to hardware phase).

### Phase 5 – Swarm Center GCS Completion [DONE]
- [x] `field_model_loader.py` — loads Phase 3 outputs (heightmap, obstacles) for overlay rendering.
- [x] `avoidance_panel.py` — per-drone NOMINAL/WARN/CRITICAL/BLOCKED state with animated pulse.
- [x] `field_view.py` — overlay layers: no-go zones, obstacles, terrain heatmap; sector preview before mission.
- [x] `control_panel.py` — overlay toggle checkboxes; `export_report_clicked` signal; "Export Report" button.
- [x] `report_generator.py` — pure-Python post-mission HTML report:
  - Aggregates: coverage stats, spray summary, blocked events, per-drone summary (flight distance estimate).
  - SVG grid heatmap (green/red/orange/yellow by cell status) + spray dose overlay.
  - Saves `grid_snapshot.json` for re-generation after session.
  - Output: `reports/<mission_id>/report.html` (self-contained, offline-ready).
  - Auto-opened in browser on mission_complete; re-generable via "Export Report" button.
- [x] `paths.py` — added `FIELD_MODEL_DIR`, `NO_GO_FILE`, `OBSTACLES_FILE`, `TERRAIN_FILE`, `REPORTS_DIR`.

## 16. Known Risks & Compatibility Notes

- `navigation_backend=direct` remains compiled-in; do not extend it. New mission features go through the runtime command/status contract.
- Bridge protocol is duplicated in two source trees — keep both copies in lockstep.
- Some scenario YAMLs historically referenced `~/scout_ws/install/setup.bash`; Phase 2D normalized the active set, but any newly added scenario should use the absolute workspace path or `scout_control.utils.paths` equivalents.
- `task_allocator.yaml` does not match the `setup.py` console_scripts and is not standalone-runnable without rework.
- Worktree may carry uncommitted changes; do not revert third-party changes without explicit instruction.

## 17. Documentation Conventions

- Long-lived human docs live under `docs/`.
- Runbooks and manual launch procedures live under `launch_files/`.
- `CLAUDE.md` is the authoritative project map and workflow reference.
- `codex.md` is the running changelog / discovery log for AI sessions.
- This file (`scout_ws_e2e_architecture.md`) is the architectural source of truth and tracks roadmap status.
