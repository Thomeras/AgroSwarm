# Scout WS E2E Architecture Status

This document tracks the implementation progress of the "scout_ws_e2e_architecture" plan.

## Phase 1: Stable Onboard Runtime & Ownership [DONE]
**Goal:** Establish `obstacle_avoidance_runtime` as the single authoritative flight owner and `swarm_agent` as a mission delegator.

- [x] **Single Flight Owner:** `obstacle_avoidance_runtime` is the only node publishing PX4 setpoints in autonomous mode.
- [x] **Swarm Agent Refactor:** `swarm_agent` delegates to runtime via `target_cmd` and tracks completion via `last_completed_target_id`.
- [x] **TelemetryHub:** Centralized ROS2 topic registry in `scout_control.avoidance.telemetry_hub`.
- [x] **Health Monitoring:** Runtime gates flight on EKF/Sensor health (stale pose, depth dropout).
- [x] **Map/Plan Validity:** Planner rejects invalid/stale local maps.
- [x] **Legacy Cleanup:** Legacy nodes (`offboard_control`, `terrain_follower`, `obstacle_avoidance_mission`) removed from installation.
- [x] **Scan Pipeline:** Unified `DepthProjector` and artifacts with metadata.

## Phase 2: Boundary Capture & Home Pad [TODO]
**Goal:** Formalize perimeter capture and precision landing.

- [ ] **Perimeter Capture Tooling:** Refactor `manual_controller` or create specialized tool for GPS perimeter recording.
- [ ] **Home Pad Metadata:** Extend `home_positions.json` with pad type, orientation, and visual marker ID.
- [ ] **Visual Landing:** Integration of camera-based home pad alignment.
- [ ] **E2E Validation:** Full swarm field mission verification in simulation.

## Phase 3: Fleet Management & GCS Polish [PLANNED]
**Goal:** Advanced swarm reallocation and GCS feedback.

- [ ] **Dynamic Reassignment:** Improved allocator response to `SOFT/HARD` blocked events.
- [ ] **GCS Detailed Status:** Visualization of local plans and obstacle maps in `swarm_center`.
- [ ] **Cell Data Enrichment:** Automated upload of visited cell artifacts to GCS.

---

## Current Module Map (Post-Phase 1)

```text
ManualControlTool ──events only──▶ swarm_agent ──target_cmd──▶ obstacle_avoidance_runtime
                                                                    │
                    ┌──TelemetryHub (Topic Central)─────────────────┤
                    ├──SensorHub (Health/Readiness)─────────────────┤
                    ├──HealthMonitor (EKF Gate)─────────────────────┤
                    ├──DepthProjector (Projection Path)─────────────┤
                    └──AltitudeController (NED/Terrain)─────────────┘
```
