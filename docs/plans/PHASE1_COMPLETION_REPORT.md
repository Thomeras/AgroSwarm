# Phase 1 Completion Report — Stable Onboard Runtime

**Date:** 2026-04-24
**Project:** scout_ws
**Status:** Phase 1 DONE

## Executive Summary
Phase 1 of the `scout_ws` E2E architecture has been successfully completed. The core objective was to establish a stable onboard runtime where `obstacle_avoidance_runtime` acts as the single authoritative flight-control owner, and `swarm_agent` is transformed into a pure mission delegator.

## Key Achievements

### 1. Authority & Ownership (Single Flight Owner)
- **`obstacle_avoidance_runtime`** is now the exclusive publisher of PX4 setpoints in the avoidance-enabled path.
- **`swarm_agent`** has been refactored to use `navigation_backend=avoidance_runtime` by default. It no longer manages low-level PX4 control loops or publishers in this mode.
- **Ownership Guard:** Implemented a system where the runtime explicitly reports its `flight_control_owner` status.

### 2. TelemetryHub & Topic Centralization
- Created **`telemetry_hub.py`** as the single source of truth for all ROS2 topic contracts.
- Centralized PX4, sensor, avoidance, swarm, and GCS bridge topics.
- Updated all nodes (runtime, agent, coordinator, recorder, etc.) to use the `TelemetryHub` for topic discovery, eliminating hardcoded strings across the codebase.

### 3. Health & Safety (Runtime Readiness)
- Implemented **`RuntimeHealthMonitor`** and **`RuntimeReadiness`** logic.
- The runtime now recognizes and gates navigation on:
  - Stale pose/depth data.
  - EKF degradation (heading validity, reset counters).
  - Dead reckoning status.
- **Safety Gate:** `_step_toward` no longer defaults to (0,0) on EKF dropout, preventing dangerous fly-aways.

### 4. Planning & Mapping Robustness
- **`LocalPlanner`** now refuses to plan over empty, stale, or unready maps.
- **`DepthProjector`** unified projection logic (used by both Mapper and ScanManager).
- **Scan Retention:** Implemented retention policies and metadata storage for scan artifacts (NPZ files with intrinsics).

### 5. Legacy Node Cleanup
- Removed all legacy flight nodes from `setup.py` (they remain in the repo as archived source but are not installed as console scripts).
- Replaced legacy node entry points with archive notices in scenario files.
- **`manual_controller`** refactored to focus on event-based UI interaction rather than direct flight ownership during autonomous missions.

### 6. Multi-Drone Support
- Improved **Isaac Sim** launch files to correctly handle `drone_count` (e.g., spawning runtime/agent for `drone_1` only when requested).
- Centralized topic templates for camera/depth in `TelemetryHub`.

## Verification Results
- **Unit/Integration Tests:** 38 passed (including new tests for TelemetryHub, Typed Readiness, and Stale Map rejection).
- **Build:** `colcon build` successful with no missing dependencies in core logic.
- **Static Analysis:** `python3 -m compileall` passed for all modified directories.

## Next Steps: Phase 2
1. **Live E2E Verification:** Comprehensive flight tests in Gazebo/Isaac Sim to verify the full hand-off from Coordinator -> Agent -> Runtime.
2. **Boundary Capture Workflow:** Implementation of perimeter boundary capture and home pad metadata persistence.
3. **Home Pad Vision:** Integration of visual docking/landing on markers.
4. **Final Direct-Path Removal:** Deleting the legacy `navigation_backend=direct` code in `swarm_agent` once E2E stability is confirmed.
