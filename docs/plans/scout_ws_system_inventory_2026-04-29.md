# Scout WS System Inventory and Code Reality Check

Generated: 2026-04-29

This document is a static inventory of the current repository. It describes the ROS2 nodes, topic wiring, launch-level topology, Python modules, classes, and places where comments, log strings, docs, or launch files do not fully match the implementation.

Scope inspected:

- `src/scout_control`: active ROS2 package and Python modules.
- `src/scout_control_msgs`: custom message contracts used by the active package.
- `src/px4_msgs`: vendored/generated PX4 message package.
- `swarm_center`: PySide ground control station and TCP bridge client.
- `src/scout_control/launch`, `scenarios`, and existing docs where they affect runtime topology.

## Executive Summary

The current system is a ROS2/PX4 swarm autonomy stack for agricultural field setup, mission allocation, obstacle avoidance, mapping, spraying simulation, ML-data collection, and GCS visualization.

The active mission path is:

1. Operator/GCS or setup tool assigns landing pads and boundary points.
2. `field_setup_coordinator` writes `perimeters/field_boundary.json`, `perimeters/field_grid.json`, home positions, and publishes `/swarm/mission_ready`.
3. `swarm_coordinator` reloads the generated grid and wraps pure-Python `TaskAllocator`.
4. `swarm_agent` translates allocated cells into high-level avoidance target commands.
5. `obstacle_avoidance_runtime` owns PX4 offboard input topics and navigates each drone while publishing typed and JSON status.
6. `spray_controller`, `cell_data_recorder`, `ml_interface`, `home_manager`, `gcs_bridge`, and `swarm_center` consume mission state for spraying, records, UI, and RTH.

Important code-reality findings:

- `src/scout_control/launch/swarm_mission.launch.py` launches executable `task_allocator`, and `setup.py` registers `task_allocator = scout_control.utils.task_allocator:main`, but `utils/task_allocator.py` has no `main()` and no ROS `Node`. Actual working wrapper is `swarm_coordinator`.
- Existing operator docs mention `legacy_manual_controller`, `legacy_manual_commander`, `legacy_field_commander`, `legacy_perimeter_flight`, and `legacy_terrain_follower`, but current `setup.py` does not register those `legacy_*` console scripts. It registers non-legacy names for `field_setup_tool` and `manual_controller`, and does not register `manual_commander`, `field_commander`, or legacy modules.
- `field_setup_tool` has a `drone_count` parameter, but `isaac_e2e_mission.launch.py` starts it without passing `drone_count`; for non-default drone counts this can diverge from the rest of the launch.
- `gcs_bridge` declares `avoidance_status_topic_template` and `avoidance_events_topic_template`; status subscription honors the status template, but events are built from `TelemetryHub` defaults rather than the event template parameter.
- `swarm_agent` still declares `navigation_backend`, but code forces `avoidance_runtime`; the log line prints a backend value that can look configurable, while the implementation ignores the parameter intentionally.
- `ml_interface` is explicitly a dummy/tooling placeholder. It publishes plausible-looking field health, anomalies, and dose data, but no trained model is loaded.
- `precision_landing` publishes advisory offsets only. `advisory_only` is included in the payload; the runtime does not currently consume it for closed-loop landing control.

## Repository Topology

### ROS packages

`src/scout_control`

- Main Python ROS2 package.
- Entry points in `setup.py` include active core, mapping, visualization, manual setup, and utility nodes.
- Launch files live in `src/scout_control/launch`.
- Runtime data paths are centralized in `scout_control.utils.paths`.

`src/scout_control_msgs`

- Custom ROS messages:
  - `AvoidanceStatus.msg`
  - `DroneHealth.msg`
  - `DroneReadiness.msg`
  - `PeerTelemetry.msg`
  - `TargetCommand.msg`
- Used mainly by the typed avoidance runtime path.

`src/px4_msgs`

- Vendored PX4 message definitions.
- The active runtime uses:
  - `VehicleLocalPosition`
  - `VehicleStatus`
  - `VehicleControlMode`
  - `VehicleCommandAck`
  - `OffboardControlMode`
  - `TrajectorySetpoint`
  - `VehicleCommand`
  - plus several legacy/manual modules.

### Non-ROS application

`swarm_center`

- PySide/PyQt ground control application.
- Talks to ROS via `scout_control.core.gcs_bridge` over TCP using newline-delimited JSON.
- Talks to PX4 SITL directly through MAVLink for telemetry, arm/disarm, and UI status.

## Canonical Topic Contracts

The central source for active per-drone topics is `scout_control.avoidance.telemetry_hub.TelemetryHub`.

For `drone_id=0`:

- `drone_ns`: `drone_0`
- `px4_ns`: empty string
- PX4 input topics:
  - `/fmu/in/offboard_control_mode`
  - `/fmu/in/trajectory_setpoint`
  - `/fmu/in/vehicle_command`
- PX4 output topics:
  - `/fmu/out/vehicle_local_position_v1`
  - `/fmu/out/vehicle_status_v3`
  - `/fmu/out/vehicle_control_mode`
  - `/fmu/out/vehicle_command_ack_v1`

For `drone_id=N` where `N > 0`:

- `drone_ns`: `drone_N`
- `px4_ns`: `/px4_N`
- PX4 input topics:
  - `/px4_N/fmu/in/offboard_control_mode`
  - `/px4_N/fmu/in/trajectory_setpoint`
  - `/px4_N/fmu/in/vehicle_command`
- PX4 output topics:
  - `/px4_N/fmu/out/vehicle_local_position_v1`
  - `/px4_N/fmu/out/vehicle_status_v3`
  - `/px4_N/fmu/out/vehicle_control_mode`
  - `/px4_N/fmu/out/vehicle_command_ack_v1`

Per-drone autonomy topics:

- `/{drone_ns}/camera/image_raw`
- `/{drone_ns}/depth/image_raw`
- `/{drone_ns}/camera/camera_info`
- `/{drone_ns}/downward_lidar/scan`
- `/{drone_ns}/avoidance/target_cmd`
- `/{drone_ns}/avoidance/target_cmd_json`
- `/{drone_ns}/avoidance/status`
- `/{drone_ns}/avoidance/status_json`
- `/{drone_ns}/avoidance/events`
- `/{drone_ns}/avoidance/active`
- `/{drone_ns}/avoidance/planned_path`
- `/{drone_ns}/avoidance/actual_path`
- `/{drone_ns}/obstacles/detected`
- `/{drone_ns}/obstacles/clear`
- `/{drone_ns}/next_cell`
- `/{drone_ns}/rth_target`
- `/{drone_ns}/precision_landing/offset`

Swarm-level topics:

- `/swarm/peer_telemetry`
- `/swarm/drone_status`
- `/swarm/rth_request`
- `/swarm/landed_confirmation`
- `/swarm/pad_assignment`
- `/swarm/pad_query`
- `/swarm/pad_response`
- `/swarm/home_positions`
- `/swarm/task_status`
- `/swarm/mission_complete`
- `/swarm/cell_override`
- `/swarm/mode`
- `/swarm/peer_cells`
- `/swarm/manual_control`
- `/swarm/mission_ready`
- `/swarm/start_mission`

Field/setup topics:

- `/field/setup_status`
- `/field/setup_complete`
- `/field/corner_marked`
- `/field/boundary_point`
- `/field/boundary_close`
- `/field/mission_confirm`
- `/field/generate_grid`
- `/field/grid`
- `/field/anomaly`
- `/field/cell_health`

ROS services/actions:

- Static scan found no `create_service`, `create_client`, `ActionServer`, or `ActionClient` usage in `src/scout_control/scout_control` or `swarm_center`.
- Current inter-node control is topic-based plus the TCP JSON bridge between `gcs_bridge` and `swarm_center`.

## Active ROS Nodes

### `field_setup_coordinator`

File: `src/scout_control/scout_control/core/field_setup_coordinator.py`

Class: `FieldSetupCoordinator(Node)`

Purpose:

- Orchestrates the setup phase before autonomous field coverage.
- Handles pad assignment, polygon boundary capture, optional legacy four-corner fallback, grid generation, home-position persistence, RTH gating, and mission-ready publication.

Parameters:

- `cell_size_m`: default `5.0`.
- `drone_count`: default `2`.
- `boundary_inset_m`: default `1.0`.

Publishes:

- `/field/setup_status` (`std_msgs/String` JSON): current setup state and hints.
- `/field/setup_complete` (`std_msgs/String` JSON): latched setup result.
- `/swarm/rth_request` (`std_msgs/String` JSON): requests return-home after setup.
- `/swarm/mission_ready` (`std_msgs/String` JSON): latched mission-start gate.

Subscribes:

- `/swarm/pad_assignment`: pad assignment events from setup tool or GCS.
- `/field/corner_marked`: legacy four-corner capture.
- `/field/boundary_point`: polygon vertex capture.
- `/field/boundary_close`: polygon close/generate trigger.
- `/swarm/landed_confirmation`: confirms drones reached/landed at pads.
- `/field/mission_confirm`: operator confirmation.
- `/field/generate_grid`: explicit grid generation request from GCS.

Connected nodes:

- Receives operator actions from `field_setup_tool`, `manual_controller`, and `gcs_bridge`.
- Feeds `/swarm/mission_ready` to `swarm_coordinator`, `swarm_agent`, `mission_launcher`, `gcs_bridge`, and `swarm_center`.
- Feeds `/swarm/rth_request` to `home_manager` and `swarm_agent`.
- Produces field-grid files consumed by `swarm_coordinator`, `ml_interface`, `swarm_center`, and mapping/refinement helpers.

Implementation notes:

- The default path is polygon-aware boundary capture.
- The first `/field/corner_marked` locks the node into legacy four-corner bounding-box mode.
- It writes persistent setup artifacts under `perimeters`.
- It can call `GridRefiner` if mapping artifacts are present.

### `home_manager`

File: `src/scout_control/scout_control/core/home_manager.py`

Classes:

- `PadRegistry`: pure in-memory pad state machine.
- `HomeManager(Node)`: ROS wrapper.

Purpose:

- Stores pad assignments and home positions.
- Handles RTH requests, pad allocation/query, landed confirmation, charge completion, and RTH target publication.

Publishes:

- `/swarm/home_positions` (`std_msgs/String` JSON): latched known pads/home positions.
- `/swarm/pad_response` (`std_msgs/String` JSON): pad query/allocation responses.
- `/{drone_ns}/rth_target` (`geometry_msgs/Point`): per-drone return target.

Subscribes:

- `/swarm/rth_request`
- `/swarm/landed_confirmation`
- `/swarm/pad_assignment`
- `/swarm/pad_query`
- `/swarm/charge_complete`

Connected nodes:

- Consumes pad assignments from setup tools/GCS.
- Publishes `/{drone}/rth_target` for `obstacle_avoidance_runtime`.
- Publishes home positions for `mapping_mission`, `precision_landing`, and GCS UI.

Implementation notes:

- Dynamic per-drone RTH publishers are created on demand.
- `PadRegistry` is ROS-free and unit-testable.

### `obstacle_avoidance_runtime`

File: `src/scout_control/scout_control/core/obstacle_avoidance_runtime.py`

Class: `ObstacleAvoidanceRuntime(Node)`

Purpose:

- Per-drone navigation runtime.
- Owns PX4 offboard heartbeat, trajectory setpoints, and vehicle commands.
- Fuses PX4 pose/status, depth camera, camera info, downward/obstacle lidar, peer telemetry, target commands, and RTH targets.
- Runs local mapping, scan manager, local planner, altitude controller, health monitor, and PX4 publisher adapter.

Key parameters:

- `drone_id`: default `0`.
- `default_altitude_m`: default `5.0`.
- `default_cruise_speed`: default `2.5`.
- `default_clear_dist`: default `2.5`.
- `avoid_offset_m`: default `3.0`.
- `home_dist`: default `1.5`.
- `setpoint_lookahead_s`, `min_command_step_m`, drift/replan thresholds.
- Local planner cost parameters.
- Scan parameters: hover ticks, spin ticks, free distance, point stride, camera HFOV.
- Sensor gate parameters including `require_depth_for_navigation`.
- Relaxation/test parameters used in Isaac launches.
- `publish_legacy_obstacle_topics`: default `False`.

Publishes:

- `/{drone_ns}/obstacles/detected` (`std_msgs/String` JSON).
- `/{drone_ns}/obstacles/clear` (`std_msgs/Bool`).
- `/{drone_ns}/avoidance/status` (`scout_control_msgs/AvoidanceStatus` if available, else `String`).
- `/{drone_ns}/avoidance/status_json` (`String`).
- `/{drone_ns}/avoidance/planned_path` (`nav_msgs/Path`).
- `/{drone_ns}/avoidance/actual_path` (`nav_msgs/Path`).
- `/{drone_ns}/avoidance/active` (`Bool`).
- `/{drone_ns}/avoidance/events` (`String` JSON).
- `/swarm/peer_telemetry` (`scout_control_msgs/PeerTelemetry`) when custom type is available.
- PX4 inputs:
  - `/fmu/in/offboard_control_mode` or `/px4_N/fmu/in/offboard_control_mode`
  - `/fmu/in/trajectory_setpoint` or `/px4_N/fmu/in/trajectory_setpoint`
  - `/fmu/in/vehicle_command` or `/px4_N/fmu/in/vehicle_command`
- Optional legacy drone-0 visualization topics if `publish_legacy_obstacle_topics=True`:
  - `/obstacle_avoidance/status`
  - `/obstacle_avoidance/planned_path`
  - `/obstacle_avoidance/actual_path`
  - `/obstacle_avoidance/avoidance_active`
  - `/obstacle_avoidance/events`

Subscribes:

- PX4 outputs:
  - vehicle local position
  - vehicle status
  - vehicle control mode
  - vehicle command ack
- Sensors:
  - camera image
  - depth image
  - camera info
  - downward lidar scan
  - optional obstacle lidar topic
- Commands:
  - `/{drone_ns}/avoidance/target_cmd`
  - `/{drone_ns}/avoidance/target_cmd_json`
  - `/{drone_ns}/rth_target`
- Swarm:
  - `/swarm/peer_telemetry`

Connected nodes:

- Receives target commands from `swarm_agent`, `mapping_mission`, and `gcs_bridge` manual goto commands.
- Receives RTH targets from `home_manager`, `field_setup_tool`, or manual tools.
- Feeds status to `swarm_agent`, `gcs_bridge`, `precision_landing`, `mapping_mission`, and visualizers.
- Feeds peer telemetry to other runtime instances.
- Publishes PX4 commands; should not be run with other PX4 offboard controllers for the same drone.

Implementation notes:

- Uses `Px4InputOwnershipGuard` to detect competing publishers on PX4 input topics.
- Publishes both typed status and JSON status for compatibility.
- `RosIOAdapter` mirrors runtime events/status to legacy publishers when enabled.
- `AltitudeController` converts terrain-follow or fixed-altitude policy into PX4 NED `z`.
- `RuntimeHealthMonitor` gates readiness based on pose/depth freshness.

### `swarm_agent`

File: `src/scout_control/scout_control/core/swarm_agent.py`

Class: `SwarmAgent(Node)`

Purpose:

- Per-drone mission executor between allocation and runtime navigation.
- Converts assigned cells into avoidance target commands.
- Translates runtime status into `/swarm/drone_status` events.
- Handles RTH request publication toward runtime.

Parameters:

- `drone_id`: default `0`.
- `altitude_m`: default `5.0`.
- `home_ned_x`: default `0.0`.
- `home_ned_y`: default `0.0`.
- `cruise_speed`: default `2.0`.
- `navigation_backend`: declared but deprecated; implementation always uses `avoidance_runtime`.

Publishes:

- `/{drone_ns}/avoidance/target_cmd` (`TargetCommand` if available, else `String` JSON).
- `/{drone_ns}/avoidance/target_cmd_json` (`String` JSON).
- `/swarm/drone_status` (`String` JSON).
- `/swarm/landed_confirmation` (`String` JSON).

Subscribes:

- `/{drone_ns}/next_cell`
- `/swarm/rth_request`
- `/{drone_ns}/avoidance/status`
- `/{drone_ns}/avoidance/status_json`
- `/swarm/mission_ready`

Connected nodes:

- Receives next-cell assignments from `swarm_coordinator`.
- Sends target commands to `obstacle_avoidance_runtime`.
- Sends status to `swarm_coordinator`, `spray_controller`, `cell_data_recorder`, `gcs_bridge`, and `swarm_center`.
- Receives mission-ready from `field_setup_coordinator`.

Implementation notes:

- `navigation_backend` is not truly configurable now; it is hardwired to runtime.
- Status names normalize runtime navigation phases into allocator-friendly events such as `READY`, `CELL_COMPLETE`, blocked variants, RTH, etc.

### `swarm_coordinator`

File: `src/scout_control/scout_control/core/swarm_coordinator.py`

Class: `SwarmCoordinator(Node)`

Purpose:

- ROS2 wrapper around pure-Python `TaskAllocator`.
- Loads field grid, assigns cells, monitors drone status, publishes next cells and mission progress.

Parameters:

- `drone_count`: default `2`.
- `ready_timeout`: default `30.0`.
- `nfz_radius`: default `3.0`.
- `deferred_retry_delay_s`: default `12.0`.
- `hard_block_cooldown_s`: default `30.0`.
- `max_deferrals_per_cell`: default `3`.

Publishes:

- `/{drone_ns}/next_cell` (`String` JSON), one publisher per drone.
- `/swarm/task_status` (`String` JSON).
- `/swarm/mission_complete` (`String` JSON).
- `/swarm/rth_request` (`String` JSON).

Subscribes:

- `/swarm/drone_status`
- `/swarm/cell_override`
- `/swarm/mission_ready`
- per-drone PX4 local position topics.

Connected nodes:

- Consumes drone state from `swarm_agent`.
- Sends cell assignments to `swarm_agent`.
- Sends progress to `gcs_bridge`, `mission_launcher`, and `swarm_center`.
- Sends RTH requests to `home_manager` and `swarm_agent`.

Implementation notes:

- Starts with an empty placeholder if grid file is not yet available.
- Reloads generated grid on `/swarm/mission_ready`.
- The actual allocator is not a ROS node.

### `mission_launcher`

File: `src/scout_control/scout_control/core/mission_launcher.py`

Class: `MissionLauncher(Node)`

Purpose:

- Small mission trigger/monitor node.
- Publishes mission start once setup reports mission-ready and monitors completion/status.

Publishes:

- `/swarm/start_mission` (`String` JSON).

Subscribes:

- `/swarm/mission_ready`
- `/swarm/mission_complete`
- `/swarm/task_status`

Connected nodes:

- Consumes setup/coordinator status.
- Can be used by launch flows as a simple mission lifecycle helper.

Implementation notes:

- The allocator does not depend on `/swarm/start_mission` in the current code path; `swarm_coordinator` starts readiness timeout directly from `/swarm/mission_ready`.

### `gcs_bridge`

File: `src/scout_control/scout_control/core/gcs_bridge.py`

Class: `GcsBridge(Node)`

Purpose:

- TCP server bridge between ROS2 graph and Swarm Center.
- Serializes mission, setup, camera, depth, avoidance, and drone-status events to newline JSON.
- Receives GCS commands and republishes them into ROS topics.

Parameters:

- `host`: default `127.0.0.1`.
- `port`: default bridge port.
- `drone_count`: default `2`.
- `camera_fps_limit`: default `5.0`.
- `depth_fps_limit`: default `2.0`.
- `camera_topic_template`: default `/{drone_id}/camera/image_raw`.
- `depth_topic_template`: default `/{drone_id}/depth/image_raw`.
- `camera_info_topic_template`: default `/{drone_id}/camera/camera_info`.
- `avoidance_status_topic_template`: default `/{drone_id}/avoidance/status`.
- `avoidance_events_topic_template`: declared, but not consistently used for subscription construction.

Publishes:

- `/swarm/mode`
- `/swarm/peer_cells`
- `/swarm/rth_request`
- `/field/mission_confirm`
- `/field/generate_grid`
- `/swarm/cell_override`
- `/swarm/manual_control`
- `/{drone_ns}/avoidance/target_cmd` for direct GCS target commands.

Subscribes:

- `/swarm/task_status`
- `/swarm/drone_status`
- `/swarm/mission_ready`
- `/swarm/mission_complete`
- `/field/setup_status`
- `/field/setup_complete`
- per-drone avoidance status and status JSON.
- per-drone avoidance events.
- per-drone camera image, depth image, and camera info.

Connected nodes:

- TCP client is `swarm_center.core.ros2_bridge.Ros2BridgeClient`.
- Receives mission/status streams from nearly all active mission nodes.
- Sends operator commands into setup, swarm, and per-drone runtime topics.

Implementation notes:

- Uses a background socket server and queue.
- Comments explicitly note QoS matching to active publishers.
- Bridges typed `AvoidanceStatus` or `String` fallback.

### `field_setup_tool`

File: `src/scout_control/scout_control/manual/field_setup_tool.py`

Class: `FieldSetupTool(Node)`

Purpose:

- Production setup-only manual helper.
- Provides curses/local controls and remote `/swarm/manual_control` handling for pad assignment, boundary marking, polygon close, and mission confirmation.

Parameters:

- `ui`: default `True`.
- `reject_origin_pad`: default `True`.
- `drone_count`: default `2`.

Publishes:

- `/swarm/pad_assignment`
- `/field/corner_marked`
- `/field/boundary_point`
- `/field/boundary_close`
- `/field/mission_confirm`
- `/{drone_ns}/rth_target`

Subscribes:

- `/swarm/manual_control`
- per-drone PX4 local position topics.

Connected nodes:

- Feeds `field_setup_coordinator` and `home_manager`.
- Receives remote commands from `gcs_bridge` and Swarm Center.
- Provides direct RTH targets for runtime during setup.

### `spray_controller`

File: `src/scout_control/scout_control/core/spray_controller.py`

Class: `SprayController(Node)`

Purpose:

- Simulated spray actuator.
- Watches cell-completion events and publishes per-drone spray commands with configured dose.
- Persists spray events in `reports/spray_events.json`.

Parameter:

- `dose_ml`: default constant from module.

Publishes:

- `/{drone_id}/spray_command` (`String` JSON), dynamically per drone. Here `drone_id` is the string from `/swarm/drone_status`, e.g. `drone_0`.

Subscribes:

- `/swarm/drone_status`

Connected nodes:

- Consumes `swarm_agent` cell completion.
- Output can be consumed by simulation/UI/reporting; no active ROS consumer was found in the code scan.

Implementation notes:

- It only sprays on `CELL_COMPLETE`.
- Log and JSON include dose, cell, drone, timestamp, and running total.

### `cell_data_recorder`

File: `src/scout_control/scout_control/core/cell_data_recorder.py`

Class: `CellDataRecorder(Node)`

Purpose:

- Passive ML training-data collector.
- Captures latest camera frame and position per drone, then records snapshots when `/swarm/drone_status` indicates a relevant cell event.

Parameters:

- `drone_count`: default `2`.
- `camera_topic_template`: default empty, resolved through `TelemetryHub`.
- `vehicle_position_topic_template`: default empty, resolved through `TelemetryHub`.

Publishes:

- None.

Subscribes:

- `/swarm/drone_status`
- per-drone camera topics.
- per-drone PX4 local-position topics.

Connected nodes:

- Consumes camera bridge/runtime mission status.
- Writes local dataset artifacts for later ML use.

### `ml_interface`

File: `src/scout_control/scout_control/core/ml_interface.py`

Classes:

- `DummyFieldModel`
- `DummySprayModel`
- `MLInterface(Node)`

Purpose:

- Tooling-only ML stub.
- Publishes dummy field-health, anomaly, and spray-dose signals from field grid.

Parameters:

- `publish_hz`: default `1.0`.
- `drone_count`: default `2`.
- `anomaly_threshold`: default `0.35`.
- `max_spray_dose`: default `3.0`.

Publishes:

- `/field/anomaly` (`String` JSON).
- `/field/cell_health` (`String` JSON).
- `/drone/spray_dose` (`String` JSON).

Subscribes:

- None.

Connected nodes:

- Reads `field_grid.json` from disk.
- Data is primarily for UI/tooling; no active control dependency was found.

Code-reality note:

- The module name and logs are honest: this is dummy data. Do not treat outputs as real agronomic inference.

### `grid_generator`

File: `src/scout_control/scout_control/utils/grid_generator.py`

Class: `GridGenerator(Node)`

Purpose:

- Builds a Cartesian occupancy grid from perimeter data, boundary polygon, or simulation preset.
- Publishes and writes the field grid.

Parameters:

- `cell_size`: default `5.0`.
- `sim_mode`: default `False`.
- `boundary_mode`: default `False`.
- `boundary_inset_m`: default `1.0`.
- `sim_field_size`: default `100.0`.
- `sim_field_width_m`: default `0.0`.
- `sim_field_height_m`: default `0.0`.
- `sim_origin_x`: default `20.0`.
- `sim_origin_y`: default `-50.0`.

Publishes:

- `/field/grid` (`nav_msgs/OccupancyGrid`).

Subscribes:

- None.

Connected nodes:

- Legacy/utility path for grid generation.
- Active E2E path usually uses `field_setup_coordinator` directly.

### `mapping_mission`

File: `src/scout_control/scout_control/missions/mapping_mission.py`

Classes:

- `MappingPhase`
- `DroneMappingState`
- `MappingMission(Node)`

Purpose:

- Generates and dispatches mapping routes before spray missions.
- Sends per-drone route targets through avoidance runtime rather than direct PX4.

Parameters:

- `drone_count`: default `1`.
- `altitude_m`: default `8.0`.
- `line_spacing_m`: default `4.0`.
- `side_overlap_pct`: default `30.0`.
- `cruise_speed_mps`: default `2.5`.
- `auto_start`: default `True`.
- `tick_hz`: default `2.0`.

Publishes:

- `/swarm/mapping_progress` (`String` JSON).
- `/swarm/mapping_complete` (`String` JSON).
- `/{drone_ns}/avoidance/target_cmd` (`TargetCommand` or JSON `String`).

Subscribes:

- `/swarm/home_positions`
- `/{drone_ns}/avoidance/status`
- `/{drone_ns}/avoidance/status_json`

Connected nodes:

- Commands `obstacle_avoidance_runtime`.
- Completion is consumed by `field_model_builder`.

### `field_model_builder`

File: `src/scout_control/scout_control/mapping/field_model_builder.py`

Class: `FieldModelBuilder(Node)`

Purpose:

- Accumulates mapping points, depth projections, and poses.
- Writes heightmap and obstacle artifacts.
- Publishes manifest path/metadata.

Parameters:

- `origin_x`: default `-50.0`.
- `origin_y`: default `-50.0`.
- `width_m`: default `100.0`.
- `height_m`: default `100.0`.
- `cell_size_m`: default `0.5`.
- `obstacle_cell_size_m`: default `0.75`.
- `min_obstacle_points`: default `3`.
- `drone_count`: default `1`.
- `depth_stride`: default `8`.

Publishes:

- `/swarm/field_model_manifest` (`String` JSON).

Subscribes:

- `/swarm/mapping_points`
- `/swarm/mapping_complete`
- per-drone PX4 local position.
- per-drone camera info.
- per-drone depth image.

Connected nodes:

- Receives completion from `mapping_mission`.
- Uses `DepthProjector`, `Heightmap2D`, and `extract_obstacles`.
- Artifacts can be consumed by `GridRefiner`, `swarm_center`, and reports.

### `precision_landing`

File: `src/scout_control/scout_control/vision/precision_landing.py`

Class: `PrecisionLanding(Node)`

Purpose:

- Advisory node for detecting landing pad marker and publishing offset payloads.

Parameters:

- `drone_id`: default `0`.
- `marker_size_m`: default `0.35`.
- `active_phase`: default `RETURN_HOME`.
- `max_active_altitude_m`: default `5.0`.
- `advisory_only`: default `True`.

Publishes:

- `/{drone_ns}/precision_landing/offset` (`String` JSON).

Subscribes:

- `/{drone_ns}/camera/image_raw`
- `/{drone_ns}/camera/camera_info`
- `/{drone_ns}/avoidance/status_json`
- `/swarm/home_positions`

Connected nodes:

- Consumes runtime status and camera stream.
- Publishes advisory offsets; current runtime control path does not consume this topic.

### `camera_hud`

File: `src/scout_control/scout_control/viz/camera_hud.py`

Class: `CameraHud(Node)`

Purpose:

- OpenCV camera viewer with HUD and optional minimap for field setup/manual mapping.

Parameters:

- `drone_id`: default `0`.
- `show_minimap`: default `True`.
- `camera_topic`: default empty; falls back to `TelemetryHub`.
- `pos_topic`: default empty; falls back to PX4 local position.

Publishes:

- None.

Subscribes:

- camera image topic.
- PX4 local position topic.

Connected nodes:

- Consumes image bridge and PX4 output.

### `gimbal_cam_viz`

File: `src/scout_control/scout_control/viz/gimbal_cam_viz.py`

Class: `GimbalCamViz(Node)`

Purpose:

- OpenCV camera viewer with digital pan/tilt/zoom and status overlay.

Parameters:

- `drone_id`: default `0`.
- `camera_topic`, `pos_topic`, `status_topic`, `avoid_topic`: optional overrides.
- `subscribe_legacy_topics`: default `False`.
- `pan_step_deg`: default `3.0`.
- `tilt_step_deg`: default `2.0`.
- `zoom_step`: default `0.1`.

Publishes:

- None.

Subscribes:

- camera image.
- PX4 local position.
- avoidance status.
- avoidance active.
- optional legacy `/obstacle_avoidance/*` topics.

### `obstacle_viz`

File: `src/scout_control/scout_control/viz/obstacle_viz.py`

Class: `ObstacleViz(Node)`

Purpose:

- RViz visualization adapter for obstacle detections, drone marker, planned path, actual path, and obstacle cloud.

Parameters:

- `drone_id`: default `0`.
- `subscribe_legacy_topics`: default `False`.

Publishes:

- `/visualization/obstacle_markers` (`MarkerArray`).
- `/visualization/drone_marker` (`Marker`).
- `/visualization/planned_path` (`Path`).
- `/visualization/actual_path` (`Path`).
- `/visualization/obstacle_cloud` (`PointCloud2`).

Subscribes:

- PX4 local position.
- `/{drone_ns}/avoidance/active`
- `/{drone_ns}/avoidance/planned_path`
- `/{drone_ns}/avoidance/actual_path`
- optional legacy runtime topics.

### `scan_cloud_viz`

File: `src/scout_control/scout_control/viz/scan_cloud_viz.py`

Purpose:

- Simple viewer for saved obstacle scan point clouds.
- No ROS node class was found; `main()` is a standalone visualization entry point.

### `manual_controller`

File: `src/scout_control/scout_control/manual/manual_controller.py`

Classes:

- `Phase`
- `DroneCtrl`
- `ManualController(Node)`

Purpose:

- Legacy dual-drone manual PX4 flight controller.
- Provides curses UI and remote manual-control handling.

Parameters:

- `altitude`: default module constant.
- `ui`: default `True`.

Publishes:

- Per-PX4 namespace:
  - `.../fmu/in/offboard_control_mode`
  - `.../fmu/in/trajectory_setpoint`
  - `.../fmu/in/vehicle_command`
- `/swarm/pad_assignment`
- `/field/corner_marked`
- `/field/mission_confirm`
- `/swarm/landed_confirmation`
- `/drone_0/rth_target`
- `/drone_1/rth_target`

Subscribes:

- Per-PX4 namespace local position.
- `/{drone}/rth_target`
- `/swarm/mission_ready`
- `/swarm/landed_confirmation`
- `/swarm/manual_control`

Connected nodes:

- Can drive setup and manual PX4 control directly.
- Must not be run together with `obstacle_avoidance_runtime` for the same drones unless intentionally testing PX4 input ownership conflicts.

### `manual_commander`

File: `src/scout_control/scout_control/manual/manual_commander.py`

Class: `ManualCommander(Node)`

Purpose:

- Single-drone WSAD manual flight for field perimeter mapping.

Parameter:

- `altitude`: default module constant.

Publishes:

- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`

Subscribes:

- `/fmu/out/vehicle_local_position_v1`

Registration note:

- Has a `main()`, but is not registered in current `setup.py`.

### `field_commander`

File: `src/scout_control/scout_control/manual/field_commander.py`

Class: `FieldCommander(Node)`

Purpose:

- Interactive single-drone grid-based commander for manual field-cell navigation.

Publishes:

- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`
- `/field/grid_state`
- `/swarm/rth_request`
- `/swarm/landed_confirmation`

Subscribes:

- `/fmu/out/vehicle_local_position_v1`
- `/drone_0/rth_target`

Registration note:

- Has a `main()`, but is not registered in current `setup.py`.

## Legacy ROS Nodes

These modules have `main()` functions but are not registered as console scripts in the current `setup.py`.

### `legacy/offboard_control.py`

Class: `OffboardControl(Node)`

Publishes:

- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`

Subscribes:

- `/fmu/out/vehicle_local_position_v1`
- `/fmu/out/vehicle_attitude`

Purpose:

- Simple waypoint/offboard test controller.

### `legacy/position_monitor.py`

Class: `PositionMonitor(Node)`

Subscribes:

- `/fmu/out/vehicle_local_position_v1`

Purpose:

- Logs PX4 local position.

### `legacy/perimeter_flight.py`

Class: `PerimeterFlight(Node)`

Publishes:

- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`
- `/field/perimeter`

Subscribes:

- `/fmu/out/vehicle_local_position_v1`
- `/fmu/out/vehicle_global_position`

Purpose:

- Legacy PX4 perimeter survey.

### `legacy/terrain_follower.py`

Class: `TerrainFollower(Node)`

Parameter:

- `desired_height`: default `3.0`.

Publishes:

- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`

Subscribes:

- `/fmu/out/vehicle_local_position_v1`
- `/downward_lidar/scan`

Purpose:

- Single-drone terrain-following offboard controller.

### `legacy/obstacle_detector.py`

Class: `ObstacleDetector(Node)`

Parameters:

- `drone_id`, `warn_distance`, `stop_distance`, `cell_size`, `map_decay_secs`, camera geometry, log label.

Publishes:

- `/{drone_ns}/obstacles/detected`
- `/{drone_ns}/obstacles/clear`

Subscribes:

- `/{drone_ns}/depth/image_raw`
- PX4 local position.
- `/field/grid`

Purpose:

- Older OakD-Lite depth obstacle detector.

### `legacy/obstacle_avoidance_mission.py`

Class: `ObstacleAvoidanceMission(Node)`

Parameters:

- `drone_id`, altitude/speed/clearance, home behavior, blocked timeout, log label.

Publishes:

- Runtime command topic, derived from drone id.

Subscribes:

- Runtime status topic, derived from drone id.

Purpose:

- Test mission that feeds route targets to the obstacle avoidance runtime.

## Launch Files and Topology

### `full_e2e_mission.launch.py`

Starts the full production-like stack:

- `field_setup_coordinator`
- `home_manager`
- `field_setup_tool`
- `obstacle_avoidance_runtime` for each drone
- `swarm_agent` for each drone
- `swarm_coordinator`
- `cell_data_recorder`
- `spray_controller`
- `ml_interface`
- `mission_launcher`
- `gcs_bridge`
- optional Gazebo image/lidar bridges per drone

Notable parameters:

- Runtime gets `require_depth_for_navigation=False` and `altitude_policy_mode='TerrainFollow'`.
- `field_setup_tool` receives `drone_count`.
- `swarm_coordinator` receives long `ready_timeout=600.0`.

### `isaac_e2e_mission.launch.py`

Starts an Isaac-oriented E2E stack:

- Same broad set as full E2E.
- Explicit `obstacle_avoidance_runtime` and `swarm_agent` for drones 0 and 1.
- Runtime has relaxed heading/XY/dead-reckoning gates and `force_arm=True`.
- `gcs_bridge` receives camera/depth templates and FPS limits.

Code-reality note:

- `field_setup_tool` is launched with `ui=False`, `reject_origin_pad=False`, but no `drone_count`; default remains `2`.

### `mapping_mission.launch.py`

Starts:

- `obstacle_avoidance_runtime` per drone.
- `field_model_builder`.
- `mapping_mission`.

Purpose:

- Pre-operational mapping route collection and field-model artifact generation.

### `obstacle_avoidance_test.launch.py`

Starts:

- Single `obstacle_avoidance_runtime`.
- `obstacle_viz`.
- Gazebo image bridge.

Purpose:

- Local runtime avoidance testing and RViz visualization.

### `precision_landing_test.launch.py`

Starts:

- `obstacle_avoidance_runtime`.
- `precision_landing`.
- camera image bridge.

Purpose:

- Tests advisory landing-marker detection flow.

### `swarm_mission.launch.py`

Starts:

- `task_allocator`
- `swarm_agent` per drone

Code-reality issue:

- This launch is stale/broken as written. The executable `task_allocator` points to `scout_control.utils.task_allocator:main`, but that module has no `main()` and no ROS node. Use `swarm_coordinator` as the ROS wrapper around `TaskAllocator`.

### Bridge launches

`camera_bridge.launch.py`

- Starts `ros_gz_image image_bridge`.

`lidar_bridge.launch.py`

- Starts camera image bridge and `ros_gz_bridge parameter_bridge` for lidar scan.

`gimbal_bridge.launch.py`

- Starts `ros_gz_bridge parameter_bridge` for gimbal-related topics.

`camera_hud.launch.py`

- Starts camera image bridge and `camera_hud`.

## Python Modules and Classes

### Avoidance package

`avoidance/types.py`

- Shared datatypes and compatibility helpers.
- Classes:
  - `LocalMapperState`: mapper lifecycle enum.
  - `ScanState`: scan lifecycle enum.
  - `TargetCommand`: high-level runtime command; has `from_payload()` and `to_payload()`.
  - `PlannerConfig`: shared planner/perception defaults.
  - `PlanResult`: typed planner output with payload conversion.
  - `LocalGridSnapshot`: planner-ready occupancy/cost snapshot.
  - `BlockedEvent`: structured blocked-state record.
  - `PointBatch`: shared point batch between projectors and mapper.
  - `ScanArtifactPaths`, `ScanMeta`, `ScanCompleteEvent`, `ScanCommand`, `ScanStepResult`.
  - `AvoidanceStatus`: typed view of avoidance status JSON/typed payload.
  - `SwarmDroneStatusEvent`, `SwarmTaskStatus`, `PadAssignment`, `FieldSetupComplete`, `ReturnHomeRequest`, `MissionReadySignal`.
- Functions convert between JSON payloads, ROS `String`, and custom typed messages where available.

`avoidance/telemetry_hub.py`

- Central topic construction.
- `DroneTopicContract`: immutable per-drone topic list.
- `SwarmTopicContract`: immutable swarm topic list.
- `TelemetryHub`: builds topic contracts and supports overrides for camera/depth/info/range.
- `TopicOwnership`: status object for PX4 publisher counts.
- `Px4InputOwnershipGuard`: detects more publishers than expected on PX4 input topics.

`avoidance/px4_publisher_adapter.py`

- `PX4MessageTypes`: injectable message-class bundle.
- `PX4PublisherAdapter`: creates and owns publishers for PX4 offboard mode, trajectory setpoint, and vehicle command; publishes heartbeats/setpoints/commands.
- Keeps PX4 publishing isolated from runtime phase logic.

`avoidance/ros_io_adapter.py`

- `RosIOAdapter`: helper for JSON messages, `Bool`, runtime events, typed/JSON status publication, and optional legacy mirroring.

`avoidance/altitude_controller.py`

- `AltitudeSetpoint`: resolved altitude command.
- `AltitudeController`: converts mission altitude and terrain reference into PX4 NED setpoint; supports fixed and terrain-follow style policy.

`avoidance/health_monitor.py`

- `HealthConfig`, `PoseHealth`, `RuntimeReadiness`, `RuntimeHealthMonitor`.
- Tracks freshness/validity of pose and depth inputs.
- Produces readiness payloads used by runtime status.

`avoidance/depth_projector.py`

- `CameraIntrinsics`: pinhole intrinsics from HFOV or `CameraInfo`.
- `DepthProjector`: projects depth frames into body/local/world points, applies collision-band filters, records projection metadata.

`avoidance/lidar_projector.py`

- `laser_scan_to_body_points()`.
- `body_to_world_points()`.
- Lightweight LaserScan projection helper for optional obstacle ingestion.

`avoidance/local_mapper.py`

- `LocalMapperConfig`, `LocalClearanceSummary`, `LocalMapperSnapshot`, `LocalMapper`.
- Rolling local occupancy/cost map.
- Ingests sensor points, peer positions, blocked zones, and decays/free-space raytracing.
- Provides clearance summaries and planner masks.

`avoidance/local_planner.py`

- `PlannerResultStatus`, `LocalPlannerState`.
- `PlannerPose`, `PlannerTarget`, `BlockedHistoryEntry`, `DynamicMaskDisk`, `LocalGridSnapshot`, `PlanResult`, `LocalPlannerConfig`.
- `LocalPlanner`: tries direct path, drift path, A* candidates, blocked-history masks, peer masks, local-trap checks, and hard-block decisions.

`avoidance/scan_manager.py`

- `ScanManager`: controls local scan lifecycle.
- Starts scans, captures observations, processes points, writes artifacts, prunes artifacts.

`avoidance/peer_tracks.py`

- `PeerTrack`: latest peer position/speed/age.
- `SafetyDiskZone`: hard/soft peer no-go disk.
- `PeerTrackStore`: buffers peer histories and derives planner masks.

`avoidance/avoidance_logging.py`

- `AvoidanceRunLogger`: JSONL logger for obstacle avoidance events.

### Core package

`core/swarm_agent.py`

- Node described above.
- `Phase` enum represents local mission/executor state.

`core/swarm_coordinator.py`

- Node described above.
- Builds `TaskAllocator` from `field_grid.json`.

`core/field_setup_coordinator.py`

- Node described above.
- `SetupState` enum describes setup state machine.

`core/home_manager.py`

- Node described above.
- `normalize_pad()` normalizes incoming pad payloads.

`core/gcs_bridge.py`

- Node described above.
- Also owns TCP lifecycle and incoming command dispatch.

`core/mission_launcher.py`

- Node described above.

`core/spray_controller.py`

- Node described above.

`core/cell_data_recorder.py`

- Node described above.

`core/ml_interface.py`

- Node described above.

`core/obstacle_avoidance_runtime.py`

- Node described above.
- `RuntimePhase` enum represents runtime lifecycle.

### Mapping package

`mapping/heightmap.py`

- `Heightmap2D`: numpy-backed 2.5D terrain heightmap.
- Converts world/grid coordinates, updates from points, serializes to/from dict.

`mapping/obstacle_extractor.py`

- `Obstacle`: serializable static obstacle.
- `extract_obstacles()`: grid-based obstacle extraction from mapping points.

`mapping/grid_refiner.py`

- `GridRefiner`: creates no-go/caution zones from field-model obstacles and refines grid cell classifications.

`mapping/mission_package_builder.py`

- `MissionPackageBuilder`: filters cells and builds per-drone mission packages.
- Supports sector and round-robin style distribution and boustrophedon sorting.

`mapping/field_model_builder.py`

- Node described above.

### Missions package

`missions/mapping_mission.py`

- Node described above.
- Uses `lawnmower.generate_lawnmower()` route generation and runtime target commands.

### Utils package

`utils/task_allocator.py`

- Pure-Python allocator. Not a ROS node.
- `DroneStatus`: allocator state enum.
- `DroneRecord`: mutable per-drone allocation state.
- `TaskAllocator`: owns mission start, sector assignment, next-cell publishing callbacks, rebalancing, blocked/deferred cell handling, progress payloads, and mission completion.
- External I/O is injected via callbacks:
  - `on_next_cell(drone_id, cell)`
  - `on_task_status(payload)`
  - `on_mission_complete(payload)`
  - `on_rth(drone_id)`

`utils/grid_generator.py`

- Node described above.

`utils/lawnmower.py`

- `generate_lawnmower()`: pure route-generation helper for mapping missions.

`utils/polygon.py`

- Pure 2D geometry helpers:
  - signed area, orientation, bounding box.
  - point-in-polygon.
  - polygon inset.
  - cell/edge overlap and cell classification.
  - vertex conversion from dicts.

`utils/paths.py`

- Canonical paths for runtime data files.

`utils/bridge_protocol.py`

- Shared JSON message schema between `gcs_bridge` and `swarm_center`.

### Vision package

`vision/pad_detector.py`

- `CameraIntrinsics`, `PadDetection`.
- `detect_pad_marker()`: marker/pad detection helper used by `precision_landing`.

`vision/precision_landing.py`

- Node described above.

### Visualization package

`viz/camera_hud.py`, `viz/gimbal_cam_viz.py`, `viz/obstacle_viz.py`, `viz/scan_cloud_viz.py`

- Nodes/tools described above.

### Manual package

`manual/field_setup_tool.py`, `manual/manual_controller.py`, `manual/manual_commander.py`, `manual/field_commander.py`

- Setup/manual control tools described above.

## Swarm Center Application

### `swarm_center/main.py`

- Entry point for PySide GCS.
- Parses args, constructs managers/UI, starts Qt event loop.

### Core modules

`swarm_center/core/ros2_bridge.py`

- `Ros2BridgeClient(QObject)`: TCP client for `gcs_bridge`.
- Sends:
  - set mode
  - RTH all/drone
  - peer cells
  - start mission
  - generate grid
  - emergency stop
  - goto cell/drone
  - manual control
  - camera control
- Receives newline-delimited JSON and dispatches signals to UI/state managers.
- `Ros2BridgeThreadRunner`: owns QThread.

`swarm_center/core/bridge_protocol.py`

- Local copy of bridge message protocol.
- Must stay in sync with `scout_control.utils.bridge_protocol`.

`swarm_center/core/mavlink_manager.py`

- `DroneTelemetry`: latest MAVLink snapshot.
- `MavlinkWorker(QObject)`: per-drone UDP MAVLink connection, heartbeat loop, telemetry decode, arm/disarm.
- `SwarmMavlinkManager(QObject)`: owns workers/threads for all drones.

`swarm_center/core/swarm_manager.py`

- `DroneRecord`, `MissionState`, `SwarmManager`.
- Central in-memory UI state for drones, grid, mission status, avoidance status, selected drone, and listeners.

`swarm_center/core/field_manager.py`

- `Cell`, `FieldGrid`.
- Loads field grid, creates synthetic grid, regrids, finds cells by NED or ID.

`swarm_center/core/depth_mapper.py`

- `CameraIntrinsics`, `DepthMapper`.
- Accumulates lightweight terrain height map from depth frames for UI overlay.

`swarm_center/core/field_model_loader.py`

- Loads field-model overlay JSONs.

`swarm_center/core/report_generator.py`

- Generates self-contained HTML mission report from grid/swarm state and persisted spray events.

`swarm_center/core/app_logger.py`

- Qt-friendly app logging.

### UI modules

`swarm_center/ui/main_window.py`

- Top-level `QMainWindow`.
- Wires grid loading, bridge events, MAVLink telemetry, report generation, field model overlays, camera/depth mapping, and control callbacks.

`swarm_center/ui/field_view.py`

- Top-down field view.
- Draws grid, drones, trails, sector preview, terrain overlay, no-go zones, obstacles, and markers.

`swarm_center/ui/viewport_3d.py`

- Software-rendered 3D scene for grid, trails, altitude columns, terrain, and drones.

`swarm_center/ui/control_panel.py`

- Right-hand control column.
- Mission status, bridge/MAVLink status, logs, cell size, grid load/apply.

`swarm_center/ui/drone_list.py`

- Drone sidebar with telemetry, selection, and context actions.

`swarm_center/ui/camera_view.py`

- Multi-drone camera/depth feed viewer.
- Per-drone `_DroneCamera` supports enable/FPS/control widgets.

`swarm_center/ui/manual_control.py`

- Manual control widget.
- Sends manual movement/action commands through `Ros2BridgeClient`.

`swarm_center/ui/avoidance_panel.py`

- Collapsible avoidance status and blocked-event history panel.

## End-to-End Data Flow

### Setup and mission start

1. `field_setup_tool` or Swarm Center emits pad and boundary commands.
2. `gcs_bridge` republishes remote GCS commands into `/swarm/manual_control`, `/field/generate_grid`, `/field/mission_confirm`, or direct target topics.
3. `field_setup_coordinator` consumes setup events.
4. `field_setup_coordinator` writes field grid and home positions.
5. `field_setup_coordinator` publishes `/field/setup_complete` and `/swarm/mission_ready`.
6. `swarm_coordinator` receives mission-ready, reloads grid, starts ready timeout.
7. `swarm_agent` receives mission-ready and announces/keeps readiness via `/swarm/drone_status`.
8. `TaskAllocator` starts once drones are ready or timeout permits partial start.

### Cell allocation and flight

1. `swarm_coordinator` publishes `/{drone}/next_cell`.
2. `swarm_agent` converts cell payload into `TargetCommand`.
3. `obstacle_avoidance_runtime` receives target, enters navigation phases, and owns PX4 setpoints.
4. Runtime publishes typed/JSON avoidance status.
5. `swarm_agent` converts runtime completion/blockage into `/swarm/drone_status`.
6. `swarm_coordinator` marks cells visited, defers/reassigns blocked cells, rebalances queues, and emits new cells.

### Obstacle and peer handling

1. Runtime ingests depth/lidar/camera info and PX4 pose.
2. `DepthProjector` and `lidar_projector` turn sensor frames into point batches.
3. `LocalMapper` updates rolling occupancy and clearance summaries.
4. `PeerTrackStore` derives peer safety disks from `/swarm/peer_telemetry`.
5. `LocalPlanner` chooses direct/drift/A* plans or blocked results.
6. Runtime publishes status/events/paths and updates PX4 setpoints.

### RTH and mission completion

1. `TaskAllocator` calls `on_mission_complete`.
2. `swarm_coordinator` publishes `/swarm/mission_complete` and per-drone `/swarm/rth_request`.
3. `home_manager` publishes `/{drone}/rth_target`.
4. `swarm_agent` also translates RTH requests into runtime commands.
5. Runtime navigates home and emits landing/RTH status.
6. `swarm_agent` publishes landed confirmation when appropriate.

### GCS flow

1. `gcs_bridge` streams setup/mission/drone/avoidance/camera/depth events to Swarm Center.
2. `swarm_center.core.ros2_bridge` receives and dispatches JSON.
3. `SwarmManager`, `FieldGrid`, `DepthMapper`, and UI widgets update.
4. Operator commands go back over the TCP bridge to ROS topics.

## Message Contracts

### `/swarm/drone_status`

Producer:

- `swarm_agent`.

Consumers:

- `swarm_coordinator`
- `spray_controller`
- `cell_data_recorder`
- `gcs_bridge`
- `swarm_center`

Payload:

- JSON string.
- `SwarmDroneStatusEvent` supports parsing and compatibility fields.
- Status values are normalized by `TaskAllocator`, including:
  - `READY`
  - `CELL_COMPLETE`
  - `NAV_ACTIVE` -> `NAVIGATING`
  - `NAV_COMPLETED` -> `CELL_COMPLETE`
  - `NAV_BLOCKED_SOFT`/`NAV_BLOCKED_HARD` -> `BLOCKED`
  - `RTH`, `LANDING`, `ABORT`.

### `/{drone}/avoidance/status`

Producer:

- `obstacle_avoidance_runtime`.

Consumers:

- `swarm_agent`
- `mapping_mission`
- `gcs_bridge`
- visualizers

Payload:

- Custom `AvoidanceStatus` if available, else JSON string.
- `/{drone}/avoidance/status_json` is always JSON string compatibility channel.

### `/{drone}/avoidance/target_cmd`

Producers:

- `swarm_agent`
- `mapping_mission`
- `gcs_bridge`

Consumer:

- `obstacle_avoidance_runtime`

Payload:

- Custom `TargetCommand` if available, else JSON string.
- `target_cmd_json` provides string compatibility.

### `/swarm/task_status`

Producer:

- `swarm_coordinator`.

Consumers:

- `gcs_bridge`
- `mission_launcher`
- `swarm_center`

Payload:

- JSON mission progress, drone queues/statuses, deferred count, rebalance count.

## Code-Reality and Consistency Findings

### Stale or broken launch/entry-point wiring

1. `swarm_mission.launch.py` launches `task_allocator`.

Actual code:

- `TaskAllocator` is pure Python, has no `main()`, and is intended to be driven by `SwarmCoordinator`.
- `setup.py` registers `task_allocator = scout_control.utils.task_allocator:main`, but no such function exists.

Impact:

- `ros2 launch scout_control swarm_mission.launch.py` is likely to fail when it tries to start the `task_allocator` executable.

Suggested fix:

- Replace `task_allocator` launch action with `swarm_coordinator`, or add a real ROS wrapper `main()` if a separate allocator node is desired.

2. Legacy command names in docs do not match current console scripts.

Actual code:

- Docs mention `ros2 run scout_control legacy_manual_controller`, etc.
- `setup.py` registers `manual_controller`, not `legacy_manual_controller`.
- `manual_commander`, `field_commander`, and `legacy/*` modules have `main()` functions but are not registered.

Impact:

- Operator guide commands can fail even though code exists.

Suggested fix:

- Either register explicit legacy aliases, or update docs to current names and clearly mark unregistered modules.

### Parameters that look configurable but are forced or partially used

1. `swarm_agent.navigation_backend`

Actual code:

- Parameter is declared.
- `_navigation_backend` is set to constant `avoidance_runtime`.
- `_runtime_backend_active` is always `True`.
- Log line prints backend/runtime state.

Impact:

- Launch or scenario values cannot switch to a direct backend. This appears intentional after migration, but the parameter still suggests configurability.

Suggested fix:

- Remove parameter or log it as deprecated/ignored.

2. `gcs_bridge.avoidance_events_topic_template`

Actual code:

- Parameter is declared in the same group as status/camera/depth templates.
- Status topic template is used.
- Event subscription path is built from `TelemetryHub` default event topic.

Impact:

- If a launch overrides event topic template, status and event streams can diverge.

Suggested fix:

- Apply event template consistently.

3. `field_setup_tool.drone_count` in Isaac launch

Actual code:

- `field_setup_tool` declares and uses `drone_count`.
- `full_e2e_mission.launch.py` passes it.
- `isaac_e2e_mission.launch.py` does not pass it.

Impact:

- With `drone_count != 2`, Isaac setup tool behavior can diverge from coordinator/runtime count.

Suggested fix:

- Pass `{'drone_count': drone_count}` in Isaac launch.

### Dummy or advisory modules

1. `ml_interface`

- Publishes dummy health/anomaly/dose data.
- Uses fallback 4x4 dummy grid if no field grid exists.
- Should be documented as tooling/demo only.

2. `precision_landing`

- Publishes advisory offset payloads.
- `advisory_only=True` default is honest.
- Runtime does not currently consume `/precision_landing/offset`.

### Compatibility and legacy behavior

1. Runtime status and commands are intentionally dual-path:

- Typed messages when custom package is available.
- JSON strings for compatibility.

2. Legacy `/obstacle_avoidance/*` topics are off by default:

- Runtime only publishes them for drone 0 when `publish_legacy_obstacle_topics=True`.
- Visualizers can subscribe to them when `subscribe_legacy_topics=True`.

3. Manual PX4 controllers can conflict with runtime:

- `manual_controller`, `field_commander`, `manual_commander`, and legacy offboard controllers publish PX4 input topics.
- Runtime owns the same topics in active autonomy.
- Runtime includes publisher ownership diagnostics, but launch discipline is still required.

## Recommended Cleanup Backlog

1. Fix or remove `task_allocator` console script and `swarm_mission.launch.py` stale node action.
2. Align operator-guide legacy commands with `setup.py`, or register explicit legacy aliases.
3. Pass `drone_count` to `field_setup_tool` in `isaac_e2e_mission.launch.py`.
4. Apply `avoidance_events_topic_template` in `gcs_bridge`.
5. Remove or clearly rename deprecated `swarm_agent.navigation_backend`.
6. Add a short documented consumer status for `/drone/spray_dose`, `/field/anomaly`, and `/field/cell_health`.
7. Decide whether `precision_landing` remains advisory or becomes runtime-integrated; wire runtime subscription only if closed-loop landing is intended.
8. Add a launch-level guard or docs warning for PX4 input ownership when running manual controllers.
