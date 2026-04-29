# Scout ROS2 Topic Contract

**Status:** active contract  
**Last updated:** 2026-04-29

This document is the human-readable contract for ROS2 topics used by the
production Scout mission stack. The executable source of truth is
`scout_control.avoidance.telemetry_hub.TelemetryHub`; this document must not
define alternate names.

## Ownership Rules

- `obstacle_avoidance_runtime` is the only production flight owner. It is the
  only production node that may publish PX4 input setpoints on `/fmu/in/*` or
  `/px4_N/fmu/in/*`.
- `swarm_agent`, `swarm_coordinator`, `mission_launcher`, GCS bridge, and
  operator tools delegate intent; they do not publish PX4 setpoints in
  production flow.
- `field_setup_tool` is setup-only. It may publish setup, boundary, pad, mission
  confirmation, and RTH target messages, but it must not publish PX4 setpoints.
- Legacy/manual PX4 controllers are debug tools only. They must not be included
  by production E2E launch files.

## TelemetryHub Rule

All new drone, swarm, setup, runtime, and GCS-facing topics must be added to
`TelemetryHub` first, then consumed from that contract in runtime code. Launch
files may pass backend-specific sensor topic templates or overrides, but they
must not create a competing naming convention.

When renaming a topic, keep the old topic only as an explicit compatibility
publisher/subscriber with a deprecation note and a removal condition. Do not
silently dual-publish from production launch files.

## Per-Drone Topics

`{drone_ns}` is `drone_0`, `drone_1`, etc. Drone 0 uses bare PX4 topics; drone
N uses `/px4_N` for PX4 input/output topics.

| Topic | Direction | Payload | QoS | Owner / Notes |
| :--- | :--- | :--- | :--- | :--- |
| `/{drone_ns}/avoidance/target_cmd` | command to runtime | `scout_control_msgs/TargetCommand` or `std_msgs/String` JSON fallback | reliable volatile | Published by `swarm_agent`; consumed by runtime. |
| `/{drone_ns}/avoidance/target_cmd_json` | command to runtime | `std_msgs/String` JSON | reliable volatile | Compatibility mirror while typed `TargetCommand` rolls out. |
| `/{drone_ns}/avoidance/status` | runtime status | `scout_control_msgs/AvoidanceStatus` or string fallback | best-effort transient-local | Stable status subset; latched-like for late subscribers. |
| `/{drone_ns}/avoidance/status_json` | runtime status | `std_msgs/String` JSON | best-effort transient-local | GCS/debug-compatible structured status. |
| `/{drone_ns}/avoidance/events` | runtime events | `std_msgs/String` JSON/text | best-effort volatile | Ephemeral runtime events. |
| `/{drone_ns}/avoidance/active` | runtime viz | `std_msgs/Bool` | best-effort volatile | Avoidance active flag. |
| `/{drone_ns}/avoidance/planned_path` | runtime viz | `nav_msgs/Path` | best-effort volatile | Planned path visualization. |
| `/{drone_ns}/avoidance/actual_path` | runtime viz | `nav_msgs/Path` | best-effort volatile | Actual path visualization. |
| `/{drone_ns}/obstacles/detected` | runtime viz | `std_msgs/String` | best-effort volatile | Obstacle summary. |
| `/{drone_ns}/obstacles/clear` | runtime viz | `std_msgs/Bool` | best-effort volatile | Obstacle clear flag. |
| `/{drone_ns}/next_cell` | mission assignment | `std_msgs/String` | reliable transient-local | Published by `swarm_coordinator`; consumed by `swarm_agent`. |
| `/{drone_ns}/rth_target` | setup/RTH target | `geometry_msgs/Point` | reliable transient-local | Published by setup tooling/home flow; consumed by runtime/agent. |
| `/{drone_ns}/precision_landing/offset` | advisory landing | `std_msgs/String` JSON | volatile or local node default | Advisory only; no PX4 setpoints. |
| `/{drone_ns}/camera/image_raw` | sensor | `sensor_msgs/Image` | best-effort volatile | Native sim or bridge topic. Backend may override template. |
| `/{drone_ns}/depth/image_raw` | sensor | `sensor_msgs/Image` | best-effort volatile | Native sim depth; runtime may gate navigation on freshness. |
| `/{drone_ns}/camera/camera_info` | sensor | `sensor_msgs/CameraInfo` | best-effort volatile | Camera calibration/info. |
| `/{drone_ns}/downward_lidar/scan` | sensor | `sensor_msgs/LaserScan` | best-effort volatile | Terrain/range input. May also be used as optional obstacle point input only when `obstacle_avoidance_runtime.enable_lidar_obstacle_points=true`; disabled by default. |

Optional LiDAR obstacle ingestion is configured on `obstacle_avoidance_runtime`
with `enable_lidar_obstacle_points`, `lidar_obstacle_topic`,
`lidar_obstacle_confidence`, `lidar_obstacle_stride`, and
`lidar_obstacle_stale_after_s`. If `lidar_obstacle_topic` is empty, the runtime
uses the canonical `/{drone_ns}/downward_lidar/scan` topic when obstacle
ingestion is enabled.

## PX4 Topics

| Topic | Direction | Payload | QoS | Owner / Notes |
| :--- | :--- | :--- | :--- | :--- |
| `/fmu/in/offboard_control_mode` | runtime to PX4 | PX4 `OffboardControlMode` | best-effort transient-local | Drone 0 only; runtime-owned. |
| `/fmu/in/trajectory_setpoint` | runtime to PX4 | PX4 `TrajectorySetpoint` | best-effort transient-local | Drone 0 only; runtime-owned. |
| `/fmu/in/vehicle_command` | runtime to PX4 | PX4 `VehicleCommand` | best-effort transient-local | Drone 0 only; runtime-owned. |
| `/px4_N/fmu/in/offboard_control_mode` | runtime to PX4 | PX4 `OffboardControlMode` | best-effort transient-local | Drone N; runtime-owned. |
| `/px4_N/fmu/in/trajectory_setpoint` | runtime to PX4 | PX4 `TrajectorySetpoint` | best-effort transient-local | Drone N; runtime-owned. |
| `/px4_N/fmu/in/vehicle_command` | runtime to PX4 | PX4 `VehicleCommand` | best-effort transient-local | Drone N; runtime-owned. |
| `/fmu/out/vehicle_local_position_v1` | PX4 to runtime/tools | PX4 local position | best-effort transient-local | Drone 0 output. |
| `/fmu/out/vehicle_status_v3` | PX4 to runtime/tools | PX4 vehicle status | best-effort transient-local | Drone 0 output. |
| `/fmu/out/vehicle_control_mode` | PX4 to runtime/tools | PX4 control mode | best-effort transient-local | Drone 0 output. |
| `/fmu/out/vehicle_command_ack_v1` | PX4 to runtime/tools | PX4 command ack | best-effort transient-local | Drone 0 output. |

Apply the same `/px4_N` prefix for drone N PX4 output topics.

## Swarm Topics

| Topic | Payload | QoS | Notes |
| :--- | :--- | :--- | :--- |
| `/swarm/peer_telemetry` | `scout_control_msgs/PeerTelemetry` | best-effort volatile | Runtime peer sharing. |
| `/swarm/drone_status` | `std_msgs/String` JSON/text | reliable volatile | Agent status to coordinator. |
| `/swarm/task_status` | `std_msgs/String` JSON | reliable transient-local | Coordinator progress, GCS-visible. |
| `/swarm/mission_complete` | `std_msgs/String` JSON | volatile | Mission completion event. |
| `/swarm/mission_ready` | `std_msgs/String` | latched-like | Setup complete and mission can start. |
| `/swarm/start_mission` | `std_msgs/String` | latched-like | Mission launcher trigger. |
| `/swarm/rth_request` | `std_msgs/String` JSON/text | reliable volatile | Return-home request. |
| `/swarm/landed_confirmation` | `std_msgs/String` JSON/text | best-effort volatile | Landing/pad confirmation. |
| `/swarm/pad_assignment` | `std_msgs/String` JSON/text | best-effort volatile | Setup pad assignment. |
| `/swarm/pad_query` | `std_msgs/String` JSON | reliable volatile | Pad allocation query. |
| `/swarm/pad_response` | `std_msgs/String` JSON | reliable transient-local | Pad allocation response. |
| `/swarm/home_positions` | `std_msgs/String` JSON | reliable transient-local | Home pad registry. |
| `/swarm/cell_override` | `std_msgs/String` JSON/text | reliable volatile | Operator/GCS cell override. |
| `/swarm/manual_control` | `std_msgs/String` JSON/text | best-effort volatile | Operator intent, not PX4 setpoints. |
| `/swarm/mode` | `std_msgs/String` | reliable transient-local | GCS mode state. |
| `/swarm/peer_cells` | `std_msgs/String` JSON | reliable transient-local | Peer/sector preview. |

## Field Setup Topics

| Topic | Payload | QoS | Notes |
| :--- | :--- | :--- | :--- |
| `/field/boundary_point` | `std_msgs/String` JSON/text | best-effort volatile | Polygon vertex capture. |
| `/field/boundary_close` | `std_msgs/String` | best-effort volatile | Finalize polygon boundary. |
| `/field/corner_marked` | `std_msgs/String` JSON/text | best-effort volatile | Legacy 4-corner fallback only. |
| `/field/mission_confirm` | `std_msgs/String` | best-effort volatile or reliable transient-local by bridge | Operator confirmation. |
| `/field/generate_grid` | `std_msgs/String` | reliable volatile | GCS/setup request. |
| `/field/setup_status` | `std_msgs/String` JSON/text | best-effort volatile | Setup state machine status. |
| `/field/setup_complete` | `std_msgs/String` JSON/text | best-effort transient-local | Setup completion gate. |

## QoS Policy Names

- **Latched state:** reliable or best-effort + transient-local + keep-last.
  Use for state late subscribers must see, such as home positions, mission ready,
  task status, setup complete, and avoidance status.
- **Commands:** reliable + volatile + keep-last unless command loss is acceptable.
  Use for target commands, cell overrides, and explicit GCS requests.
- **Sensors/images:** best-effort + volatile + keep-last. Use for camera, depth,
  lidar, visualization, and high-rate telemetry.
- **PX4 input/output:** best-effort + transient-local + keep-last to match the
  project PX4 bridge conventions.
- **Events:** best-effort + volatile + keep-last. Events are not state.

## Payload Migration Policy

- The Python compatibility adapters for the P0 core contract set live in
  `scout_control.avoidance.types`. They cover:
  `TargetCommand`, `AvoidanceStatus`, `SwarmDroneStatusEvent`,
  `SwarmTaskStatus`, `PadAssignment`, `FieldSetupComplete`,
  `ReturnHomeRequest`, and `MissionReadySignal`.
- Until generated ROS interfaces are available in every runtime environment,
  the production wire format remains JSON-compatible. Typed helpers must round
  trip through `std_msgs/String` JSON mirrors without dropping existing fields.
- Persisted JSON payloads must be backwards-compatible. New fields require
  defaults when older files are loaded.
- Topic JSON payloads may add optional fields. Consumers must ignore unknown
  fields and provide defaults for missing optional fields.
- Required field removals or type changes require a new version field or a new
  topic. Do not reuse a topic name for an incompatible payload.
- Keep typed ROS messages and JSON mirror topics aligned when both exist.
- Deprecation must be explicit in docs and code comments, with a migration path
  and a removal condition.

## Launch Contract

Production E2E launch files start backend/autonomy nodes and sensor bridges only.
Operator tooling must be launched explicitly from scenario `extra_terminal_commands`
or by passing `include_operator_tools:=true`. Debug/legacy/manual PX4 controllers
must remain outside production launch files.
