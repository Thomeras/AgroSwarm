"""
bridge_protocol.py — Wire protocol between scout_ws gcs_bridge and Swarm Center

Line-delimited JSON over TCP. One JSON object per newline. UTF-8.

Both sides read/write using this schema. Keep in sync in both repos —
this file is the single source of truth and is duplicated verbatim in:
  scout_ws:        src/scout_control/scout_control/utils/bridge_protocol.py
  swarm_center:    swarm_center/core/bridge_protocol.py

Default endpoint:
  127.0.0.1:17845   (localhost only — no network exposure)

Message envelope (both directions):
  {"type": "<name>", "t": <unix_s>, "data": {...}}

  - "type"   mandatory. Name from MSG_* constants below.
  - "t"      mandatory. float seconds since epoch (time.time()).
  - "data"   mandatory. dict with type-specific payload.

ROS2 → Swarm Center (published by gcs_bridge):

  MSG_TASK_STATUS
    data: {mission_progress, drones:{id:{current_cell, completed, queue_remaining, status}},
           total_cells, completed_cells, cell_size_m, rebalance_count}
    From /swarm/task_status (1 Hz)

  MSG_DRONE_STATUS
    data: {drone_id, status:"READY"|"CELL_COMPLETE", cell_id?}
    From /swarm/drone_status (event-driven)
    Avoidance runtime status is typed inside ROS2 and re-serialized here as:
    {drone_id, status:"AVOIDANCE_STATUS", avoidance_status:{...}}

  MSG_MISSION_READY
    data: {drones: [drone_id, ...]}
    From /swarm/mission_ready

  MSG_MISSION_COMPLETE
    data: {cells_completed, total_time_s, area_covered_m2, cell_size_m}
    From /swarm/mission_complete

  MSG_SETUP_STATUS
    data: {text}
    From /field/setup_status (1 Hz heartbeat)

  MSG_SETUP_COMPLETE
    data: {cells, field_size, cell_size_m}
    From /field/setup_complete (latched)

  MSG_GRID_RELOAD
    data: {path}
    Fired once at startup and whenever field_grid.json is regenerated.
    Swarm Center reads the file directly (same filesystem, localhost).

  MSG_HELLO
    data: {bridge_version, ros_distro, node_name}
    First message sent after client connects. Identifies the bridge.

Swarm Center → ROS2 (received by gcs_bridge):

  MSG_SET_MODE
    data: {mode: "MAPPING"|"SPRAYING"|"CHECKING"}
    Publishes std_msgs/String on /swarm/mode (latched).

  MSG_RTH_ALL
    data: {reason?}
    Publishes /swarm/rth_request for every known drone.

  MSG_PEER_CELLS
    data: {cells: {drone_id: "x4_y2" | null}}
    Publishes /swarm/peer_cells (latched). Used by drones for
    cell-granularity awareness of the rest of the swarm.

  MSG_PING
    data: {}
    Bridge replies with MSG_PONG. For heartbeat / connection health check.

  MSG_PONG
    data: {}

--- v1.3 payloads (planned, not yet implemented) ---
  MSG_NO_GO_OVERLAY
    data: {zones: [{bbox_inflated: [xmin, ymin, xmax, ymax], confidence: float}, ...]}
    Sent after refined_grid.json is generated. Swarm Center renders no-go zones on map.

  MSG_REFINED_GRID_EVENT
    data: {path, no_go_count, caution_count, total_cells}
    Notification that refined_grid.json was written (Phase 4A output available).
"""

# Message type constants — string values are wire format

MSG_HELLO              = "hello"
MSG_TASK_STATUS        = "task_status"
MSG_DRONE_STATUS       = "drone_status"
MSG_MISSION_READY      = "mission_ready"
MSG_MISSION_COMPLETE   = "mission_complete"
MSG_SETUP_STATUS       = "setup_status"
MSG_SETUP_COMPLETE     = "setup_complete"
MSG_GRID_RELOAD        = "grid_reload"

MSG_SET_MODE           = "set_mode"
MSG_RTH_ALL            = "rth_all"
MSG_PEER_CELLS         = "peer_cells"
MSG_PING               = "ping"
MSG_PONG               = "pong"

# Milestone 3 — Mission control
MSG_START_MISSION      = "start_mission"   # {} → /field/mission_confirm
MSG_GENERATE_GRID      = "generate_grid"   # {} → /field/generate_grid
MSG_EMERGENCY_STOP     = "emergency_stop"  # {reason?} → RTH all drones
MSG_GOTO_CELL          = "goto_cell"       # {drone_id, cell_id} → /swarm/cell_override
MSG_MANUAL_CONTROL     = "manual_control"  # {action, ...} → /swarm/manual_control

# Milestone 4 — Camera & 3D
MSG_CAMERA_FRAME       = "camera_frame"    # {drone_id, seq, jpeg_b64, width, height}
MSG_DEPTH_FRAME        = "depth_frame"     # {drone_id, seq, data_b64, width, height, encoding}
MSG_CAMERA_INFO        = "camera_info"     # {drone_id, width, height, k}
MSG_CAMERA_CONTROL     = "camera_control"  # GCS→ROS2: {drone_id|"all", enabled, fps_limit}

# Protocol version — wire format stays JSON; 1.3 adds field model overlay payloads.
BRIDGE_VERSION = "1.3"
PROTOCOL_VERSION = "1.3"   # alias for forward-compatibility checks

# Default endpoint (localhost-only)
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 17845
