"""
bridge_protocol.py — Wire protocol between scout_ws gcs_bridge and Swarm Center

Line-delimited JSON over TCP. One JSON object per newline. UTF-8.

Both sides read/write using this schema. Keep in sync in both repos —
this file is the single source of truth and is duplicated verbatim in:
  scout_ws:        src/scout_control/scout_control/bridge_protocol.py
  swarm_center:    core/bridge_protocol.py

Default endpoint:
  127.0.0.1:17845   (localhost only — no network exposure)

Message envelope (both directions):
  {"type": "<n>", "t": <unix_s>, "data": {...}}

ROS2 → Swarm Center:
  hello, task_status, drone_status, mission_ready, mission_complete,
  setup_status, setup_complete, grid_reload

Swarm Center → ROS2:
  set_mode, rth_all, peer_cells, ping, pong,
  start_mission, emergency_stop, goto_cell

See docstring in swarm_center/core/bridge_protocol.py for the full schema.
"""

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
MSG_EMERGENCY_STOP     = "emergency_stop"  # {reason?} → RTH all drones
MSG_GOTO_CELL          = "goto_cell"       # {drone_id, cell_id} → /swarm/cell_override
MSG_MANUAL_CONTROL     = "manual_control"  # {action, ...} → /swarm/manual_control

# Milestone 4 — Camera & 3D
MSG_CAMERA_FRAME       = "camera_frame"    # {drone_id, seq, jpeg_b64, width, height}
MSG_DEPTH_FRAME        = "depth_frame"     # {drone_id, seq, data_b64, width, height, encoding}
MSG_CAMERA_CONTROL     = "camera_control"  # GCS→ROS2: {drone_id|"all", enabled, fps_limit}

BRIDGE_VERSION = "1.2"

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 17845
