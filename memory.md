# Project Memory — scout_ws

## Core Architecture Facts
- **Flight Ownership:** `obstacle_avoidance_runtime` is the single source of PX4 setpoints for autonomous flight.
- **Mission Execution:** `swarm_agent` delegates to the runtime.
- **Topic Contracts:** Centralized in `scout_control.avoidance.telemetry_hub`.
- **Coordinate System:** PX4 uses NED (North-East-Down).
- **Workspace Root:** `/home/tj/_Data/_Projekty/TJlabs/scout_ws`.
- **Phase Status:** Phase 1 (Stable Runtime) is DONE.

## Hardware & Simulation Contracts
- **Isaac Sim Camera:** RGB/Depth published via `simulation_cam.py` in-session helper.
- **Gazebo Bridge:** `ros_gz_image` for camera streams.
- **Telemetry Hub:** Handles drone-prefixed namespaces (e.g., `drone_0`, `drone_1`).

## Key CLI Workflows
- **Build:** `colcon build --packages-select scout_control`
- **Tests:** `PYTHONPATH=src/scout_control pytest src/scout_control/test/`
- **Launcher:** `python3 scout_launcher.py`

## Active Decisions
- [x] Use `navigation_backend=avoidance_runtime` by default in `swarm_agent`.
- [x] Legacy nodes (`offboard_control`, `terrain_follower`) are archived and not installed.
- [x] Runtime gates navigation on EKF/Sensor health.
