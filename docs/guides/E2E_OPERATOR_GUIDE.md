# Full E2E Mission — Tilted Field: Operator Guide

End-to-end guide for running the full autonomous swarm spray mission on the `tilted_field` world.

---

## Prerequisites

- PX4-Autopilot built with `gz_x500_mono_cam_down_lidar` model
- `tilted_field.world` in `~/PX4-Autopilot/Tools/simulation/gz/worlds/`
- `scout_ws` built (`colcon build --packages-select scout_control`)

---

## Step 1 — Launch via scout_launcher

```bash
cd /home/tj/_Data/_Projekty/TJlabs/scout_ws
python3 scout_launcher.py
```

In the launcher:
1. **World:** select `tilted_field`
2. **Drone model:** select `gz_x500_mono_cam_down_lidar` (downward camera + lidar)
3. **Launch mode:** select `Swarm (2 drones)`
4. Wait for PX4 + Gazebo + MicroXRCE-DDS + QGroundControl to start (~30 s)
5. **Scenario:** select `Full E2E Mission — Tilted Field`

This opens two terminals:
- **ROS2 launch terminal** — all background nodes (field_setup_coordinator, swarm_agents, task_allocator, spray_controller, ml_interface, mission_launcher, sensor bridges)
- **field_setup_tool terminal** — setup-only UI for pad/corner marking and mission confirmation

Flight ownership boundary:
- `obstacle_avoidance_runtime` is the production flight owner. It arms, takes off and publishes PX4 setpoints.
- `field_setup_tool` is setup/debug tooling only. It reads PX4 local position and publishes setup topics; it does **not** publish PX4 setpoints or vehicle commands.
- Legacy manual flight tools are available only through explicit `legacy_*` commands and must not be started together with the production autonomy path unless you are intentionally debugging flight ownership.

---

## Step 2 — Set landing pad positions

Both drones spawn near origin. Move each drone to its designated landing pad using the production runtime/GCS controls for the active autonomy stack. Use `field_setup_tool` only to record the current position.

### Assign pad_0 (drone_0)

In the field_setup_tool terminal:
1. Drone_0 is the active drone by default (shown with `◄`)
2. Move drone_0 over the orange landing pad at Gazebo(-8, 10) = NED(10, -8)
3. Press **H** → pad_0 is assigned and saved

Expected output: `pad_0 set for drone_0: NED(-8.00, 10.00)` *(actual values reflect drone's current position)*

### Assign pad_1 (drone_1)

1. Press **Tab** → switch active drone to drone_1 (shown with `◄`)
2. Move drone_1 over the blue landing pad at Gazebo(-8, 40) = NED(40, -8)
3. Press **J** → pad_1 is assigned and saved

Expected output: `pad_1 set for drone_1: NED(-8.00, 40.00)`

> The `field_setup_coordinator` automatically advances to `MAP_FIELD` state once both pads are set.

---

## Step 3 — Map the field corners (drone_0 only)

Switch back to drone_0 (**Tab**) and move it to each of the 4 field corners using the production runtime/GCS controls.

For each corner:
1. Move drone_0 over the corner location at cruising altitude (~5 m)
2. Press **C** → corner submenu appears: `[1]NE  [2]NW  [3]SE  [4]SW`
3. Press the number for the current corner (1/2/3/4)
4. Confirm the flash message: `Corner NE marked at x=… y=…`

Mark all 4 corners in any order:
- **NE** (north-east corner of field)
- **NW** (north-west corner of field)
- **SE** (south-east corner of field)
- **SW** (south-west corner of field)

The status line shows progress: `MAP_FIELD — corners marked: 3/4 (NE, NW, SW)`

> Once all 4 corners are marked, `field_setup_coordinator` automatically:
> - Computes the field bounding box
> - Generates `perimeters/field_grid.json`
> - Sends drone_0 RTH back to pad_0
> - Advances to `READY_FOR_MISSION`

---

## Step 4 — Start the mission

Once drone_0 has landed on pad_0, the mission starts automatically.

**Alternatively**, if you want to start immediately (before drone_0 lands):
- Press **M** in field_setup_tool → confirms mission start immediately

At this point:
- `/swarm/mission_ready` is published
- `task_allocator` assigns grid cells in snake pattern to both drones
- Both drones execute their sectors autonomously (TAKEOFF → ROTATE → CRUISE → RTH)
- `spray_controller` logs each sprayed cell to `spray_log.json`
- `field_setup_tool` remains passive because it never published offboard setpoints

Status message: `Mission confirmed`

---

## Step 5 — Monitor progress

### In the ROS2 launch terminal

Progress is logged every 30 s:
```
[PROGRESS] 42/100 cells (42.0%) in 120s | rebalances=1
  drone_0: status=WORKING done=21 queued=8 current=x4_y3
  drone_1: status=WORKING done=21 queued=8 current=x14_y3
```

Mission launcher logs at 10% intervals:
```
[MISSION]  40% — 40/100 cells | rebalances=1
```

### Swarm Center (PyQt6 GCS)

For a visual overview, you can launch the **Swarm Center** GCS:
1. Open a new terminal.
2. Run: `ros2 run scout_control gcs_bridge` (if not already running via launch).
3. Run: `cd swarm_center && python3 main.py`.

The Swarm Center provides:
- Live top-down map with drone trails.
- Real-time grid cell status (visited, assigned, remaining).
- Mission progress bar and telemetry table.
- Mode selection (Mapping/Spraying/Checking).
- RTH All button for emergency return.

### Spray log

Live spray events written to `spray_log.json`:
```bash
cat spray_log.json | python3 -m json.tool
```

### ML topics (in a separate terminal)
```bash
ros2 topic echo /field/anomaly
ros2 topic echo /field/cell_health
ros2 topic echo /drone/spray_dose
```

### Lidar check (terrain following active)
```bash
ros2 topic echo /drone_0/downward_lidar/scan --once
ros2 topic echo /drone_1/downward_lidar/scan --once
```

---

## Step 6 — Mission complete

When all cells are covered:
- `task_allocator` publishes `/swarm/mission_complete`
- Both drones receive RTH requests and return to their landing pads
- `mission_launcher` logs the full summary and shuts down gracefully

```
[MISSION] 100% — 100/100 cells | rebalances=2
MISSION COMPLETE
  Total time:      245.3 s
  Cells completed: 100
  Area covered:    2500.0 m²
  Cell size:       5.0 m
Drones are returning to home pads. Shutting down.
```

Press **Q** in field_setup_tool to exit.

---

## Key controls (field_setup_tool)

| Key | Action |
|-----|--------|
| Tab | Switch active drone (drone_0 ↔ drone_1) |
| H | Assign current drone_0 position as pad_0 |
| J | Assign current drone_1 position as pad_1 |
| C | Corner marking submenu → 1=NE 2=NW 3=SE 4=SW |
| M | Confirm mission start immediately |
| Q | Quit field_setup_tool |

## Legacy/debug manual flight tools

Use these only for isolated debugging or old manual workflows. They publish PX4 setpoints/commands and are not part of the production autonomy path:

```bash
ros2 run scout_control legacy_manual_controller
ros2 run scout_control legacy_manual_commander
ros2 run scout_control legacy_field_commander
ros2 run scout_control legacy_perimeter_flight
ros2 run scout_control legacy_terrain_follower
```

The obstacle-avoidance route provider is also explicit test harness tooling:

```bash
ros2 launch scout_control obstacle_avoidance_test.launch.py
```

---

## Troubleshooting

**Drones don't arm / take off**
- Check MicroXRCE-DDS is running: `MicroXRCEAgent udp4 -p 8888`
- Check PX4 topics: `ros2 topic echo /fmu/out/vehicle_local_position_v1 --once`

**Lidar not working (runaway climbing)**
- Verify lidar bridge is running: `ros2 topic echo /drone_0/downward_lidar/scan --once`
- swarm_agent has a 20-second TAKEOFF timeout safety net

**field_setup_coordinator stuck in IDLE**
- Ensure field_setup_tool terminal is running
- Press H with drone_0 over pad_0, then J (Tab first) with drone_1 over pad_1

**task_allocator won't start mission**
- It waits up to 300 seconds for drones to publish READY
- Check: `ros2 topic echo /swarm/drone_status`

**Reset everything**
```bash
pkill -9 -f "gz"; pkill -9 -f "px4"; pkill -9 -f "gzserver"
# Then restart via scout_launcher
```

---

## File outputs

| File | Content |
|------|---------|
| `perimeters/home_positions.json` | Landing pad positions (written by field_setup_coordinator) |
| `perimeters/field_grid.json` | Grid cell map (written by field_setup_coordinator) |
| `spray_log.json` | Spray event log, one entry per cell (written by spray_controller) |
