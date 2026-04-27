# Isaac Phase 1+2+3 E2E Preparation

Updated: 2026-04-26

## Prepared Entry Points

- Scenario:
  - `scenarios/isaac_e2e_mission.yaml`
  - Runs `ros2 launch scout_control isaac_e2e_mission.launch.py` with
    `drone_count:=1`, `altitude:=5.0`, `cell_size_m:=5.0`, and
    Isaac-safe runtime defaults from the launch file.
  - Opens extra terminals for:
    - `field_setup_tool`
    - Swarm Center
    - `scripts/isaac_phase123_checks.sh`

- Phase 3 scenario:
  - `scenarios/mapping_mission.yaml`
  - Run after Phase 2 has produced `perimeters/field_boundary.json`.
  - Stop the Phase 1+2 backend first so there is only one runtime flight owner.

- Launch:
  - `src/scout_control/launch/isaac_e2e_mission.launch.py`
  - Already starts the Phase 1 runtime owner and Phase 2 setup stack:
    `field_setup_coordinator`, `home_manager`, `field_setup_tool`
    headless bridge, `obstacle_avoidance_runtime`, `swarm_agent`,
    `swarm_coordinator`, `mission_launcher`, `gcs_bridge`.

- Live verification script:
  - `scripts/isaac_phase123_checks.sh`
  - Checks PX4 setpoint ownership, swarm_agent publisher separation,
    avoidance status, Isaac camera/depth streams, setup status, pad schema,
    boundary artifact, grid artifact, Phase 3 field-model artifacts, and
    Phase 1+2+3 unit tests.

- Runbook:
  - `docs/launch_files/isaac_phase123_e2e_test.txt`
  - Updated to match current `field_setup_tool` keys:
    `H`, `J`, `Tab`, `B`, `F`, `C`, `M`, `Q`.

## Code Fixes Needed For A Valid Phase 2 Check

- `field_setup_coordinator` now persists Phase 2 pad metadata into
  `perimeters/home_positions.json`:
  - `charging_capable`
  - `orientation_deg`
  - `service_priority`
  - `allowed_drone_classes`

- `scout_launcher.py` now accepts both old string-style
  `extra_terminal_commands` and the newer `{title, command}` scenario form.

## Manual Isaac Prerequisites

Before selecting the scenario, run manually:

1. PX4 SITL:
   `PX4_SIM_MODEL=gazebo-classic_iris ./build/px4_sitl_default/bin/px4 ...`
2. MicroXRCE:
   `MicroXRCEAgent udp4 -p 8888`
3. Isaac Sim:
   load `worlds/agro_field.usd`, load PX4 vehicle through Pegasus, run
   `Pegasus_scenarios/simulation_cam.py` once, then press Play.

The ROS2 scenario should be started only after PX4 stops printing
`Waiting for simulator to accept connection on TCP port 4560`.

## Verification Performed

```bash
python3 -m py_compile scout_launcher.py
bash -n scripts/isaac_phase123_checks.sh
source /opt/ros/jazzy/setup.bash && \
  PYTHONPATH=src/scout_control:$PYTHONPATH python3 -m pytest \
    src/scout_control/test/test_local_planner.py \
    src/scout_control/test/test_avoidance_helpers.py \
    src/scout_control/test/test_home_manager.py \
    src/scout_control/test/test_grid_from_polygon.py \
    src/scout_control/test/test_boundary_capture.py \
    src/scout_control/test/test_e2e_setup_flow.py \
    src/scout_control/test/test_lawnmower.py \
    src/scout_control/test/test_heightmap.py \
    src/scout_control/test/test_obstacle_extractor.py \
    src/scout_control/test/test_mapping_mission.py \
    src/scout_control/test/test_pad_detector.py -q
source /opt/ros/jazzy/setup.bash && colcon build --packages-select scout_control
```

Results:

- YAML parse: OK
- Launcher compile: OK
- Live-check script syntax: OK
- Phase 1+2+3 tests: `80 passed`
- `scout_control` build: OK
