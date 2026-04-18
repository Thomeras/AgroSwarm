# Technické poznámky — scout_ws

Detailní zápisky z vývoje. Pro přehled struktury viz root [CLAUDE.md](../CLAUDE.md).

---

## Prostředí

| Komponenta | Verze / detail |
|---|---|
| ROS2 | Jazzy |
| Gazebo | Harmonic / gz-sim 8.10.0 |
| PX4 SITL | nová Gazebo (`gz_` prefix), ne `gazebo-classic` |
| QGroundControl | `/home/tj/QGroundControl-x86_64.AppImage` |
| ros_gz_bridge | 1.0.18 |
| ros_gz_image | 1.0.18 |

---

## Architektura pohybu — Moving Virtual Setpoint (VSP) ✓

Funguje spolehlivě. VSP se posouvá kroky `CRUISE_SPEED × DT` k cílovému waypointu.
PX4 dostává setpoint blízko skutečné polohy → malý position error → tilt max ~5°.

```python
CRUISE_SPEED = 1.0   # m/s
DT           = 0.1   # s (10 Hz)
step = CRUISE_SPEED * DT  # = 0.1 m/tick

# VSP krok (volat v publish callbacku):
dx = target_x - vsp[0]; dy = target_y - vsp[1]; dz = target_z - vsp[2]
d  = math.sqrt(dx*dx + dy*dy + dz*dz)
if d > step:
    vsp[0] += (dx/d)*step; vsp[1] += (dy/d)*step; vsp[2] += (dz/d)*step
else:
    vsp = [target_x, target_y, target_z]

msg.position = vsp
msg.velocity = [nan, nan, nan]   # POVINNĚ nan, ne 0.0
```

## Co NEFUNGUJE (nezkoušet znovu)

| Přístup | Proč nefunguje |
|---|---|
| `velocity=True, position=False` | Integrator windup na zemi → flip 180° při vzletu |
| `position + velocity feedforward` | PX4 přičítá position controller output → 35°+ tilt |
| Čistý static position setpoint (bez VSP) | PX4 agresivně akceleruje → přestřelí → osciluje |

---

## Opravené chyby

| Chyba | Příčina | Fix |
|---|---|---|
| Dron „kinknul" | `yaw=0.0` přinutilo rotaci na sever | `yaw=float('nan')` |
| Předčasné přepnutí WP | `REACH_DIST=1.0 m` příliš velký | `ALT_TOL` guard + `REACH_DIST=0.4 m` |
| Žádná pozice dat | subscriber QoS VOLATILE ≠ PX4 TRANSIENT_LOCAL | `QOS_SUB` s `TRANSIENT_LOCAL` |
| Žádná pozice dat | špatný topic bez `_v1` suffixu | `/fmu/out/vehicle_local_position_v1` |
| Agresivní oscilace | subscriber nefungoval → `_x/_y/_z=0` stále | viz QoS fix |
| Build error logy | printf styl v ROS2 loggeru | všude f-string |
| Dron zaseknutý na posledním WP | podmínka `_wp < len-1` vynechala poslední wp | `_landing` flag |
| `rqt_image_view: command not found` | není v PATH, je jako ros2 pkg | `ros2 run rqt_image_view rqt_image_view` |
| gnome-terminal not found | není nainstalován | launcher auto-detekuje xterm/konsole/... |
| PX4 „already running" | stará instance z předchozí session | `_kill_stale()` před launch (pkill -9) |
| Ready timeout příliš brzy (Bug 1) | `_ready_deadline` nastavena při init `TaskAllocator`, před `mission_ready` | `start_ready_timeout()` metoda, timer startuje až po `/swarm/mission_ready` |
| První buňka přeskočena 18/20 (Bug 2) | Subscriber `depth=1` na `/drone_N/next_cell` — primary cell zahozena před zpracováním callbacku | `depth=10` + queue-first v TAKEOFF, aktivace při přechodu TAKEOFF→ROTATE |
| QoS mismatch `/swarm/rth_request` (Bug 3) | `field_setup_coordinator` pub BEST_EFFORT, `swarm_agent` sub RELIABLE | `field_setup_coordinator.QOS_VOL.reliability = RELIABLE` |
| Camera recorder `img=NO` (Bug 4) | Sub na `/camera/image_raw`, bridge outputuje `/drone_N/camera/image_raw` | Per-drone camera subscriber + 10s TTL check s WARN |
| Pad uložen na (0,0) místo skutečné polohy (Bug 6) | `pos_valid=True` i bez `xy_valid` z PX4 EKF | Kontrola `msg.xy_valid`, blokování H/J dokud EKF není ready, `[xy OK]` v UI |

---

## Known Limitations (Fáze 1)

### Obstacle Avoidance — NENÍ implementováno (Bug 7)

`swarm_agent.py` řídí výšku fixní `altitude_m` přes NED position setpoint bez obstacle avoidance.
`terrain_follower.py` existuje jako standalone node ale **není integrován** do `swarm_agent`.

**Kontrola světa:** `swarm_field.sdf` neobsahuje žádný model překážky (ověřeno — žádný `<model name="obstacle*">`).
Existující modely (landing pads, swarm_center) mají `<collision>` elementy správně.

**Plán:** Krok 19 — `obstacle_detector.py` (samostatná implementační fáze).

**Workflow pro přidání překážky do světa:**
```xml
<!-- Přidat do swarm_field.sdf jako nový <model> -->
<model name="obstacle_box">
  <static>true</static>
  <pose>5 5 1 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry><box><size>1 1 2</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>1 1 2</size></box></geometry>
      <material><ambient>1 0 0 1</ambient></material>
    </visual>
  </link>
</model>
```
Bez `<collision>` je model pouze vizuální — dron jím proletí, PX4 ani ROS2 kolizi nedetekují.

---

## ROS2 Topics

| Topic | Směr | Zpráva |
|---|---|---|
| `/fmu/in/offboard_control_mode` | → PX4 | `OffboardControlMode` |
| `/fmu/in/trajectory_setpoint` | → PX4 | `TrajectorySetpoint` |
| `/fmu/in/vehicle_command` | → PX4 | `VehicleCommand` |
| `/fmu/out/vehicle_local_position_v1` | ← PX4 | `VehicleLocalPosition` |
| `/fmu/out/vehicle_global_position` | ← PX4 | `VehicleGlobalPosition` (lat/lon/alt) |
| `/fmu/out/vehicle_attitude` | ← PX4 | `VehicleAttitude` (quaternion) |
| `/camera/image_raw` | ← Gz bridge | `sensor_msgs/Image` |
| `/field/perimeter` | pub | `Float32MultiArray` [x0,y0, x1,y1, ...] |
| `/field/grid` | pub | `nav_msgs/OccupancyGrid` |
| `/field/grid_state` | pub | `std_msgs/String` (JSON) |

---

## Kamera — integrace

- **Model:** `x500_mono_cam` — 1280×960 px, FOV 1.74 rad, 30 Hz
- **SDF:** `/home/tj/PX4-Autopilot/Tools/simulation/gz/models/x500_mono_cam/model.sdf`
- **Gz topic:** `/world/<world>/model/x500_mono_cam_0/link/camera_link/sensor/camera/image`
  - `_0` suffix přidává gz-sim automaticky při spawnu
- **Bridge:** `ros_gz_image image_bridge` → `/camera/image_raw`
- **Launch:** `ros2 launch scout_control camera_bridge.launch.py world:=agricultural_field`

---

## Agricultural Field World

- **Soubor:** `src/scout_control/worlds/agricultural_field.world`
- **Symlink:** `~/PX4-Autopilot/Tools/simulation/gz/worlds/agricultural_field.sdf`
- **Spuštění:** `PX4_GZ_WORLD=agricultural_field make px4_sitl gz_x500_mono_cam`
- **Obsah:** 25×25 m zemědělská půda + tráva (300×300 m), bez stromů, slunce + obloha

---

## Nody — přehled

### offboard_control.py
- Čtverec 10×10 m ve výšce 2 m (NED z=-2)
- Arm po 10 ticích, waypoint přepnutí: ALT_TOL + REACH_DIST

### perimeter_flight.py
- Perimeter survey 25×25 m ve výšce 5 m (NED z=-5), rychlost 2 m/s
- Výstup: `~/scout_ws/field_perimeter.json` (každé 2s local NED + GPS)
- Topic: `/field/perimeter` (Float32MultiArray) po přistání

### grid_generator.py
- Single-shot node: načte `field_perimeter.json` → vygeneruje mřížku → uloží `field_grid.json`
- Buňky: 1×1 m (konfigurovatelné `--ros-args -p cell_size:=0.5`)
- ID formát: `"x{col}_y{row}"` kde col=NED x/North, row=NED y/East
- Publikuje `/field/grid` (nav_msgs/OccupancyGrid, RELIABLE + TRANSIENT_LOCAL)

### field_commander.py
- Curses TUI: interaktivní grid s dronovou kontrolou po buňkách
- Threading: curses na main threadu, rclpy.spin na daemon threadu
- VSP krok: CRUISE_SPEED(2 m/s) × DT(0.1s) = 0.2 m/tick
- Výstup po ukončení: uloží aktualizovaný `field_grid.json` (visited/hovering status)
- Klávesy: šipky = pohyb kurzoru, ENTER = přesuň dron, Q = quit, L = land+quit

### scout_launcher.py
- Curses TUI launcher — spouští PX4+Gazebo, DDS, QGC, pak loop scénářů
- Auto-detekce terminálu: gnome-terminal → xterm → konsole → xfce4-terminal
- Scénáře: načítá `.yaml` soubory ze `scenarios/` dynamicky
- Loop: po spuštění scénáře nabídne další bez restartu systémů

---

## scout_launcher — formát scénáře (YAML)

```yaml
name: "Název scénáře"
description: "Popis co to dělá"
ros2_command: "ros2 run scout_control my_node"
# nebo: ros2 launch scout_control my_launch.py
```

---

## TODO / nápady

- [ ] Přidat VehicleStatus subscriber pro detekci přistání
- [ ] Bezpečnostní timeout — pokud wp není dosažen za N sekund, přistát
- [ ] Camera subscriber node pro zpracování obrazu (detekce, SLAM...)
- [ ] camera_info topic přes parameter_bridge (pro kalibraci)
- [ ] Spawn dron na konkrétní souřadnice (SW roh pole) přes SDF pose

---

*Aktualizováno: 2026-04-12*
