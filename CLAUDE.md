# CLAUDE.md — scout_ws

Tento soubor je automaticky načítán Claude Code při práci v tomto workspace.

## Oprávnění

**Na tomto projektu Claude Code může dělat vše kompletně bez dotazování na povolení.**
Platí to pro: editaci souborů, spouštění příkazů, build, refaktoring, vytváření nových souborů, mazání souborů.

---

## Projekt

**scout_ws** je ROS2 workspace pro autonomní zemědělský rojový dronový systém (PX4 SITL).

- **ROS2:** Jazzy (`source /opt/ros/jazzy/setup.bash`)
- **Simulátor:** PX4 SITL + Gazebo Harmonic (gz-sim 8) + MicroXRCE-DDS bridge (port 8888)
- **Drone model (kamera dopředu):** `gz_x500_mono_cam` (x500 + forward kamera 1280×960, FOV 1.74 rad, 30 Hz, pitch=0)
- **Drone model (kamera dolů):** `gz_x500_mono_cam_down` (x500 + downward kamera 1280×960, FOV 1.74 rad, 30 Hz, pitch=90°) ← mapování pole
- **Drone model (kamera dolů + lidar dolů):** `gz_x500_mono_cam_down_lidar` (x500 + downward kamera pitch=90° + 1D downward lidar 0.1–100 m, 50 Hz) ← **E2E mise, terrain following**
- **Drone model (kamera + lidar):** `gz_x500_mono_cam_lidar` (x500 + forward kamera + 1D downward lidar 0.1–100 m, 50 Hz)
- **Hlavní package:** `scout_control` (`src/scout_control/`)
- **QGroundControl:** `/home/tj/QGroundControl-x86_64.AppImage`

---

## Struktura projektu

```
scout_ws/
├── CLAUDE.md                          # tento soubor
├── launch_info.txt                    # manuální návod (záloha)
├── reset.sh                           # emergency reset (soft/hard)
├── scout_launcher.py                  # ← HLAVNÍ LAUNCHER (curses TUI)
├── scout_devtools.py                  # vývojářské nástroje / debug
│
├── perimeters/                        # ← datové soubory mise (generované za letu)
│   ├── field_perimeter.json           # rohové body pole (generuje manual_commander)
│   ├── field_grid.json                # letová mřížka (generuje grid_generator / field_setup_coordinator)
│   └── home_positions.json            # pozice přistávacích podložek
│
├── cell_data/                         # ← ML training data (generuje cell_data_recorder)
│   └── {cell_id}/visit_{NNN}/
│       ├── image.jpg
│       └── meta.json
│
├── scenarios/                         # scénáře pro scout_launcher (auto-discovery)
│   ├── camera_bridge.yaml
│   ├── camera_hud.yaml                # ← kamera feed s HUD overlay (crosshair, minimap, kompas)
│   ├── camera_view.yaml
│   ├── field_commander.yaml           # interaktivní curses grid control
│   ├── full_e2e_mission.yaml          # ← FULL E2E mise (tilted_field, manual_controller + launch)
│   ├── grid_generator.yaml            # generuje field_grid.json z perimeter.json
│   ├── grid_generator_sim.yaml        # grid_generator pro simulované scénáře
│   ├── home_manager.yaml              # správce přistávacích podložek + RTH
│   ├── lidar_bridge.yaml              # ROS bridge pro downward lidar
│   ├── manual_commander.yaml          # WSAD manuální létání + mapování perimetru (1 dron)
│   ├── ml_interface.yaml              # spustí ml_interface nod
│   ├── perimeter_flight.yaml          # autonomní perimeter survey (starý způsob)
│   ├── spray_controller.yaml          # spustí spray_controller nod
│   ├── swarm_coordinator.yaml         # spustí swarm_coordinator nod
│   ├── swarm_field_commander.yaml     # field_commander pro swarm_field world
│   ├── swarm_mission.yaml             # swarm spray mise (swarm_field, 2× swarm_agent)
│   ├── task_allocator.yaml            # spustí task_allocator nod standalone
│   └── terrain_follower.yaml          # spustí terrain_follower nod
│
├── docs/
│   └── notes.md                       # detailní technické poznámky
│
└── src/scout_control/
    ├── launch/
    │   ├── camera_bridge.launch.py    # offboard_control + camera bridge
    │   ├── camera_hud.launch.py       # camera bridge + camera_hud node
    │   ├── full_e2e_mission.launch.py # ← FULL E2E (tilted_field): setup + swarm + bridges
    │   ├── lidar_bridge.launch.py     # downward lidar bridge
    │   └── swarm_mission.launch.py    # task_allocator + swarm_agent[0] + swarm_agent[1]
    ├── worlds/
    │   └── agricultural_field.world   # 25×25 m pole (bez stromů)
    └── scout_control/
        ├── paths.py                   # ← CENTRÁLNÍ CESTY k datovým souborům
        ├── offboard_control.py        # waypoint mise (čtverec 10×10 m)
        ├── perimeter_flight.py        # autonomní perimeter survey 25×25 m
        ├── manual_commander.py        # WSAD manuální létání + mapování perimetru (1 dron)
        ├── manual_controller.py       # ← DUAL-DRONE manual controller pro E2E setup
        ├── grid_generator.py          # generuje kartézskou mřížku z perimeter.json
        ├── field_commander.py         # interaktivní curses ovládání dronu po buňkách
        ├── field_setup_coordinator.py # ← E2E: orchestrátor setupu pole (SM: IDLE→READY)
        ├── home_manager.py            # správce přistávacích podložek + RTH koordinátor
        ├── terrain_follower.py        # terrain following pomocí downward lidar
        ├── position_monitor.py        # debug: výpis aktuální pozice
        ├── camera_hud.py              # ← kamera feed s HUD (crosshair, minimap, kompas)
        ├── cell_data_recorder.py      # ← ML training data: JPG + meta.json per cell visit
        ├── ml_interface.py            # ML placeholder: /field/anomaly, /field/cell_health, /drone/spray_dose
        ├── mission_launcher.py        # ← fires /swarm/start_mission, loguje mission summary
        ├── spray_controller.py        # simulovaný postřikovač: CELL_COMPLETE → spray_log.json
        ├── task_allocator.py          # ← PURE PYTHON: sektory, snake pattern, dynamic rebalancing
        ├── swarm_coordinator.py       # ← ROS2 WRAPPER nad TaskAllocator (nahrazuje starý task_allocator ROS nod)
        └── swarm_agent.py             # ← SWARM: autonomní agent jednoho dronu (IDLE→TAKEOFF→ROTATE→CRUISE→RTH)
```

---

## Doporučený workflow — klasický (swarm_field)

```
1. scout_launcher.py
   → Vyber svět (agricultural_field nebo swarm_field)
   → Vyber model:
       • terrain following / E2E mise → gz_x500_mono_cam_down_lidar
       • mapování pole              → gz_x500_mono_cam_down
       • forward kamera             → gz_x500_mono_cam
   → Spustí PX4 SITL + Gazebo + MicroXRCE + QGC

2. Manual Commander (scenarios/manual_commander.yaml)
   → Dron vzlétne automaticky
   → Fly to home pad → press H
   → Fly to each field corner → press R
   → Press ENTER → uloží field_perimeter.json + home_positions.json

3. Grid Generator (scenarios/grid_generator.yaml)
   → Načte perimeters/field_perimeter.json
   → Uloží perimeters/field_grid.json

4. Home Manager (scenarios/home_manager.yaml)
   → Načte perimeters/home_positions.json
   → Publikuje RTH targety

5. Field Commander (scenarios/field_commander.yaml)
   → Načte perimeters/field_grid.json
   → WSAD grid navigation, R=RTH, L=land
```

---

## Doporučený workflow — Full E2E mise (tilted_field)

```
1. scout_launcher.py
   → Vyber svět: tilted_field
   → Vyber model: gz_x500_mono_cam_down_lidar
   → Spustí PX4 SITL (drone_0 na Gz(-8,10), drone_1 na Gz(-8,40))

2. Scénář: Full E2E Mission (scenarios/full_e2e_mission.yaml)
   → Spustí ros2 launch scout_control full_e2e_mission.launch.py (background nody)
   → Otevře 2 extra terminály:
       • manual_controller  (curses TUI, dual-drone)
       • camera_hud         (HUD overlay pro drone_0)

3. V manual_controller:
   a. Tab = přepnout aktivní dron (drone_0 ↔ drone_1)
   b. H   = zaznamenat aktuální pozici drone_0 jako pad_0
              → publikuje /swarm/pad_assignment + /drone_0/rth_target
   c. J   = zaznamenat aktuální pozici drone_1 jako pad_1
              → publikuje /swarm/pad_assignment + /drone_1/rth_target
   d. C + [1/2/3/4] = označit roh pole (NE/NW/SE/SW) — drone_0 pozice
              → publikuje /field/corner_marked
   e. Po všech 4 rozích field_setup_coordinator automaticky:
              • vygeneruje field_grid.json
              • odešle drone_0 RTH
              • čeká na přistání drone_0
   f. M   = potvrdit start mise (jen po přistání drone_0)
              → publikuje /field/mission_confirm
              → field_setup_coordinator publishne /swarm/mission_ready
              → manual_controller přestane publishovat offboard setpointy
              → swarm_coordinator přiděluje buňky swarm_agentům

4. Swarm agenti autonomně pokryjí pole, spray_controller loguje postřiky,
   cell_data_recorder ukládá JPG snímky per buňka.

5. Po dokončení mise: mission_launcher loguje summary, drony RTH.
```

---

## Spuštění (primárně přes scout_launcher.py)

```bash
cd /home/tj/_Data/_Projekty/TJlabs/scout_ws
python3 scout_launcher.py
```

Launcher postupně:
1. Vybere svět (world)
2. Vybere model dronu
3. Spustí PX4 SITL + Gazebo + MicroXRCE + QGC
4. Nabídne menu scénářů (loop — lze spouštět více scénářů za sebou)
5. V každém scénáři lze togglem C zapnout camera feed (rqt_image_view)

---

## Kritické konvence (PX4 SITL)

```python
# SPRÁVNĚ — topic s _v1 suffixem
'/fmu/out/vehicle_local_position_v1'

# SPRÁVNĚ — yaw jako nan (PX4 drží aktuální heading, neskáče na sever)
msg.yaw = float('nan')

# SPRÁVNĚ — velocity jako nan (0.0 způsobuje oscilace / integrator windup)
msg.velocity = [float('nan'), float('nan'), float('nan')]

# SPRÁVNĚ — ROS2 logger s f-stringem
self.get_logger().info(f'Waypoint {self._wp} reached')

# ŠPATNĚ — printf styl hodí varování
self.get_logger().info('Waypoint %d reached', self._wp)

# SPRÁVNĚ — source_system/source_component pro VehicleCommand
msg.source_system    = 1
msg.source_component = 1
msg.from_external    = True
# (PX4 ignoruje příkazy ze source 255/190 — špatný GCS default)
```

---

## paths.py — centrální cesty

Všechny nody importují cesty z `scout_control.paths`:

```python
from scout_control.paths import PERIMETER_FILE, GRID_FILE, HOME_POS_FILE, PERIMETERS_DIR, CELL_DATA_DIR
```

- `PERIMETER_FILE` → `<ws_root>/perimeters/field_perimeter.json`
- `GRID_FILE`      → `<ws_root>/perimeters/field_grid.json`
- `HOME_POS_FILE`  → `<ws_root>/perimeters/home_positions.json`
- `PERIMETERS_DIR` → `<ws_root>/perimeters/`
- `SPRAY_LOG_FILE` → `<ws_root>/spray_log.json`
- `CELL_DATA_DIR`  → `<ws_root>/cell_data/`   ← ML training data

Workspace root se detekuje automaticky hledáním `CLAUDE.md` od `__file__` nahoru.
**Nikdy nepoužívat `~/scout_ws/...` — to je špatná cesta!**

---

## QoS profily

```python
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

QOS_SUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
QOS_PUB = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
# Pro latched publish:
QOS_LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
# Pro swarm ephemeral eventy:
QOS_SWARM = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
# Pro /drone_N/rth_target (RELIABLE+LATCHED — QoS mismatch by dropped BEST_EFFORT pub):
QOS_LATCHED_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
```

---

## Waypoint logika (platí pro všechny nody)

- **Arm + offboard:** po 10 ticích (1 s od startu nodu)
- **Přepnutí waypointu:** `abs(z - target_z) < ALT_TOL` AND `3D_dist < REACH_DIST`
- **Pohyb:** Moving Virtual Setpoint — VSP se posouvá o `CRUISE_SPEED × DT` m/tick
- **Yaw:** používat `nan` pro udržení headingu; explicitní yaw pouze při ROTATE fázi

---

## FlightPhase state machine (field_commander + manual_commander)

```
IDLE → TAKEOFF → ROTATE → CRUISE → HOVER
                    ↓
                   RTH → IDLE (land)
```

- **ROTATE:** Postupný yaw VSP (`YAW_RATE = 35°/s`). Přechod do CRUISE až když `|yaw_error| < 8°` (nebo timeout 8 s).
- **CRUISE:** VSP se pohybuje v XY rovině k cíli. Z fixní na letové výšce.
- **RTH:** Klesání na `RTH_HOVER_Z = -0.5 m` rychlostí `RTH_SPEED = 0.8 m/s`, pak NAV_LAND.

Klíčové konstanty (field_commander.py):
```python
ROTATE_TICKS     = 10      # min. tiků před kontrolou yaw (1 s)
MAX_ROTATE_TICKS = 80      # bezpečnostní strop (8 s)
YAW_RATE         = math.radians(35)   # ~35°/s
YAW_TOL          = math.radians(8)    # 8° tolerance
RTH_HOVER_Z      = -0.5   # m NED
RTH_SPEED        = 0.8    # m/s sestup
```

---

## PX4 koordinátový systém (NED)

- **x** = North (sever)
- **y** = East (východ)
- **z** = Down (dolů, výška jako záporné číslo — z=-5 znamená 5 m nad zemí)

**Gazebo ENU ↔ PX4 NED konverze:**
- `Gazebo_X = NED_y` (East)
- `Gazebo_Y = NED_x` (North)

---

## Kamera

- **Model:** `x500_mono_cam` (součást PX4 SITL)
- **Gazebo topic:** `/world/<world>/model/x500_mono_cam_0/link/camera_link/sensor/camera/image`
- **ROS2 topic:** `/camera/image_raw` (via `ros_gz_image image_bridge`), nebo `/<drone_id>/camera/image_raw` (swarm)
- **Bridge launch:** `ros2 launch scout_control camera_bridge.launch.py world:=agricultural_field`

### camera_hud.py
- Zobrazuje kamera feed s HUD overlay (OpenCV okno)
- **Crosshair** (aiming reticle), **NED pozice** v rohu, **kompas rose** (yaw), **mini-mapa** (perimeter + drone)
- Subscribe: `/camera/image_raw`, `/fmu/out/vehicle_local_position_v1` (konfigurovatelné parametry)
- Klávesy: `Q`/`ESC` = quit, `R` = reload corners ze souboru
- Parametry: `show_minimap` (default True), `camera_topic`, `pos_topic`
- `Q` profil: kamera musí být `RELIABLE+VOLATILE` (ros_gz_image image_bridge výchozí)

```bash
ros2 run scout_control camera_hud
ros2 run scout_control camera_hud --ros-args \
  -p camera_topic:=/drone_0/camera/image_raw \
  -p pos_topic:=/fmu/out/vehicle_local_position_v1
```

---

## Downward Lidar

Dva modely s downward lidarem (lidar SDF je v obou identický — stejná orientace):

| Model (make target) | Kamera | Lidar | Použití |
|---|---|---|---|
| `gz_x500_mono_cam_lidar` | forward | downward | terrain_follower standalone |
| `gz_x500_mono_cam_down_lidar` | **down** | downward | **E2E mise, swarm spray** |

- **Gazebo topic (lidar):** `/world/<world>/model/<model_instance>/link/lidar_sensor_link/sensor/lidar/scan`
- **ROS2 topic:** `/downward_lidar/scan` (standalone) nebo `/<drone_id>/downward_lidar/scan` (swarm)
- **Bridge typ:** `sensor_msgs/msg/LaserScan[gz.msgs.LaserScan` ← POZOR: `Range` typ nefunguje!
- **Bridge launch:** přes launcher (Lidar Bridge scénář) — model se předá automaticky z výběru dronu
- **Orientace senzoru:** `lidar_sensor_link` má `pitch=1.57` na LINK úrovni + `roll=π` na sensor úrovni → paprsek míří dolů.
- **Airframe `gz_x500_mono_cam_down_lidar`:** `4023_gz_x500_mono_cam_down_lidar` v PX4 CMakeLists
- **Bridge delay:** 5 s TimerAction v launch souborech — Gazebo musí model spawnout před bridge připojením

### terrain_follower.py
- Udržuje konstantní výšku nad terénem pomocí lidar dat (standalone, 1 dron)
- Subscribe: `/downward_lidar/scan` — musí před spuštěním běžet lidar_bridge scénář!
- Parametr: `desired_height` (default 3.0 m)
- Korekce: `target_z = drone_z + (range - desired_height)` ← NED: kladné z = dolů
- Fallback bez lidar dat: stoupá na `-desired_height` (abs. výška nad startem)
- **Safety cap:** `MAX_ALT_M = 15.0` — dron nikdy nestoupne výše (chrání před runaway climbing)
- **Spuštění:** `ros2 run scout_control terrain_follower --ros-args -p desired_height:=3.0`

### Runaway climbing bug (opraveno)
Vzorec `target_z = drone_z + error` způsobuje nekonečné stoupání pokud lidar hlásí
konzistentně nízkou hodnotu (zaseknutý senzor, self-hit, špatný topic bridge):

**Oprava (v obou nodech):**
```python
target_z = min(target_z, 0.0)           # nikdy underground (NED: z > 0)
target_z = max(target_z, -MAX_ALT_M)    # nikdy výše než MAX_ALT_M metrů
```
`swarm_agent` navíc má **TAKEOFF timeout** (`MAX_TAKEOFF_TICKS = 200` = 20 s) — forced transition s varováním.

---

## Landing pad systém (swarm)

### home_manager.py
- Načte `perimeters/home_positions.json`
- Publikuje `/swarm/home_positions` (String JSON, latched)
- Subscribe `/swarm/rth_request` → označí pad obsazený, publishne `/<drone_id>/rth_target` (Point NED)
- Subscribe `/swarm/landed_confirmation` → uvolní pad

### home_positions.json formát
```json
{
  "home_positions": [{
    "pad_id": "pad_0",
    "drone_id": "drone_0",
    "ned": {"x": 0.0, "y": 0.0, "z": -0.5},
    "gz_pose": {"x": 0.0, "y": 0.0, "z": 0.0},
    "status": "available"
  }]
}
```

### field_perimeter.json formát (vstup pro grid_generator)
```json
{
  "altitude_m": 5.0,
  "waypoints_ned": [[x0, y0, z0], [x1, y1, z1], ...]
}
```

---

## Světy Gazebo

| World | Soubor | Popis |
|-------|--------|-------|
| `agricultural_field` | `worlds/agricultural_field.world` | 25×25 m travnaté pole |
| `swarm_field` | `~/PX4-Autopilot/Tools/simulation/gz/worlds/swarm_field.sdf` | 300×300 m + landing pad + swarm center |
| `tilted_field` | PX4 SITL worlds | 5° slope + terrain bump, 2 landing pads mimo pole |

**swarm_field.sdf:**
- Landing pad 0 (oranžový): r=0.75 m + bílé H — Gazebo(0,0) = NED(0,0) ← drone_0 spawn
- Landing pad 1 (modrý):   r=0.75 m + bílé H — Gazebo(3,0) = NED(0,3)  ← drone_1 spawn
- Swarm center: šedá budova 4×3×2.5 m na Gazebo(-6,0) = NED(0,-6)

**tilted_field (Full E2E mise):**
- Pad 0: Gazebo ENU(-8, 10) = NED(10, -8) ← drone_0 spawn
- Pad 1: Gazebo ENU(-8, 40) = NED(40, -8) ← drone_1 spawn
- 5° svah + terrain bump — terrain following nutný

---

## Field Setup Coordinator (E2E)

### field_setup_coordinator.py

Orchestrátor E2E mise — řídí setup pole přes state machine:

```
IDLE → ASSIGN_PADS → MAP_FIELD → GENERATE_GRID → WAITING_FOR_LANDING → READY_FOR_MISSION
```

| Stav | Podmínka vstupu | Akce |
|---|---|---|
| IDLE | start | čeká na pad_assignment (H a J v manual_controller) |
| ASSIGN_PADS | oba pady přijaty | uloží home_positions.json |
| MAP_FIELD | pady uloženy | čeká na 4 rohy pole (C+1/2/3/4 v manual_controller) |
| GENERATE_GRID | 4 rohy přijaty | inline grid generace → uloží field_grid.json |
| WAITING_FOR_LANDING | grid uložen | odešle drone_0 RTH, čeká na landed_confirmation |
| READY_FOR_MISSION | drone_0 přistál | čeká na M press (mission_confirm) |

**Topicy:**

| Směr | Topic | Typ |
|---|---|---|
| Subscribe | `/swarm/pad_assignment` | String JSON (z manual_controller H/J) |
| Subscribe | `/field/corner_marked` | String JSON (z manual_controller C+1234) |
| Subscribe | `/swarm/landed_confirmation` | String JSON |
| Subscribe | `/field/mission_confirm` | String JSON (z manual_controller M) |
| Publish | `/field/setup_status` | String — 1 Hz heartbeat |
| Publish | `/field/setup_complete` | String JSON (latched) |
| Publish | `/swarm/rth_request` | String JSON |
| Publish | `/swarm/mission_ready` | String JSON (latched) |

Parametry: `cell_size_m` (default 5.0 m)

```bash
ros2 run scout_control field_setup_coordinator
ros2 run scout_control field_setup_coordinator --ros-args -p cell_size_m:=3.0
```

---

## Manual Controller (dual-drone E2E)

### manual_controller.py

Rozšíření `manual_commander.py` pro dual-drone E2E setup. Curses TUI.

**Klávesová mapa:**

| Klávesa | Akce |
|---|---|
| Tab | přepnout aktivní dron (drone_0 ↔ drone_1) |
| W/S/A/D | fly N/S/W/E (NED) — aktivní dron |
| ↑↓ | altitude up/down — aktivní dron |
| H | zaznamenat drone_0 pozici jako pad_0 → `/swarm/pad_assignment` + `/drone_0/rth_target` |
| J | zaznamenat drone_1 pozici jako pad_1 → `/swarm/pad_assignment` + `/drone_1/rth_target` |
| C + 1/2/3/4 | označit roh pole NE/NW/SE/SW (drone_0) → `/field/corner_marked` |
| M | potvrdit start mise → `/field/mission_confirm` |
| L | přistát aktivní dron |
| Q | quit |

**Důležité detaily:**
- H/J guard: pad se uloží jen pokud `xy_valid=True` (EKF konvergoval) a dron není u originu
- Po M: `manual_controller` přestane publishovat offboard setpointy → swarm_agent přebírá
- `/drone_N/rth_target` publikuje s `RELIABLE+LATCHED` (QoS mismatch by zahodil BEST_EFFORT)
- `source_system=1, source_component=1` v VehicleCommand (opraveno oproti default 255/190)

```bash
ros2 run scout_control manual_controller
ros2 run scout_control manual_controller --ros-args -p altitude:=5.0
```

---

## Swarm Coordinator + Task Allocator

### Architektura (E2E)

```
field_setup_coordinator  ──/swarm/mission_ready──►  swarm_coordinator
                                                          │
                                              /drone_N/next_cell
                                                          │
swarm_coordinator  ◄──/swarm/drone_status──  swarm_agent[0]  ──►  drone_0 (PX4)
                                             swarm_agent[1]  ──►  drone_1 (PX4)
```

**task_allocator.py** — čistý Python (bez ROS2):
- Sektory, snake pattern, dynamic rebalancing, prefetch
- Instantiated a řízený přes `SwarmCoordinator`

**swarm_coordinator.py** — ROS2 wrapper nad `TaskAllocator`:
- Načte `field_grid.json`, předá `TaskAllocator`
- Vlastní všechny ROS2 publishers/subscribers
- Drivuje `TaskAllocator` tick metody přes ROS2 timery
- Subscribe drone pozice (VehicleLocalPosition) pro NFZ support
- Čeká na `/swarm/mission_ready` před startem ready-timeout (countdown nesmí startovat dřív)

| Topic | Směr | Popis |
|---|---|---|
| `/swarm/drone_status` | Sub | READY / CELL_COMPLETE ze swarm_agent |
| `/drone_N/next_cell` | Pub | cell assignment per drone (latched) |
| `/swarm/task_status` | Pub | 1 Hz progress (latched) |
| `/swarm/mission_complete` | Pub | jednou po dokončení |
| `/swarm/rth_request` | Pub | RTH per drone po misi |

Parametry: `drone_count` (default 2), `ready_timeout` (default 30 s; Full E2E: 600 s), `nfz_radius` (default 3.0 m)

```bash
ros2 run scout_control swarm_coordinator
ros2 run scout_control swarm_coordinator --ros-args -p drone_count:=2 -p ready_timeout:=600.0
```

### Snake pattern — optimalizace směru

Směr se volí automaticky podle tvaru sektoru:
- `by_cols=True`  → letí po sloupcích (sektor výšší než širší)
- `by_cols=False` → letí po řádcích (sektor širší než vyšší)

Minimalizuje počet otočení (`min(rows-1, cols-1)`).

### Prefetch — plynulý let bez zastavení

task_allocator vždy posílá **dvě buňky dopředu**: aktuální + prefetch.
swarm_agent ukládá do lokální fronty (`deque`) a přechází seamlessly pokud není změna směru.

Změna směru (`angle_diff > 45°`) → ROTATE fáze. Jinak zůstává v CRUISE.

---

## swarm_agent — FlightPhase state machine

```
IDLE ──arm──► TAKEOFF ──alt reached──► ROTATE ──settled──► CRUISE
                                          ▲                    │
                                          │◄─── turn >45° ─────┘
                                          │
                                       RTH active → RTH ──►  IDLE
                                                    (XY→home, descend, AUTO.LAND)
```

- **IDLE:** čeká na buňku; po `IDLE_RTH_TICKS=60` (6 s) self-trigger RTH (safety net)
- **TAKEOFF:** VSP klesá na `target_z = -altitude_m`; terrain following aktivní
- **ROTATE:** postupný yaw VSP (40°/s); settled = `|yaw_error| < 10°` AND `ticks ≥ 8`, nebo timeout 6 s
- **CRUISE:** VSP posouvá se k cíli; yaw průběžně aktualizován (nos vždy dopředu)
- **RTH:** VSP XY → home, pak sestup na `RTH_HOVER_Z = -1.5 m`, pak AUTO.LAND

### Landing — kritická implementace (opraveno)

**Problém:** `VEHICLE_CMD_NAV_LAND` ignorováno v offboard módu.

**Oprava:**
1. RTH dosáhne `RTH_HOVER_Z = -1.5 m` → `_landing = True`
2. Přestane publishovat offboard heartbeat (~0.5 s PX4 opustí offboard)
3. Pošle `VEHICLE_CMD_DO_SET_MODE` → AUTO.LAND

```python
# SPRÁVNĚ — AUTO.LAND v PX4 custom mode
self._send_command(
    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
    param1=1.0,   # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    param2=4.0,   # PX4_CUSTOM_MAIN_MODE_AUTO
    param3=6.0,   # PX4_CUSTOM_SUB_MODE_AUTO_LAND
)

# ŠPATNĚ — NAV_LAND se ignoruje pokud běží offboard heartbeat
self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
```

### Klíčové konstanty (swarm_agent.py)

```python
DT               = 0.1    # s — 10 Hz control loop
ARM_TICKS        = 10     # pre-arm setpoints před armingem
ALT_TOL          = 0.4    # m — altitude reached threshold
REACH_DIST       = 0.8    # m — buňka dosažena
RTH_HOVER_Z      = -1.5   # m NED — výška při triggeru AUTO.LAND
RTH_SPEED        = 0.8    # m/s — rychlost sestupu při RTH
YAW_RATE         = 40°/s  # rychlost otáčení při ROTATE
YAW_TOL          = 10°    # tolerance yaw při ROTATE
ROTATE_TICKS     = 8      # min. tiků před přechodem z ROTATE
MAX_ROTATE_TICKS = 60     # timeout ROTATE (6 s)
IDLE_RTH_TICKS   = 60     # 6 s idle → self-trigger RTH (safety net)
MAX_TAKEOFF_TICKS= 200    # 20 s timeout TAKEOFF (fallback bez lidar dat)
```

---

## Spray Controller

### spray_controller.py

- Reaguje na `CELL_COMPLETE` eventy ze všech swarm agentů (`/swarm/drone_status`)
- Na každý event publishne `/<drone_id>/spray_command` s konstantní dávkou
- Loguje každý postřik do `spray_log.json` (flush po každém eventu)
- Publisher pro každý dron se vytváří **lazy** při prvním eventu

| Směr | Topic | Typ |
|---|---|---|
| Subscribe | `/swarm/drone_status` | String JSON (CELL_COMPLETE) |
| Publish | `/{drone_id}/spray_command` | String JSON |

**Payload spray_command:**
```json
{
  "drone_id":    "drone_0",
  "cell_id":     "x2_y3",
  "dose_ml":     50.0,
  "dose_source": "constant",
  "timestamp":   "2026-04-14T10:00:00+00:00"
}
```

Parametry: `dose_ml` (default 50.0)

```bash
ros2 run scout_control spray_controller
ros2 run scout_control spray_controller --ros-args -p dose_ml:=75.0
```

### Fáze 2 — kde nahradit dummy hodnotou

`_execute_spray()` → volat ML model místo konstanty. Vstup: `cell_id`. Výstup: `dose_ml`, `dose_source: "ml_model"`.

---

## Cell Data Recorder (ML training data)

### cell_data_recorder.py

Pasivní ML-data kolektor — bez interference s letem.

- Subscribe: `/swarm/drone_status` (CELL_COMPLETE), `/drone_N/camera/image_raw`, position topics
- Na každý CELL_COMPLETE uloží: `<ws_root>/cell_data/{cell_id}/visit_{NNN}/image.jpg` + `meta.json`
- Detekuje stale kamera frame (TTL = 10 s) — varování pokud bridge nefunguje
- `meta.json`: `{timestamp_utc, drone_id, cell_id, visit, ned: {x,y,z}}`

```bash
ros2 run scout_control cell_data_recorder
```

Kamera topics (swarm): `/drone_0/camera/image_raw`, `/drone_1/camera/image_raw`

---

## Mission Launcher

### mission_launcher.py

- Čeká na `/swarm/mission_ready` → publishne `/swarm/start_mission` (latched)
- Subscribe `/swarm/task_status` → loguje progress po 10% krocích
- Subscribe `/swarm/mission_complete` → loguje summary + shutdown po 5 s
- Nespouští task_allocator ani agenty — pouze loguje a signalizuje start

| Směr | Topic | Typ |
|---|---|---|
| Subscribe | `/swarm/mission_ready` | String JSON (latched) |
| Subscribe | `/swarm/mission_complete` | String JSON |
| Subscribe | `/swarm/task_status` | String JSON (1 Hz) |
| Publish | `/swarm/start_mission` | String JSON (latched) |

```bash
ros2 run scout_control mission_launcher
```

---

## ML Interface (Fáze 1 — placeholder)

### ml_interface.py

- Publikuje dummy ML výstupy na 3 topicy (JSON String)
- Načítá `field_grid.json` automaticky — **reload při změně souboru** (mtime)
- Hodnoty per buňka jsou deterministické z `hash(cell_id)`

| Topic | QoS | Formát |
|---|---|---|
| `/field/anomaly` | volatile | `{"stamp": float, "anomalies": [{"cell_id", "type", "confidence"}]}` |
| `/field/cell_health` | latched | `{"stamp": float, "scores": {"x0_y0": 0.92, ...}}` |
| `/drone/spray_dose` | volatile | `{"stamp": float, "doses": {"drone_0": 1.2, "drone_1": 2.7}}` |

Parametry: `publish_hz` (1.0), `drone_count` (2), `anomaly_threshold` (0.35), `max_spray_dose` (3.0)

### Fáze 2 — kde nahradit dummy daty
- `DummyFieldModel.cell_health_scores()` → CNN na kamerovém obrazu
- `DummyFieldModel.anomalies()` → detektor anomálií
- `DummySprayModel.drone_doses()` → agro optimalizační model

---

## Důležité příkazy

```bash
# Prostředí
source /opt/ros/jazzy/setup.bash
source /home/tj/_Data/_Projekty/TJlabs/scout_ws/install/setup.bash

# PX4 SITL — single drone (bez lidaru, mapování)
cd ~/PX4-Autopilot
PX4_GZ_WORLD=agricultural_field make px4_sitl gz_x500_mono_cam_down

# PX4 SITL — single drone (s lidarem, terrain following)
PX4_GZ_WORLD=agricultural_field make px4_sitl gz_x500_mono_cam_down_lidar

# PX4 SITL — swarm (swarm_field, 2 drony)
# Terminal 1:
PX4_GZ_WORLD=swarm_field make px4_sitl gz_x500_mono_cam_down_lidar
# Terminal 2:
cd ~/PX4-Autopilot/build/px4_sitl_default && \
  PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=swarm_field \
  PX4_SIM_MODEL=gz_x500_mono_cam_down_lidar \
  PX4_GZ_MODEL_POSE="3,0,0,0,0,0" \
  ./bin/px4 -i 1 -s etc/init.d-posix/rcS
# Namespace: drone_0 = bare topics, drone_1 = /px4_1/fmu/...

# PX4 SITL — Full E2E mise (tilted_field, 2 drony)
# Terminal 1:
PX4_GZ_WORLD=tilted_field make px4_sitl gz_x500_mono_cam_down_lidar
# Terminal 2:
cd ~/PX4-Autopilot/build/px4_sitl_default && \
  PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=tilted_field \
  PX4_SIM_MODEL=gz_x500_mono_cam_down_lidar \
  PX4_GZ_MODEL_POSE="-8,40,0,0,0,0" \
  ./bin/px4 -i 1 -s etc/init.d-posix/rcS

# MicroXRCE bridge (jeden agent pro oba drony)
MicroXRCEAgent udp4 -p 8888

# Build
cd /home/tj/_Data/_Projekty/TJlabs/scout_ws
colcon build --packages-select scout_control

# Reset zombie procesů
pkill -9 -f "gz" ; pkill -9 -f "px4" ; pkill -9 -f "gzserver"

# Camera feed (single drone)
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# Camera HUD
ros2 run scout_control camera_hud --ros-args \
  -p camera_topic:=/drone_0/camera/image_raw \
  -p pos_topic:=/fmu/out/vehicle_local_position_v1

# Lidar debug
ros2 topic echo /downward_lidar/scan --once
ros2 topic echo /drone_0/downward_lidar/scan --once

# Spray log — sledování
cat spray_log.json | python3 -m json.tool

# Cell data debug
ls cell_data/

# ML debug — sledování topicu
ros2 topic echo /field/anomaly
ros2 topic echo /field/cell_health
ros2 topic echo /drone/spray_dose

# Field setup status (E2E)
ros2 topic echo /field/setup_status

# Full E2E mise (launch)
ros2 launch scout_control full_e2e_mission.launch.py
ros2 launch scout_control full_e2e_mission.launch.py altitude:=5.0 cell_size_m:=5.0
```

---

## Detailní poznámky

Viz [docs/notes.md](docs/notes.md) — opravené chyby, co nefunguje, architektura.
