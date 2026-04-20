# CLAUDE.md — scout_ws

Aktuální stav workspace k 2026-04-19.

Tento dokument je určený pro práci nad repozitářem. Popisuje reálnou strukturu
projektu, běžné entry pointy a známé odchylky proti starší dokumentaci.

## Co Ten Projekt Dělá

`scout_ws` je ROS2 Jazzy workspace pro autonomní zemědělský roj dronů nad PX4
SITL. V repu dnes existují dvě paralelní runtime větve:

- Gazebo / PX4 SITL workflow přes `scout_launcher.py`
- Isaac Sim / Pegasus workflow přes `isaac_e2e_mission.launch.py`

Nad tím nově běží samostatná desktopová GCS aplikace `swarm_center/`, která se
nepouští jako ROS2 package, ale jako zvláštní PyQt6 aplikace připojená přes
MAVLink a vlastní TCP bridge.

## Stack

- ROS2: Jazzy
- Autopilot: PX4 SITL
- Gazebo větev: Gazebo Harmonic + `ros_gz_bridge` / `ros_gz_image`
- Isaac větev: Isaac Sim / Pegasus, senzory publikuje přímo do ROS2
- DDS bridge: MicroXRCE-DDS (`udp4`, typicky port `8888`)
- GCS: `swarm_center` (`PyQt6` + `pymavlink`)

## Hlavní Vstupní Body

- Launcher simulace: `scout_launcher.py`
- ROS2 package: `src/scout_control`
- Hlavní swarm wrapper: `src/scout_control/scout_control/swarm_coordinator.py`
- Pure-Python alokátor: `src/scout_control/scout_control/task_allocator.py`
- TCP bridge do GCS: `src/scout_control/scout_control/gcs_bridge.py`
- Samostatná GCS app: `swarm_center/main.py`

## Aktuální Struktura Repo

```text
scout_ws/
├── CLAUDE.md
├── E2E_OPERATOR_GUIDE.md
├── scout_launcher.py
├── isaac_launcher.py
├── reset.sh
├── scenarios/
├── perimeters/
├── cell_data/
├── worlds/                       # Isaac / overlay assety, ne Gazebo worlds
├── swarm_center/                 # samostatná PyQt6 GCS
└── src/
    ├── px4_msgs/
    └── scout_control/
        ├── launch/
        ├── scout_control/
        ├── worlds/
        └── config/
```

## Důležité Adresáře A Soubory

### `src/scout_control/scout_control/`

Aktivní nody a moduly:

- `manual_commander.py` — single-drone ruční mapování perimetru
- `manual_controller.py` — dual-drone curses UI pro E2E setup
- `field_setup_coordinator.py` — stavový automat setupu pole
- `grid_generator.py` — generace `field_grid.json`
- `home_manager.py` — RTH / landing pad koordinace
- `swarm_agent.py` — autonomní agent jednoho dronu
- `swarm_coordinator.py` — ROS2 vrstva nad `TaskAllocator`
- `task_allocator.py` — alokace buněk, snake pattern, rebalance
- `mission_launcher.py` — sleduje ready/complete a loguje průběh mise
- `spray_controller.py` — zapisuje události postřiku do `spray_log.json`
- `cell_data_recorder.py` — ukládá snímky a metadata po buňkách
- `ml_interface.py` — placeholder ML výstupy
- `gcs_bridge.py` — TCP bridge pro `swarm_center`
- `bridge_protocol.py` — sdílený wire protokol, verze `1.2`
- `camera_hud.py` — OpenCV HUD pro živý kamerový obraz
- `terrain_follower.py` — standalone terrain following přes downward lidar
- `obstacle_detector.py`, `obstacle_avoidance_mission.py`, `obstacle_viz.py`,
  `gimbal_cam_viz.py` — oddělená obstacle-avoidance / gimbal větev

### `src/scout_control/launch/`

- `full_e2e_mission.launch.py` — hlavní Gazebo E2E mise pro `tilted_field`
- `isaac_e2e_mission.launch.py` — Isaac Sim varianta bez Gazebo bridge
- `swarm_mission.launch.py` — starší / jednodušší swarm launch
- `camera_bridge.launch.py`, `camera_hud.launch.py`, `lidar_bridge.launch.py`
- `gimbal_bridge.launch.py`
- `obstacle_avoidance_test.launch.py`

### `swarm_center/`

Samostatná aplikace mimo ROS2 package:

- `main.py` — CLI a start Qt aplikace
- `core/mavlink_manager.py` — MAVLink příjem telemetrie
- `core/ros2_bridge.py` — TCP klient na `gcs_bridge`
- `core/swarm_manager.py` — centrální stav UI
- `core/field_manager.py` — práce s `field_grid.json`
- `ui/` — Mission / Camera / 3D pohledy

Závislosti nejsou řešené přes `colcon`; používá se:

```bash
cd swarm_center
pip install -r requirements.txt
python3 main.py
```

## Reálné Workflow Dnes

## 1. Gazebo E2E Mise

Nejběžnější plná cesta je:

```bash
cd /home/tj/_Data/_Projekty/TJlabs/scout_ws
python3 scout_launcher.py
```

Typický průběh:

1. V launcheru se vybere world a model.
2. Launcher spustí PX4 SITL, Gazebo, MicroXRCE a QGC.
3. Potom se z menu pustí scénář `Full E2E Mission — Tilted Field`.
4. `full_e2e_mission.launch.py` spustí backend nody.
5. `manual_controller` běží v extra terminálu, protože potřebuje skutečné TTY.
6. `camera_hud` se může otevřít zvlášť pro `drone_0`.
7. `gcs_bridge` se spouští v launchi automaticky a `swarm_center` se může připojit.

Gazebo E2E launch dnes obsahuje:

- `field_setup_coordinator`
- `home_manager`
- `swarm_agent` pro `drone_0` a `drone_1`
- `swarm_coordinator`
- `cell_data_recorder`
- `spray_controller`
- `ml_interface`
- `mission_launcher`
- `gcs_bridge`
- bridges pro lidar a kameru obou dronů

`manual_controller` v tomto launchi záměrně není.

## 2. Isaac Sim E2E Mise

Scénář `isaac_e2e_mission.yaml` používá:

```bash
ros2 launch scout_control isaac_e2e_mission.launch.py
```

Rozdíl proti Gazebo větvi:

- PX4 SITL, MicroXRCE a Isaac Sim se mají spustit ručně předem
- launch nepouští `ros_gz_bridge`
- kamera a případná depth data mají přijít přímo z Isaac Sim
- `gcs_bridge` se i tady spouští automaticky
- default je dnes `drone_count:=1`, i když launch umí i druhý dron

V repu jsou pro Isaac i overlay assety:

- `worlds/agro_field.usd`
- `worlds/agro_field_overlay.png`
- `worlds/agro_field_overhead.png`

## 3. Swarm Center

`swarm_center` je teď integrální součást workflow, ale technicky je mimo ROS2
package. Připojuje se dvěma směry:

- MAVLink UDP na PX4 instance, default `14540 + N`
- TCP na `gcs_bridge`, default `127.0.0.1:17845`

Umí dnes:

- živou top-down mapu a grid
- progress mise
- assignment / current cell per drone
- mode přepínač přes `/swarm/mode`
- `RTH all`
- start mise přes `/field/mission_confirm`
- manual `goto_cell` override
- kamera stream přes bridge
- základní 3D view, pokud jsou dostupné UI závislosti

Bridge protokol je sdílený v:

- `src/scout_control/scout_control/bridge_protocol.py`
- `swarm_center/core/bridge_protocol.py`

Verze protokolu je aktuálně `1.2`.

## Setup Pole A Mise

Aktuální setup flow pro plnou E2E misi:

1. `manual_controller` uloží landing pady:
   `H` pro `drone_0`, `J` pro `drone_1`
2. `field_setup_coordinator` uloží `home_positions.json`
3. operátor označí 4 rohy pole přes `C` + `1/2/3/4`
4. `field_setup_coordinator` vygeneruje `field_grid.json`
5. `drone_0` dostane RTH
6. po potvrzení mise se publikuje `/swarm/mission_ready`
7. `swarm_coordinator` spustí countdown a čeká na READY od dronů
8. `TaskAllocator` rozdělí grid a swarm přebere řízení

`swarm_coordinator` je dnes jediná aktivní ROS2 vrstva pro alokaci; samotný
`task_allocator.py` je interní pure-Python modul.

## Důležité Runtime Konvence

### Cesty

Používej `scout_control.paths`, ne ručně zadrátované cesty.

Exportované konstanty:

- `WS_ROOT`
- `PERIMETERS_DIR`
- `PERIMETER_FILE`
- `GRID_FILE`
- `HOME_POS_FILE`
- `SPRAY_LOG_FILE`
- `CELL_DATA_DIR`

Workspace root se hledá podle přítomnosti `CLAUDE.md` při průchodu nahoru od
modulu. Fallback stále existuje na `~/scout_ws`, ale ten je jen kvůli
kompatibilitě se starším layoutem.

### PX4 / NED

- PX4 používá NED
- `z` je down, tedy výška je záporné číslo
- pro držení headingu se používá `yaw = nan`
- topicy pro PX4 používat se suffixem `_v1`, např.:
  `/fmu/out/vehicle_local_position_v1`

### QoS

Projekt hodně spoléhá na přesný match QoS. Obzvlášť citlivé jsou:

- latched / reliable command topics
- `/swarm/task_status`
- `/field/setup_complete`
- `/swarm/mission_ready`
- obrazová data z bridge

Při úpravách bridge nebo coordinatoru vždy nejdřív ověřit QoS na obou stranách.

## Data, Která Mise Produkuje

- `perimeters/field_perimeter.json`
- `perimeters/field_grid.json`
- `perimeters/home_positions.json`
- `spray_log.json`
- `cell_data/<cell_id>/...`

`cell_data` už v workspace obsahuje historická data, není to prázdná složka.

## Scenario Soubory

`scout_launcher.py` načítá scénáře auto-discovery z `scenarios/*.yaml`.

Aktuálně tam jsou tři typy scénářů:

- aktivní a používané: `full_e2e_mission`, `isaac_e2e_mission`, `gcs_bridge`,
  `camera_*`, `lidar_bridge`, `manual_commander`, `home_manager`,
  `swarm_coordinator`, `terrain_follower`
- specializované: `obstacle_avoidance_test`, `obstacle_detector`,
  `gimbal_bridge`
- historické nebo podezřelé: `task_allocator.yaml`, část starších standalone flow

Pozor: samotný `task_allocator` dnes není registrovaný jako `console_script` v
`src/scout_control/setup.py`. Pokud někdo spustí scénář `task_allocator.yaml`,
pravděpodobně selže. Pro reálný swarm flow se má používat `swarm_coordinator`.

## Build A Spouštění

ROS2 package:

```bash
cd /home/tj/_Data/_Projekty/TJlabs/scout_ws
colcon build --packages-select scout_control
source install/setup.bash
```

Launcher:

```bash
python3 scout_launcher.py
```

Swarm Center:

```bash
cd swarm_center
pip install -r requirements.txt
python3 main.py
```

## Světy A Assety

V repo jsou dnes dva různé typy world assetů:

- `src/scout_control/worlds/*.world` — ROS2 package assety, např.
  `agricultural_field.world`, `obstacle_course.world`
- `worlds/` v rootu — Isaac / overlay soubory, ne Gazebo `.world`

`scout_launcher.py` čte seznam worldů z `~/PX4-Autopilot/Tools/simulation/gz/worlds`.
Takže existence world souboru v repu sama o sobě nestačí; často musí být
soubor i v PX4 worlds adresáři.

## Známé Rozchody A Rizika

- `CLAUDE.md` byl předtím výrazně zastaralý; staré reference na
  `scout_devtools.py` a některé workflow už neplatí.
- `swarm_center` je už reálná součást systému, ne jen budoucí milestone.
- `gcs_bridge` je součást plného launch flow, nejen volitelný doplněk.
- `task_allocator.yaml` neodpovídá registraci v `setup.py`.
- některé scénáře nebo popisy stále používají historickou cestu `~/scout_ws`
  místo skutečné workspace cesty
- v worktree jsou necommitnuté změny i nové soubory kolem Isaac/GCS; při úpravách
  nevracet cizí změny bez ověření záměru

## Když Budeš V Tomto Repo Něco Měnit

- nejdřív ověř skutečný stav v `scenarios/`, `setup.py` a launch souborech
- nepředpokládej, že YAML scénář znamená funkční entry point
- změny bridge protokolu dělej vždy synchronně v `scout_control` i `swarm_center`
- u UI / GCS změn kontroluj i dopad na `gcs_bridge`
- u swarm logiky kontroluj návaznost:
  `manual_controller` → `field_setup_coordinator` → `swarm_coordinator` →
  `TaskAllocator` → `swarm_agent`
