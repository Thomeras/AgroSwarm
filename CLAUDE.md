# CLAUDE.md — scout_ws

Aktualni stav workspace k 2026-04-28. Phase 1–5 dokončeny; historicky node
audit je uzavren a smazan.

Tento dokument je urceny pro praci nad timto repozitarem. Popisuje realnou
strukturu projektu, aktualni workflow a dulezite rozdily mezi produkcni cestou,
test harnessy a historickou dokumentaci.

## Co Ten Projekt Dela

`scout_ws` je ROS2 Jazzy workspace pro autonomni zemedelsky roj dronu nad PX4
SITL. Dnes v repu soubezne existuji ctyri prakticky dulezite vetve:

- Gazebo / PX4 SITL full E2E workflow pres `scout_launcher.py` a
  `full_e2e_mission.launch.py`
- Isaac Sim / Pegasus E2E workflow pres `isaac_e2e_mission.launch.py` a
  in-session helper `Pegasus_scenarios/simulation_cam.py`
- samostatna desktopova GCS aplikace `swarm_center/`, ktera bezi mimo ROS2
  package a pripojuje se pres MAVLink + vlastni TCP bridge
- nova obstacle-avoidance runtime vetev kolem
  `src/scout_control/scout_control/obstacle_avoidance_runtime.py` a modulu v
  `src/scout_control/scout_control/avoidance/`

Dulezite: pro avoidance-enabled navigation path je flight-control owner
`obstacle_avoidance_runtime`. `swarm_agent` je v tomto rezimu mission
executor/route provider a neposila PX4 flight setpointy.
`swarm_agent` ma mit default `navigation_backend=avoidance_runtime`.
Kompatibilni `navigation_backend=direct` muze byt stale dostupny jako
prechodovy fallback.

Node-audit cleanup k 2026-04-28 doplnil runtime hranice
`FlightPhaseMachine`, `PX4PublisherAdapter` a `RosIOAdapter`. Topic kontrakty
maji lidsky zdroj pravdy v `docs/topic_contract.md`; kodovy zdroj pravdy pro
nazvy topicu zustava `TelemetryHub`.

## Stack

- ROS2: Jazzy
- Autopilot: PX4 SITL
- Gazebo vetev: Gazebo Harmonic + `ros_gz_bridge` / `ros_gz_image`
- Isaac vetev: Isaac Sim / Pegasus, senzory publikuji primo do ROS2
- DDS bridge: MicroXRCE-DDS (`udp4`, typicky port `8888`)
- GCS: `swarm_center` (`PyQt6` + `pymavlink`)
- Computer vision / bridge utility: `opencv-python`, `cv_bridge`

## Repo Map

```text
scout_ws/
├── CLAUDE.md
├── codex.md
├── docs/
│   ├── guides/
│   ├── internal/
│   ├── launch_files/             # operatorni runbooky (isaac_phase123_e2e_test.txt atd.)
│   └── plans/
├── perimeters/                   # perimeter, grid, home JSON data + field_model/
│   └── field_model/              # Phase 3 vystup: heightmap, obstacles, manifest
├── cell_data/                    # historicka data po navstivenych bunkach
├── logs/
│   └── avoidance_logs/
├── scout_launcher.py
├── isaac_launcher.py
├── reset.sh
├── scenarios/
├── worlds/                       # Isaac / overlay assety
├── Pegasus_scenarios/
├── swarm_center/                 # samostatna PyQt6 GCS
└── src/
    ├── px4_msgs/
    └── scout_control/
        ├── launch/
        ├── scout_control/
        │   ├── avoidance/
        │   ├── mapping/          # Phase 3: Heightmap2D, ObstacleExtractor, FieldModelBuilder
        │   ├── missions/         # Phase 3: MappingMission node
        │   ├── vision/           # pomocne vizualni moduly
        │   └── utils/
        ├── test/
        ├── worlds/
        └── config/
```

## Klicove Vstupni Body

- hlavni terminal launcher: `scout_launcher.py`
- ROS2 package: `src/scout_control`
- Gazebo full E2E launch: `src/scout_control/launch/full_e2e_mission.launch.py`
- Isaac full E2E launch: `src/scout_control/launch/isaac_e2e_mission.launch.py`
- Isaac Phase 1+2+3 E2E protokol: `docs/launch_files/isaac_phase123_e2e_test.txt`
- Phase 3 mapping launch: `src/scout_control/launch/mapping_mission.launch.py`
- obstacle test launch:
  `src/scout_control/launch/obstacle_avoidance_test.launch.py`
- swarm allocator wrapper:
  `src/scout_control/scout_control/core/swarm_coordinator.py`
- interni allocator:
  `src/scout_control/scout_control/task_allocator.py`
- GCS TCP bridge:
  `src/scout_control/scout_control/gcs_bridge.py`
- samostatna GCS app: `swarm_center/main.py`

## Aktivni Moduly A Jejich Role

### Production Core

Tyto komponenty jsou dnes nejbliz aktualni produkcni / operatorni ceste:

- `field_setup_coordinator.py`
- `grid_generator.py`
- `home_manager.py`
- `swarm_agent.py` (Default: `navigation_backend=avoidance_runtime`)
- `swarm_coordinator.py`
- `mission_launcher.py`
- `gcs_bridge.py`
- `spray_controller.py`
- `cell_data_recorder.py`
- `ml_interface.py`

### Mapping Pipeline (Phase 3 — k 2026-04-27 funkční)

- `mapping/field_model_builder.py` — ROS2 node; akumuluje depth frames do
  `Heightmap2D`, na `/swarm/mapping_complete` persistuje do `perimeters/field_model/`
- `mapping/heightmap.py` — Heightmap2D: 2.5D mřížka, min NED-z per cell
- `mapping/obstacle_extractor.py` — klasifikace překážek z point cloudu
- `missions/mapping_mission.py` — generuje lawnmower trasu z `field_boundary.json`,
  posílá waypoints do runtime, hlásí progress na `/swarm/mapping_progress`
- `vision/precision_landing.py` — advisory node pro detekci přistávacího padu
  (ArUco marker), publikuje offset v `RETURN_HOME` fázi.

Klíčová gotcha depth→heightmap:
- `depth_projector.project_to_world_points()` filtruje přes collision band —
  terrain body na world_z≈0 jsou vyhozeny. Pro heightmap vždy volat
  `depth_to_body_points()` + ruční world projekce (opraveno 2026-04-27).

### Runtime-Centric Obstacle Avoidance Branch (Phase 1 Finalized)

Nova avoidance architektura je stabilizovana jako cilovy flight-control model:

- `obstacle_avoidance_runtime.py`
  - **Autoritativni Flight Owner**
  - Vlastni obstacle detection pipeline
  - Lokalni mapu, scan / replanning
  - PX4 offboard setpoint publishing
- `avoidance/telemetry_hub.py`
  - **Centralni Topic Registry** (Single source of truth pro vsechny ROS2 topicy)
- `avoidance/flight_phase_machine.py`
  - phase state, phase ticks a transition metadata pro runtime orchestrator
- `avoidance/px4_publisher_adapter.py`
  - jedina pomocna hranice pro runtime PX4 input publishing
- `avoidance/ros_io_adapter.py`
  - runtime status/event/bool ROS vystupy a JSON/typed status publikace
- `avoidance/depth_projector.py`
  - Depth frame -> body/world point batches (unified path)
- `avoidance/local_mapper.py`
  - Rolling obstacle memory a clearance summary
- `avoidance/local_planner.py`
  - Rejekce nevalidnich map, detour / blocked rozhodovani
- `avoidance/scan_manager.py`
  - 360 scan orchestrace a scan body points
- `avoidance/peer_tracks.py`
  - Safety zony ostatnich dronu
- `avoidance/types.py`
  - Sdilene datove typy, readiness parsery, payload normalizace a typed
    JSON-compatible adaptery pro P0 core kontrakty

Soucasny conceptual split:

- `obstacle_avoidance_runtime` je cilovy flight-control owner pro
  avoidance-enabled misi
- `obstacle_avoidance_mission.py` je test harness / route provider, ne obecny
  produkcni navigator
- `obstacle_detector.py` zustava izolovany debug / srovnavaci node, ne hlavni
  produkcni obstacle controller

### Support, Manual A Debug Nody

- `manual_controller.py`
- `manual_commander.py`
- `field_commander.py`
- `position_monitor.py`
- `offboard_control.py`
- `camera_hud.py`
- `terrain_follower.py`
- `obstacle_detector.py`
- `obstacle_avoidance_mission.py`
- `obstacle_viz.py`
- `gimbal_cam_viz.py`
- `scan_cloud_viz.py`
- `perimeter_flight.py`

Pozor: `task_allocator.py` je interni pure-Python modul. Neni registrovany jako
`console_script` v `src/scout_control/setup.py`, takze standalone scenar
`task_allocator.yaml` je podezrely a pravdepodobne nespustitelny bez dalsich
uprav.

## Launch A Scenario Prehled

### `src/scout_control/launch/`

- `full_e2e_mission.launch.py`
  - hlavni Gazebo swarm mise pro `tilted_field`
  - pousti `field_setup_coordinator`, `home_manager`, dva `swarm_agent`,
    `swarm_coordinator`, `cell_data_recorder`, `spray_controller`,
    `ml_interface`, `mission_launcher`, `gcs_bridge` a bridge pro kamery a
    lidary
  - `manual_controller` zamerne nepousti, protoze potrebuje skutecne TTY
- `isaac_e2e_mission.launch.py`
  - Isaac / Pegasus varianta bez `ros_gz_bridge`
  - pousti headless `manual_controller` s `ui:=False`
  - defaultne `drone_count:=1`
  - predava `camera_topic_template` a `depth_topic_template` do `gcs_bridge`
- `mapping_mission.launch.py`
  - Phase 3 mapping pipeline: `obstacle_avoidance_runtime` + `field_model_builder`
    + `mapping_mission`
  - parametry: `altitude_m`, `line_spacing_m`, `side_overlap_pct`, `cruise_speed`
  - spousti se az po ukonceni Phase 1+2 backendu (samostatny terminal)
- `obstacle_avoidance_test.launch.py`
  - pousti `obstacle_avoidance_runtime`, `obstacle_avoidance_mission`,
    `obstacle_viz`
  - bridgeuje Gazebo kameru pres `ros_gz_image`
  - `gimbal_cam_viz` se pousti separatne

### `scenarios/*.yaml`

`scout_launcher.py` scenare autodiscoveruje z `scenarios/*.yaml`.

Prakticky dulezite scenare:

- `full_e2e_mission.yaml`
- `isaac_e2e_mission.yaml`
- `obstacle_avoidance_test.yaml`
- `camera_*`, `lidar_bridge`, `gcs_bridge`, `manual_commander`,
  `home_manager`, `swarm_coordinator`, `terrain_follower`

Historicky / rizikovy detail:

- nektere scenario YAML soubory stale referencuji `~/scout_ws/install/setup.bash`
  misto skutecneho workspace path
- `scout_launcher.py` ma `WS_DIR` natvrdo na
  `/home/tj/_Data/_Projekty/TJlabs/scout_ws`

## Realtime Workflow Dnes

## 1. Gazebo Full E2E Mise

Nejbeznejsi operatorni cesta:

```bash
cd /home/tj/_Data/_Projekty/TJlabs/scout_ws
python3 scout_launcher.py
```

Typicky prubeh:

1. V launcheru se vybere world a model.
2. Launcher spusti PX4 SITL, Gazebo, MicroXRCE a QGroundControl.
3. Potom se spusti scenar `Full E2E Mission — Tilted Field`.
4. `full_e2e_mission.launch.py` rozbehne backend nody.
5. `manual_controller` bezi v extra terminalu kvuli curses UI.
6. `gcs_bridge` se spousti automaticky a `swarm_center` se muze pripojit.

V teto ceste ma byt avoidance runtime path autoritativni pro flight execution:

- `swarm_agent` drzi mission stav, frontu targetu a swarm reporting
- `swarm_agent` posila high-level commandy (`goto`, `return_home`) do runtime
- `swarm_agent` odvozuje `CELL_COMPLETE` z runtime pole
  `last_completed_target_id` (ne z vlastniho direct flight loopu)
- `obstacle_avoidance_runtime` publikuje PX4 setpointy a rozhoduje avoidance
  execution flow

Poznamka: nektere launch/config kombinace mohou stale dovolit direct fallback
(`navigation_backend=direct`) kvuli kompatibilite. To je docasne.

## 2. Isaac Sim / Pegasus E2E Mise

Pouziva se:

```bash
ros2 launch scout_control isaac_e2e_mission.launch.py
```

Predpoklady:

- PX4 SITL a MicroXRCE jsou spustene rucne predem
- Isaac Sim je spusteny rucne v ROS2-ready envu
- world a dron se nactou rucne v bezici Isaac session

Overeny camera/depth workflow k 2026-04-22:

1. Spustit Isaac Sim.
2. Rucne otevrit world a nahrat dron pres Pegasus UI.
3. Az pote ve `Window > Script Editor` pustit:

```python
exec(open("/home/tj/_Data/_Projekty/TJlabs/scout_ws/Pegasus_scenarios/simulation_cam.py").read())
```

Tento skript:

- nevytvari novou Isaac instanci
- nenacita world
- nespawnuje dron
- jen najde existujici kamerovy prim a pripoji ROS2 publishery

Ocekavane topicy:

- `/drone_0/camera/image_raw`
- `/drone_0/depth/image_raw`

Dulezite:

- pokud se `simulation_cam.py` spusti dvakrat v jedne Isaac session, vzniknou
  duplicitni publishery
- `camera_info` neni v tomto workflow garantovany
- route/mission command `altitude_m` je kladna vyska nad zemi, ne PX4 NED `z`
  - spravne: `altitude_m: 5.0`
  - runtime pak vyrobi PX4 setpoint `z=-5.0`
  - spatne: `altitude_m: -5.0` -> runtime vyrobi `z=+5.0`
    a dron se bude snazit jit dolu do zeme misto vzletu

## 3. Phase 3 — Mapping Mission

Spousti se az po ukonceni Phase 1+2 backendu (Ctrl+C v terminalu Phase 1+2).
PX4, MicroXRCE a Isaac nechat bezet.

```bash
ros2 launch scout_control mapping_mission.launch.py \
  drone_count:=1 \
  altitude_m:=8.0 \
  line_spacing_m:=4.0 \
  side_overlap_pct:=30.0 \
  cruise_speed:=2.0
```

Sledovani:

```bash
ros2 topic echo /swarm/mapping_progress
```

Vystup v `perimeters/field_model/` — viz sekce Data nizse.

Dulezite: `field_model_builder` potrebuje tect depth z `/drone_0/depth/image_raw`
a pozici z `/drone_0/.../vehicle_local_position_v1`. Obe musi tect i v Phase 3.

## 4. Obstacle Avoidance Test Harness

Pouziva se:

```bash
ros2 launch scout_control obstacle_avoidance_test.launch.py
```

Tento test dnes oddeluje role takto:

- `obstacle_avoidance_runtime` ridi dron a vyhodnocuje lokalni prekazky
- `obstacle_avoidance_mission` jen posila high-level `goto` / `return_home`
  commandy na `/{drone_ns}/avoidance/target_cmd`
- `obstacle_viz` je jen vizualizace

Pri manualnim JSON testu pouzij:

```bash
ros2 topic pub --once /drone_0/avoidance/target_cmd_json std_msgs/String \
  '{data: "{\"target_id\":\"test1\",\"command\":\"goto\",\"target_ned\":[3.0,0.0],\"altitude_m\":5.0}"}' \
  --qos-durability transient_local \
  --qos-reliability best_effort
```

Logy bezhu jdou do:

- `logs/avoidance_logs/*.jsonl`

## Swarm Center

`swarm_center/` je samostatna aplikace mimo ROS2 package.

Spusteni:

```bash
cd swarm_center
pip install -r requirements.txt
python3 main.py
```

Pripojuje se:

- MAVLink UDP na PX4 instance, default `14540 + N`
- TCP na `gcs_bridge`, default `127.0.0.1:17845`

Umi dnes:

- top-down mapu a grid s overlay vrstvami (no-go zóny, překážky, terrain heatmap)
- sector preview před startem mise (z task_status před mission_ready)
- mission progress a per-drone stav
- assignment / current cell per drone
- avoidance panel: per-drone NOMINAL/WARN/CRITICAL/BLOCKED stav s animací
- mode prepinac pres `/swarm/mode`
- `RTH all`
- start mise pres `/field/mission_confirm`
- manual `goto_cell` override
- camera a depth stream pres bridge
- zakladni 3D view, pokud jsou dostupne UI zavislosti
- post-mission HTML report (`swarm_center/core/report_generator.py`):
  - automaticky nabídnut po `mission_complete`
  - tlačítko "Export Report" pro ruční re-generaci
  - self-contained HTML s inline CSS, SVG gridem a spray dose overlajem
  - výstup: `reports/<mission_id>/report.html`

Bridge protokol je sdileny v:

- `src/scout_control/scout_control/bridge_protocol.py`
- `swarm_center/core/bridge_protocol.py`

Aktualni verze protokolu je `1.2` a zahrnuje i `MSG_CAMERA_FRAME` a
`MSG_DEPTH_FRAME`.

## Cesty, Data A Konvence

### Pouzivej `scout_control.paths`

Na path konstanty nepouzivej hardcoded rooty tam, kde jde pouzit:

- `WS_ROOT`
- `PERIMETERS_DIR`
- `PERIMETER_FILE`
- `GRID_FILE`
- `HOME_POS_FILE`
- `SPRAY_LOG_FILE`
- `CELL_DATA_DIR`
- `FIELD_MODEL_DIR`
- `REPORTS_DIR`

`paths.py` hleda workspace root podle pritomnosti `CLAUDE.md`. Fallback na
`~/scout_ws` zustava jen kvuli kompatibilite se starsim layoutem.

### Data, Ktera Mise Produkuje

Phase 1+2 (field setup + swarm mise):
- `perimeters/field_boundary.json`
- `perimeters/field_grid.json`
- `perimeters/home_positions.json`
- `spray_log.json`
- `cell_data/<cell_id>/...`
- `logs/avoidance_logs/*.jsonl`

Phase 3 (mapping):
- `perimeters/field_model/heightmap_<ts>.json`  — 2.5D terrain mapa (NED z per cell)
- `perimeters/field_model/heightmap_<ts>.npy`   — numpy dump stejnych dat
- `perimeters/field_model/obstacles_<ts>.json`  — klasifikovane prekazky
- `perimeters/field_model/manifest.json`        — index vsech verzi, pointer na latest

### PX4 / NED

- PX4 pouziva NED
- `z` je down, takze vyska je zaporne cislo
- pro heading-hold se casto pouziva `yaw = nan`
- PX4 topic naming tady standardne pouziva suffix `_v1`, napr.:
  `/fmu/out/vehicle_local_position_v1`

### QoS

Projekt je citlivy na QoS match. Opatrne hlavne u:

- latched command / status topicu
- `/swarm/task_status`
- `/field/setup_complete`
- `/swarm/mission_ready`
- `/{drone_ns}/avoidance/status`
- obrazovych dat pres `gcs_bridge`

Pri upravach bridge, coordinatoru nebo avoidance runtime nejdriv zkontrolovat
QoS na obou stranach.

## Build, Test A Overy

ROS2 package:

```bash
cd /home/tj/_Data/_Projekty/TJlabs/scout_ws
colcon build --packages-select scout_control
source install/setup.bash
```

Targeted pure-Python avoidance testy:

```bash
PYTHONPATH=src/scout_control pytest \
  src/scout_control/test/test_local_planner.py \
  src/scout_control/test/test_avoidance_helpers.py
```

Launcher:

```bash
python3 scout_launcher.py
```

## Dulezite Rozchody A Rizika

- nektere launch/config kombinace stale drzi direct fallback
  (`navigation_backend=direct`) pro kompatibilitu; cilovy model je runtime-owned
  execution (`navigation_backend=avoidance_runtime`)
- `task_allocator.yaml` neodpovida registraci v `setup.py`
- bridge protokol je duplikovany ve dvou souborech; zmeny musi zustat
  synchronizovane. Aktualni v1.3 overlay payloady (`MSG_NO_GO_OVERLAY`,
  `MSG_REFINED_GRID_EVENT`) jsou GCS konzumenty napojene.
- nektere scenare a starsi poznamky stale pocitaji s `~/scout_ws`
- `scout_launcher.py` ma workspace root natvrdo, neni plne prenositelny
- `docs/plans/scout_ws_node_audit.md` byl uzavren a smazan; pro topic kontrakty
  pouzivej `docs/topic_contract.md`
- worktree muze byt spinavy; nevracet cizi zmeny bez explicitniho zadani

### Runtime RTH — kriticky poznatek (opraveno 2026-04-27)

- `obstacle_avoidance_runtime` musi dostat home pozici z `/drone_N/rth_target`
  (posila `home_manager`, `geometry_msgs/Point`, TRANSIENT_LOCAL).
- Bez tohoto subscriberu runtime pouzival fyzickou spawn pozici dronu jako home.
- V Isaac Sim dron spawni na NED(0,0), pad muze byt na NED(0,-5) → RTH smer jinam.
- Po pridani `_rth_target_cb` subscriberu je home vzdy roven prirazenemu padu.
- `_complete_target()` po `return_home` prechazi do `LANDING` (drive IDLE → dron
  hoveroal bez pristani).

## Dokumentacni Konvence V Teto Repu

- dlouhodoba lidska dokumentace je v `docs/`
- operatorni runbooky a rucni launch postupy jsou v `launch_files/`
- `CLAUDE.md` ma byt aktualni mapa projektu a workflow
- `codex.md` ma byt prubezny changelog / log dulezitych zjisteni pro dalsi AI
  session

## Update 2026-04-22 (A/B/C/D Integrace)

Aktualni implementacni stav po koordinovane integraci:

- `obstacle_avoidance_runtime` je stabilizovany jako single flight owner pro
  avoidance-enabled flow.
- `local_mapper + local_planner + scan_manager` jsou aktivne integrovane
  pipeline moduly uvnitr runtime.
- `swarm_agent` umi backend volbu:
  - `navigation_backend=direct`
  - `navigation_backend=avoidance_runtime`
- default backend je `navigation_backend=avoidance_runtime`
- Pri `avoidance_runtime` backendu je direct PX4 ownership path ve `swarm_agent`
  vypnuta (zadne PX4 pubs/subs/timer/control loop).
- `swarm_agent` posila high-level `target_cmd` do runtime a `CELL_COMPLETE`
  odvozuje z runtime `last_completed_target_id`.
- `task_allocator` umi pracovat s blocked/deferred semantikou:
  - `blocked_severity` (`NONE|SOFT|HARD`)
  - `CELL_DEFERRED`
  - `TEMP_BLOCKED` cooldown.
- `gcs_bridge` forwarduje i avoidance detail stream
  (`AVOIDANCE_STATUS`, `AVOIDANCE_EVENT`) v payloadu `MSG_DRONE_STATUS`.

Aktualni stabilni `avoidance/status` kontrakt obsahuje mimo jine:
- `phase`, `state`, `result`
- `planner_mode`, `planner_state`
- `scan_state`, `scan_active`
- `no_path_streak`, `scan_attempts_for_target`
- `blocked_reason`, `blocked_since_s`, `blocked_severity`
- `reassign_recommended`
- `last_scan`, `last_runtime_event`
- aditivni ownership/mission feedback pole pro mission vrstvu
  (napr. accepted/active/completed target identifikatory)

Poznamka:
- full E2E launch (`full_e2e_mission.launch.py`) je stale historicky
  kompatibilni i s direct backendem a muze vyzadovat dalsi wiring/polish pro
  plne odstraneni stare direct-control cesty.

## Update 2026-04-22 (Swarm-Agent Ownership Refactor)

Aktualni cilovy model pro dalsi iterace:

- `swarm_agent`:
  - mission state owner
  - queue bunek/targetu
  - swarm reporting (`/swarm/drone_status` a souvisejici udalosti)
  - route provider: high-level commandy do runtime
- `obstacle_avoidance_runtime`:
  - production flight-control owner pro tuto path
  - planner/mapper/scan execution
  - jedine misto, ktere ma publikovat PX4 navigation commandy pro tuto path

Prakticka compatibility poznamka:

- direct path v `swarm_agent` muze byt docasne ponechana za backend gate, ale
  nema se dale rozsirovat funkcionalitou
- nove changes mají jit primarne do runtime command/status kontraktu
- runtime command ingestion akceptuje command aliasy, envelope payloady a target
  aliasy; invalidni commandy emituji `command_rejected`, validni
  `command_accepted`

Otevrene body:

- launch/config defaulty nemusi byt ve vsech scenarich prepnute na
  `navigation_backend=avoidance_runtime`
- pred finalnim odstraneni direct path je potreba E2E overeni ve full swarm
  scenari

In-progress poznamka:

- Tento dokument uz bere runtime-owned model jako cilovy a preferovany.
- Pokud nektera cast kodu/launchu stale pouziva direct path, brat to jako
  prechodovy compatibility stav, ne jako cilovou architekturu.

## Update 2026-04-28 (Node Audit Closure)

Historicky `docs/plans/scout_ws_node_audit.md` je uzavren a smazan. Otevrene
body z auditu jsou presunute do implementace a aktualnich zdroju pravdy:

- runtime god-object cleanup:
  - `avoidance/flight_phase_machine.py`
  - `avoidance/px4_publisher_adapter.py`
  - `avoidance/ros_io_adapter.py`
- typed core contract cleanup:
  - `avoidance/types.py` obsahuje typed JSON-compatible helpers pro
    `TargetCommand`, `AvoidanceStatus`, `SwarmDroneStatusEvent`,
    `SwarmTaskStatus`, `PadAssignment`, `FieldSetupComplete`,
    `ReturnHomeRequest`, `MissionReadySignal`
  - wire kompatibilita pres JSON/String zustava zachovana
- topic contract doc:
  - `docs/topic_contract.md`
- core hardening hotovo:
  - `cell_data_recorder` podporuje `drone_count` a topic templates
  - `grid_generator` pouziva explicitni `run()`
  - `mission_launcher` uz nema shutdown pres `SystemExit` z timeru
  - `spray_controller` zapisuje atomicky
  - `ml_interface` je oznaceny jako tooling stub
- bridge v1.3:
  - `MSG_NO_GO_OVERLAY` a `MSG_REFINED_GRID_EVENT` jsou synchronni v obou
    protocol souborech a GCS je konzumuje

Overeni:

```bash
PYTHONPATH=src/scout_control pytest \
  src/scout_control/test/test_flight_phase_machine.py \
  src/scout_control/test/test_ros_io_adapter.py \
  src/scout_control/test/test_typed_ros_contract_adapters.py \
  src/scout_control/test/test_px4_publisher_adapter.py \
  src/scout_control/test/test_swarm_core_hardening.py \
  src/scout_control/test/test_grid_generator_lifecycle.py

colcon build --packages-select scout_control
git diff --check
cmp -s src/scout_control/scout_control/utils/bridge_protocol.py \
  swarm_center/core/bridge_protocol.py
```

Vysledek targeted sady: `25 passed`.
