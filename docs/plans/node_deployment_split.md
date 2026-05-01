# SCOUT node deployment split

Tento vypis rozdeluje nody a moduly podle ciloveho nasazeni:

- **Dron / onboard**: bezi primo na companion computeru konkretniho dronu a pracuji v jeho namespace, typicky `drone_N`.
- **Swarm Center / ground station**: bezi na pozemnim pocitaci operatora, koordinuji vice dronu, ukladaji model pole, ovladaji UI nebo generuji reporty.
- **Sim / dev only**: existuji hlavne pro Gazebo/Isaac testy, vizualizaci nebo vyvoj.

## Dron / onboard

Tyto nody maji byt spoustene per dron.

| Node / executable | Cilovy nazev | Role |
|---|---|---|
| `obstacle_avoidance_runtime` | `avoidance_runtime_N` | Hlavni vlastnik autonomniho letu. Vydava PX4 offboard setpointy, resi local avoidance, scan, replan, RTH/land jako high-level targety. |
| `swarm_agent` | `swarm_agent_N` | Per-drone delegator mise. Prevadi pridelenou praci na high-level targety pro `obstacle_avoidance_runtime`; nema vlastnit PX4 setpointy. |
| `precision_landing` | `precision_landing_N` | Volitelna per-drone presna pristavaci logika / pad detector workflow. |

Onboard moduly pouzivane uvnitr runtime nebo per-drone logiky:

| Modul | Role |
|---|---|
| `scout_control.avoidance.local_mapper` | Rolling 2.5D lokalni mapa kolem dronu. |
| `scout_control.avoidance.local_planner` | Kratkodobe planovani obchvatu a subgoal. |
| `scout_control.avoidance.scan_manager` | 360 scan pri critical/blocking situacich. |
| `scout_control.avoidance.depth_projector` | Projekce depth kamery do bodu pro lokalni mapu. |
| `scout_control.avoidance.lidar_projector` | Projekce lidar/range dat, pokud je senzor dostupny. |
| `scout_control.avoidance.telemetry_hub` | Slouceni odometrie, health a runtime stavu. |
| `scout_control.avoidance.peer_tracks` | Lokalni reprezentace ostatnich dronu jako dynamickych prekazek. |
| `scout_control.avoidance.health_monitor` | Runtime health/readiness stav. |
| `scout_control.avoidance.flight_phase_machine` | Faze letu a prechody mezi takeoff/mission/RTH/land. |
| `scout_control.avoidance.px4_publisher_adapter` | Adapter pro publikaci PX4 prikazu/setpointu. |
| `scout_control.avoidance.ros_io_adapter` | ROS IO hranice runtime. |
| `scout_control.vision.pad_detector` | Detekce landing padu pro precision landing. |

## Swarm Center / ground station

Tyto nody/moduly patri na pozemni pocitac, kde bezi Swarm Center a orchestrace.

### ROS 2 nody z `scout_control`

| Node / executable | Role |
|---|---|
| `field_setup_coordinator` | Orchestrace setup flow: pady, hranice pole, grid, readiness. |
| `home_manager` | Centralni zdroj home pad / RTH targetu pro vsechny drony. |
| `grid_generator` | Vytvoreni zakladniho gridu z polygonu pole. |
| `field_model_builder` | Skladani mapping vystupu do field modelu: terrain, obstacles, no-go zones. |
| `mapping_mission` | Dedicated mapping workflow pred produkcni misi. |
| `swarm_coordinator` | Koordinace prace mezi drony, blocked/deferred work, mission progress. |
| `task_allocator` | Rozdeleni bunek/sektoru mezi dostupne drony. |
| `mission_launcher` | Spousteni mise a logovani mission summary. |
| `gcs_bridge` | ROS 2 <-> Swarm Center TCP line-JSON bridge. |
| `cell_data_recorder` | Ukladani evidence navstev bunek, snapshotu a `meta.json`. |
| `spray_controller` | Payload/spray controller a log spray eventu. |
| `ml_interface` | Placeholder/payload inference vrstva pro NDVI, anomalii a davkovani. |

### Swarm Center aplikace

Tyto soubory nejsou ROS 2 nody; bezi jako samostatna PyQt6 aplikace pres `python3 swarm_center/main.py`.

| Modul | Role |
|---|---|
| `swarm_center/main.py` | Entry point Swarm Center aplikace. |
| `swarm_center/core/swarm_manager.py` | Centralni stav dronu, mise a gridu. |
| `swarm_center/core/ros2_bridge.py` | TCP klient pro `gcs_bridge`. |
| `swarm_center/core/mavlink_manager.py` | MAVLink spojeni per dron. |
| `swarm_center/core/field_manager.py` | Field grid/model nacitani a regrid. |
| `swarm_center/core/field_model_loader.py` | Nacitani heightmap, obstacles a no-go overlayu. |
| `swarm_center/core/report_generator.py` | Post-mission HTML report. |
| `swarm_center/core/depth_mapper.py` | Depth frame -> NED pointy pro UI/3D vizualizaci. |
| `swarm_center/ui/*` | UI: mapa, control panel, drone list, manual control, kamera, 3D viewport, avoidance panel. |

## Operator/setup utility

Tyto nody patri spis na pozemni stanici, ale nejsou hlavni produkcni autonomie.

| Node / executable | Role |
|---|---|
| `manual_controller` | Manual intent bridge pro Swarm Center setup/takeoff ovladani; nema vydavat primo konkurecni PX4 setpointy v produkcni autonomii. |
| `field_setup_tool` | Rucni setup utility pro pole/pady. |
| `legacy_manual_controller` | Legacy manual controller, drzet mimo produkcni E2E cestu. |

## Sim / dev only

Tyto nody se spousti pro simulaci, testy nebo vizualizaci. Na realnem dronu by je nahradily realne senzory/drivery nebo by vubec nebezely.

| Node / executable | Role |
|---|---|
| `lidar_bridge_drone_N` (`ros_gz_bridge parameter_bridge`) | Gazebo LaserScan -> `/drone_N/downward_lidar/scan`. |
| `camera_bridge_drone_N` (`ros_gz_image image_bridge`) | Gazebo camera image -> `/drone_N/camera/image_raw`. |
| `depth_bridge_drone_N` (`ros_gz_image image_bridge`) | Gazebo depth image -> `/drone_N/depth/image_raw`. |
| `depth_info_bridge_drone_N` (`ros_gz_bridge parameter_bridge`) | Gazebo camera info -> `/drone_N/camera/camera_info`. |
| `camera_hud` | Developer/operator HUD vizualizace. |
| `obstacle_viz` | Debug vizualizace prekazek. |
| `gimbal_cam_viz` | Debug gimbal/camera vizualizace. |
| `scan_cloud_viz` | Debug scan cloud vizualizace. |
| `obstacle_avoidance_mission` | Test-only route provider/harness podle architektury. |
| `obstacle_detector` | Legacy/debug detector, ne produkcni cesta. |
| `perimeter_flight` | Volitelny/legacy helper pro assisted boundary mode. |

## Prakticky produkcni split

Minimalni realne nasazeni pro jeden dron:

- **Na dronu**: `obstacle_avoidance_runtime`, `swarm_agent`, realne PX4/MAVLink/ROS 2 sensor drivery, volitelne `precision_landing`.
- **Na Swarm Center PC**: `gcs_bridge`, `field_setup_coordinator`, `home_manager`, `swarm_coordinator`, `task_allocator`, `mission_launcher`, `cell_data_recorder`, `spray_controller`, `field_model_builder`, `mapping_mission`, `ml_interface`, aplikace `swarm_center/main.py`.

Minimalni realne nasazeni pro vice dronu:

- **Na kazdem dronu**: vlastni instance `obstacle_avoidance_runtime` a `swarm_agent` s parametrem `drone_id`.
- **Na Swarm Center PC**: jedna instance centralnich orchestracnich nodu a jedna Swarm Center UI aplikace.
