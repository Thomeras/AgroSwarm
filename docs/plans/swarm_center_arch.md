# Swarm Center — architektura a schopnosti

> Stav k 2026-04-27. Samostatná PyQt6 GCS aplikace mimo ROS2 workspace.

## Spuštění

```bash
cd swarm_center
pip install -r requirements.txt
python3 main.py [--drones 2] [--base-port 14540] [--bridge-port 17845] [--world-image ../worlds/agro_field_overhead.png]
```

## Připojení

| Kanál | Protokol | Default |
|---|---|---|
| PX4 SITL per dron | MAVLink UDP | `127.0.0.1:14540+N` |
| scout_ws gcs_bridge | TCP line-JSON | `127.0.0.1:17845` |

Bridge se auto-reconnectuje při restartu simulace. Swarm Center může běžet nezávisle.

## Struktura souborů

```
swarm_center/
├── main.py                   # argparse entry point
├── core/
│   ├── app_logger.py         # centrální log s levely
│   ├── bridge_protocol.py    # MSG_* konstanty, verze protokolu (1.2)
│   ├── depth_mapper.py       # DepthMapper: depth frame → NED body pointy
│   ├── field_manager.py      # FieldGrid: load/from_file/synthetic/regrid
│   ├── mavlink_manager.py    # SwarmMavlinkManager: threaded MAVLink per dron
│   ├── ros2_bridge.py        # Ros2BridgeClient: TCP recv + send commands
│   └── swarm_manager.py      # SwarmManager: centrální stav (drony, mise, grid)
└── ui/
    ├── main_window.py        # QMainWindow — wire-up všeho
    ├── field_view.py         # 2D top-down mapa s gridem a drony
    ├── control_panel.py      # pravý sloupec — status, progress, tlačítka
    ├── drone_list.py         # tabulka dronů s telemetrií
    ├── manual_control.py     # klávesnicové ovládání + field setup flow
    ├── camera_view.py        # live camera + depth streamy
    └── viewport_3d.py        # 3D trail vizualizace
```

## Datový tok

```
MAVLink (per-drone threads)  ──► SwarmManager ──► FieldView / DroneListPanel / Viewport3D
ROS2 bridge (task_status,
             drone_status,
             camera_frame,
             depth_frame…)   ──► SwarmManager ──► ControlPanel / MissionState

ControlPanel / ManualControl ──► Ros2BridgeClient ──► scout_ws
SwarmManager (peer cells)    ──► 1× /s broadcast  ──► Ros2BridgeClient
```

## Taby

### Mission (FieldView)
- Top-down 2D mapa s NED souřadnicemi, overhead PNG s alignmentem
- Grid overlay — stav buněk: volná / přiřazená / dokončená / blokovaná
- Live pozice dronů (20 fps repaint, trail)
- Klik na dron → výběr
- Pravý klik na buňku → GOTO override (→ `/swarm/cell_override`)

### Manual (ManualControlWidget)
- Výběr aktivního dronu (combo + klik na mini mapu)
- Klávesnicové řízení: W/S = forward/back, A/D = strafe, Up/Down = výška
  - Posílá velocity commands přes `MSG_MANUAL_CONTROL`
- Field setup flow: assign_pad_0/1 → mark corners (NE/NW/SE/SW) → generate grid → start mission
- Land per dron
- Live camera stream vybraného dronu (JPEG přes bridge)

### Camera (CameraView)
- Live barevný camera stream (JPEG) per dron
- Live depth stream (PNG)
- Ovládání streamu: enable/disable, fps limit per dron nebo pro všechny (`MSG_CAMERA_CONTROL`)

### 3D Map (Viewport3D)
- 3D vizualizace pole s gridem
- Trail dronů v prostoru
- Napojeno na `DepthMapper` (depth frame → NED body pointy)

## Control Panel (pravý sloupec)

| Prvek | Funkce |
|---|---|
| MAVLink status | počet připojených dronů |
| Bridge status | zelená/červená indikace |
| Setup label | aktuální fáze field_setup_coordinator |
| Progress bar | completed_cells / total_cells |
| Mode selector | MAPPING / SPRAYING / CHECKING → `/swarm/mode` |
| Start Mission | `/field/mission_confirm` (aktivní jen když field_ready) |
| RTH all drones | `MSG_RTH_ALL` s potvrzovacím dialogem |
| EMERGENCY STOP | `MSG_EMERGENCY_STOP` okamžitě |
| Load grid JSON | ruční načtení field_grid.json |
| Cell size spinner | vizuální regrid (nemění běžící misi) |
| Log panel | posledních 500 řádků MAVLink + bridge logů |

## Drone List Panel

- Tabulka: ID, přiřazená buňka, telemetrie (pozice, výška, stav)
- ARM / DISARM per dron přes MAVLink (s potvrzovacím dialogem pro disarm)

## Přijímané zprávy z bridge (inbound)

| Zpráva | Akce |
|---|---|
| `MSG_HELLO` | verze bridge a ROS2 distro do status baru |
| `MSG_TASK_STATUS` | stav buněk per dron → SwarmManager |
| `MSG_DRONE_STATUS` | avoidance detail, phase, planner_state, blocked info → SwarmManager |
| `MSG_SETUP_STATUS` | progress field setup → MissionState |
| `MSG_SETUP_COMPLETE` | auto-reload gridu z canonical path |
| `MSG_MISSION_READY` | mise běží → MissionState |
| `MSG_MISSION_COMPLETE` | mise dokončena → MissionState |
| `MSG_GRID_RELOAD` | přenačtení gridu (path nebo default) |
| `MSG_CAMERA_FRAME` | JPEG bytes per dron → CameraView + ManualControl |
| `MSG_DEPTH_FRAME` | PNG bytes per dron → CameraView + DepthMapper |
| `MSG_CAMERA_INFO` | intrinsics per dron → DepthMapper |

## Odesílané zprávy do bridge (outbound)

| Zpráva | Trigger |
|---|---|
| `MSG_SET_MODE` | změna mode combo |
| `MSG_START_MISSION` | tlačítko Start Mission |
| `MSG_RTH_ALL` | tlačítko RTH all |
| `MSG_EMERGENCY_STOP` | tlačítko Emergency Stop |
| `MSG_GOTO_CELL` | pravý klik na buňku |
| `MSG_MANUAL_CONTROL` | klávesy W/S/A/D/Up/Down nebo setup akce |
| `MSG_GENERATE_GRID` | tlačítko Generate Grid |
| `MSG_PEER_CELLS` | 1× za sekundu (grid-level swarm awareness) |
| `MSG_CAMERA_CONTROL` | enable/disable/fps per dron |
| `MSG_PING` | keepalive každých 5 s |

## Co chybí / není hotové

- Avoidance status panel — bridge data (`blocked_reason`, `planner_state` atd.) přijdou, ale nejsou zobrazena v samostatném widgetu
- Phase 3 field model vizualizace — heightmap a obstacles nejsou zobrazeny
- 3D depth mapping — `DepthMapper` je zabudován, ale Viewport3D ho plně nevykresluje
- `task_allocator.yaml` scénář není spustitelný bez registrace v `setup.py`
