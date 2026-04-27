# Swarm Center — architektura a schopnosti

> Stav k 2026-04-27 (Phase 5 dokončena). Samostatná PyQt6 GCS aplikace mimo ROS2 workspace.

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
│   ├── bridge_protocol.py    # MSG_* konstanty, verze protokolu (1.3)
│   ├── depth_mapper.py       # DepthMapper: depth frame → NED body pointy
│   ├── field_manager.py      # FieldGrid: load/from_file/synthetic/regrid
│   ├── field_model_loader.py # Phase 3 výstupy → overlay data (heightmap, obstacles)
│   ├── mavlink_manager.py    # SwarmMavlinkManager: threaded MAVLink per dron
│   ├── report_generator.py   # ReportGenerator: post-mission HTML report (Phase 5)
│   ├── ros2_bridge.py        # Ros2BridgeClient: TCP recv + send commands
│   └── swarm_manager.py      # SwarmManager: centrální stav (drony, mise, grid)
└── ui/
    ├── main_window.py        # QMainWindow — wire-up všeho
    ├── field_view.py         # 2D top-down mapa + overlay vrstvy (Phase 5)
    ├── control_panel.py      # pravý sloupec — status, progress, tlačítka
    ├── avoidance_panel.py    # per-drone avoidance stav s animací (Phase 5)
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
- Sector preview: barevné přiřazení sektorů per dron před startem mise
- Overlay vrstvy (Phase 5):
  - No-go zóny (z `field_model/`) — červené transparentní obdélníky
  - Obstacles — oranžové markery
  - Terrain heatmap — modrý gradient výšky
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
| **Export Report** | generuje HTML report posledního mission_id; aktivní po mission_complete |
| Load grid JSON | ruční načtení field_grid.json |
| Cell size spinner | vizuální regrid (nemění běžící misi) |
| Overlay checkboxy | No-go / Obstacles / Terrain / Sector preview |
| Log panel | posledních 500 řádků MAVLink + bridge logů |

## Avoidance Panel (pod drone list)

- Zobrazuje per-drone avoidance stav: NOMINAL / WARN / CRITICAL / BLOCKED
- Animovaný pulse pro BLOCKED stav
- Pole: planner_state, blocked_severity, no_path_streak, blocked_since

## Report Generator (Phase 5)

`swarm_center/core/report_generator.py` — pure Python, žádné ROS2 závislosti.

**Trigger:** po `MSG_MISSION_COMPLETE` se automaticky zobrazí dialog "Generovat report?".
Tlačítko "Export Report" v Control Panelu umožňuje ruční regeneraci kdykoli po misi.

**Zdroje dat (čte přímo z disku):**
- In-memory stav gridu (buňky s finálními statusy) → předán z SwarmManager
- `spray_log.json` — spray eventy per buňka per dron
- `cell_data/*/visit_N/meta.json` — drone_id, NED, timestamp per visit
- `perimeters/home_positions.json` — pad registry

**Výstup:**
```
reports/<mission_id>/
  ├── report.html          # self-contained HTML (inline CSS, SVG heatmap)
  └── grid_snapshot.json   # snapshot gridu pro regeneraci po restartu
```

**Obsah reportu:**
- Coverage: total / visited / missed / blocked / skipped s % hodnotami
- Spray: celková dávka, buňky postříkány, průměrná dávka
- Blocked events: seznam cell_id s rozlišením blocked/skipped
- Per-drone: navštívené buňky, spray eventy, celková dávka, odhadnutá vzdálenost letu
- SVG NED grid heatmap: zelená = visited, červená = missed, oranžová = blocked, žlutá = skipped
- Spray overlay: semi-transparentní modré kruhy, průměr ∝ dávce

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
| `MSG_MISSION_COMPLETE` | mise dokončena → MissionState → dialog pro generování reportu |
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

- Bridge v1.3 overlay payloads (`MSG_NO_GO_OVERLAY`, `MSG_REFINED_GRID_EVENT`) nejsou
  na GCS straně ještě konzumovány — field model se načítá přímo ze souboru, ne přes bridge
- 3D depth mapping — `DepthMapper` je zabudován, ale Viewport3D ho plně nevykresluje
- `task_allocator.yaml` scénář není spustitelný bez registrace v `setup.py`
