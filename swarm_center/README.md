# Scout Swarm Center

Samostatná PyQt6 ground control station pro Scout Autonomous System.

Dva kanály:
- **MAVLink UDP** na PX4 SITL — pozice, telemetrie, armování (hotovo v M1)
- **ROS2 bridge TCP** na `gcs_bridge` v scout_ws — task_status, mission events, mode, peer cells (hotovo v M2)

## Architektura

```
┌──────────────────────── Isaac Sim / Gazebo ────────────────────────┐
│                                                                    │
│   PX4 SITL #0 ──MAVLink UDP 14540──┐                               │
│   PX4 SITL #1 ──MAVLink UDP 14541──┤                               │
│                                    │                               │
│   scout_ws (ROS2):                 │                               │
│     swarm_agent_0/1                │                               │
│     task_allocator                 │                               │
│     field_setup_coord              │                               │
│     gcs_bridge ────TCP 17845──┐    │                               │
│                               │    │                               │
└───────────────────────────────┼────┼───────────────────────────────┘
                                ▼    ▼
                         ┌────────────────────────┐
                         │   Swarm Center (PyQt6) │
                         │                        │
                         │  MavlinkManager        │ ← pozice, telemetrie
                         │  Ros2Bridge            │ ← task_status, grid, mode
                         │  SwarmManager          │ ← centrální stav
                         │  FieldView             │ ← top-down + live grid
                         │  ControlPanel          │ ← progress, RTH, mode
                         └────────────────────────┘
```

## Instalace

```bash
cd swarm_center
pip install -r requirements.txt
```

Závislosti: `PyQt6`, `pymavlink`, `numpy`, `Pillow`, `pyqtgraph`, `PyOpenGL`.

## Spuštění

```bash
# Default: 2 drony, MAVLink na 14540+, bridge na 17845
python3 main.py

# Plná konfigurace
python3 main.py --drones 2 --base-port 14540 \
                --bridge-host 127.0.0.1 --bridge-port 17845 \
                --grid-file ~/_Data/_Projekty/TJlabs/scout_ws/perimeters/field_grid.json
```

Pokud `gcs_bridge` neběží, Swarm Center se bude periodicky pokoušet připojit
a logovat to v panelu „Log". Grid a MAVLink pozice fungují i bez bridge.

## Topic map

### ROS2 → Swarm Center (přijímáno přes bridge)

| ROS2 topic               | Co Swarm Center dělá |
|--------------------------|----------------------|
| `/swarm/task_status`     | Update mission progress, assigned cells per drone, grid status (hovering) |
| `/swarm/drone_status`    | CELL_COMPLETE → cell = visited |
| `/swarm/mission_ready`   | Označí misi jako aktivní |
| `/swarm/mission_complete`| Flip všech hovering → visited |
| `/field/setup_status`    | Zobrazí v horním řádku mission panelu |
| `/field/setup_complete`  | Automatický reload `field_grid.json` |

### Swarm Center → ROS2 (posíláno přes bridge)

| Payload         | ROS2 topic publishnutý v `gcs_bridge` |
|-----------------|--------------------------------------|
| Mode dropdown   | `/swarm/mode` JSON `{"mode":"MAPPING"}` |
| RTH all button  | `/swarm/rth_request` (1× per drone) |
| Peer cells (1 Hz)| `/swarm/peer_cells` JSON `{"cells":{"drone_0":"x4_y2",...}}` |

### MAVLink (oba směry)

- `GLOBAL_POSITION_INT`, `LOCAL_POSITION_NED`, `ATTITUDE`, `SYS_STATUS`, `HEARTBEAT`
- GCS heartbeat posíláme my (1 Hz) — PX4 to vyžaduje

## Struktura kódu

```
swarm_center/
├── main.py                      entry point, CLI args
├── core/
│   ├── bridge_protocol.py       wire format (shared s scout_ws)
│   ├── ros2_bridge.py           TCP client v QThread
│   ├── mavlink_manager.py       MAVLink thread per drone
│   ├── swarm_manager.py         centrální state + mission state
│   └── field_manager.py         načítání field_grid.json, NED → cell
└── ui/
    ├── main_window.py           QMainWindow
    ├── field_view.py            top-down mapa, grid status, drony
    ├── drone_list.py            tabulka (telemetrie + assigned cell)
    └── control_panel.py         progress bar, mode, RTH, bridge status
```

## Integrace do scout_ws

Samostatný `scout_ws_patch/` obsahuje dva soubory co patří do `scout_control`:

- `bridge_protocol.py` — duplikát wire formátu
- `gcs_bridge.py` — ROS2 node, TCP server

Instrukce v `scout_ws_patch/INTEGRATION.md`.

## Hotovo

### Milestone 1 — skeleton
- PyQt6 UI, MAVLink per dron (threaded), live pozice, grid overlay, pan/zoom, trails

### Milestone 2 — ROS2 bridge
- TCP bridge protokol (line-delimited JSON), auto-reconnect, ping heartbeat
- `gcs_bridge.py` v scout_ws (subscribe + publish all relevant topics)
- Live mission progress bar
- Setup status indicator
- Assigned cell highlighting in field view (drone's colour)
- Drone list s allocator statusem (WORKING / SECTOR_DONE / RTH)
- RTH all button
- Mode selector → `/swarm/mode`
- Peer cells 1 Hz broadcast → `/swarm/peer_cells`
- Automatický grid reload po `/field/setup_complete`

### Milestone 3 — Mission control
- Start mission button → `/field/mission_confirm` (enabled po `READY_FOR_MISSION`)
- Emergency stop → RTH all drones přes bridge
- Per-drone ARM / DISARM přes MAVLink (pravý klik v drone listu)
- Manual waypoint editor — pravý klik na cell v mapě → GOTO override na vybraném dronu
- Výběr dronu: klik na řádek v drone listu nebo na dron v mapě (bílý halo kroužek)
- bridge_protocol verze 1.1

### Milestone 4 — Kamera a 3D
- Camera bridge — `MSG_CAMERA_FRAME` / `MSG_DEPTH_FRAME` přes TCP bridge (JPEG base64, rate-limited)
- `gcs_bridge` subscribuje `/drone_N/camera/image_raw` + `/drone_N/depth/image_raw`
  — konvertuje do JPEG/PNG, posílá max N fps (parametr `camera_fps_limit`)
- `MSG_CAMERA_CONTROL` — GCS může zapnout/vypnout stream nebo změnit fps za běhu
- `CameraView` — per-drone JPEG viewer (taby), FPS counter, stale-frame warning, stream controls
- `Viewport3D` — 3D drone trail viewer (pyqtgraph.opengl); graceful fallback pokud pyqtgraph chybí
  — drone pozice z MAVLink, field grid outline, NED→GL konverze
- Hlavní okno přepracováno na QTabWidget (Mission / Camera / 3D Map)
- BRIDGE_VERSION bumped na 1.2

## Co bude dál

### Milestone 5 — AI overlay (next)
- Cell health map (NDVI, 0..1)
- Pest detection markers
- Variabilní spray dose recommendations

## Testování bez simulace

Swarm Center se spouští i bez žádného PX4 nebo bridge. Uvidíš:
- Syntetický 20×20 grid (pokud nenajde `field_grid.json`)
- Prázdnou drone list
- Bridge status: „disconnected" (červeně)
- MAVLink status: „waiting for HEARTBEAT"

Až pustíš simulaci a `gcs_bridge`, všechno se přepne na živě za pár sekund.
