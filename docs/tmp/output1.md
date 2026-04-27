# Scout WS — Porovnání plánů vs. aktuální stav
**Datum:** 2026-04-27  
**Zdroje:** `scout_ws_e2e_arch.txt`, `scout_ws_e2e_architecture.md`, `update_plans.txt`, `CLAUDE.md`, memory

---

## 1. Roadmap Status (dle `scout_ws_e2e_architecture.md`)

| Fáze | Název | Plán | Stav |
|------|-------|------|------|
| Phase 1 | Stable Onboard Runtime | Finish runtime split, make `swarm_agent` delegator, single flight owner | **DONE** |
| Phase 2 | Boundary to Base Grid | Pad registration, polygon capture, grid generation, E2E verify | **DONE** |
| Phase 3 | Mapping Mission Pipeline | Lawnmower flight, field model outputs, terrain/obstacle map | **DONE** |
| Phase 4 | Operational Hardening | Multi-drone classes, charging lifecycle, bridge v1.3, path portability | PLANNED |
| Phase 5 | Swarm Center Planning | Field model v GCS, dashboards, post-mission reports | PLANNED |
| Phase 6 | Context Slices to Drones | Statické výřezy mapy posílat na drony per-region | PLANNED |
| Phase 7 | Agronomic Intelligence | Reálné modely growth, pest detection, treatment recommendations | PLANNED |
| Phase 8 | Sensor & Infrastructure | Lidar, terrain sensing, automated refill, advanced scheduling | PLANNED |

---

## 2. Co bylo naplánováno vs. co je hotovo

### Phase 1 — Stable Onboard Runtime ✅

**Plánováno (`update_plans.txt` M1+M2):**
- Rozdělit monolit `obstacle_avoidance_runtime.py` do interních modulů
- Rolling local mapa 40×40 m / 0.25 m
- A* short-horizon planner
- `swarm_agent` jako pure delegator bez přímého PX4 ownership

**Hotovo:**
- `obstacle_avoidance_runtime.py` — Single Flight Owner
- `avoidance/local_mapper.py` — Rolling obstacle memory
- `avoidance/local_planner.py` — A* planner s subgoal selection
- `avoidance/scan_manager.py` — 360° scan orchestrace
- `avoidance/depth_projector.py` — depth frame → point batch
- `avoidance/peer_tracks.py` — safety zóny ostatních dronů
- `avoidance/types.py` — sdílené datové typy
- `avoidance/telemetry_hub.py` — centrální topic registry
- `swarm_agent` má `navigation_backend=avoidance_runtime` jako default
- Avoidance status kontrakt: `phase`, `state`, `result`, `planner_mode`, `blocked_severity`, `reassign_recommended`, atd.

**Odchylka od plánu:** Žádná zásadní. RTH bug (home pozice bez `rth_target` subscriberu) byl objeven a opraven 2026-04-27.

---

### Phase 2 — Boundary to Base Grid ✅

**Plánováno:**
- 2A: Home pad registration s metadata + pad state machine
- 2B: Polygon boundary capture (`/field/boundary_point`, `/field/boundary_close`)
- 2C: Grid generation z polygonu (ray-cast point-in-polygon, `cell_class`)
- 2D: Full E2E swarm field mission verify

**Hotovo:**
- `home_manager.py` — pad registry, occupancy SM, RTH coordination
- `field_setup_coordinator.py` — setup orchestrace, IDLE→ASSIGN_PADS→CAPTURE_BOUNDARY→GENERATE_GRID→WAITING_FOR_LANDING→READY_FOR_MISSION
- `grid_generator.py` — `boundary_mode` s inset buffer a cell classification
- `perimeters/field_boundary.json`, `perimeters/field_grid.json`, `perimeters/home_positions.json`
- E2E ověřeno v Isaac Sim (phase 1+2 test 2026-04-27)

**Odchylka od plánu:** Žádná.

---

### Phase 3 — Mapping Mission Pipeline ✅ (nové vs. plán)

**Plánováno (`scout_ws_e2e_architecture.md`):**
- Mapping mission flight pattern (lawnmower nad polygonem)
- Field model outputs: terrain map (2.5D heightmap), static obstacle extraction
- Persist pod `perimeters/field_model/`
- Precision landing / home pad vision (deferred)
- Odstranit `navigation_backend=direct` po Phase 3 verify

**Hotovo:**
- `mapping/heightmap.py` — `Heightmap2D`, 2.5D grid (min NED-z per cell)
- `mapping/obstacle_extractor.py` — klasifikace překážek z point cloudu
- `mapping/field_model_builder.py` — ROS2 node, akumuluje depth frames, persistuje do `perimeters/field_model/`
- `missions/mapping_mission.py` — lawnmower trasa z `field_boundary.json`, posílá waypoints do runtime
- `launch/mapping_mission.launch.py` — parametrizovaný launch
- Výstupy: `heightmap_<ts>.json`, `heightmap_<ts>.npy`, `obstacles_<ts>.json`, `manifest.json`

**Nový bug nalezený a opraven:** `depth_projector.project_to_world_points()` filtruje přes collision band — terrain body na `world_z≈0` byly vyhazovány. Opraveno: pro heightmap se volá `depth_to_body_points()` + ruční world projekce.

**Odchylka od plánu:**
- Precision landing / home pad vision: stále deferred (neimplementováno)
- `navigation_backend=direct` stále přítomno (nebyl spuštěn dedicated ověřovací run pro jeho odstranění)

---

### Milestone M3 — Swarm Integration přes Runtime Backend ✅

**Plánováno (`update_plans.txt`):**
- `swarm_agent` dostane `navigation_backend=avoidance_runtime`
- místo přímých PX4 setpointů posílá `avoidance/target_cmd`
- čte `avoidance/status`, publikuje `swarm/drone_status`

**Hotovo:**
- `swarm_agent` má obě backend větve, default `avoidance_runtime`
- direct PX4 control path uvnitř `swarm_agent` je gated za backend switch
- `CELL_COMPLETE` odvozováno z `last_completed_target_id` (ne z interního flight loopu)
- `task_allocator` má `TEMP_BLOCKED`, deferred queue, `CELL_DEFERRED`
- `gcs_bridge` forwarduje `AVOIDANCE_STATUS`, `AVOIDANCE_EVENT` v payloadu `MSG_DRONE_STATUS`

---

### Milestone M4 — Peer Drone Dynamic Mask + Reassign Policy ✅

**Plánováno:**
- peer tracks do local mapperu
- `BLOCKED_SOFT/HARD`
- `CELL_DEFERRED`
- allocator respektuje dočasně unavailable dron

**Hotovo:**
- `peer_tracks.py` — ingest + smoothing/clamp
- `blocked_severity`: `NONE|SOFT|HARD`
- allocator: hard blocked → deferred + cooldown + reassign queue

---

## 3. Co z plánů ještě není hotovo

### Přechodové technické dluhy

| Položka | Stav | Poznámka |
|---------|------|----------|
| `navigation_backend=direct` odstranění | ❌ | Čeká na dedicated E2E verify run |
| `task_allocator.yaml` vs `setup.py` | ❌ | Neodpovídá konzolám skriptů, standalone nespustitelný |
| `WS_DIR` hardcoded v `scout_launcher.py` | ❌ | Plánováno odstratnit v Phase 4 |
| Precision landing / pad vision | ❌ | Deferred z Phase 2, stále nerealizováno |
| Bridge protokol v1.3 | ❌ | Plánováno Phase 4 |
| Typed `scout_msgs` package | ❌ | M5 milník, stále `std_msgs/String` JSON |
| Bridge protokol duplikace (2 soubory) | ⚠️ | Existuje, nutno udržovat ručně synchronizované |

### Phase 4+ (plánováno, nezahájeno)

- Multi-drone class support v pad allocatoru (`allowed_drone_classes`)
- Charging lifecycle s real hardware feedback
- Workspace path portability
- Swarm Center planning views (field model v GCS, boundary editor, mission refinement)
- Context slices ze Swarm Center zpět na drony (per-region static map extract)
- Agronomic intelligence (growth/pest modely)
- Lidar integration (extensionpoint připraven přes `PointBatch` API)

---

## 4. Porovnání s high-level architekturou (`scout_ws_e2e_arch.txt`)

### Co je splněno ze sekce "Node roles and target state"

| Node | Plánovaný target | Aktuální stav |
|------|-----------------|---------------|
| `field_setup_coordinator` | Keep as setup orchestrator | ✅ Aktivní |
| `grid_generator` | Base grid + extend metadata | ✅ `boundary_mode` + `cell_class` |
| `home_manager` | Central pad source for all drones | ✅ Aktivní, metadata, pad SM |
| `mission_launcher` | Mission trigger + progress logging | ✅ Aktivní |
| `swarm_coordinator` | Task allocator + blocked/deferred | ✅ Lightly refactored |
| `swarm_agent` | Mission delegator, no direct flight | ✅ `avoidance_runtime` backend |
| `obstacle_avoidance_runtime` | Single flight owner | ✅ Finalizováno |
| `obstacle_avoidance_mission` | Test-only harness | ✅ Test harness |
| `obstacle_detector` | Deprecate from production path | ✅ Debug-only |
| `gcs_bridge` | Forward mission/runtime/report events | ✅ v1.2, avoidance forwarding |
| `cell_data_recorder` | Evidence store | ✅ Aktivní |
| `spray_controller` | Payload action controller | ✅ Aktivní |

### Co je naplánováno v arch dokumentu, ale ještě neprobíhá

- **Refined grid a no-go zones**: field model existuje (Phase 3), ale nejsou ještě integrovány do grid refinement (Phase 4 úkol)
- **Mission packages per drone**: terrain-aware mission packages z field modelu neexistují
- **GCS planning surface**: Swarm Center má základní mapu a progress, ale ne boundary editor, no-go zone editor, 3D terrain view (Phase 5)
- **Context slices zpět na drony**: Swarm Center → drone local mapper static layer (Phase 6)
- **Charging lifecycle v mission planning**: jen pad SM, ne mission-time battery scheduling
- **Agronomic reports**: placeholder, žádná real implementace

---

## 5. Shrnutí a doporučení pro další session

### Silné stránky aktuálního stavu
- Celá runtime avoidance pipeline je funkční a ověřená E2E testem
- Field model (Phase 3) je sice nový, ale Swarm Center ho již umí konzumovat (heightmap, obstacles, no-go zones)
- Architektura dobře připravena pro rozšíření: `PointBatch` API, plugin-friendly local mapper, modularní avoidance

### Kritické otevřené body (doporučené pro Phase 4)
1. **Integrace field model → grid refinement** — no-go zones z Phase 3 nejsou ještě input do `grid_generator`
2. **Odstranění / uzavření `navigation_backend=direct`** — po jednom ověřovacím E2E runu
3. **Precision landing** — home pad vision stále chybí, RTH je jen NED goto
4. **Workspace path portability** — `WS_DIR` hardcoded je riziko při přesunu projektu

### Co Swarm Center už umí ze strany field modelu
- Vyčtení heightmap, obstacles, manifest
- Potenciál pro vizualizaci a plánování v 2D/3D view

### Milníky k Phase 4
- M5: typed `scout_msgs` (nízká priorita, jen pokud bude čas)
- Phase 4: multi-drone classes, charging lifecycle, bridge v1.3
- Phase 5: Swarm Center planning views (boundary editor, mission refinement, agronomic reports)
