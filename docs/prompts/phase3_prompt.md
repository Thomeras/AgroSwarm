# Phase 3 — Mapping Mission Pipeline

## Kontext

Phase 1 (DONE) stabilizovala onboard runtime: `obstacle_avoidance_runtime` je
single flight owner, `swarm_agent` je mission delegator, topic contracts jsou
v `TelemetryHub`.

Phase 2 (DONE) finalizovala pre-mission setup: home pad metadata, polygon
boundary capture, grid generation z polygonu, E2E verifikace.

**Phase 3 zavadi mapping mission pipeline** — autonomni nalet roje pres pole
s cilem postavit field model (terrain heightmap + staticke prekazky) PRED
operacni misi. Vystup mapping mise je vstup pro budouci spray/scout misi.

Vsechny zmeny musi zachovat single-flight-owner architekturu a nesmí narusit
beh existujicich nodu nebo Phase 2 setup flow.

---

## Phase 3 — Prehled zmen

### 3A — Mapping Flight Pattern (Lawnmower)
Nova mission strategie nad existujicim polygon gridem: "boustrophedon" /
lawnmower pattern s konfigurovatelnym bocnim prekryvem. Roj si rozdeli pas
na drone-line a leti rovnobezne. Vsechna flight execution jde pres
`obstacle_avoidance_runtime` (zadny novy PX4 publisher).

### 3B — Field Model Outputs
Behem mapping mise se z onboard senzoru (depth + odometrie) skladaji
artefakty:
- 2.5D terrain heightmap (rasterized DEM nad polygon AABB)
- staticke obstacle extraction (clusters z `local_mapper` snapshot)

Persist do `perimeters/field_model/` jako versionovane JSON + numpy raw.

### 3C — Precision Landing / Home Pad Vision (deferred z Phase 2)
Onboard detekce home pad markeru (ArUco / kontrastni vzor) v poslednich
metrech RTH. Vystup feeduje runtime jako fine-tuned target offset, ne jako
samostatny flight controller.

### 3D — Direct Backend Removal & Cleanup
Po E2E overeni Phase 3 odstranit `navigation_backend=direct` cestu ze
`swarm_agent` a souvisejici dead code. Sjednotit `task_allocator.yaml`
scenare s `setup.py` entry pointy.

---

## Rozdeleni na paralelni agenty (sessions)

Prace je rozdelena na **3 nezavisle paralelni session** + **1 finalizujici**.
Kazda session ma jasne ohraniceny scope souboru.

```
Session A (mapping pattern + mission node)  ──┐
                                                │
Session B (field model builder + persistence) ─┼──► Session D (cleanup + direct removal)
                                                │
Session C (precision landing vision)         ──┘
```

A, B, C jsou plne nezavisle. D bezi az po dokonceni A+B+C a po E2E overeni.

---

### SESSION A — Mapping Mission Node [todo]

**Scope:** Novy mission node pro lawnmower pattern; pouziva existujici grid a
`obstacle_avoidance_runtime` pro flight execution.

**Soubory k editaci:**
- `src/scout_control/scout_control/missions/mapping_mission.py` (novy)
- `src/scout_control/scout_control/utils/lawnmower.py` (novy — pure Python plan generator)
- `src/scout_control/launch/mapping_mission.launch.py` (novy)
- `src/scout_control/setup.py` (registrace `mapping_mission` console_script)
- `scenarios/mapping_mission.yaml` (novy)
- `src/scout_control/test/test_lawnmower.py` (novy)
- `src/scout_control/test/test_mapping_mission.py` (novy)

**Soubory k cteni (read-only):**
- `src/scout_control/scout_control/core/swarm_agent.py`
- `src/scout_control/scout_control/core/swarm_coordinator.py`
- `src/scout_control/scout_control/obstacle_avoidance_runtime.py`
- `src/scout_control/scout_control/avoidance/telemetry_hub.py`
- `src/scout_control/scout_control/utils/grid_generator.py`
- `src/scout_control/scout_control/utils/paths.py`
- `perimeters/field_grid.json` (priklad vystupu Phase 2)
- `CLAUDE.md`, `docs/plans/scout_ws_e2e_architecture.md`

**Co udelat:**

1. **Lawnmower plan generator (`utils/lawnmower.py`):**
   - Pure-Python funkce `generate_lawnmower(polygon_vertices_ned, drone_count, line_spacing_m, altitude_m, side_overlap_pct)`.
   - Rozdelit polygon na `drone_count` rovnomernych pasu (po hlavni ose AABB).
   - Pro kazdy pas vygenerovat boustrophedon vlnovku (sudé linie one-way,
     liché reverse) s `line_spacing_m` rozestupem.
   - Output: `dict[drone_id -> list[NED waypoint]]`.
   - Klipovat waypointy uvnitr polygonu (point-in-polygon, reuse z Phase 2).
   - `side_overlap_pct` (default 30%) zmensi `line_spacing_m`.

2. **Mapping mission node (`missions/mapping_mission.py`):**
   - ROS2 Node `MappingMission`.
   - Subscriby: `/swarm/home_positions`, `/{drone}/avoidance/status`.
   - Publishers: `/{drone}/avoidance/target_cmd` (delegace runtime), `/swarm/mapping_progress`.
   - Pri starte: nacist `field_boundary.json` + `home_positions.json`,
     vygenerovat lawnmower plan.
   - State machine: `IDLE → TAKEOFF → MAPPING → RTH → DONE`.
   - Per-drone: posilat waypointy postupne, cekat na `last_completed_target_id`
     z avoidance status (stejny pattern jako `swarm_agent`).
   - Pri `blocked_severity=HARD` na waypointu: skip + log do
     `/swarm/mapping_progress`.
   - **NEPUBLIKUJE PX4 setpointy** — vsechno pres avoidance runtime.

3. **Launch & scenario:**
   - `mapping_mission.launch.py` — runtime + mapping_mission per dron, podobny
     pattern jako `obstacle_avoidance_test.launch.py`.
   - `scenarios/mapping_mission.yaml` — pouzit absolutni workspace path
     (`/home/tj/_Data/_Projekty/TJlabs/scout_ws/install/setup.bash`).

4. **Testy:**
   - `test_lawnmower.py`: pure-Python testy na generator (pocet waypointu,
     prekryv, klipovani na polygon, drone_count rozdeleni).
   - `test_mapping_mission.py`: state machine transitions, skip-on-blocked.

**DULEZITE OMEZENI:**
- Zadny novy PX4 setpoint publisher. Vsechno pres `avoidance/target_cmd`.
- Nemen `swarm_agent.py`, `obstacle_avoidance_runtime.py` ani `avoidance/`.
- Nemen Phase 2 setup soubory (`field_setup_coordinator`, `home_manager`,
  `grid_generator`).
- Pouzivat `scout_control.utils.paths`.
- NED vsude.

---

### SESSION B — Field Model Builder [todo]

**Scope:** Onboard agregace senzorickych dat behem mapping mise do
persistovaneho field modelu.

**Soubory k editaci:**
- `src/scout_control/scout_control/mapping/field_model_builder.py` (novy)
- `src/scout_control/scout_control/mapping/heightmap.py` (novy)
- `src/scout_control/scout_control/mapping/obstacle_extractor.py` (novy)
- `src/scout_control/scout_control/utils/paths.py` (jen pridat
  `FIELD_MODEL_DIR` konstantu)
- `src/scout_control/test/test_heightmap.py` (novy)
- `src/scout_control/test/test_obstacle_extractor.py` (novy)

**Soubory k cteni (read-only):**
- `src/scout_control/scout_control/avoidance/local_mapper.py`
- `src/scout_control/scout_control/avoidance/depth_projector.py`
- `src/scout_control/scout_control/avoidance/types.py`
- `src/scout_control/scout_control/avoidance/telemetry_hub.py`
- `CLAUDE.md`, `docs/plans/scout_ws_e2e_architecture.md`

**Co udelat:**

1. **Heightmap (`mapping/heightmap.py`):**
   - Trida `Heightmap2D(origin_ned, cell_size_m, width, height)` —
     2D pole `min_z` per buňka.
   - Metoda `update_from_points(points_ned: np.ndarray)` — pro kazdy bod
     spocitat (i,j) bunku a aktualizovat `min_z` (NED → nizsi z je vys).
   - Metoda `to_dict()` / `from_dict()` pro JSON persistence.
   - Numpy-only, zadne ROS2 zavislosti — testovatelne pure-Python.

2. **Obstacle extractor (`mapping/obstacle_extractor.py`):**
   - Vstup: `local_mapper` snapshot (rolling occupancy points) nebo primo
     point batches z `depth_projector`.
   - Cluster pomoci jednoduche grid-based agregace (ne DBSCAN — bez sklearn).
   - Vystup: `list[Obstacle]` s polozkami `centroid_ned`, `bbox_ned`,
     `point_count`, `confidence`.
   - JSON serializace.

3. **Field model builder node (`mapping/field_model_builder.py`):**
   - ROS2 Node, subscribe `/{drone}/depth/image_raw` + odometry, nebo lepe
     reuse `local_mapper` snapshot topic (pokud existuje, jinak pridat
     read-only consumer cestu).
   - Akumulovat heightmap a obstacle clustery v pameti behem mapping mise.
   - Na trigger (`/swarm/mapping_complete`) zapsat:
     - `perimeters/field_model/heightmap_<timestamp>.json` + `.npy`
     - `perimeters/field_model/obstacles_<timestamp>.json`
     - `perimeters/field_model/manifest.json` (latest pointer)
   - Versionovani: stary manifest neprepisovat, jen pridat novy entry.

4. **Testy:**
   - `test_heightmap.py`: update_from_points correctness, JSON roundtrip,
     bounds checking.
   - `test_obstacle_extractor.py`: clustering na synthetic point cloud,
     edge cases (prazdny vstup, single point).

**DULEZITE OMEZENI:**
- Nemen `avoidance/local_mapper.py` ani jine `avoidance/` moduly. Pokud
  potrebujes data z local_mapperu, pridej read-only consumer / topic
  expozici jen pokud uz existuje; jinak konzumuj raw depth + odometry.
- Zadne nove pip dependencies (numpy uz je v repu, scipy/sklearn ne).
- Pouzivat `scout_control.utils.paths` pro vsechny file paths.
- Vystupni JSON formaty navrhnout dopredne s `version: 1` polem pro budouci
  zpetnou kompatibilitu.

---

### SESSION C — Precision Landing Vision [todo]

**Scope:** Onboard detekce home pad markeru. Feed jako fine-tuned offset do
runtime; runtime zustava flight owner.

**Soubory k editaci:**
- `src/scout_control/scout_control/vision/pad_detector.py` (novy)
- `src/scout_control/scout_control/vision/precision_landing.py` (novy ROS2 node)
- `src/scout_control/launch/precision_landing_test.launch.py` (novy)
- `src/scout_control/test/test_pad_detector.py` (novy)
- `worlds/markers/` (novy — ArUco PNG textury, pokud potreba)

**Soubory k cteni (read-only):**
- `src/scout_control/scout_control/obstacle_avoidance_runtime.py`
- `src/scout_control/scout_control/avoidance/telemetry_hub.py`
- `src/scout_control/scout_control/core/home_manager.py`
- `CLAUDE.md`, `docs/plans/scout_ws_e2e_architecture.md`

**Co udelat:**

1. **Pad detector (`vision/pad_detector.py`):**
   - Pure-Python OpenCV detector ArUco markeru (cv2.aruco DICT_4X4_50).
   - Vstup: BGR frame + camera_info (fx, fy, cx, cy).
   - Vystup: `Optional[PadDetection]` s `marker_id`, `offset_xy_body_m`,
     `range_m`, `confidence`.
   - Pomoci `cv2.solvePnP` na znamou velikost markeru → 3D pose v body
     frame.
   - Robustni na chybejici camera_info (default focal length warning).

2. **Precision landing node (`vision/precision_landing.py`):**
   - Subscribe: `/{drone}/camera/image_raw`, `/{drone}/camera_info` (best
     effort), `/{drone}/avoidance/status`, `/swarm/home_positions`.
   - Aktivni jen kdyz avoidance status `phase == "RTH_FINAL"` (nebo
     ekvivalent) a vyska pod prahem (default 5 m).
   - Pri detekci publikovat `/{drone}/avoidance/target_offset_cmd` (string
     JSON `{"dx_m":..., "dy_m":..., "valid_for_s": 1.0}`) — runtime to
     **muze** integrovat (pridat do contraktu) NEBO publikovat jako
     advisory bez side effectu, pokud runtime endpoint zatim neexistuje.
   - **NEPRIPISUJ DO RUNTIME PRIMO.** Pokud runtime nema dnes endpoint,
     publikuj na novy advisory topic `/{drone}/precision_landing/offset`
     a v dokumentaci poznamenat ze integrace do runtime je follow-up.

3. **Launch & test world:**
   - `precision_landing_test.launch.py` spousti runtime + precision_landing
     node + Gazebo bridge na kameru.
   - Volitelne: pridat marker do `worlds/markers/` jako texturu, nebo
     pouzit existujici Gazebo modely.

4. **Testy:**
   - `test_pad_detector.py`: ArUco detekce na syntetickem frame
     (cv2.aruco.generateImageMarker), pose roundtrip toleranci.
   - Mock camera_info handling.

**DULEZITE OMEZENI:**
- Zadny novy PX4 setpoint publisher. Vystup je advisory data, runtime
  rozhoduje.
- Nemen `obstacle_avoidance_runtime.py`. Integrace offsetu do runtime je
  follow-up task.
- Nemen `home_manager.py` ani Phase 2 moduly.
- Opencv-python a cv_bridge uz jsou v repu — zadne nove deps.

---

### SESSION D — Direct Backend Removal & Cleanup [todo, RUNS LAST]

**Scope:** Po E2E overeni Phase 3 odstranit dead code a sjednotit konfiguraci.

**Predpoklad:** Sessions A, B, C dokonceny a E2E overen v full_e2e_mission +
mapping_mission. Phase 3 features stabilni.

**Soubory k editaci:**
- `src/scout_control/scout_control/core/swarm_agent.py` (odstranit direct
  backend cestu)
- `src/scout_control/launch/full_e2e_mission.launch.py` (odstranit
  `navigation_backend` parametr, pokud uz je hardcoded na `avoidance_runtime`)
- `src/scout_control/launch/isaac_e2e_mission.launch.py` (totez)
- `scenarios/*.yaml` (cleanup direct-backend zminek)
- `src/scout_control/setup.py` (sladit `task_allocator` registraci se
  scenarem nebo scenar smazat)
- `scenarios/task_allocator.yaml` (smazat NEBO opravit aby fungoval)
- `CLAUDE.md` (aktualizovat — direct backend pryc)
- `docs/plans/scout_ws_e2e_architecture.md` (Phase 3 [DONE], direct backend
  pryc z §3 a §16)

**Soubory k cteni (read-only):**
- Vsechno ostatni — jen overeni dependency

**Co udelat:**

1. **Audit direct backend usage:**
   - `grep -rn "navigation_backend" src/ scenarios/ launch/`
   - Identifikovat kazde misto a rozhodnout (smazat / hardcode na runtime).

2. **Remove direct path from `swarm_agent`:**
   - Smazat PX4 publisher/subscriber/timer/control loop kod.
   - Smazat `navigation_backend` parametr (default & only path = runtime).
   - Zachovat mission state owner + reporting + delegation logic.

3. **Launch & scenario sjednoceni:**
   - Vsechny production launches: bez `navigation_backend` argumentu.
   - Scenare bez direct-backend zminek.

4. **`task_allocator.yaml`:**
   - Bud smazat (allocator je interni modul, ne node).
   - Nebo registrovat jako legitimni standalone node v `setup.py` a
     poskytnout opravdovou main funkci.
   - Rozhodnuti dokumentovat v PR.

5. **Dokumentace:**
   - `CLAUDE.md` §"Update 2026-04-22 (A/B/C/D Integrace)" prepsat na
     reflect direct removal.
   - `scout_ws_e2e_architecture.md` Phase 3 [DONE], §3 Backend Modes
     ponechat jen `avoidance_runtime` radek.

6. **E2E sanity:**
   - `colcon build --packages-select scout_control && source install/setup.bash`
   - Spustit `full_e2e_mission` a `mapping_mission` scenare, overit clean
     start/stop.
   - Vsechny existujici testy musi projit.

**DULEZITE OMEZENI:**
- Nesmi se rozbit Phase 1 / Phase 2 / Phase 3 funkcionalita.
- Pokud audit ukaze ze direct path je nekde stale referenced bez snadne
  cesty pryc, ZAPISI to jako follow-up TODO a NEMAZE silou.
- Bridge protokol verze v1.2 — nemenit (Phase 4 territorium).

---

## Spolecna pravidla pro vsechny session

1. **Single flight owner:** `obstacle_avoidance_runtime` je jediny PX4
   setpoint publisher. Zadna session nesmi pridat dalsiho.

2. **TelemetryHub source of truth:** Per-drone topicy registrovat v
   `TelemetryHub`. Field-level / mission-level (`/swarm/*`, `/field/*`)
   mohou zustat jako stringy.

3. **QoS kompatibilita:** Nemenit existujici QoS profily. Nove topicy:
   `QOS_VOL` pro ephemeral, `QOS_LATCHED` pro stateful.

4. **JSON zpetna kompatibilita:** Vsechny nove JSON formaty obsahuji pole
   `"version": 1`. Cteni stareho payloadu bez chyby (defaulty).

5. **NED souradnice:** Vsechno v NED. Z je down (vyska = zaporne cislo).

6. **Paths:** Pouzivat `scout_control.utils.paths`. Pridat nove konstanty
   tam, ne hardcoded stringy.

7. **Testy:** Kazda session musi mit unit testy. pytest.
   `PYTHONPATH=src/scout_control pytest src/scout_control/test/test_<name>.py`

8. **Zadne nove dependencies:** Pouzit standardni knihovnu + numpy +
   opencv-python + cv_bridge + ROS2. Zadne pip nebo apt instaly.

9. **Kod anglicky** (comments, variables, docstrings, commit messages).

10. **Pred editaci si soubor VZDY precti.** Nepredpokladat obsah.

11. **Po dokonceni session aktualizovat status [todo] → [done]** v tomto
    promptu a v `docs/plans/scout_ws_e2e_architecture.md` odpovidajici
    checkbox.

---

## Akceptacni kriteria Phase 3

- [ ] Mapping mission projde nad polygon polem s 1+ drony, vse pres
      avoidance runtime, bez PX4 conflictu.
- [ ] `perimeters/field_model/manifest.json` obsahuje validni heightmap +
      obstacles entry po dokoncene mapping misi.
- [ ] Precision landing detector vraci stabilni offset na ArUco markeru
      v Gazebo SITL (nemusi byt zatim integrovany do runtime — advisory ok).
- [ ] `navigation_backend=direct` odstranen ze `swarm_agent` a launch
      souboru. Build zelena, vsechny testy prochazeji.
- [ ] `docs/plans/scout_ws_e2e_architecture.md` Phase 3 [DONE].
- [ ] CLAUDE.md aktualni (direct backend reference odstraneny).
