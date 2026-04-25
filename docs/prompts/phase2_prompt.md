# Phase 2 — Boundary to Base Grid Workflow

## Kontext

Phase 1 (DONE) stabilizovala onboard runtime: `obstacle_avoidance_runtime` je
single flight owner, `swarm_agent` je mission delegator, topic contracts jsou
centralizovane v `TelemetryHub`. Vsechny zmeny Phase 2 musi zachovat tuto
architekturu a nesmí narusit beh existujicich nodu.

Phase 2 finalizuje **pre-mission setup workflow**: od manualni boundary capture
pres home pad registraci az po generovani base gridu a overeni full E2E mise.

---

## Phase 2 — Prehled zmen

### 2A — Home Pad Registration s Metadata
Rozsireni `home_manager` a `field_setup_coordinator` o pad metadata: pad ID,
orientace, charging capability, occupancy state, service priority.

### 2B — Manual Boundary Capture Finalizace
Nahrazeni 4-corner bounding boxu za polygon boundary capture. Operator leti po
obvodu pole a body se zaznamenavaji jako polygon. Safety inset buffer pro
operacni cesty.

### 2C — Grid Generation z Polygonu
`grid_generator` a `field_setup_coordinator` musi umet generovat grid
z polygon boundary (ne jen bounding box ze 4 rohu). Bunky oznacit jako
inside/outside/edge/restricted. Ulozit `field_boundary.json`.

### 2D — Full E2E Swarm Verification
Overeni kompletniho E2E flow: setup → boundary → grid → mission start →
swarm execution → RTH → reporting. Fix launch/config pro default
`navigation_backend=avoidance_runtime` ve vsech scenarich.

---

## Rozdeleni na paralelni agenty (sessions)

Prace je rozdelena na **3 nezavisle paralelni session**. Kazda session ma jasne
ohraniceny scope souboru a topic kontraktu, takze si navzajem nelezou do cesty.

---

### SESSION A — Home Pad Metadata & Registration [done]

**Scope:** Rozsireni pad modelu o metadata a zlepseni pad lifecycle.

**Soubory k editaci:**
- `src/scout_control/scout_control/core/home_manager.py`
- `src/scout_control/scout_control/avoidance/telemetry_hub.py`
  (pouze `SwarmTopicContract` — pridani novych pad-related topicu, pokud potreba)

**Soubory k cteni (read-only, nesmis editovat):**
- `src/scout_control/scout_control/core/field_setup_coordinator.py` (read-only pro pochopeni pad flow)
- `src/scout_control/scout_control/manual/field_setup_tool.py` (read-only)
- `src/scout_control/scout_control/core/swarm_agent.py` (read-only — konzumuje RTH target)
- `CLAUDE.md`, `docs/plans/scout_ws_e2e_architecture.md`

**Co udelat:**

1. **Rozsirit pad datovy model** v `home_manager.py`:
   - Kazdy pad musi mit: `pad_id`, `drone_id`, `ned` (x/y/z), `status`
     (available/occupied/charging/maintenance), `charging_capable` (bool),
     `orientation_deg` (float, heading), `service_priority` (int, 0=highest),
     `allowed_drone_classes` (list[str], default `["*"]`).
   - Zpetne kompatibilni: stare `home_positions.json` bez novych poli musi
     nacist bez chyby (defaultni hodnoty).

2. **Occupancy state machine:**
   - `available` → `occupied` (RTH request prijat)
   - `occupied` → `charging` (landed confirmation + pad je charging_capable)
   - `charging` → `available` (charge complete / manual release)
   - `occupied` → `available` (landed confirmation + pad neni charging_capable)
   - `*` → `maintenance` (manual override)
   - Stav publikovat v existujicim `/swarm/home_positions` payloadu.

3. **Pad query topic** (optional, pokud jednoduchy):
   - `/swarm/pad_query` (sub) — request: `{"drone_id":"drone_0","reason":"low_battery"}`
   - `/swarm/pad_response` (pub) — response: `{"drone_id":"drone_0","pad_id":"pad_1","ned":{...}}`
   - Alokace: nejblizsi volny pad s `charging_capable=true` (pokud reason=low_battery).

4. **Testy:**
   - `test/test_home_manager.py` — unit testy pro pad state machine, zpetnou
     kompatibilitu, alokaci.

**DULEZITE OMEZENI:**
- NEMEN `field_setup_coordinator.py`, `field_setup_tool.py`, `manual_controller.py`,
  `swarm_agent.py`, ani zadne `avoidance/` moduly krome `telemetry_hub.py`.
- Pad assignment topic `/swarm/pad_assignment` payload format musi zustat zpetne
  kompatibilni (nove pole jsou optional).
- `home_positions.json` musi byt zpetne kompatibilni.
- Vsechny QoS profily zachovat beze zmeny.

---

### SESSION B — Polygon Boundary Capture & Grid from Polygon [done]

**Scope:** Nahrazeni 4-corner capture za polygon + grid generation z polygonu.

**Soubory k editaci:**
- `src/scout_control/scout_control/core/field_setup_coordinator.py`
- `src/scout_control/scout_control/utils/grid_generator.py`
- `src/scout_control/scout_control/manual/field_setup_tool.py`

**Soubory k cteni (read-only, nesmis editovat):**
- `src/scout_control/scout_control/core/home_manager.py` (read-only)
- `src/scout_control/scout_control/avoidance/telemetry_hub.py` (read-only)
- `src/scout_control/scout_control/core/swarm_agent.py` (read-only)
- `src/scout_control/scout_control/utils/paths.py` (read-only — pouzij existujici konstanty)
- `CLAUDE.md`, `docs/plans/scout_ws_e2e_architecture.md`

**Co udelat:**

1. **Polygon boundary capture v `field_setup_coordinator`:**
   - Nahradit `REQUIRED_CORNERS = {"NE","NW","SE","SW"}` za dynamicky seznam
     boundary bodu (polygon).
   - Novy topic `/field/boundary_point` (String JSON):
     `{"index": 0, "ned": {"x": ..., "y": ..., "z": ...}, "type": "vertex"}`
   - Operator leti po obvodu, tiskne tlacitko pro kazdy bod.
   - `/field/boundary_close` (String JSON) — uzavre polygon.
   - State machine upravit:
     `IDLE → ASSIGN_PADS → CAPTURE_BOUNDARY → GENERATE_GRID → WAITING_FOR_LANDING → READY_FOR_MISSION`
   - **Zpetna kompatibilita:** Starou 4-corner cestu ponechat jako fallback.
     Pokud prijdou 4 cornery pres `/field/corner_marked`, prepnout do legacy
     rezimu a pouzit puvodni bounding-box grid.
   - Ulozit boundary jako `perimeters/field_boundary.json`:
     ```json
     {
       "vertices_ned": [{"x":...,"y":...,"z":...}, ...],
       "closed": true,
       "inset_buffer_m": 1.0,
       "capture_mode": "polygon"
     }
     ```

2. **Safety inset buffer:**
   - Parametr `boundary_inset_m` (default `1.0` m).
   - Pred generovanim gridu polygon zmensi o inset buffer (offsetting dovnitr).
   - Jednoducha implementace: pro kazdy segment posunout dovnitr o buffer
     vzdalenost. Netreba presnou Minkowski sum — staci priblizeni pro konvexni
     a mirne nekonvexni polygony.

3. **Grid generation z polygonu v `grid_generator.py`:**
   - Pridat rezim `boundary_mode` (vedle existujiciho `sim_mode` a perimeter mode).
   - Nacist `field_boundary.json`, vypocitat bounding box polygonu.
   - Vygenerovat grid pres bounding box, ale kazdou bunku klasifikovat:
     - `inside` — stred bunky je uvnitr polygonu (point-in-polygon test)
     - `edge` — stred uvnitr, ale bunka presahuje boundary
     - `outside` — stred mimo polygon → nevkládat do `cells` listu
   - Point-in-polygon: ray casting algoritmus (jednoduchy, spolehlovy).
   - Ulozit do `field_grid.json` s pridanym polem `"cell_class"` u kazde bunky.

4. **Aktualizovat `field_setup_tool.py`:**
   - Pridat tlacitko `B` — mark boundary point (publikuje `/field/boundary_point`).
   - Pridat tlacitko `F` — close boundary (publikuje `/field/boundary_close`).
   - Zobrazit pocet boundary bodu v curses UI.
   - Zachovat puvodni `C` (corner marking) pro zpetnou kompatibilitu.

5. **Testy:**
   - `test/test_grid_from_polygon.py` — unit testy pro point-in-polygon,
     inset buffer, klasifikaci bunek.
   - `test/test_boundary_capture.py` — unit testy pro polygon state machine.

**DULEZITE OMEZENI:**
- NEMEN `home_manager.py` (to je scope Session A).
- NEMEN zadne `avoidance/` moduly, `swarm_agent.py`, `swarm_coordinator.py`.
- Zachovat zpetnou kompatibilitu: 4-corner flow musi stale fungovat.
- `field_grid.json` format musi byt zpetne kompatibilni — existujici bunky
  bez `cell_class` pole jsou implicitne `inside`.
- Vsechny nove topicy registrovat v `TelemetryHub` jen pokud to je nutne
  (field-level topicy jako `/field/*` nemusi byt v per-drone contractu).
- Pouzivej `scout_control.utils.paths` pro vsechny file paths.

---

### SESSION C — E2E Verification & Launch/Config Cleanup [done]

**Scope:** Overeni kompletniho E2E flow, fix launch/config, cleanup.

**Soubory k editaci:**
- `src/scout_control/launch/full_e2e_mission.launch.py`
- `src/scout_control/launch/isaac_e2e_mission.launch.py`
- `scenarios/*.yaml` (kde je treba opravit paths, backend defaults)
- `src/scout_control/setup.py` (registrace novych entry points, pokud potreba)
- `src/scout_control/test/test_e2e_setup_flow.py` (novy)

**Soubory k cteni (read-only, nesmis editovat):**
- Vsechny `core/`, `avoidance/`, `manual/`, `utils/` moduly — jen cist, ne editovat
- `CLAUDE.md`, `docs/plans/scout_ws_e2e_architecture.md`

**Co udelat:**

1. **Audit launch files:**
   - Overit, ze `full_e2e_mission.launch.py` ma vsechny nody s
     `navigation_backend=avoidance_runtime` (uz je, jen overit).
   - Overit, ze `isaac_e2e_mission.launch.py` pouziva stejny pattern.
   - Overit, ze `field_setup_tool` (ne `manual_controller`) je preferovany
     setup node v produkcnich launchich.

2. **Scenario YAML cleanup:**
   - Projit vsechny `scenarios/*.yaml`.
   - Opravit `~/scout_ws/install/setup.bash` → spravna workspace cesta
     (`/home/tj/_Data/_Projekty/TJlabs/scout_ws/install/setup.bash`).
   - Overit, ze vsechny scenare pouzivaji konzistentni parametry.

3. **Entry point registrace v `setup.py`:**
   - Overit, ze vsechny pouzivane nody jsou registrovane jako `console_scripts`.
   - Pokud Session A nebo B pridaly novy node, pridat entry point.
   - Pozor: `manual_controller` je registrovany jako `legacy_manual_controller`?
     Overit a sladit s launchem.

4. **E2E integration test:**
   - Napsat `test/test_e2e_setup_flow.py` — test ktery simuluje cely setup flow:
     - Publikuje pad assignments → overí ze `field_setup_coordinator` prejde do
       `MAP_FIELD`.
     - Publikuje 4 cornery → overí prechod do `GENERATE_GRID`.
     - Overí ze `field_grid.json` byl vytvoren.
     - Overí ze RTH request byl odeslan.
   - Pouzit `launch_testing` nebo standalone ROS2 node test pattern.

5. **Dokumentace:**
   - Aktualizovat `docs/plans/scout_ws_e2e_architecture.md`:
     - Phase 2 checkboxy oznacit jako `[x]` po dokonceni.
     - Pridat nove topicy do node table pokud relevantni.

**DULEZITE OMEZENI:**
- NEMEN zadne core Python moduly (`home_manager.py`,
- Pouze launch files, scenare, setup.py, testy a dokumentace.
- Pokud najdes bug v core modulech, zapiš ho jako TODO komentář v test filu
  nebo report — neopravuj primo.

---

## Diagram zavislosti

```
Session A (home_manager, pad metadata)  ──┐
                                           ├──→  Session C (E2E verification)
Session B (boundary + grid polygon)     ──┘

A a B jsou plne nezavisle — mohou bezet paralelne.
C musi bezet az po dokonceni A a B.
```

## Spolecna pravidla pro vsechny session

1. **Single flight owner:** `obstacle_avoidance_runtime` je jediny PX4 setpoint
   publisher. Zadna session nesmi pridat dalsiho PX4 publishera.

2. **TelemetryHub je source of truth:** Vsechny nove topicy registrovat v
   `TelemetryHub` pokud jsou per-drone. Field-level topicy (`/field/*`,
   `/swarm/*`) mohou zustat jako stringy.

3. **QoS kompatibilita:** Nemenit existujici QoS profily. Nove topicy: pouzit
   QoS_VOL pro ephemeral, QOS_LATCHED pro stateful.

4. **JSON zpetna kompatibilita:** Vsechny JSON soubory a topic payloady musi
   zustat zpetne kompatibilni. Nove pole jsou optional s defaulty.

5. **NED souradnice:** Vsechno v NED. Z je down (vyska = zaporne cislo).

6. **Paths:** Pouzivat `scout_control.utils.paths` pro vsechny workspace paths.

7. **Testy:** Kazda session musi mit unit testy. Pouzit pytest.
   `PYTHONPATH=src/scout_control pytest src/scout_control/test/test_<name>.py`

8. **Nepridavat zavislosti:** Zadne nove pip/apt packages. Pouzit standardni
   knihovnu + ROS2 + to co uz je v repu.

9. **Kod pis v angličtině** (comments, variables, docstrings). Commit messages
   taky anglicky.

10. **Pred editaci souboru si ho VZDY precti.** Nepredpokladej obsah.
