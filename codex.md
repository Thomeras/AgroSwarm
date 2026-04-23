## Codex Log — 2026-04-20

### Kontext

- Resila se Isaac Sim / Pegasus kamera a depth pipeline pro Isaac E2E workflow.
- Cilem bylo dostat RGB + depth do ROS2 bez uprav funkcnich ROS nodek v `scout_control`.
- Ukazalo se, ze nejcistsi cesta neni poustet novou Isaac instanci ze standalone
  Python skriptu, ale poustet helper skript az uvnitr uz bezici Isaac + Pegasus
  session.

### Co bylo upraveno

- `Pegasus_scenarios/simulation_cam.py`
  - puvodne z nej byl pokus o standalone launcher nove Isaac appky
  - to se ukazalo jako slepa cesta:
    - jina instance
    - jina extension state
    - chyby kolem `rclpy`, `ROS2Backend`, world lifecycle a UI
  - soubor byl nakonec prepsan na in-session helper skript
  - finalni chovani:
    - nevytvari `SimulationApp`
    - nenacita world
    - nespawnuje dron
    - jen najde existujici kamerovy prim na uz rucne nactenem Pegasus dronu
      a pripoji ROS2 publishery
  - publikuje:
    - `/drone_0/camera/image_raw`
    - `/drone_0/depth/image_raw`
  - script je urceny ke spusteni z Isaac `Window > Script Editor`
  - quick run:

```python
exec(open("/home/tj/_Data/_Projekty/TJlabs/scout_ws/Pegasus_scenarios/simulation_cam.py").read())
```

- `launch_files/launch_sim_e2e.txt`
  - dokumentace byla narovnana na realne funkcni workflow
  - Isaac se spousti rucne v ROS2-ready envu
  - world i dron se nacitaji rucne pres Pegasus UI
  - `simulation_cam.py` se pousti az potom ze Script Editoru uvnitr bezici session
  - doplnena diagnostika k duplicitnim publisherum (`Publisher count: 2`)

- `CLAUDE.md`
  - dopsan aktualni, overeny Isaac/Pegasus camera workflow

### Ověřené skutečnosti

- V aktualnim Isaac workflow po rucnim nacteni sveta a dronu a po spusteni
  `simulation_cam.py` uvnitr Isaac session existuji topicy:

```text
/drone_0/camera/image_raw
/drone_0/depth/image_raw
```

- `ros2 topic info` ukazal publishery z Isaac/Replicator pipeline.
- Pokud se helper skript spusti dvakrat v jedne session, vzniknou duplicitni
  publishery a `Publisher count` bude `2`.
- `camera_info` neni v aktualnim workflow povinny a muze se lisit podle
  konkretniho Isaac build/runtime helperu.

### Důležité závěry

- Problem nebyl v `gcs_bridge`, ale v tom, ze Isaac/Pegasus session puvodne
  nepublikovala kameru/depth na kompatibilni ROS2 topicy.
- Pro tenhle projekt se nema pouzivat standalone `simulation_cam.py` jako
  launcher nove Isaac instance.
- Spravna cesta je:
  1. spustit Isaac normalne
  2. rucne nacist world
  3. rucne nacist dron pres Pegasus UI
  4. pustit `simulation_cam.py` uvnitr bezici session

### Co zůstává důležité při další práci

- Nepredelavat znovu `simulation_cam.py` na launcher nove appky, pokud k tomu
  neni velmi silny duvod.
- Pri dalsi praci v Isaac vetvi pocitat s tim, ze `simulation_cam.py` je
  session helper, ne bootstrap cele simulace.
- Pokud nekdo hlasi `Publisher count: 2`, prvni podezreni je dvojite spusteni
  helper skriptu v jedne Isaac session.

## Codex Log — 2026-04-21

### Kontext

- Resilo se testovani obstacle avoidance pres Isaac depth kameru a obstacle
  course.
- Uzivatel predtim opravil bug kolem obstacle course spawn/world setup.
- Cilem bylo overit, jestli avoidance mise opravdu ridi podle kamery, proc
  leti velmi pomalu, a pridat robustnejsi debug logy.

### Co bylo upraveno

- `launch_files/launch_sim_e2e.txt`
  - na konec byla dopsana sekce pro samostatny obstacle-avoidance test v
    Isaac vetvi
  - workflow byl doplnen o `obstacle_detector`, `obstacle_avoidance_mission`,
    `obstacle_viz` a `gimbal_cam_viz`
  - behem dalsiho overeni se potvrdilo, ze pokud je otevreny primo
    `worlds/obstacle_course.usda`, tak `obstacle_course_spawn.py` uz neni treba

- `src/scout_control/scout_control/avoidance_logging.py`
  - pridan jednoduchy JSONL logger pro obstacle test behy
  - logy se ukladaji do `logs/avoidance_logs/`

- `src/scout_control/scout_control/obstacle_detector.py`
  - doplneno per-run logovani:
    - start konfigurace
    - prvni validni pozice
    - stav detectoru
    - stats z depth framu
    - chyby `CvBridge`
    - decay udalosti occupancy gridu

- `src/scout_control/scout_control/obstacle_avoidance_mission.py`
  - opraven crash v loggeru (`mission_name` se predavalo dvakrat)
  - mission uz mezi obstacle runy nedela mezilehly navrat domu
  - home pozice se uklada z realne pozice pred takeoffem a pouziva se jen pro
    finalni land/navrat
  - `_step_toward()` uz nepocita dalsi setpoint ciste od predchoziho setpointu,
    ale od aktualni pozice dronu, pokud je k dispozici
  - pridano logovani phase transition, obstacle update, vehicle commandu a
    mission statusu
  - pri critical obstacle byl doplnen fallback detour i pro wall-like situace:
    kdyz neni zadny sektor uplne `free`, mise si vybere lepsi stranu podle
    vetsi lateralni clearance

- `logs/avoidance_logs/.gitkeep`
  - vytvorena explicitni slozka pro obstacle logy v repu

### Co bylo overene z logu

- Avoidance mise neni "cheating" v tom smyslu, ze by cetla fixni mapu prekazek
  pro rizeni:
  - `obstacle_avoidance_mission.py` ridi jen podle
    `/drone_0/obstacles/detected`
  - ten je odvozen z depth kamery v `obstacle_detector.py`
  - fixni `OBSTACLES` seznam v `obstacle_viz.py` slouzi jen pro vizualizaci

- V testu `North Wall` se ukazaly dve hlavni runtime slabiny:
  1. puvodni mezilehly `RTH_HOME` vedl dron zpet skrz prekazky
  2. po oprave RTH vetve je mise stale velmi pomala

- Duvod pomaleho letu neni jedna jedina chyba, ale kombinace:
  - mission bezi v position-setpoint rezimu a generuje jen maly posun na tick
  - pri priblizeni k waypointu se rychlost dale snizuje pres `min(speed, d)`
  - v `warn` zone se navic pouziva `WARN_SLOWDOWN = 0.5`
  - realna ground speed tak byla v logu o dost mensi nez nominalni
    `cruise_speed:=2.5`

- Posledni test (logy `20260421_211901_*` a `20260421_212004_*`) ukazal:
  - mise dosla k prvnimu `North Wall` runu
  - critical avoidance vyrobila detour a probehl prechod `APPROACH -> AVOIDING`
  - po dosazeni detour waypointu se mise vratila do `APPROACH`
  - nasledne dron zustal v `warn` zone kolem `closest ~= 2.21 m`
  - `critical` uz se znovu nespustil, protoze `stop_distance` zustava `2.0 m`
  - `free_directions` byly prazdne, takze mise pred zdi prakticky ustrnula

### Dulezite zavery

- Isaac obstacle-avoidance vetev je dnes dobra pro:
  - validaci camera/depth pipeline
  - ladeni reaktivni obstacle logiky
  - sbirani logu a reprodukovatelnych failu

- Neni jeste dobra pro:
  - robustni oblet dlouhych sten
  - "real-world ready" obstacle avoidance bez dalsi lokalni pameti/mapovani

- Pokud nekdo hlasi "dron leti hrozne pomalu", prvni vec ke kontrole jsou:
  - `mission_status` v `logs/avoidance_logs/*obstacle_avoidance_mission*.jsonl`
  - rozdil mezi `drone_ned` a `setpoint_ned`
  - jestli mise neni v `warn` creep rezimu nebo tesne pred waypointem

### Co zustava jako dalsi rozumna prace

- zvazit agresivnejsi position guidance, pokud ma mission pusobit realisticky
  i bez planneru
- rozhodnout, jestli se ma v `warn` zone:
  - drzet pomaly creep
  - nebo jit drive do aktivniho bočniho detour rezimu
- edge-following "z dalky" zatim neimplementovat; bez lokalni pameti/slamu by to
  bylo krehke a silne heuristicke

## Codex Log — 2026-04-22

### Kontext

- Uzivatel chtel znovu projit cely projekt a srovnat AI instrukcni soubory se
  skutecnym stavem repo.
- Mezi 2026-04-20 a 2026-04-22 pribyla nova obstacle-avoidance runtime vetev,
  testy, relokovana dokumentace do `docs/` a runbooky do `launch_files/`.
- Cilem nebylo menit runtime logiku, ale opravit `CLAUDE.md` a `codex.md`, aby
  dalsi session nevychazely ze zastaralych predpokladu.

### Co bylo overeno primo v kodu

- `src/scout_control/setup.py` dnes registruje mimo jine:
  - `obstacle_avoidance_runtime`
  - `obstacle_avoidance_mission`
  - `scan_cloud_viz`
  - `gcs_bridge`
- `task_allocator.py` stale neni registrovany jako `console_script`; scenar
  `task_allocator.yaml` je proto stale podezrely.
- `full_e2e_mission.launch.py` zustava swarm-agent-centric launch:
  - setup + mission nody pro full swarm misi
  - `gcs_bridge` se spousti automaticky
  - `manual_controller` se zamerne spousti bokem kvuli TTY
- `isaac_e2e_mission.launch.py` uz pocita s topic templaty pro kameru a depth:
  - `camera_topic_template`
  - `depth_topic_template`
  - default topicy odpovidaji `Pegasus_scenarios/simulation_cam.py`
- nova avoidance architektura je uz fyzicky pritomna v:
  - `src/scout_control/scout_control/avoidance/depth_projector.py`
  - `src/scout_control/scout_control/avoidance/local_mapper.py`
  - `src/scout_control/scout_control/avoidance/local_planner.py`
  - `src/scout_control/scout_control/avoidance/scan_manager.py`
  - `src/scout_control/scout_control/avoidance/peer_tracks.py`
  - `src/scout_control/scout_control/avoidance/types.py`
- `obstacle_avoidance_runtime.py` uz dnes opravdu dela vic nez jen reaktivni
  stopku:
  - lokalni mapovani
  - planner integration
  - peer drone safety masky
  - status / path / subgoal publikaci
- existuji uz focused testy pro avoidance helpery a planner:
  - `src/scout_control/test/test_local_planner.py`
  - `src/scout_control/test/test_avoidance_helpers.py`
- `paths.py` stale hleda workspace root podle pritomnosti `CLAUDE.md` a drzi
  fallback na `~/scout_ws`.
- bridge protokol je stale duplikovany ve dvou kopiich a verze zustava `1.2`
  s podporou `camera_frame` a `depth_frame`.

### Co bylo upraveno

- `CLAUDE.md`
  - prepsan na aktualni mapu projektu k 2026-04-22
  - oddeleny production core, runtime-centric avoidance branch a debug/tooling
    nody
  - doplneny realne launch cesty a roli jednotlivych souboru
  - dopsany aktualni Isaac camera/depth workflow
  - dopsany obstacle-avoidance runtime test harness workflow
  - zapsany aktualni path / data / QoS / bridge konvence
  - zvyrazneny stale otevreny rozdil mezi full swarm misi a novou avoidance
    runtime architekturou

- `codex.md`
  - doplnen tento auditni zaznam z 2026-04-22

### Dulezite zavery pro dalsi praci

- Nepredpokladat, ze `obstacle_avoidance_runtime` uz nahradil `swarm_agent`
  v plne E2E misi. Zatim jde o vedlejsi, ale uz dost realnou runtime vetev.
- Pri dalsich upravach obstacle avoidance drzet oddeleni:
  - `obstacle_avoidance_runtime` = flight-control owner
  - `obstacle_avoidance_mission` = route provider / test harness
  - `obstacle_detector` = debug / srovnavaci node
- Pri upravach cest preferovat `scout_control.paths`, ale pocitat s tim, ze
  cast launcheru a scenaru stale obsahuje historicke `~/scout_ws` reference.
- Pokud se meni bridge wire format, musi zustat synchronni:
  - `src/scout_control/scout_control/bridge_protocol.py`
  - `swarm_center/core/bridge_protocol.py`

## Codex Log — 2026-04-22 (A/B/C/D merge)

### Kontext

- Uzivatel chtel paralelni realizaci 4 workstreamu:
  - A: core avoidance runtime
  - B: swarm integrace
  - C: peer/reassign/GCS
  - D: typed payloady + cleanup + tuning
- Cilem bylo dotahnout runtime-centric avoidance flow tak, aby
  `obstacle_avoidance_runtime` byl jedinym flight ownerem.

### Co bylo finalne zapojeno

- Runtime:
  - stabilizovane faze `CRUISE_TO_TARGET`, `WARN_DRIFT`, `STOP_HOVER`,
    `SCAN_360`, `LOCAL_REPLAN`, `DETOUR_EXECUTION`, `BLOCKED`
  - scan enrichment je skutecne navazan do mapperu a ovlivnuje nasledujici
    replan
  - `scan_complete(success=True)` vede do replanu
  - `scan_complete(success=False)` / repeated no-path eskaluje do `BLOCKED`
- Swarm:
  - `swarm_agent` ma `navigation_backend=direct|avoidance_runtime`, default je
    `avoidance_runtime`
  - v runtime backendu je direct PX4 ownership path vypnuta (zadne PX4
    pubs/subs/timer/control loop)
  - `swarm_agent` posila runtime high-level `target_cmd` a `CELL_COMPLETE`
    odvozuje z runtime `last_completed_target_id`
  - finalni mapping `/{drone}/avoidance/status` -> `/swarm/drone_status`
- Allocator / reassign:
  - `blocked_severity` sjednoceno na `NONE|SOFT|HARD`
  - zavedeno `CELL_DEFERRED` + `TEMP_BLOCKED` cooldown + deferred queue
- GCS:
  - bridge forwarduje `AVOIDANCE_STATUS` a `AVOIDANCE_EVENT` jako subtype
    payloady pres `MSG_DRONE_STATUS`
- Typed + cleanup:
  - typed parsery avoidance/swarm status payloadu
  - legacy `/obstacle_avoidance/*` publikace je gateovana parametrem
    `publish_legacy_obstacle_topics` (default `false`)
  - runtime command ingestion rozsiren o command aliasy, envelope payloady a
    target aliasy
  - runtime emituje `command_accepted` / `command_rejected` eventy
  - runtime status ma aditivni ownership/mission feedback pole pro mission layer

### Overy

- Lokalni test batch po integraci:
  - `test_local_planner.py`
  - `test_local_mapper_scan_pipeline.py`
  - `test_task_allocator.py`
  - `test_typed_status_payloads.py`
  - `test_avoidance_helpers.py`
- Vysledek: `23 passed`

### Otevrene body

- Chybi plny E2E ROS/PX4 launch-level overeni celeho runtime backend flow.
- GCS consumeri musi filtrovat `status` subtype (`AVOIDANCE_STATUS`,
  `AVOIDANCE_EVENT`) v `MSG_DRONE_STATUS`.
- Po prekroceni `max_deferrals_per_cell` je bunka zmrazena v deferred stavu bez
  samostatneho alert topicu.

## Codex Log — 2026-04-22 (Swarm Agent Ownership Refactor Finalized)

### Finalni smer a stav

- `swarm_agent` se prevadi na mission executor/route provider nad
  `obstacle_avoidance_runtime`.
- V teto path nema `swarm_agent` byt PX4 flight-control owner a nema publikovat
  low-level flight setpointy.
- `obstacle_avoidance_runtime` je produkcni flight-control owner pro execution,
  avoidance rozhodovani a navazne replany.

### Potvrzene implementacni body

- `swarm_agent` default backend je `navigation_backend=avoidance_runtime`.
- V runtime mode je direct PX4 ownership path v `swarm_agent` vypnuta:
  - zadne PX4 pubs/subs
  - zadny direct control timer/loop
- `swarm_agent` pouziva runtime command topic (`target_cmd`) a completion
  vyhodnocuje z `last_completed_target_id`.
- Runtime command ingestion podporuje:
  - command aliasy
  - envelope payload shape
  - target aliasy
- Runtime emituje command lifecycle eventy:
  - `command_accepted`
  - `command_rejected`
- Runtime status je rozsireny o aditivni ownership/mission feedback pole pro
  nadrazenou mission vrstvu.

### Co je stale otevrene

- Doladeni launch/config defaultu tak, aby runtime backend byl konzistentni
  default i mimo test harness.
- Finalni odstraneni stare direct-control cesty az po E2E overeni v plne swarm
  misi.

## Codex Log — 2026-04-23 (Phase 1 dokončení)

### Kontext

- Cílem bylo uzavřít Phase 1 z roadmapy `scout_ws_e2e_architecture.docx`:
  "Stable onboard runtime — swarm_agent jako čistý delegátor,
  obstacle_avoidance_runtime jako jediný flight owner."

### Co bylo provedeno

- `obstacle_avoidance_runtime.py`
  - přidána `_normalize_target_command_payload()` — normalizace aliasů, envelope
    payloadů a target aliasů příchozích příkazů
  - přidána `_publish_command_feedback()` — emituje `command_accepted` /
    `command_rejected` events do runtime event streamu
  - `_target_cmd_cb()` přepsán na `TargetCommand.from_payload()` pipeline
  - přidán `_last_completed_target_ts` — timestamp dokončení targetu
  - status payload rozšířen o: `mission_feedback_state`, `command_active`,
    `target_reached` (3s okno), `flight_control_owner`, `execution_owner`

- `swarm_agent.py`
  - default backend změněn na `navigation_backend=avoidance_runtime`
  - PX4 publishers/subscribers gateovány za `not self._runtime_backend_active`
  - timer `_timer_cb` spouštěn pouze v direct backend módu
  - přidán `_target_cmd_pub` pro high-level příkazy do runtime
  - přidány runtime stavové proměnné:
    `_runtime_active_cell_id`, `_runtime_last_completed_target_id`,
    `_runtime_rth_requested`, `_runtime_return_home_sent`
  - přidány metody:
    `_maybe_build_runtime_cmd_locked()`, `_publish_runtime_cmd()`,
    `_build_runtime_return_home_cmd_locked()`
  - `CELL_COMPLETE` odvozován z `last_completed_target_id` v runtime statusu
  - RTH callback posílá runtime command místo přímého PX4 řízení
  - `mission_ready` callback resetuje runtime stav
  - guardy pro `None` publishers (`_pub_offboard`, `_pub_setpoint`, `_send_command`)

- `full_e2e_mission.launch.py`
  - přidány `avoidance_runtime_0` a `avoidance_runtime_1` nodes (spouštěny dříve
    než swarm agents, s 1s zpoždením)
  - `swarm_agent_0` a `swarm_agent_1` mají explicitně nastaveno
    `navigation_backend: avoidance_runtime`
  - doc string aktualizován — popsány runtimes jako flight owners

- `isaac_e2e_mission.launch.py`
  - stejné přidání `avoidance_runtime_0` a `avoidance_runtime_1`
  - `swarm_agent_0` a `swarm_agent_1` s `navigation_backend: avoidance_runtime`
  - přidána poznámka o depth camera závilosti na `simulation_cam.py`

### Testy

- `23 passed` (test_local_planner, test_avoidance_helpers, test_task_allocator,
  test_typed_status_payloads, test_local_mapper_scan_pipeline)

### Stav Phase 1

Phase 1 je architektonicky kompletní:
- `obstacle_avoidance_runtime` = jediný flight owner ve všech launch scénářích
- `swarm_agent` = čistý delegátor bez PX4 ownership v runtime backend módu
- direct-control path v `swarm_agent` zachována za backend gate pro přechodovou
  kompatibilitu — odstraní se po plném E2E ověření

### Co zůstává otevřené pro Phase 2

- Plné live E2E ověření v Gazebo nebo Isaac Sim s runtime backendem
- Finální odstranění direct-control path po E2E ověření
- Boundary capture workflow a home pad metadata (Phase 2)

## Codex Log — 2026-04-24 (Phase 1 Final — Architecture & Safety)

### Kontext
- Finalizace Phase 1: "Stable onboard runtime".
- Vyřešení zbývajících findingů F1-F15 (topic kontrakty, legacy cleanup, health gates).

### Co bylo provedeno
- **`telemetry_hub.py`**: Vytvořen centrální registr všech ROS2 topiců. Všechny nody přepojeny na tento hub.
- **`RuntimeHealthMonitor`**: Implementováno hlídání EKF zdraví (heading, reset counters, dead reckoning) a stale sensor dat. Runtime nyní bezpečně gateuje setpointy.
- **`LocalPlanner`**: Přidána pojistka proti plánování nad nevalidní/prázdnou mapou.
- **Legacy Cleanup**:
  - `setup.py` už neinstaluje legacy flight nody (`offboard_control`, `terrain_follower`, atd.).
  - Scénáře YAML přepnuty na "archived" notice.
- **Isaac Sim Multi-drone**:
  - `isaac_e2e_mission.launch.py` nyní korektně spawnuje runtime/agent nody jen pro existující drony.
- **Typed Readiness**: Opraven parsing nested readiness payloadů (depth.ready, depth.age_s).

### Výsledky
- **Phase 1 DONE**: `obstacle_avoidance_runtime` je jediný flight owner.
- **TelemetryHub**: Jediné místo pro správu topiců a HW kontraktů.
- **Bezpečnost**: Systém rozpozná výpadek sensorů/EKF a zastaví navigaci místo havárie.

### Co dál (Phase 2)
- Live E2E swarm mise v simulaci pro ověření souhry všech nových komponent.
- Nástroje pro sběr perimetru a metadata pro home pady.
- Postupné mazání `navigation_backend=direct` kódu ze `swarm_agent`.
