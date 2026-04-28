# SCOUT WS — Multi-Agent Completion Prompt

Pouzij tento prompt pro koordinovany multi-agent dobeh historickych auditnich
bodu pri respektovani architektury v `docs/plans/scout_ws_e2e_architecture.docx`,
`CLAUDE.md`, `codex.md` a `memory.md`.

## Globalni zadani pro vsechny agenty

Pracujete v ROS2 Jazzy workspace `scout_ws` pro autonomni agro-droni roj nad
PX4 SITL. Cilem neni prepsat system, ale opatrne dodelat otevrene body z
node auditu bez rozbiti produkcni flow.

Zakladni architektonicke pilire:

- `obstacle_avoidance_runtime` je jediny autonomous flight-control owner.
- `swarm_agent` a dalsi mission/route provideri posilaji jen high-level targety
  do `/{drone_ns}/avoidance/target_cmd`; nepublikuji PX4 setpointy.
- Swarm vrstva prideluje praci, resi blocked/deferred cells a pady; nedela
  heavy central path planning.
- Runtime vrstva resi bezpecne provedeni: warn drift, stop-hover, scan,
  local replan, blocked, return-home a landing.
- Tooling/manual/legacy nody nesmi byt potichu vraceny do produkcni flight path.
- Topic naming a QoS drzte pres `scout_control.avoidance.telemetry_hub`, pokud
  to konkretni zmena umoznuje.
- PX4 souradnice jsou NED: `z` je down, vyska nad zemi je typicky zaporne `z`.
- Zmeny v bridge protokolu musi zustat synchronizovane mezi:
  - `src/scout_control/scout_control/utils/bridge_protocol.py`
  - `swarm_center/core/bridge_protocol.py`
- Uprednostnete kompatibilni, inkrementalni migrace. Pokud menite topic payload,
  pridejte adapter nebo dual-publish/subscriber prechod, dokud nejsou vsechny
  consumers aktualizovane.
- Nevracejte cizi zmeny ve worktree. Upravujte jen vlastni scope.

Spolecna verifikace po integraci:

```bash
PYTHONPATH=src/scout_control pytest src/scout_control/test/
colcon build --packages-select scout_control
```

Pokud full ROS/PX4 E2E neni dostupne, minimalne dolozte, ktere unit/integration
testy prosly a co zustava pouze launch-level riziko.

## Prioritizovany backlog

### P0

1. Ztencit `obstacle_avoidance_runtime` z god objectu na orchestrator.
   Oddelit zejmena flight phase machine, PX4 publishing adapter a ROS IO hranice.

2. Pripravit typed messages/adapters pro core kontrakt topicy, dnes stale
   JSON-over-String:
   - `/swarm/drone_status`
   - `/swarm/task_status`
   - `/{drone_ns}/avoidance/target_cmd`
   - `/{drone_ns}/avoidance/status`
   - `/swarm/pad_assignment`
   - `/field/setup_complete`
   - `/swarm/rth_request`
   - `/swarm/mission_ready`

### P1

3. Parametrizovat `cell_data_recorder`, aby nemel hardcoded `drone_0` a
   `drone_1`. Pridat `drone_count` a topic template/TelemetryHub integraci.

4. Oddelit production vs operator/tooling launch flow. Dokumentacne je smer
   hotovy, ale launch soubory maji byt citelne a nesmi nechtene tahat manual
   nebo legacy nody do produkcni cesty.

### P2

5. Opravit lifecycle `grid_generator`: prace nema bezet v `__init__`, ale v
   explicitni `run()`/callback metode.

6. Zalozit centralni dokumentaci topic kontraktu: naming, QoS, durability,
   compatibility policy a migration rules pro typed payloady.

7. Vyresit `mission_launcher` lifecycle: neukoncovat node pres `SystemExit` z
   timeru; pouzit cisty ROS lifecycle/shutdown signal.

8. Zkontrolovat `spray_controller`, protoze prepisuje cely log per event.
   Navrhnout nebo implementovat bezpecnejsi append/atomic persistence podle
   existujicich projektovych vzoru.

9. Oznacit `ml_interface` jako stub/tooling placeholder, aby ho dalsi agenti
   nepovazovali za produkcni ML vrstvu.

10. Do budoucna odstranit deprecated `navigation_backend=direct` ze
    `swarm_agent`, ale az po uspesnem E2E overeni runtime backendu.

11. Doresit GCS consumption bridge v1.3 overlay payloadu
    `MSG_NO_GO_OVERLAY` a `MSG_REFINED_GRID_EVENT`, pokud to patri do aktualni
    iterace; jinak dokumentovat jako navazujici GCS workstream.

12. `task_allocator.yaml` neni registrovany v `setup.py`; bud ho opravit, nebo
    explicitne oznacit jako nespustitelny/archivni scenar.

## Agent A — Runtime Decomposition

Scope:

- `src/scout_control/scout_control/core/obstacle_avoidance_runtime.py`
- nove pomocne moduly v `src/scout_control/scout_control/avoidance/` nebo
  `src/scout_control/scout_control/core/`, pokud zapadaji do existujiciho stylu
- targeted tests v `src/scout_control/test/`

Ukol:

- Navrhni a implementuj prvni opatrny refactor, ktery zmensi runtime god object,
  ale nezmeni wire behavior.
- Oddel minimalne jednu jasnou oblast:
  - `FlightPhaseMachine` pro phase transitions a command lifecycle, nebo
  - `PX4PublisherAdapter` pro offboard/setpoint/vehicle command publishing, nebo
  - `RosIOAdapter`/topic boundary pro publishers/subscribers a QoS.
- Zachovej existujici runtime phases a stavove eventy:
  `CRUISE_TO_TARGET`, `WARN_DRIFT`, `STOP_HOVER`, `SCAN_360`,
  `LOCAL_REPLAN`, `DETOUR_EXECUTION`, `BLOCKED`, `LANDING`.
- Zachovej `return_home` jako high-level target, ktery stale prochazi runtime
  safety vrstvou a po dokonceni vede do landing flow.
- Nezavadet zadne PX4 publishery do `swarm_agent`.

Acceptance:

- Existing avoidance tests stale prochazi.
- Novy modul ma focused unit test, pokud obsahuje netrivialni logiku.
- Runtime status payload zustane backward compatible.

## Agent B — Typed Core Contracts

Scope:

- `src/scout_control/scout_control/avoidance/telemetry_hub.py`
- `src/scout_control/scout_control/avoidance/types.py`
- core nody, ktere publish/subscribe uvedene core topics
- testy typed payload/adapters
- dokumentace topic kontraktu

Ukol:

- Zmapuj aktualni JSON-over-String payloady pro P0 topics.
- Navrhni migracni strategii typed kontraktu bez jednorazoveho rozbiti:
  centralni dataclass/parser/adapters, aliasy a kompatibilni JSON fallback.
- Implementuj nejmene jeden uzavreny vertical slice s testem, idealne
  `/{drone_ns}/avoidance/target_cmd` nebo `/{drone_ns}/avoidance/status`.
- Pokud skutecne ROS `.msg` soubory nejsou vhodne pro tuto iteraci, jasne
  oddel "typed Python payload adapter now" od "ROS msg migration later".
- Dokumentuj QoS a durability pro vsechny P0 topicy.

Acceptance:

- Neexistuje consumer, ktery by po zmene prisel o stavajici JSON payload.
- Testy pokryvaji normalizaci aliasu/envelope payloadu a reject path.
- Dokumentace rika, ktery topic je transient/local, reliable/best-effort a proc.

## Agent C — Swarm/Core Hardening

Scope:

- `src/scout_control/scout_control/core/cell_data_recorder.py`
- `src/scout_control/scout_control/core/mission_launcher.py`
- `src/scout_control/scout_control/core/spray_controller.py`
- `src/scout_control/scout_control/core/ml_interface.py`
- relevantni launch/config/test soubory

Ukol:

- Odstran hardcoded `drone_0`/`drone_1` z `cell_data_recorder`.
  Pridej `drone_count` parametr a topic template, idealne pres TelemetryHub.
- Oprav `mission_launcher` tak, aby lifecycle nekoncil pres `SystemExit` z
  timeru.
- Zkontroluj persistence ve `spray_controller`; pokud je rizikove prepisovani
  celeho logu per event, navrhni/implementuj atomic write nebo append model.
- Oznac `ml_interface` jako stub/placebo placeholder v kodu nebo dokumentaci
  bez zavadejici produkcni semantiky.

Acceptance:

- `cell_data_recorder` funguje pro `drone_count=1`, `2` i vyssi pocet bez
  hardcoded seznamu.
- Launch defaults zustanou kompatibilni se stavajicim full E2E flow.
- Pridane testy nepotrebuji PX4/Gazebo/Isaac.

## Agent D — Launch, Tooling Boundaries, Docs

Scope:

- `src/scout_control/launch/*.launch.py`
- `scenarios/*.yaml`
- `docs/`
- `CLAUDE.md`, `codex.md`, pokud se meni dlouhodobe instrukce

Ukol:

- Oddel a pojmenuj production launch flow vs operator/tooling/debug flow.
- Zkontroluj, ze production launch nepousti legacy/debug/manual nody, pokud to
  neni explicitne zamyslene.
- Vytvor centralni topic-contract dokument se seznamem core topicu, QoS,
  payload policy, deprecation policy a TelemetryHub pravidlem.
- Zkontroluj historicke `~/scout_ws` reference ve scenarich/runboocich.
  Preferuj `scout_control.paths` nebo jasne vysvetli kompatibilni fallback.
- Vyres nebo zdokumentuj `task_allocator.yaml` mismatch s `setup.py`.

Acceptance:

- Dokumentace rozlisuje production, operator tooling a legacy/test harness.
- Topic contract doc je pouzitelny pro Agent B i budouci GCS praci.
- Zadna zmena launch souboru nemeni flight ownership model.

## Agent E — GCS / Bridge Follow-Up

Scope:

- `src/scout_control/scout_control/core/gcs_bridge.py`
- `src/scout_control/scout_control/utils/bridge_protocol.py`
- `swarm_center/core/bridge_protocol.py`
- relevantni `swarm_center/` consumers

Ukol:

- Over aktualni bridge protocol verzi a synchronizaci obou kopii.
- Pokud jsou v1.3 zpravy `MSG_NO_GO_OVERLAY` a `MSG_REFINED_GRID_EVENT` jen
  placeholdery, bud je zapoj do GCS consumeru, nebo jasne dokumentuj pending
  stav.
- Pri jakemkoliv wire-format zasahu zachovej zpetnou kompatibilitu a pridat
  minimalni test/fixture.
- Nepresouvej mission planning authority do GCS runtime streamu; GCS je
  operator/planning surface, runtime zustava onboard safety owner.

Acceptance:

- Bridge protocol soubory jsou synchronni.
- GCS se nerozbije na starych status subtype payloads
  `AVOIDANCE_STATUS` / `AVOIDANCE_EVENT`.
- Pokud neni implementovano, existuje presny navazujici TODO s payload shape.

## Integracni pravidla pro koordinatora

1. Nejdriv sloucit zmeny s nejmensim blast radius:
   `cell_data_recorder`, docs, launch labeling.
2. Runtime decomposition sloucit po focused testech, bez soubezne zmeny topic
   wire formatu.
3. Typed contract migraci sloucit po runtime refactoru nebo jako adapter-only
   zmenu, aby se nemichaly phase-machine zmeny s ROS contract zmenami.
4. GCS bridge zmeny sloucit az po potvrzeni protocol version a payload shape.
5. Po kazdem merge spustit targeted pytest subset; na konci full
   `PYTHONPATH=src/scout_control pytest src/scout_control/test/`.

Stop conditions:

- Jakykoliv agent zjisti, ze potrebuje pridat PX4 setpoint publisher mimo
  `obstacle_avoidance_runtime`.
- Zmena vyzaduje jednorazove prepnout vsechny ROS topic consumers bez fallbacku.
- Launch zmena by potichu presunula legacy/manual node do produkcni flight path.
- Neni jasne, jestli pracujeme v NED nebo ENU souradnicich.

V takovem pripade agent zastavi implementaci a doda maly design note s navrhem
bezpecne migrace.
