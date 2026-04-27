# SCOUT WS — NODE AUDIT (aktualizováno)

**Originál:** 2026-04-22  
**Aktualizace:** 2026-04-27 (po Phase 3 + Phase 4A + Phase 4B)  
**Scope:** ROS2 nody v `src/scout_control/scout_control/`, porovnání s cílovou architekturou

---

## 1. Executive Summary

Cílová architektura je jasná a z velké části implementovaná:

- **Mission vrstva** rozhoduje *kam* — `swarm_agent`, `mapping_mission`
- **Runtime vrstva** rozhoduje *jak bezpečně* — `obstacle_avoidance_runtime`
- **Tooling/operator** nody jsou odděleny do `manual/` a `legacy/`

Největší zbývající technický dluh:

1. `obstacle_avoidance_runtime` je stále "god object" (2214 řádků).
2. JSON-over-String protokol přetrvává na všech core kontrakt topicích.
3. Hardcoded `drone_0`/`drone_1` v `cell_data_recorder`.

---

## 2. Co se změnilo od původního auditu (2026-04-22)

### Vyřešeno ✅

| Problém | Řešení |
|---|---|
| `swarm_agent` jako flight-control owner | Phase 3D: direct backend odstraněn, jen mission executor (500 řádků, bylo 1071) |
| `scout_launcher.py` hardcoded WS path | Phase 4B: `_find_ws_root()` podle CLAUDE.md |
| `home_manager` ignoruje `allowed_drone_classes` | Phase 4B: filtrování dle třídy dronu, default `"survey"` |
| `bridge_protocol.py` duplikace out of sync | Phase 4B: oba soubory synchronizovány, `PROTOCOL_VERSION = "1.3"` |
| Field model → grid pipeline chybí | Phase 4A: `GridRefiner` + `MissionPackageBuilder` |
| Balík bez struktury — vše v root | Refaktor: `core/`, `legacy/`, `manual/`, `mapping/`, `missions/`, `avoidance/`, `vision/`, `viz/` |

### Otevřené ⚠️

| Problém | Priorita | Poznámka |
|---|---|---|
| `obstacle_avoidance_runtime` god object (2214 řádků) | P0 | Správná role, špatná velikost; phase machine, PX4 adapter, ROS IO v jednom |
| JSON-over-String na core kontrakt topicích | P0 | Viz seznam níže |
| `cell_data_recorder` hardcoded `drone_0`/`drone_1` | P1 | `cell_data_recorder.py:64-74` |
| `grid_generator` spouští práci v `__init__` | P2 | `grid_generator.py:42-64` |
| `mission_launcher` shutdown přes `SystemExit` z timeru | P2 | Nečistý lifecycle |

---

## 3. Stav struktury balíku (k 2026-04-27)

```
src/scout_control/scout_control/
├── core/                        # Production core
│   ├── obstacle_avoidance_runtime.py   (2214 řádků — god object)
│   ├── swarm_agent.py                  (500 řádků — mission executor ✅)
│   ├── field_setup_coordinator.py      (717 řádků)
│   ├── home_manager.py                 (drone_class filter ✅)
│   ├── swarm_coordinator.py
│   ├── gcs_bridge.py
│   ├── mission_launcher.py
│   ├── spray_controller.py
│   ├── cell_data_recorder.py           (hardcoded drone_0/1 ⚠️)
│   └── ml_interface.py                 (placeholder/stub)
├── avoidance/                   # Runtime interní moduly
│   ├── telemetry_hub.py
│   ├── depth_projector.py
│   ├── local_mapper.py
│   ├── local_planner.py
│   ├── scan_manager.py
│   ├── peer_tracks.py
│   └── types.py
├── mapping/                     # Phase 3+4 — pure Python, bez ROS2 závislostí
│   ├── field_model_builder.py
│   ├── heightmap.py
│   ├── obstacle_extractor.py
│   ├── grid_refiner.py          (Phase 4A ✅)
│   └── mission_package_builder.py (Phase 4A ✅)
├── missions/
│   └── mapping_mission.py       (Phase 3 ✅)
├── vision/
│   └── precision_landing.py     (Phase 3C ✅)
├── manual/                      # Operator tools — TOOLING, ne production path
│   ├── manual_controller.py
│   ├── manual_commander.py
│   ├── field_commander.py
│   └── field_setup_tool.py
├── legacy/                      # Deprecated / debug / test harness
│   ├── obstacle_avoidance_mission.py
│   ├── obstacle_detector.py
│   ├── offboard_control.py
│   ├── perimeter_flight.py
│   ├── position_monitor.py
│   └── terrain_follower.py
├── viz/                         # Tooling vizualizace
└── utils/
```

---

## 4. Node-by-node hodnocení (aktuální stav)

### Production Core

| Node | Stav | Zbývá |
|---|---|---|
| `obstacle_avoidance_runtime` | REFACTOR — správná role, velký | Oddělit phase machine + PX4 adapter do tříd |
| `swarm_agent` | KEEP ✅ — čistý mission executor | Nic kritického |
| `field_setup_coordinator` | KEEP | Grid generation delegovaná přes `GridRefiner`; JSON kontrakty |
| `home_manager` | KEEP ✅ | drone_class filter hotový; `/swarm/charge_complete` subscriber hotový |
| `swarm_coordinator` | KEEP | JSON payloady; `cell_override` ownership |
| `gcs_bridge` | KEEP/REFACTOR | Velký (transport + serialization + ROS adapter v jednom) |
| `mission_launcher` | KEEP | `SystemExit` lifecycle; velmi tenký |
| `spray_controller` | KEEP | Přepisuje celý log per event |
| `cell_data_recorder` | REFACTOR | Hardcoded `drone_0`/`drone_1` |
| `ml_interface` | TOOLING/PLACEHOLDER | Označit jako stub |

### Mapping Pipeline (Phase 3+4, pure Python)

| Modul | Stav |
|---|---|
| `field_model_builder` | KEEP ✅ |
| `heightmap` | KEEP ✅ |
| `obstacle_extractor` | KEEP ✅ |
| `grid_refiner` | KEEP ✅ Phase 4A |
| `mission_package_builder` | KEEP ✅ Phase 4A |
| `mapping_mission` | KEEP ✅ Phase 3 |

### Manual / Tooling

| Node | Stav |
|---|---|
| `manual_controller` | TOOLING — velký (911 řádků), UI+PX4+setup v jednom |
| `manual_commander` | TOOLING — překrývá se s `manual_controller` |
| `field_commander` | TOOLING — 884 řádků, vlastní PX4 offboard loop |

### Legacy (jen debug/reference)

`obstacle_avoidance_mission`, `obstacle_detector`, `offboard_control`, `perimeter_flight`, `position_monitor`, `terrain_follower`

---

## 5. Otevřené body — prioritizovaný backlog

### P0 — nejvyšší dopad

**1. `obstacle_avoidance_runtime` refactor (god object → tenký orchestrátor)**
- Oddělit: `FlightPhaseMachine`, `PX4PublisherAdapter`, `RosIOAdapter` do samostatných tříd
- Node sám by měl být jen orchestrátor volající tyto vrstvy

**2. Typed messages pro core kontrakty**

Kandidáti (nejvýše prioritní):
```
/swarm/drone_status
/swarm/task_status
/{drone_ns}/avoidance/target_cmd
/{drone_ns}/avoidance/status
/swarm/pad_assignment
/field/setup_complete
/swarm/rth_request
/swarm/mission_ready
```

### P1

**3. `cell_data_recorder` — parametrizovat drony**
- `cell_data_recorder.py:64-74` hardcoded `drone_0`, `drone_1`
- Přidat `drone_count` parametr + topic template

**4. Tooling separace v launchích**
- Dokumentačně (CLAUDE.md) hotovo; ideálně oddělit launch soubory pro production vs operator

### P2

**5. `grid_generator` lifecycle**
- Přesunout práci z `__init__` do explicitní `run()` metody

**6. Topic contracts — centrální dokumentace**
- Zapsat naming conventions a QoS requirements do jednoho docs souboru

---

## 6. Aktuální produkční cesta (Flight Path)

```
swarm_agent (mission queue, cell assignment)
    │
    │  target_cmd (JSON/String → budoucí typed msg)
    ▼
obstacle_avoidance_runtime (flight owner, PX4 setpoints)
    ├── avoidance/local_planner
    ├── avoidance/local_mapper
    ├── avoidance/scan_manager
    └── avoidance/depth_projector
```

`swarm_agent` odvozuje `CELL_COMPLETE` z `last_completed_target_id` v runtime statusu.
`navigation_backend=direct` je odstraněn (Phase 3D).

---

## 7. Bridge Protocol Stav

- **Verze:** `1.3` (alias `PROTOCOL_VERSION`)
- **Soubory synchronizovány:**
  - `src/scout_control/scout_control/utils/bridge_protocol.py`
  - `swarm_center/core/bridge_protocol.py`
- **v1.3 placeholdery:** `MSG_NO_GO_OVERLAY`, `MSG_REFINED_GRID_EVENT` (připraveno pro Phase 5)
