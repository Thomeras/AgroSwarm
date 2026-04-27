# Phase 4 — Implementační prompt
**Datum:** 2026-04-27  
**Prerekvizita:** Phase 3 C+D dokončeny (precision landing testy + direct backend smazán)  
**Branch:** main

---

## Kontext a stav projektu

### Co je hotovo (Phase 1–3)
- Phase 1: `obstacle_avoidance_runtime` jako single flight owner, modularní avoidance pipeline.
- Phase 2: polygon boundary, pad registry, grid_generator s `boundary_mode`.
- Phase 3A: `MappingMission` node, lawnmower trajectory.
- Phase 3B: `FieldModelBuilder`, `Heightmap2D`, `ObstacleExtractor` — výstupy v `perimeters/field_model/`.
- Phase 3C: precision landing advisory node (soubory existují, integrace advisory-only).
- Phase 3D: `navigation_backend=direct` odstraněn ze `swarm_agent`.

### Datové formáty Phase 3 (vstup pro Phase 4)

**`perimeters/field_model/manifest.json`** — index mapovacích běhů:
```json
{
  "version": 1,
  "latest": {
    "heightmap_json": "heightmap_<ts>.json",
    "obstacles_json": "obstacles_<ts>.json",
    "obstacle_count": N,
    "point_count": N,
    "created_at_s": 1777275470.0
  },
  "entries": [...]
}
```

**`perimeters/field_model/obstacles_<ts>.json`** — seznam překážek:
```json
{
  "version": 1,
  "created_at_s": ...,
  "obstacles": [
    {
      "centroid_ned": [x, y, z],
      "bbox_ned": [x_min, y_min, z_min, x_max, y_max, z_max],
      "point_count": 47,
      "confidence": 0.85
    }
  ]
}
```

**`perimeters/field_model/heightmap_<ts>.json`** — 2.5D terrain grid s `min_z` per cell.

### Klíčová architektura (neměnit)
- **Single flight owner:** `obstacle_avoidance_runtime.py`.
- **Route provider pattern:** mission nody posílají `target_cmd` na `/{drone}/avoidance/target_cmd`.
- **TelemetryHub:** nové per-drone topicy sem patří.
- **Paths:** vždy `from scout_control.utils.paths import ...`. Žádné hardcoded stringy.

---

## PHASE 4A — Field Model → No-Go Zones → Refined Grid

### Cíl

Propojit výstupy Phase 3 (field_model) s grid systémem. Výsledek: `refined_grid.json`
se čtyřmi typy buněk a `no_go_zones.json` jako inflated polygony překážek. Toto je vstup
pro Swarm Center a budoucí mission planning.

### Nové soubory / moduly

| Soubor | Typ | Účel |
|--------|-----|------|
| `src/scout_control/scout_control/mapping/grid_refiner.py` | Pure Python | `GridRefiner` class: obstacles + base grid → refined grid |
| `src/scout_control/scout_control/mapping/mission_package_builder.py` | Pure Python | `MissionPackageBuilder`: refined grid + drone count → mission packages |
| `perimeters/refined_grid.json` | Datový výstup | Upřesněný grid s rozšířenou `cell_class` |
| `perimeters/field_model/no_go_zones.json` | Datový výstup | Inflated obstacle polygony |
| `perimeters/mission_packages/<mission_id>/<drone_id>.json` | Datový výstup | Per-drone ordered cell list |
| `src/scout_control/test/test_grid_refiner.py` | Test | Unit testy GridRefineru |
| `src/scout_control/test/test_mission_package_builder.py` | Test | Unit testy MissionPackageBuilder |

---

### Krok 1 — `GridRefiner` (`mapping/grid_refiner.py`)

#### Vstup
- `obstacles: list[Obstacle]` — z `obstacles_<ts>.json`
- `base_grid_path: Path` — cesta k `field_grid.json`
- `inflation_m: float = 1.5` — nafukování překážky (platform size + GPS uncertainty)
- `caution_buffer_m: float = 1.0` — buffer okolo no-go pro caution zónu

#### Výstup
- `refined_grid.json` — stejný formát jako `field_grid.json` ale s rozšířenou `cell_class`
- `no_go_zones.json` — seznam inflated AABB polygonů

#### `cell_class` rozšíření (zpětně kompatibilní)
Stávající hodnoty: `inside`, `edge`  
Nové hodnoty: `no_go`, `caution`  
Pravidlo priority: `no_go` > `caution` > `edge` > `inside`

Starší `field_grid.json` bez nových hodnot → zůstanou `inside`/`edge` beze změny.

#### Algoritmus
```python
class GridRefiner:
    def __init__(self, inflation_m: float = 1.5, caution_buffer_m: float = 1.0): ...

    def build_no_go_zones(self, obstacles: list[Obstacle]) -> list[dict]:
        """Inflated AABB polygon per obstacle (bbox_ned + inflation_m)."""
        # Pro každý obstacle: rozšiř bbox_ned o inflation_m ve všech horizontálních směrech.
        # Vrať seznam {"centroid_ned": [...], "bbox_inflated": [xmin, ymin, xmax, ymax],
        #               "confidence": float, "original_bbox": [...]}

    def refine_grid(
        self,
        cells: list[dict],  # z field_grid.json
        no_go_zones: list[dict],
    ) -> list[dict]:
        """Přiřaď cell_class no_go / caution / zachovej stávající."""
        # Pro každou cell:
        #   center_x, center_y = cell["x"] + cell_size/2, cell["y"] + cell_size/2
        #   Pokud center leží uvnitř libovolného bbox_inflated → cell_class = "no_go"
        #   Pokud center leží uvnitř bbox_inflated + caution_buffer_m → cell_class = "caution"
        #   Jinak: ponechat stávající cell_class
        # Vrátit nový seznam buněk (nemutovat vstup).

    def save(self, refined_cells: list[dict], no_go_zones: list[dict],
             output_dir: Path) -> None:
        """Uložit refined_grid.json a no_go_zones.json."""
        # refined_grid.json: stejný root jako field_grid.json + "refined": true, "version": 2
        # no_go_zones.json: {"version": 1, "created_at_s": ..., "zones": [...]}
```

**DŮLEŽITÉ:**
- `GridRefiner` je pure-Python class bez ROS2 závislostí.
- Nesmí modifikovat `field_grid.json` — výstup je nový `refined_grid.json`.
- Pokud není k dispozici žádný field model (manifest neexistuje nebo `obstacle_count == 0`):
  refined_grid = kopie base grid (bez no_go/caution). Logovat varování.
- `bbox_ned` formát: `[x_min, y_min, z_min, x_max, y_max, z_max]` (6 hodnot).
  Horizontální inflation: jen `x_min -= inflation_m`, `x_max += inflation_m`,
  `y_min -= inflation_m`, `y_max += inflation_m`. Vertikální (z) neměnit.

---

### Krok 2 — Integrace do `field_setup_coordinator.py`

Po `GENERATE_GRID` stavu (po zapsání `field_grid.json`) zavolat `GridRefiner`:

```python
# V field_setup_coordinator — po grid generaci:
if field_model_manifest_exists:
    obstacles = load_latest_obstacles(manifest)
    refiner = GridRefiner()
    no_go_zones = refiner.build_no_go_zones(obstacles)
    refined_cells = refiner.refine_grid(base_cells, no_go_zones)
    refiner.save(refined_cells, no_go_zones, PERIMETERS_DIR / "field_model")
    self.get_logger().info(f"Refined grid: {sum(c['cell_class']=='no_go' for c in refined_cells)} no_go cells")
else:
    self.get_logger().info("No field model found — skipping grid refinement")
```

Podmínka spuštění: pokud `perimeters/field_model/manifest.json` existuje a `latest.obstacle_count > 0`
NEBO `latest.point_count > 0` (heightmap data existují i bez překážek).

**Omezení:** Nezpomalovat setup flow — `GridRefiner` musí být synchronní a rychlý (<200ms).

---

### Krok 3 — `MissionPackageBuilder` (`mapping/mission_package_builder.py`)

Cíl: přiřadit `inside` a `edge` buňky (ne `no_go`) dronům jako ordered lists.

#### Vstup
- `refined_grid.json` (nebo `field_grid.json` jako fallback)
- `drone_ids: list[str]`
- `strategy: str = "sector"` — dostupné: `"sector"`, `"round_robin"`

#### Výstup
- `perimeters/mission_packages/<mission_id>/<drone_id>.json` per drone

#### Package formát (per drone JSON)
```json
{
  "version": 1,
  "mission_id": "mission_20260427_093750",
  "drone_id": "drone_0",
  "created_at_s": 1777275470.0,
  "strategy": "sector",
  "cells": [
    {
      "cell_id": "x4_y2",
      "x": 4.0, "y": 2.0,
      "cell_class": "inside",
      "altitude_m": 5.0,
      "service_type": "survey"
    }
  ],
  "total_cells": 42,
  "estimated_flight_time_s": null
}
```

#### Algoritmus `"sector"` strategie
1. Filtrovat buňky: ponechat jen `cell_class in ("inside", "edge")`.
2. Seřadit buňky po sloupcích (boustrophedon / lawnmower order) pro minimalizaci pohybu.
3. Rozdělit na N přibližně stejných sektorů (po sloupcích nebo řádcích).
4. Každý sektor přiřadit jednomu dronu.

**Poznámka:** Toto je první verze — není třeba řešit battery rotation, payload type nebo
terrain-aware altitude per cell. `altitude_m` je konstantní (default 5.0, parametr).

**DŮLEŽITÉ:** `MissionPackageBuilder` nesmí mít ROS2 závislosti. Je to pure-Python utility
volatelná z `field_setup_coordinator` nebo standalone.

---

### Krok 4 — Unit testy

**`test_grid_refiner.py`**
```python
# Test 1: buňka přímo uvnitř inflated bbox → cell_class == "no_go"
# Test 2: buňka v caution buffer → cell_class == "caution"
# Test 3: buňka daleko od překážky → cell_class zachována ("inside")
# Test 4: prázdný obstacles list → refined == base grid beze změny
# Test 5: refined_grid.json má správný JSON formát (version, cells, refined=True)
# Test 6: no_go_zones.json má správný formát (zones list)
# Test 7: GridRefiner nemutuje vstupní cells list
```

**`test_mission_package_builder.py`**
```python
# Test 1: sector strategie rozdělí buňky rovnoměrně mezi drony
# Test 2: no_go buňky nejsou v žádném package
# Test 3: package soubory mají správný JSON formát
# Test 4: round_robin přiřazuje buňky střídavě
# Test 5: 1 dron dostane všechny buňky
# Test 6: více dronů než buněk → někteří dostanou prázdný package
```

Spuštění:
```bash
PYTHONPATH=src/scout_control pytest src/scout_control/test/test_grid_refiner.py \
  src/scout_control/test/test_mission_package_builder.py -v
```

---

### Acceptance criteria — Phase 4A

- [ ] `GridRefiner` je pure Python, žádné ROS2 závislosti.
- [ ] `refined_grid.json` se generuje po `GENERATE_GRID` pokud existuje field model.
- [ ] `no_go_zones.json` se generuje v `perimeters/field_model/`.
- [ ] Pokud field model neexistuje → `refined_grid.json` == kopie `field_grid.json`.
- [ ] `MissionPackageBuilder` generuje per-drone JSON packakges.
- [ ] Oba unit test soubory existují a procházejí.
- [ ] `colcon build --packages-select scout_control` zelený.
- [ ] `CLAUDE.md` sekce "Mapping Pipeline" aktualizována.
- [ ] `docs/plans/scout_ws_e2e_architecture.md` Phase 4A označena jako `[DONE]`.

---

## PHASE 4B — Operational Hardening

### Cíl
Infrastrukturní cleanup: multi-drone class support v pad allocatoru, WS_DIR portability,
a bridge verze příprava. Charging lifecycle zůstává bez real hardware — jen schema a SM.

### Prerekvizita
Phase 4A musí být dokončena a build zelený.

---

### Krok 1 — Multi-Drone Class Support

#### Aktuální stav
`home_positions.json` má pole `allowed_drone_classes` definováno v `scout_ws_e2e_architecture.md`
ale `home_manager.py` a `task_allocator.py` ho ignorují.

#### Co implementovat

**`home_manager.py`:**
- Při pad query (`/swarm/pad_query`) přijímat volitelné pole `"drone_class": str`.
- Pokud pad má `allowed_drone_classes: []` (prázdný) → přijímá libovolnou třídu.
- Pokud `allowed_drone_classes: ["sprayer", "survey"]` → odmítnout query od dronu jiné třídy.
- Default drone_class = `"survey"` (backwards compat — staré query bez pole = survey).

**`task_allocator.py`:**
- Přidat volitelné `drone_class: str = "survey"` k drone registration.
- Předávat drone_class do pad query.

**`home_positions.json` schema** (zpětně kompatibilní):
```json
{
  "pad_id": "pad_0",
  "drone_id": "drone_0",
  "ned": {"x": 0.0, "y": 0.0, "z": 0.0},
  "allowed_drone_classes": [],
  "charging_capable": true,
  "orientation_deg": 0.0,
  "service_priority": 1,
  "status": "available"
}
```
Starší JSON bez `allowed_drone_classes` → default `[]` (přijímá vše).

**Testy:** `test_home_manager.py` rozšířit o:
- Test: dron správné třídy dostane pad.
- Test: dron špatné třídy je odmítnut, dostane dalšípad nebo None.
- Test: pad s `allowed_drone_classes: []` přijme libovolnou třídu.

---

### Krok 2 — WS_DIR Portability (`scout_launcher.py`)

#### Aktuální stav
`scout_launcher.py` má hardcoded:
```python
WS_DIR = "/home/tj/_Data/_Projekty/TJlabs/scout_ws"
```

#### Co implementovat
Použít stejnou logiku jako `scout_control.utils.paths` — hledat `CLAUDE.md` jako workspace anchor:

```python
def _find_ws_root() -> Path:
    """Walk up from this file's location until CLAUDE.md is found."""
    candidate = Path(__file__).resolve().parent
    for _ in range(6):
        if (candidate / "CLAUDE.md").exists():
            return candidate
        candidate = candidate.parent
    # Fallback: use file location itself
    return Path(__file__).resolve().parent

WS_DIR = _find_ws_root()
```

**Omezení:** Nezměnit žádné jiné chování `scout_launcher.py`. Jen nahradit hardcoded string.

---

### Krok 3 — Charging Lifecycle Schema

#### Aktuální stav
Pad SM má stavy `available → occupied → charging → available` ale bez real hardware feedbacku.

#### Co implementovat (schema only, bez hardware)
Přidat pad topic pro charging completion:
- Nový topic: `/swarm/charge_complete` (publisher: operator nebo hardware adapter, subscriber: `home_manager`)
- Message: `std_msgs/String` JSON: `{"pad_id": "pad_0", "drone_id": "drone_0"}`
- `home_manager` přechod při přijetí: `charging → available`

Aktuálně `charging → available` probíhá jen přes manual override. Toto přidá programatický trigger.

**Soubory:** `home_manager.py` — přidat subscriber na `/swarm/charge_complete`.

**Test:** `test_home_manager.py` — test přechodu `charging → available` přes topic.

---

### Krok 4 — Bridge Protocol příprava (v1.3 placeholder)

#### Aktuální stav
Bridge protokol je v1.2, duplikovaný ve dvou souborech:
- `src/scout_control/scout_control/bridge_protocol.py`
- `swarm_center/core/bridge_protocol.py`

#### Co implementovat
**Nepřidávat nové payloady** — Phase 4B pouze připravit verzi bump mechanismus:
- Přidat `PROTOCOL_VERSION = "1.2"` konstantu do obou souborů (pokud neexistuje).
- Přidat komentář kde přidat v1.3 payloady (no-go zone overlay, refined grid event).
- **Synchronizovat oba soubory** pokud jsou out of sync.

Ověřit:
```bash
diff src/scout_control/scout_control/bridge_protocol.py swarm_center/core/bridge_protocol.py
```
Pokud diff není prázdný — synchronizovat (bridge_protocol.py je authoritative).

---

### Acceptance criteria — Phase 4B

- [ ] `home_manager.py` filtruje pads dle `allowed_drone_classes`.
- [ ] `home_positions.json` starší bez `allowed_drone_classes` → loads OK (default `[]`).
- [ ] `scout_launcher.py` neobsahuje hardcoded `/home/tj/...` cesty.
- [ ] `/swarm/charge_complete` subscriber v `home_manager` existuje a přechází `charging → available`.
- [ ] `bridge_protocol.py` v obou stromech je synchronizovaný.
- [ ] Unit testy pro multi-drone class a charge complete procházejí.
- [ ] `colcon build --packages-select scout_control` zelený.
- [ ] `docs/plans/scout_ws_e2e_architecture.md` Phase 4 označena jako `[DONE]`.

---

## Pořadí provedení

```
[Phase 3C+D dokončeny — precision landing testy, direct backend smazán]
        ↓
Phase 4A — Grid Refiner + Mission Package Builder
  Prerekvizita: colcon build zelený, pytest zelený
        ↓
Phase 4B — Operational Hardening
  Prerekvizita: Phase 4A zelená
        ↓
[E2E verify: full_e2e_mission nebo isaac_e2e_mission]
        ↓
[Phase 5 — Swarm Center Planning Views — vyžaduje design session]
```

---

## Poznámky k Phase 5+

Phase 5 (Swarm Center planning views) a Phase 6 (context slices zpět na drony) nejsou
ještě dostatečně naplánované pro implementaci. Před zahájením Phase 5 je třeba:

1. **Design session** — jakými konkrétními UI prvky Swarm Center rozšiřovat?
   - Boundary editor? No-go zone visualizer? Refined grid view? Mission package editor?
2. **Bridge protocol v1.3** — jaké nové zprávy budou potřeba pro field model → GCS flow?
3. **Context slices API** — jaký formát má static map extract pro drona? Kdo ho posílá?

Bez těchto rozhodnutí nelze napsat validní implementační prompt pro Phase 5+.

---

## Společná pravidla

1. **Přečíst soubor před editací.** Nikdy nepředpokládat obsah.
2. **TDD:** testy napsat před implementací.
3. **NED všude.** Z je dolů (výška = záporné číslo).
4. **Paths:** `from scout_control.utils.paths import ...`. Žádné hardcoded rooty.
5. **Žádné nové ROS2 závislosti** v mapping utility modulech (pure Python).
6. **Zpětná kompatibilita:** JSON formáty rozšiřovat přes nová volitelná pole + `version` bump.
7. **Žádný nový PX4 setpoint publisher.** Single flight owner = `obstacle_avoidance_runtime`.
8. **Commit po každé session** (`feat:`, `refactor:`, `fix:` prefix).
9. **Synchronizovat bridge_protocol.py** v obou stromech při každé změně.
