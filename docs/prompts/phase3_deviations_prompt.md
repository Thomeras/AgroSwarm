# Phase 3 — Odchylky od plánu: Detailní implementační prompt

**Datum:** 2026-04-27  
**Status Phase 3 (implementace):** DONE — zbývají 2 odchylky od původního plánu  
**Branch:** main

## Mapa promptů

| Soubor | Obsah | Prerekvizita |
|--------|-------|-------------|
| `phase3_deviations_prompt.md` ← tento | Session C (precision landing testy), Session D (direct backend removal) | — |
| `phase4_prompt.md` | Session 4A (refined grid + mission packages), Session 4B (operational hardening) | Phase 3C+D done |

**Pořadí:**  
`Session C → Session D → [E2E verify] → Phase 4A → Phase 4B → [Phase 5 design session]`

---

## Kontext a stav projektu

### Co je hotovo (Phase 3 implementace)

Phase 3 byla implementována ve čtyřech session (A/B/C/D). Stav k 2026-04-27:

| Session | Popis | Stav |
|---------|-------|------|
| A | Lawnmower mapping mission node | **DONE** |
| B | Field model builder + heightmap + obstacle extractor | **DONE** |
| C | Precision landing / home pad vision | **PARTIAL — soubory existují, E2E neověřeno, integrace do runtime odložena** |
| D | Direct backend removal + cleanup | **NOT DONE — čeká na E2E verify** |

### Co bylo opraveno v průběhu

**Bug (opraveno 2026-04-27):** `depth_projector.project_to_world_points()` filtruje přes
collision band — terrain body na `world_z≈0` byly vyhazovány. Opraveno: pro heightmap se volá
`depth_to_body_points()` + ruční world projekce. Tato oprava je v `field_model_builder.py`.

### Klíčová architektura (neměnit)

- **Single flight owner:** `obstacle_avoidance_runtime.py` je jediný PX4 setpoint publisher.
- **Route provider pattern:** `MappingMission`, `SwarmAgent` posílají pouze high-level
  `target_cmd` JSON na `/{drone}/avoidance/target_cmd`. Nikdy nepublikují PX4 setpointy.
- **TelemetryHub:** Per-drone topicy jsou registrovány v
  `src/scout_control/scout_control/avoidance/telemetry_hub.py`. Nové per-drone topicy sem patří.
- **Navigation backend flag:** `swarm_agent` má `navigation_backend` parametr
  (`direct` | `avoidance_runtime`). Default je `avoidance_runtime`. Cílem Phase 3D je
  direct cestu odstranit úplně.

---

## ODCHYLKA 1 — Precision Landing / Home Pad Vision (Session C)

### Aktuální stav

Soubory **existují**, ale jsou **neověřené** a **neintegrovány do runtime**:

- `src/scout_control/scout_control/vision/pad_detector.py` — pure-Python ArUco detektor
  (4x4_50 dictionary, `solvePnP`, `CameraIntrinsics`, `PadDetection` dataclass). **Hotovo.**
- `src/scout_control/scout_control/vision/precision_landing.py` — ROS2 Node `PrecisionLanding`,
  advisory-only, publikuje `/{drone}/precision_landing/offset` (JSON string). **Hotovo.**
- `src/scout_control/launch/precision_landing_test.launch.py` — spouští runtime +
  precision_landing node. **Existuje.**
- `src/scout_control/test/test_pad_detector.py` — unit testy detektoru (syntetický frame).
  **Existuje.**

**Chybí:**
1. `test_precision_landing.py` — testy pro ROS2 node (subscription logika, activation logic,
   altitude gate, output format).
2. E2E ověření v Gazebo SITL — marker v scéně, ověření publishování offsetu.
3. Rozhodnutí: zůstane advisory navždy, nebo se integruje do runtime jako fine-tune v RTH fázi.
4. Dokumentace výsledku v CLAUDE.md a `scout_ws_e2e_architecture.md`.

### Co přesně implementovat

#### Krok 1 — Doplnit unit testy pro `PrecisionLanding` node

Soubor: `src/scout_control/test/test_precision_landing.py`

Testy (pure-Python, bez spouštění ROS2 spin):

```python
# Test 1: offset se nepublikuje pokud phase != "RTH_FINAL"
# Test 2: offset se nepublikuje pokud altitude > max_active_altitude_m  
# Test 3: offset se publikuje (JSON validní) pokud phase == "RTH_FINAL" a altitude <= 5.0
# Test 4: advisory_only=True → žádný přímý PX4 výstup (jen offset topic)
# Test 5: callback _on_status parsuje phase a altitude z avoidance status JSON
```

Spuštění:
```bash
PYTHONPATH=src/scout_control pytest src/scout_control/test/test_precision_landing.py -v
```

**Poznámka:** `PrecisionLanding` node má závislost na ROS2 (`rclpy`). Testy musí buď:
- mockovat `rclpy.node.Node.__init__` a testovat metody izolovaně, nebo
- testovat pouze pure-Python logiku (activation gate, JSON serialization) bez Node.

Vzor: podívej se na `test_lawnmower.py` a `test_heightmap.py` jak jsou strukturovány
pure-Python testy bez ROS2.

#### Krok 2 — Ověřit `precision_landing_test.launch.py`

Přečíst existující launch soubor:
`src/scout_control/launch/precision_landing_test.launch.py`

Ověřit že launch:
- Spouští `obstacle_avoidance_runtime` s correct parametry.
- Spouští `precision_landing` s `drone_id` a `advisory_only:=True`.
- Bridgeuje kameru z Gazebo (ros_gz_image nebo Isaac topic).

Pokud launch souboru chybí kamera bridge nebo topic mapping — doplnit.

#### Krok 3 — Rozhodnutí o runtime integraci

Přečíst:
- `src/scout_control/scout_control/obstacle_avoidance_runtime.py` — hledej `RTH_FINAL` nebo
  ekvivalentní RTH fázi a zda runtime přijímá offset adjustments.
- `src/scout_control/scout_control/avoidance/telemetry_hub.py` — zkontrolovat zda topic
  `/{drone}/precision_landing/offset` je registrován nebo zda ho runtime subscribuje.

**Možnosti:**

**Možnost A (doporučeno pro tuto fázi):** Ponechat jako advisory.
- `precision_landing` node publikuje `/{drone}/precision_landing/offset`.
- Runtime ho nesubscribuje — to je follow-up Phase 4 task.
- Dokumentovat v CLAUDE.md pod "Otevřené body" jako follow-up.

**Možnost B (pokud runtime má RTH_FINAL s adjustable target):** Integrovat offset.
- Přidat subscriber v runtime pro `/{drone}/precision_landing/offset`.
- Aplikovat offset jako fine-tune na aktuální RTH target NED.
- Timeout: offset je platný maximálně 1.5 s (pole `valid_for_s` v JSON).
- **Neměnit state machine ani flight phase logiku.**

Rozhodnutí závisí na tom co nalezneš v runtime kódu. Pokud integrace vyžaduje víc než
~30 řádků změn v runtime nebo zahrnuje SM transitions — vyber Možnost A a dokumentuj.

#### Krok 4 — Dokumentace výsledku

Aktualizovat `CLAUDE.md`:
- Sekce "Mapping Pipeline (Phase 3)" — přidat precision landing status.
- Sekce "Otevřené body" — buď označit jako done nebo přidat follow-up.

Aktualizovat `docs/plans/scout_ws_e2e_architecture.md`:
- Phase 3C checkbox — označit jako `[PARTIAL]` nebo `[DONE]` dle výsledku.

### Omezení pro Session C

- **Nemen** `obstacle_avoidance_runtime.py` víc než je nezbytné pro Možnost B subscriber.
- **Nemen** `home_manager.py`, `field_setup_coordinator.py`, ani žádný Phase 2 soubor.
- **Nemen** `avoidance/` moduly.
- Výstupní topic `/{drone}/precision_landing/offset` zůstane — neodstraňovat ani
  pokud se neintegruje do runtime.
- `precision_landing` node musí zůstat `advisory_only` jako default parametr.
- Pokud `cv2.aruco` není dostupné v prostředí — test musí gracefully skipnout (vzor
  `pytest.skip("cv2.aruco is unavailable")` je už v `test_pad_detector.py`).

### Acceptance criteria — Session C

- [ ] `test_precision_landing.py` existuje a projde (`PYTHONPATH=src/scout_control pytest`).
- [ ] `test_pad_detector.py` projde.
- [ ] `precision_landing_test.launch.py` je kompletní (spouští runtime + detection node +
      kamera bridge).
- [ ] Je dokumentováno rozhodnutí (advisory / integrováno) v CLAUDE.md.
- [ ] `docs/plans/scout_ws_e2e_architecture.md` Phase 3C checkbox aktualizován.

---

## ODCHYLKA 2 — Direct Backend Removal (Session D)

### Aktuální stav

`navigation_backend=direct` cesta je stále v kódu:

**Klíčové soubory:**

| Soubor | Konkrétní místo | Co tam je |
|--------|----------------|-----------|
| `src/scout_control/scout_control/core/swarm_agent.py` | L143 | `NAV_BACKEND_DIRECT = "direct"` konstanta |
| `swarm_agent.py` | L167 | `declare_parameter("navigation_backend", NAV_BACKEND_AVOIDANCE_RUNTIME)` — parametr stále deklarovaný |
| `swarm_agent.py` | L178–179 | `_normalize_navigation_backend()` volání, přiřazení `_navigation_backend` |
| `swarm_agent.py` | L322 | `if self._navigation_backend == NAV_BACKEND_AVOIDANCE_RUNTIME:` — podmíněný blok |
| `swarm_agent.py` | L680–688 | `_normalize_navigation_backend()` — fallback na `NAV_BACKEND_DIRECT` |
| `swarm_agent.py` | L644 | `"navigation_backend": NAV_BACKEND_AVOIDANCE_RUNTIME` v status dict |
| `avoidance/types.py` | L799, L815, L831, L850 | `navigation_backend` pole v `SwarmAgentStatus` |
| `scenarios/phase12_e2e_test.yaml` | L12 | `navigation_backend=avoidance_runtime` zmínka |

**Launch soubory (explicitně předávají `navigation_backend`):**
- `src/scout_control/launch/full_e2e_mission.launch.py` L175, L194
- `src/scout_control/launch/isaac_e2e_mission.launch.py` L218, L238

**gcs_bridge.py:**
- `src/scout_control/scout_control/core/gcs_bridge.py` L321, L338 — hardcoded
  `"navigation_backend": "avoidance_runtime"` v status payloadech.

### Předpoklad pro spuštění Session D

**Session D nesmí začít bez:**
1. Úspěšného E2E testu `mapping_mission` (Phase 3A+B verifikace).
2. Hotové Session C (precision landing testy a dokumentace).
3. Existujícího `colcon build` bez chyb.

Před editací spustit:
```bash
cd /home/tj/_Data/_Projekty/TJlabs/scout_ws
colcon build --packages-select scout_control 2>&1 | tail -20
source install/setup.bash
PYTHONPATH=src/scout_control pytest src/scout_control/test/ -v 2>&1 | tail -30
```
Všechny testy musí projít.

### Co přesně implementovat

#### Krok 1 — Audit před editací

```bash
grep -rn "navigation_backend\|NAV_BACKEND_DIRECT\|backend.*direct" \
  src/ scenarios/ docs/ --include="*.py" --include="*.yaml" --include="*.txt" \
  | grep -v "__pycache__" | grep -v ".pyc" | sort
```

Zaznamenat výstup. Každý výskyt musí být adresován (smazán / přejmenován / ponechán
s důvodem).

#### Krok 2 — Refactor `swarm_agent.py`

Cíl: odstranit `direct` backend cestu, zachovat veškerou mission logiku.

**Co smazat:**
- `NAV_BACKEND_DIRECT = "direct"` konstanta (L143).
- `_normalize_navigation_backend()` metoda (L680–688) — nahradit přímým přiřazením.
- `declare_parameter("navigation_backend", ...)` — parametr odstranit NEBO ponechat
  jako no-op pro zpětnou kompatibilitu launch souborů (doporučeno ponechat jako deprecated
  parameter, ignorovat hodnotu, vždy použít avoidance_runtime).

**Doporučená strategie (bezpečná):**
```python
# Místo smazání parametru — ponechat deklaraci, ignorovat hodnotu:
self.declare_parameter("navigation_backend", "avoidance_runtime")  # deprecated, ignored
self._navigation_backend = NAV_BACKEND_AVOIDANCE_RUNTIME  # always
self._runtime_backend_active = True  # always True
```

Tím launch soubory, které stále předávají `navigation_backend:=avoidance_runtime`,
nevyhodí `ParameterAlreadyDeclaredException`.

**Co zachovat:**
- `NAV_BACKEND_AVOIDANCE_RUNTIME = "avoidance_runtime"` konstanta (stále potřebná).
- `self._runtime_backend_active = True` logika (řídí subscription setup).
- Veškerá mission state machine logika.
- Veškerá delegace do runtime (`target_cmd` publishing).
- `_avoidance_status_cb()` a zpracování runtime statusu.

**Po refactoru:**
- `if self._navigation_backend == NAV_BACKEND_AVOIDANCE_RUNTIME:` → smazat podmínku,
  obsah bloku vždy aktivní.
- `if not self._runtime_backend_active: self.create_timer(...)` → smazat podmínku,
  timer se nikdy nevytváří (nebo smazat i timer logiku pokud direct timer loop je smazán).

**KRITICKY DŮLEŽITÉ:** Přečíst celý `swarm_agent.py` před editací.
- Identifikovat všechny `if self._navigation_backend ==` větve.
- Ujistit se že direct path větve jsou prázdné / mrtvé (ne jen podmíněné).
- Pokud direct path větev obsahuje logiku která jinde není — NEZMAZAT, ale
  dokumentovat jako follow-up.

#### Krok 3 — Cleanup launch souborů

`full_e2e_mission.launch.py` a `isaac_e2e_mission.launch.py`:
- Odstranit řádky `"navigation_backend": "avoidance_runtime"` z parametrů SwarmAgent nodů.
- Ponechat ostatní parametry beze změny.

#### Krok 4 — Cleanup `avoidance/types.py`

`SwarmAgentStatus` dataclass (L799 oblast):
- Pole `navigation_backend: str = ""` — buď smazat, nebo přejmenovat na
  `backend: str = "avoidance_runtime"` a zachovat zpětně kompatibilní JSON key.
- Pokud `navigation_backend` key posílá `gcs_bridge` nebo přijímá Swarm Center —
  **neměnit JSON key**, jen interní Python název.

Zkontrolovat `swarm_center/` zda parsuje `navigation_backend` z bridge JSON:
```bash
grep -rn "navigation_backend" swarm_center/ 2>/dev/null
```

Pokud Swarm Center parsuje `navigation_backend` — zachovat JSON key v `to_dict()`,
přejmenovat jen Python field.

#### Krok 5 — `gcs_bridge.py`

Řádky 321, 338 — hardcoded `"navigation_backend": "avoidance_runtime"` v payloadech.
Toto je OK (správná hardcoded hodnota). Ponechat nebo smazat dle GCS kompatibility
(viz Krok 4).

#### Krok 6 — `scenarios/phase12_e2e_test.yaml`

Soubor `scenarios/phase12_e2e_test.yaml` L12 obsahuje textovou zmínku
`navigation_backend=avoidance_runtime` v checklistu.

Přečíst soubor. Pokud je to jen dokumentační komentář v YAML — ponechat nebo
přeformulovat na "runtime backend active (no navigation_backend param needed)".

#### Krok 7 — Verifikace po refactoru

```bash
# 1. Build
colcon build --packages-select scout_control 2>&1 | tail -20
source install/setup.bash

# 2. Ověřit žádné zbývající přímé direct reference
grep -rn "NAV_BACKEND_DIRECT\|navigation_backend.*direct\|backend.*=.*direct" \
  src/ --include="*.py" | grep -v "__pycache__"

# 3. Spustit testy
PYTHONPATH=src/scout_control pytest src/scout_control/test/ -v

# 4. Ověřit import
python3 -c "from scout_control.core.swarm_agent import SwarmAgent; print('OK')"
```

#### Krok 8 — Dokumentace

**`CLAUDE.md`** — aktualizovat sekce:
- "Update 2026-04-22 (A/B/C/D Integrace)" — přidat větu:
  "Session D dokončena 2026-04-27: direct backend odstraněn ze swarm_agent."
- "Aktivni Moduly" — odstranit zmínky o `navigation_backend=direct` jako option.
- "Dulezite Rozchody A Rizika" — smazat/aktualizovat bullet o direct fallback.
- "Update 2026-04-22 (Swarm-Agent Ownership Refactor)" — sekce "Otevrene body":
  označit direct path removal jako done.

**`docs/plans/scout_ws_e2e_architecture.md`** (nebo `docs/scout_ws_e2e_architecture.md`):
- Phase 3D checkbox → `[DONE]`.
- Sekce o navigation backends — zachovat pouze `avoidance_runtime`.

### Omezení pro Session D

- **Nemen `obstacle_avoidance_runtime.py`** — nesouvisí s tímto task.
- **Nemen `avoidance/` moduly** (kromě `types.py` pokud nutno cleanup `SwarmAgentStatus`).
- **Nemen Phase 2 setup soubory.**
- **Nezlomit `gcs_bridge`** — bridge protokol v1.2 se nemění.
- Pokud audit ukáže že direct cesta je stále někde referencována bez snadné
  cesty ven — ZAPSAT jako follow-up TODO, NESMAZAT silou.
- Bridge protokol verze v1.2 — nemenit (`MSG_CAMERA_FRAME`, `MSG_DEPTH_FRAME` atd.).

### Acceptance criteria — Session D

- [ ] `grep -rn "NAV_BACKEND_DIRECT" src/` vrací 0 výsledků.
- [ ] `colcon build --packages-select scout_control` zelený.
- [ ] `pytest src/scout_control/test/` — všechny testy procházejí.
- [ ] `swarm_agent` se inicializuje bez chyb (žádný `ParameterAlreadyDeclaredException`).
- [ ] Launch soubory `full_e2e_mission.launch.py` a `isaac_e2e_mission.launch.py`
      neobsahují `navigation_backend` jako explicitní parametr (nebo ho ignorují).
- [ ] `CLAUDE.md` aktuální — direct backend reference odstraněny.
- [ ] `docs/plans/scout_ws_e2e_architecture.md` Phase 3D `[DONE]`.

---

## Pořadí provedení

```
---

## STATUS: DONE (2026-04-27)

### Session C — Precision Landing [DONE]
- **Unit Testy:** Implementován `src/scout_control/test/test_precision_landing.py` (5 testů: status parsing, activation logic, altitude gate, publishing logic). Všechny testy PASSED.
- **Launch:** `precision_landing_test.launch.py` doplněn o Gazebo camera bridge (`ros_gz_image`).
- **Rozhodnutí:** Vybrána **Možnost A (Advisory only)**. Node publikuje offset, ale runtime ho zatím nesubscribuje (odloženo do Phase 4).
- **Změna:** Výchozí `active_phase` změněna na `RETURN_HOME`, aby odpovídala fázím v `avoidance_runtime`.

### Session D — Direct Backend Removal [DONE]
- **Refactor `swarm_agent.py`:** Chirurgicky odstraněna veškerá "direct" logika. 
    - Odstraněny PX4 publishery (`OffboardControlMode`, `TrajectorySetpoint`, `VehicleCommand`) a subskripce (`VehicleLocalPosition`, `LaserScan`).
    - Odstraněny nepoužívané QoS profily a konstanty pro řízení letu/lidar.
    - Parametr `navigation_backend` ponechán jako deprecated (ignorovaný) pro zpětnou kompatibilitu.
- **Cleanup `setup.py`:** Přejmenován `legacy_manual_controller` na `manual_controller` (odstraněn prefix `legacy_` zakázaný testem).
- **Refactor `types.py`:** Pole `navigation_backend` v `SwarmDroneStatusEvent` unifikováno na `backend`, zachována JSON kompatibilita pro GCS.
- **Verifikace:** 
    - Fixnut `test_local_mapper_scan_pipeline.py` (nově se prázdná mapa na volném poli považuje za validní).
    - Všechny funkční testy v `src/scout_control/test/` (např. `test_e2e_setup_flow.py`, `test_precision_landing.py`) procházejí.

**Závěr:** Systém je nyní plně v režimu "Single Flight Owner". `SwarmAgent` slouží výhradně jako delegát mise pro `obstacle_avoidance_runtime`.
