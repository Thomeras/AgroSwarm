# Specializace — pohled přes projekt Scout

> Tento dokument porovnává dvě konkrétní kariérní specializace **z perspektivy kódu, který
> v projektu Scout už existuje**. Není to generický robotický průvodce — každý bod
> odkazuje na skutečný soubor, topic nebo architektonické rozhodnutí z tohoto workspace.

---

## Option A — Robot Perception Engineer

### Co v Scoutu už dělám, co je Perception

Vnímání prostředí je v tomto projektu přítomné ve třech vrstvách:

**1. Depth → Body Points pipeline**

Soubor [depth_projector.py](../../src/scout_control/scout_control/avoidance/depth_projector.py)
implementuje celý pinhole back-projection stack:
- `CameraIntrinsics.from_hfov()` a `CameraIntrinsics.from_camera_info()` — kalibrační vrstva, umí pracovat jak se
  skutečným `CameraInfo` z ROS2, tak s fallbackem z horizontálního FOV (71.9°)
- `DepthProjector.depth_to_body_points()` — projekce z 2D depth obrazu do body-frame FRD souřadnic
  přes pinhole model: `forward = d`, `right = (u - cx)*d/fx`, `down = (v - cy)*d/fy`
- `project_to_world_points()` — rotace z body FRD do world NED přes matici `[cos_yaw, -sin_yaw; sin_yaw, cos_yaw]`
- `project_to_local_xy()` — filtrování collision bandu (`-1.0 m` až `+1.0 m` v ose down),
  izolace bodů ve výšce, kde může nastat kolize
- `for_shape()` — škálování intrinsics pokud se změní rozlišení framu

Toto je čistá **depth perception pipeline** — přijímá ROS `sensor_msgs/Image` depth frame a vrací
prostorové body v konzistentním souřadnicovém systému.

**2. Occupancy mapping a temporal fusion**

Soubor [local_mapper.py](../../src/scout_control/scout_control/avoidance/local_mapper.py)
implementuje 2D occupancy grid s temporal decay:
- Dvě vrstvené mřížky: `_fast_layer` (depth camera, half-life 10 s) a `_scan_layer` (360° skenovací průchod, half-life 60 s)
- Odlišná confidence při inserci: `0.55` pro depth frame, `0.90` pro dense scan
- Exponenciální decay: `layer *= 0.5^(dt / half_life_s)`
- Voxelový grid (`_dense_scan_voxels`, 0.2 m voxel size) pro akumulaci dense scan pointů
- `obstacle_inflation_radius_m: 1.2 m` — Minkowski sum expansion kolem detekovaných překážek
  přes `_build_gradient_kernel()` s hard + soft shellem
- `_build_clearance_summary()` — sektorová analýza clearance (left/center/right/forward),
  detekce warn a critical stavu

**3. Peer tracking a prediction**

Soubor [peer_tracks.py](../../src/scout_control/scout_control/avoidance/peer_tracks.py):
- `PeerTrackStore` sleduje pozice ostatních dronů
- `SafetyDiskZone` — dynamická no-go zóna s hard a soft shellem
- `peer_prediction_s: 2.0` — predikce polohy peers do budoucnosti

**4. Heightmap — terrain mapping**

Soubor [heightmap.py](../../src/scout_control/scout_control/mapping/heightmap.py):
- `Heightmap2D` — 2.5D terrain model, min NED z per cell
- `update_from_points()` — akumulace depth bodů do terrain mapy
- Výstup: `perimeters/field_model/heightmap_<ts>.json`

**5. Obstacle extraction**

Soubor [obstacle_extractor.py](../../src/scout_control/scout_control/mapping/obstacle_extractor.py):
- BFS/DFS grid clustering bez externích závislostí — čistá implementace connected-components
- `Obstacle` dataclass s centroid, bbox, point_count a confidence

**6. ArUco detekce pro přistání**

Soubor [pad_detector.py](../../src/scout_control/scout_control/vision/pad_detector.py)
a [precision_landing.py](../../src/scout_control/scout_control/vision/precision_landing.py):
- ArUco marker detekce přes OpenCV
- Výpočet offsets a publikace na `/{drone}/precision_landing/offset`

**7. 360° skenovací protokol**

Soubor [scan_manager.py](../../src/scout_control/scout_control/avoidance/scan_manager.py):
- Strukturovaný hover + spin protokol pro dense scan prostředí
- Výsledky jdou do `LocalMapper.ingest_point_batch()` s `is_dense_scan=True`

**ROS2 topiky s percepcí:**

| Topic | Popis |
|---|---|
| `/drone_N/depth/image_raw` | depth frame vstup |
| `/drone_N/camera/image_raw` | RGB vstup |
| `/drone_N/camera/camera_info` | CameraInfo pro kalibraci |
| `/drone_N/precision_landing/offset` | ArUco offset výstup |
| `/drone_N/avoidance/status` | stav mapovací pipeline |
| `/swarm/mapping_complete` | signál ukončení mapping mise |
| `/swarm/mapping_progress` | průběh mapping skenu |

---

### Co by Perception specialist dělal jinak nebo hlouběji v Scoutu

**1. Senzorová fúze — depth + LiDAR + RGB**

Současná architektura: depth camera dominuje, LiDAR se pouze bridguje přes `ros_gz_bridge` jako raw topic,
`obstacle_avoidance_runtime` ho aktivně nekonzumuje do occupancy mřížky.

Co by se změnilo: implementace Kalman filtru nebo faktor-graph fúze kde depth a LiDAR se weightují
na základě confidence a senzorové charakteristiky (depth je přesný blízko, LiDAR daleko).

**2. Dynamická kalibrace kamery**

Aktuálně: intrinsics jsou buď ze statického hfov fallbacku (71.9°) nebo z `CameraInfo` zprávy.
V Isaac Sim `camera_info` není garantován (`CLAUDE.md`).

Co by se změnilo: online kalibrace přes checkerboard nebo RANSAC-based self-calibration
z feature matches mezi framy.

**3. Sémantická segmentace terrain vs. obstacles**

`obstacle_extractor.py` dělá geometrickou klasifikaci (výška nad terénem = překážka).
Co by se přidalo: neural net segmentace která by rozlišila plodiny/stromy/budovy/lidi —
důležité pro zemědělský UAV kde plodiny nejsou překážky i když mají výšku.

**4. Dense visual odometry / VIO**

Projekt závisí na PX4 local position z IMU+GPS. Co chybí: visual-inertial odometry jako záloha
v GPS-denied prostředí nebo pro vyšší přesnost při postřiku.

**5. Probabilistický occupancy grid**

Aktuální: deterministické confidence thresholding (0.55 / 0.90 increment, clip na 3.0).
Co by se zlepšilo: Bayesian occupancy grid s log-odds reprezentací — lépe modeluje neurčitost
a umožňuje správnou fúzi z více senzorů.

**6. 3D obstacle model**

`LocalMapper` je 2D grid — ztratí výškovou informaci překážek.
Percepční specialist by implementoval voxelovou mapu (OctoMap nebo podobnou)
a plánoval ve 3D, ne jen v XY rovině.

---

### Core knowledge to master

#### Matematika a teorie

- **Projektivní geometrie** — pinhole model, homogenní souřadnice, přímka/rovina projekce
  (základ je v `depth_projector.py:240-246`, ale chybí distorce)
- **Pravděpodobnostní modely** — Bayesian update rules, log-odds representation, Gaussian mixture models
- **Rotace a transformace** — SO(3), quaternions, Lie groups (SO(3)/SE(3)) — kritické pro
  body-to-world transformace
- **Spektrální metody** — pro zpracování point cloudů (PCL, normals estimation, ICP)
- **Výpočetní geometrie** — Voronoi, Delaunay, convex hull — pro obstacle representation

#### Klíčové algoritmy

- **ICP (Iterative Closest Point)** — registrace point cloudů
- **SLAM** (GMapping, Cartographer, RTAB-Map) — simultaneous localization and mapping
- **Visual Odometry / VIO** — ORB-SLAM3, VINS-Mono, Kimera
- **Stereo matching** — Semi-Global Matching, disparity estimation
- **Object detection** — YOLO, Faster R-CNN pro real-time aerial detection
- **Semantic segmentation** — DeepLab, SegFormer pro terrain classification

#### Nástroje a frameworky

- **PCL (Point Cloud Library)** — v Scoutu se nepoužívá, ačkoliv scan_manager akumuluje body
- **Open3D** — modernější alternativa PCL
- **OpenCV** — používá se v `scan_manager.py` (import cv2) a `pad_detector.py`
- **PyTorch/TensorFlow** — pro neural perception pipeline
- **ROS2 image_transport, cv_bridge** — používá se v `precision_landing.py`
- **depth_image_proc** — ROS2 package pro point cloud z depth — nahradilo by část `depth_projector.py`

---

### Aktivní výzkumné oblasti (2024–2025)

**Neřešené problémy:**

- **Real-time semantic 3D mapping** na embedded hardware (NVIDIA Orin, Raspberry Pi 5)
  bez GPU cloudu — Scout runtime běží na CPU, neural sítě jsou zatím mimo rozsah
- **Degraded perception v adverse weather** — dron v mlze, dešti, prachu — depth kamera selhává
- **Long-term map consistency** — aktuální `local_mapper.py` je rolling window (36×36 m),
  nemá globální konzistenci
- **Multi-modal sensor fusion bez přesné kalibrace** — fúze senzorů různých typů bez
  přesné extrinsic kalibrace je otevřený problém

**Kam se obor pohybuje:**

- **3DGS (3D Gaussian Splatting)** — ultra-rychlá 3D scene rekonstrukce, 2024 exploduje
  v robotice
- **NeRF pro mapování** — implicitní neural reprezentace prostoru, pomalu se dostává do
  real-time range (Instant-NGP)
- **Foundation models pro perception** — Segment Anything (SAM), DINOv2 jako base features
  pro downstream robotic tasks
- **Event cameras** — asynchronní vision senzory, ideální pro rychlý pohyb dronů
- **4D Occupancy Prediction** — predikce dynamického prostředí do budoucnosti

**Klíčové práce a skupiny:**

- Davide Scaramuzza (ETH Zürich) — agile drone perception, event cameras
- Jitendra Malik (UC Berkeley) — visual representation learning
- CVPR 2024/2025 — robotická sekce každoročně
- RSS, IROS — přednější pro robotic perception

---

### Nejlepší masterské programy

| # | Program | Zaměření | Fit s Scout background |
|---|---|---|---|
| **1** | **ETH Zürich — Robotics, Systems and Control** | Scaramuzza lab, agile drone perception, VIO | Přímý fit — aktivně pracují s dronovou percepcí a real-time mapping |
| **2** | **TU Delft — Aerospace Engineering (MAV track)** | MAV perception, depth sensing, bio-inspired vision | Fit — kombinace aerospace a perception, otevřená pro PX4 projekty |
| **3** | **Carnegie Mellon — Robotics Institute (MRSD nebo PhD)** | SLAM, point cloud processing, semantic mapping | Silný v implementaci, výborný pro depth pipeline rozvoj |
| **4** | **MIT CSAIL / AeroAstro** | Aerial robotics, SLAM, sensor fusion | Prestiž, ale soutěž velká; výzkum je more theoretical |
| **5** | **Imperial College London — Dyson Robotics Lab** | Dense SLAM (ElasticFusion), KinectFusion navazovači | Výborný pro dense 3D mapping — přímý rozvoj `local_mapper.py` |

**Který program nejlépe sedí na Scout background:**
ETH Zürich nebo TU Delft. Scout má reálný depth pipeline, PX4 stack a outdoor drone zkušenost — přesně to, co tyto programy vyžadují od uchazečů.

---

### Kariérní cesta k zakladateli

```
Teď (Scout — depth pipeline, occupancy grid, ArUco)
         ↓
Magisterský výzkum (ETH/TU Delft, 2–3 roky)
  — publish paper na RSS/IROS o agile perception nebo semantic mapping
  — kontribuce do open-source SLAM projektu
         ↓
Seniorsní role (2–4 roky)
  Optionka A: Skydio, Zipline, Joby — perception engineer pro UAV
  Optionka B: Waymo/Motional/Mobileye — perception pro AV (přenositelné)
  Optionka C: výzkumný inženýr v akademii/labu
         ↓
Deep-tech founder
  — Percepční startup pro precision agriculture (přímé pokračování Scout)
  — Aerial inspection AI pro infrastrukturu (mosty, elektrické vedení)
  — Off-road autonomous mapping platforma
  Runway: ~7–10 let od teď s PhD/MSc cestou
  Investor pitch: "postavili jsme toto v Scout, teď to škálujeme"
```

**Kritický enabler:** Paper na tier-1 konferenci (RSS, IROS, CVPR) je vstupní vstupenka.
Bez toho je obtížné dostat funding pro deep-tech perceptual startup.

---

---

## Option B — Robot Planning & Decision-Making Engineer

### Co v Scoutu už dělám, co je Planning

Plánovací vrstva je v Scoutu nejlépe vyvinutou částí. Existuje ve čtyřech úrovních:

**1. Lokální motion planner — A* s multi-layer cost mapou**

Soubor [local_planner.py](../../src/scout_control/scout_control/avoidance/local_planner.py)
je plně funkční hierarchický plánovač:
- `_try_direct_path()` → `_try_drift_path()` → `_try_astar_candidates()` — fallback hierarchie
- A* implementace s 8-connectivity a custom cost funkcí:
  `step_cost = dist + inflation*0.1 + blocked_cost*0.1 + peer_cost*0.1 + heading_penalty + turn_penalty`
- `_simplify_path()` — string-pulling optimalizace výsledné trasy
- `_approx_clearance()` — aproximace corridor width
- Ring-based candidate generation pro subgoal sampling
- `PlannerResultStatus`: `DIRECT` / `DETOUR` / `NO_PATH` / `BLOCKED` — stavový výsledek s důvodem

**2. Swarm task allocation — boustrophedon + dynamic rebalancing**

Soubor [task_allocator.py](../../src/scout_control/scout_control/utils/task_allocator.py):
- Fáze 1: statické sektorové přiřazení (sloupce gridu rovnoměrně mezi N dronů)
- Fáze 2: dynamické vyvažování — `ceil(remaining/2)` buněk ze zatíženého dronu
- `_snake_pattern()` — boustrophedon ordering (can be row-major or col-major)
- `blocked_severity` semantika: `NONE | SOFT | HARD`
- `CELL_DEFERRED` a `TEMP_BLOCKED` cooldown — základní task retry logika

**3. Flight phase state machine**

Soubor [flight_phase_machine.py](../../src/scout_control/scout_control/avoidance/flight_phase_machine.py):
- `FlightPhaseMachine` — generický phase owner s `tick()` a `transition_to()`
- `PhaseTransition` — immutable záznam o přechodu s důvodem a timestampem
- Phases v runtime: `IDLE → TAKEOFF → CRUISE → AVOIDING → RETURN_HOME → LANDING`

**4. Mission-level state management**

Soubor [swarm_agent.py](../../src/scout_control/scout_control/core/swarm_agent.py):
- `Phase` enum: `IDLE / TAKEOFF / ROTATE / CRUISE / AVOIDING / RTH`
- Řízení fronty buněk s prefetch logikou
- Odvozování `CELL_COMPLETE` z runtime pole `last_completed_target_id`
  (ne z vlastního control loop) — klíčový architektonický vzor

**5. Mapping mission planner**

Soubor [mapping_mission.py](../../src/scout_control/scout_control/missions/mapping_mission.py)
a [lawnmower.py](../../src/scout_control/scout_control/utils/lawnmower.py):
- `generate_lawnmower()` — scan coverage plánování s polygon clipping
- MappingPhase state machine: `IDLE → TAKEOFF → MAPPING → RTH → DONE`

**6. Peer conflict avoidance**

Soubor [peer_tracks.py](../../src/scout_control/scout_control/avoidance/peer_tracks.py):
- `SafetyDiskZone` — dynamická no-go zóna pro každý peer drone
- Velocity prediction: `center_ned + velocity * lookahead_s`
- Hard/soft radius shell — graduovaná penalizace v A* cost mapu

**ROS2 topiky s plánováním:**

| Topic | Popis |
|---|---|
| `/drone_N/avoidance/target_cmd` | high-level goto příkazy do runtime |
| `/drone_N/next_cell` | přiřazení buňky od task_allocatoru |
| `/swarm/task_status` | stav celé mise pro GCS |
| `/swarm/rth_request` | signál pro RTH |
| `/drone_N/avoidance/status` | planner state + blocked_reason |
| `/field/mission_confirm` | start mise z GCS |

---

### Co by Planning specialist dělal jinak nebo hlouběji v Scoutu

**1. Formální coverage planning**

Aktuální `lawnmower.py` generuje pravidelné linie přes polygon bez ohledu na terén nebo detekované překážky.

Co by se změnilo: **Boustrophedon Cell Decomposition** nebo **Morse-based coverage** — algoritmy
které berou tvar polygonu a překážky jako vstup a generují provably-complete coverage path.
Navázání na `obstacle_extractor.py` výstupy pro adaptivní vynechání překážek.

**2. Multi-robot koordinace s formálními zárukami**

`task_allocator.py` je heuristický — static sector + steal logika. Neexistuje žádná formální
záruka optimality ani collision-free garancia na mission úrovni.

Co by se přidalo:
- **CBSH (Conflict-Based Search with Heuristics)** pro optimální multi-agent path planning
- Nebo **prioritized planning** s reservation tables
- Topiky `/{drone}/avoidance/status` s `no_path_streak` by sloužily jako vstup pro
  globální re-planning spíše než jen lokální retry

**3. Replanning při detekci no-fly zón**

Bridge protokol (v1.3) posílá `MSG_NO_GO_OVERLAY` z GCS do runtime.
Aktuálně se no-go zóny zobrazují v mapě ale nespouštějí mission-level replanning.

Co by se přidalo: reaktivní přeplánování celé mise (ne jen lokálního detour)
pokud no-go zóna zasáhne naplánovanou trasu.

**4. Temporal logic pro mission specifikaci**

Mise je aktuálně specifikovaná jako ordered list buněk z gridu.
Co by se dalo přidat: specifikace pomocí Linear Temporal Logic (LTL) —
"navštiv buňky A, B, C v libovolném pořadí, ale nikdy neleť přes D dokud E není hotové".
Překlad LTL formulí na automaty pro řízení swarm_agent.

**5. Informativní plánování pro mapping**

`mapping_mission.py` letí fixní lawnmower trajektorií bez ohledu na to, kde už je heightmap hustá.

Co by se přidalo: **informative path planning** — adaptivní trajektorie která maximalizuje
mutual information o neznámých oblastech. Runtime by dostával feedback o tom,
kde je heightmap sparse, a plánoval lety do těchto oblastí.

**6. Koordinace swarm přes centralizovaný vs. decentralizovaný model**

Aktuální: `swarm_coordinator` (centralizovaný task allocator) + `swarm_agent` (per-drone executor).
Co by bylo zajímavé: **auction-based task allocation** nebo **DCOP (Distributed Constraint Optimization)**
— každý agent optimalizuje svůj výběr buněk s ohledem na ostatní,
bez centrálního koordinátora.

---

### Core knowledge to master

#### Matematika a teorie

- **Graph theory** — shortest path (Dijkstra, A*), minimum spanning trees, planarity
- **Computational geometry** — polygon decomposition, Voronoi diagrams, visibility graphs
- **Markov Decision Processes (MDP)** a **POMDP** — decision making pod nejistotou,
  základ pro robustní task planning
- **Linear Temporal Logic (LTL) / CTL** — formální specifikace chování
- **Game theory** — Nash equilibrium pro multi-agent konflikty
- **Optimization** — LP, ILP, convex optimization pro task assignment

#### Klíčové algoritmy

- **A* a jeho varianty** — D*, Theta*, ARA* — Scout má základní A*, ale chybí anytime varianty
- **CBSH/CBS** — Conflict-Based Search pro multi-agent planning
- **RRT/RRT*** — sampling-based planning pro non-holonomic robots
- **Boustrophedon/Morse decomposition** — coverage planning (Scout má lawnmower, ale ne toto)
- **Auctions a VCG mechanismy** — task allocation v multi-robot systémech
- **SLAM + planning** — integrované mapování a plánování

#### Nástroje a frameworky

- **OMPL (Open Motion Planning Library)** — rozsáhlá planning library, ROS2 ready
- **Nav2** — ROS2 navigation stack, přímý kandidát pro nahrazení části `local_planner.py`
- **MoveIt 2** — pro manipulation planning (méně relevantní pro drony)
- **PettingZoo / RLlib** — pro multi-agent reinforcement learning experimenty
- **Spot** — IBM formal methods tool pro LTL verifikaci

---

### Aktivní výzkumné oblasti (2024–2025)

**Neřešené problémy:**

- **Scalable multi-robot coordination** — CBSH škáluje na desítky agentů, ale stovky jsou problém
- **Planning s uncertaintou ve senzorových datech** — POMDP je řešitelné jen pro malé stavové prostory
- **Safety guarantees pro learning-based planners** — neural planners jsou rychlé,
  ale bez formálních safety záruk — velký problém pro zemědělský/průmyslový UAV
- **Long-horizon planning s krátkodobou re-planningem** — jak správně oddělit tactical a strategic úroveň
- **Heterogeneous swarm coordination** — coordinating drones with different capabilities

**Kam se obor pohybuje:**

- **LLM-based task planning** — GPT-4 jako high-level task planner pro robot missions (2024 trend)
- **Neuro-symbolic planning** — kombinace neural perception s formálními planning garantiemi
- **Diffusion models pro trajectory generation** — generativní modely pro motion planning
- **MARL (Multi-Agent Reinforcement Learning)** — škálovatelná koordinace přes RL místo CBSH
- **World models** — Dreamer, RSSM — pro planning v latentním prostoru

**Klíčové práce a skupiny:**

- Nora Ayanian (USC/Stanford) — multi-robot task allocation
- Kris Hauser (UIUC → Duke) — motion planning pod nejistotou
- Sven Koenig (USC) — CBSH, multi-agent pathfinding
- ICAPS konference — dominantní pro task a motion planning výzkum
- RSS workshop on Multi-Robot Systems

---

### Nejlepší masterské programy

| # | Program | Zaměření | Fit s Scout background |
|---|---|---|---|
| **1** | **CMU Robotics Institute (MRSD/PhD)** | Task & motion planning, SLAM+planning, multi-robot | Nejsilnější implementační zázemí, přímý fit pro Scout architektuře |
| **2** | **MIT CSAIL — Robust Robotics Group** | POMDP, uncertainty-aware planning, LTL | Výborný pro formální aspekty; chce silný math background |
| **3** | **Stanford AI Lab** | RL-based planning, LLM task planning, MARL | Módní témata 2025; nebezpečí drift od robotics do AI |
| **4** | **ETH Zürich — Autonomous Systems Lab (ASL)** | MPC pro drony, trajectory optimization, NMPC | Přímý fit pro PX4/Gazebo stack; Alexey Dosovitskiy group |
| **5** | **KTH Stockholm — RPL lab** | Multi-robot planning, coverage, task allocation | Výborný fit: coverage planning a swarm jsou jejich specialty |
| **6** | **University of Pennsylvania — GRASP Lab** | Multi-robot systems, swarm, coverage planning | Legendární pro swarm robotics, Kumar group |

**Který program nejlépe sedí na Scout background:**
CMU nebo UPenn GRASP. Scout má funkční swarm koordinaci, task allocation a planner — to je přesně
to, co obhájíte jako základ pro research na téma scalable multi-robot planning.

---

### Kariérní cesta k zakladateli

```
Teď (Scout — A* planner, task allocator, flight phase SM, swarm coord)
         ↓
Magisterský výzkum (CMU/UPenn, 2–3 roky)
  — publish paper na ICAPS/RSS o multi-robot coverage nebo CBSH variantě
  — open-source planning library contribution (Nav2, OMPL)
         ↓
Seniorsní role (2–4 roky)
  Optionka A: Boston Dynamics, Locus Robotics — planning pro warehouse robot
  Optionka B: Joby, Wisk, AutoFlight — mission planning pro eVTOL
  Optionka C: autonomní zemědělský startup (John Deere AI, Monarch Tractor)
         ↓
Deep-tech founder
  — Autonomní zemědělský swarm pro precision spraying (přímé pokračování Scout)
  — Multi-UAV inspection platform (energie, infrastruktura)
  — Indoor/outdoor multi-robot logistics platforma
  Runway: ~6–9 let od teď (planning je více deployment-ready než perception)
  Investor pitch: "Scout zpracovává X hektarů autonomně, teď to nabízíme jako SaaS"
```

**Kritický enabler:** Planning je méně základní výzkum a více inženýrství —
výstup z masterky může být patent nebo open-source platform, ne jen paper.
To je pro zakladatele potenciálně silnější pozice.

---

---

## Head-to-head: který fit lépe na tento projekt a tohoto stavitele

### Přímé porovnání přes Scout architekturu

| Kritérium | Perception | Planning |
|---|---|---|
| Počet souborů přímo v doméně | 7 (`depth_projector`, `local_mapper`, `scan_manager`, `heightmap`, `obstacle_extractor`, `pad_detector`, `precision_landing`) | 8 (`local_planner`, `task_allocator`, `flight_phase_machine`, `swarm_agent`, `mapping_mission`, `lawnmower`, `swarm_coordinator`, `peer_tracks`) |
| Hloubka implementace | Solidní depth pipeline, occupancy grid | Plný A* s cost layers, multi-agent allocation, state machine |
| Chybějící vrstva | Dense 3D mapping, sensor fusion, neural perception | Formální coverage guarantees, CBSH, LTL spec |
| Co je deployment-ready | `depth_projector.py` a `local_mapper.py` fungují v produkci | `local_planner.py`, `task_allocator.py` a `flight_phase_machine.py` fungují v produkci |
| Co vyžaduje více výzkumu | Semantic perception, event cameras | Scalable multi-robot, formal verification |
| Přenositelnost mimo drony | Střední (depth/vision je broad) | Vysoká (planning algoritmy jsou domain-agnostic) |
| Time-to-market pro startup | Delší (AI/ML pipeline je slow) | Kratší (planning systém je spíše software) |

### Čestné hodnocení — která specializace má v Scoutu více základů

**Planning má v Scoutu silnější základ.**

Konkrétní čísla: `local_planner.py` má 728 řádků čistého Python planneru s A*, multi-layer cost functions,
path simplification a hierarchickým fallback systémem. `task_allocator.py` implementuje multi-agent
boustrophedon allocation s dynamic rebalancing. `flight_phase_machine.py` je produkčně nasazený
generický state machine. Toto jsou zralé implementace.

Oproti tomu perception: `depth_projector.py` je solidní pinhole projekce, ale zastavuje se tam.
Není tu probabilistický occupancy grid, není senzorová fúze, není semantic segmentation.
Perception pipeline je funkční, ale jednoúčelová — spíše jako engineering output než jako
research-grade základ.

### Finální doporučení

**Doporučení: Option B — Robot Planning & Decision-Making Engineer**

**Důvody:**

1. **Scout je planning-heavy projekt.** `local_planner.py` + `task_allocator.py` + `swarm_coordinator.py`
   jsou základ, který přímo přechází do akademického výzkumu. Obhajoba research proposal s "já mám
   v produkci fungující multi-robot task allocator pro swarm UAV" je silnější vstup pro CMU nebo UPenn
   než "mám depth camera pipeline".

2. **Planning specialista má kratší cestu k produktu.** Planning systémy jsou software —
   nevyžadují specifický hardware, jsou laditelné, testovatelné. Perception vyžaduje senzory,
   annotovaná data, compute. Pro zakladatele je to důležité.

3. **Scout agricultural use case je planning problem.** Optimální coverage, minimální overlap,
   react na nefunkční dron, no-fly zóny — to jsou planning problémy. Perception je enabler,
   ale core hodnota produktu je v tom, jak systém rozhoduje a přiřazuje práci.

4. **Complement, ne duplicate.** Jako Planning specialist nepotřebuješ obětovat percepci —
   `local_planner.py` konzumuje `LocalMapper` výstupy a ten bude stále vyvíjen.
   Planning specialist rozumí celému stacku, ale exceluje v decision layer.

5. **Timing výzkumu.** LLM-based task planning, world models a MARL jsou horká témata 2024–2025.
   Podat přihlášku na MSc s Scout background + interest v LLM+planning kombinaci
   je relevantní pro aktuální výzkumné agendy.

**Výhrada:** Pokud tě víc baví práce s daty a obrazem než s grafy a optimalizací —
Perception dává smysl i tak. Scout depth pipeline je dobrý základ. Ale čistě z hlediska
kde má Scout víc hotového a kde je kratší cesta od teď k deep-tech foundarateli,
vychází Planning lépe.

---

*Vygenerováno 2026-04-29 z analýzy kódu scout_ws workspace.*
