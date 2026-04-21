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

- `launch_sim_e2e.txt`
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
