## Codex Log — 2026-04-19

### Kontext

- Prosel jsem `CLAUDE.md` a workflow kolem `launch_sim_e2e.txt`.
- Resila se Isaac Sim / Pegasus E2E cesta, kde nefungoval camera bridge a 3D mapa pole.

### Co bylo upraveno

- `src/scout_control/scout_control/gcs_bridge.py`
  - pridan parametr `camera_topic_template`
  - pridan parametr `depth_topic_template`
  - `gcs_bridge` uz nema natvrdo jen `/{drone_id}/camera/image_raw`, ale umi topic templaty

- `src/scout_control/launch/isaac_e2e_mission.launch.py`
  - launch ted predava do `gcs_bridge`:
    - `camera_topic_template`
    - `depth_topic_template`
  - default zustava:
    - `/drone_{index}/camera/image_raw`
    - `/drone_{index}/depth/image_raw`

- `swarm_center/ui/main_window.py`
  - opraven start 3D mapy pole
  - `Viewport3D` dostane grid hned po startu pres `set_grid(grid)`

- `src/scout_control/setup.py`
  - doplnen `launch/isaac_e2e_mission.launch.py` do instalovanych launch souboru

- `launch_sim_e2e.txt`
  - byl prubezne aktualizovan podle noveho stavu projektu
  - pozor: pozdeji ho menil i dalsi agent, tak pri dalsi praci znovu overit obsah

### Ověřené skutečnosti

- `colcon build --packages-select scout_control` probehl uspesne.
- Build spam z `~` nebyl chyba `scout_control`, ale tim, ze `colcon` prochazel cizi virtualenvy:
  - `isaac_env`
  - `isaac_env_511`
  - dalsi `venv` mimo workspace
- Spravne spousteni buildu:

```bash
cd /home/tj/_Data/_Projekty/TJlabs/scout_ws
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select scout_control
source install/setup.bash
```

### Stav kamery v Isaac vetvi

Bylo overeno:

- `gcs_bridge` bezi:

```bash
ros2 node list | grep gcs_bridge
```

- topicy existuji:

```bash
/drone_0/camera/image_raw
/drone_0/depth/image_raw
/drone_1/camera/image_raw
```

- ale `ros2 topic info /drone_0/camera/image_raw -v` ukazal:

```text
Publisher count: 0
Subscription count: 4
```

Zaver:

- problem neni v `gcs_bridge`
- problem neni v topic name na ROS2 strane
- problem je na Isaac / Pegasus strane
- kamera jako prim ve scene existuje (`/World/quadrotor/body/camera`), ale z dostupnych testu nevyplyva, ze je napojena na aktivni ROS2 publisher pipeline
- samotne `Type = Camera` nestaci; chybi nebo neni aktivni ROS2 camera publisher / helper / render pipeline

### Stav 3D mapy pole

- puvodni problem ve `Swarm Center` byl, ze `Viewport3D` nedostal grid hned pri startu
- to bylo opraveno
- pokud 3D viewport stale nefunguje, dalsi podezreni:
  - neexistuje `perimeters/field_grid.json`
  - chybi `pyqtgraph` nebo `PyOpenGL` v `swarm_center`

### Co zatim NEBYLO vyreseno

- Isaac kamera stale realne nepublikuje do ROS2
- v repu nebyl nalezen zadny skript nebo config, ktery by pro `agro_field.usd` automaticky aktivoval ROS2 camera helper
- `isaac_launcher.py` jen otevre Isaac a navadi operatora:
  - otevrit `agro_field.usd`
  - loadnout Iris
  - dat Play

### Doporuceny dalsi krok

Pri dalsim sezeni jit uz po Isaac Sim / Pegasus konfiguraci:

1. V Isaac UI dohledat, jak je `/World/quadrotor/body/camera` napojena na render product.
2. Najit Action Graph / OmniGraph / ROS2 helper pro kameru.
3. Ověřit, že po `Play` vznikne realny publisher:

```bash
ros2 topic info /drone_0/camera/image_raw -v
```

Cilovy stav:

```text
Publisher count: 1
```
