# Launch Runbooks

Tato slozka obsahuje rucni operatorni postupy a poznamky ke spousteni.

## Soubory

- `phase15_Ndrone_e2e_runbook.txt` — aktualni finalni Gazebo workflow single-drone field setup → N-drone swarm mission
- `launch_info.txt` — obecny prehled spousteni workspace
- `launch_sim_e2e.txt` — Isaac/Pegasus workflow a odkazy na konkretni postupy
- `isaac_phase12_e2e_test.txt` — overeni Phase 1 + Phase 2 v Isaac Sim
- `isaac_phase123_e2e_test.txt` — overeni Phase 1 + Phase 2 + Phase 3 v Isaac Sim
- `isaac_full_e2e_mission.txt` — rucni postup pro Isaac full E2E flow
- `isaac_obstacle_avoidance_test.txt` — rucni postup pro obstacle avoidance test

Tyto soubory jsou runbooky, ne runtime launch soubory.
Skutecne ROS2 launch definice zustavaji v `src/scout_control/launch/`.

## Current Runbook

Pro finalni Phase 1-5 / Phase 15 milestone pouzivej primarne:

- `phase15_Ndrone_e2e_runbook.txt`

Isaac a starsi Phase 1-3 runbooky jsou historicke/reference workflow. Jsou
uzitecne pri ladeni, ale nejsou hlavni finalni demo cesta.
