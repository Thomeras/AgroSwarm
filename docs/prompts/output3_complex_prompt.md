Jsi seniorni systemovy architekt a ROS2/PX4 engineer. Analyzuj projekt `scout_ws` pouze z pohledu jedneho dronu, ne celeho swarmu ani field-level planningu. Zamer se jen na drone-level architekturu: letovy runtime, PX4 interface, manual ovladani, lokalni polohu, validitu estimatoru, kameru, depth/lidar, terrain following, health/readiness senzoru, topic contracts a degradaci pri vypadku dat.

Aktualni stav ber jako tento:
- Na urovni jednoho dronu je system funkcni, ale stale spise prototyp.
- Existuje PX4 local position, manual ovladani, camera bridge/HUD a zaklad terrain/depth workflow.
- Nejvetsi slabina je nesjednocena sensor vrstva.
- Chybi jasne contracts pro kameru, depth, lidar a polohu.
- Chybi robustni health/readiness checky a degradace pri vypadku dat.
- Je tam moc hardcoded topicu a predpokladu pro konkretni drony.
- Manual control je silny pro setup/debug, ale neni to jeste cista operatorska vrstva.
- Je potreba dodelat integraci downward range do hlavniho runtime, camera/depth metadata, estimator validity stav a jednotny per-drone status.

Tvuj ukol:
1. Zhodnotit, jak vypada dnes architektura jednoho dronu.
2. Pojmenovat silne stranky a hlavni technicke dluhy.
3. Presne rict, co chybi k robustni production-grade drone vrstve.
4. Navrhnout cilovy drone-level model odpovednosti mezi moduly.
5. Doporucit konkretni dalsi kroky implementace v realistickem poradi.

Pri analyze se zamer hlavne na tyto oblasti:
- PX4 offboard ownership a hranice mezi mission vrstvou, manual control vrstvou a runtime vrstvou.
- Telemetrie a poloha: `VehicleLocalPosition`, validity flags, EKF readiness, failover pri chybejicich datech.
- Kamera: image topic, camera metadata, camera_info, stale frame detection, naming conventions, per-drone namespacing.
- Depth / lidar / downward range: jejich role, integrace do runtime, quality checks, persistence a fallback chovani.
- Manual control a setup utility: co ma zustat jako tooling a co nema lezet v produkcni flight-control ceste.
- Jednotny per-drone status model: readiness, sensor health, estimator health, navigation mode, degraded mode, blocked state, landing/RTH stav.
- Topic/API kontrakty: co ma byt typed ROS message, co muze zustat JSON a co je potreba sjednotit.
- Konfigurovatelnost a odstraneni hardcoded predpokladu na `drone_0`, `drone_1` a konkretni topic names.

Pozadovany vystup:
- Strucny executive summary.
- Seznam hlavnic findingu od nejdulezitejsich po mene dulezite.
- Navrh cilove architektury pro jednoho dronu.
- Seznam konkretnich veci, ktere je potreba dodelat.
- Kratka priorizovana roadmapa po krocich.

Pis vecne, technicky a bez marketingu. Kdyz neco kritizujes, vzdy rekni i proc je to problem v praxi. Kdyz neco navrhujes, drz se realistickeho ROS2/PX4 workflow a neabstrahuj do generickych poucek. Vystup ma byt pouzitelny jako podklad pro refactor a implementacni plan.
