"""
ml_interface.py — tooling-only ML stub node (Fáze 1: dummy data)

Publikuje tři syntetické výstupní topicy pro vývoj a testování navazujících
nodů. Tento node není produkční ML inference vrstva a jeho data se nesmí
považovat za agronomickou predikci.

Topics (publish):
  /field/anomaly      std_msgs/String  JSON — seznam anomálií v buňkách pole
  /field/cell_health  std_msgs/String  JSON — skóre zdraví 0–1 per buňka
  /drone/spray_dose   std_msgs/String  JSON — variabilní dávka postřiku per dron (ml/m²)

Formát /field/anomaly:
  {
    "stamp": 1234567890.123,
    "anomalies": [
      {"cell_id": "x5_y3", "type": "drought",   "confidence": 0.87},
      {"cell_id": "x2_y7", "type": "pest",      "confidence": 0.61}
    ]
  }

Formát /field/cell_health:
  {
    "stamp": 1234567890.123,
    "scores": {
      "x0_y0": 0.92,
      "x1_y0": 0.45,
      ...
    }
  }

Formát /drone/spray_dose:
  {
    "stamp": 1234567890.123,
    "doses": {
      "drone_0": 1.2,
      "drone_1": 2.7
    }
  }

Parametry:
  publish_hz         (float, default 1.0)   — frekvence publishování
  drone_count        (int,   default 2)     — počet dronů
  anomaly_threshold  (float, default 0.35)  — skóre pod touto hodnotou = anomálie
  max_spray_dose     (float, default 3.0)   — maximální dávka postřiku (ml/m²)

Spuštění:
  ros2 run scout_control ml_interface
  ros2 run scout_control ml_interface --ros-args -p publish_hz:=0.5 -p drone_count:=2
"""

import json
import math
import os
import random
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String

from scout_control.utils.paths import GRID_FILE

# ── QoS ──────────────────────────────────────────────────────────────────────
QOS_LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
QOS_VOLATILE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

TOOLING_PLACEHOLDER = True
MODEL_MODE = "stub_tooling_placeholder"

# ── Anomálie typy pro dummy generátor ────────────────────────────────────────
ANOMALY_TYPES = ["drought", "pest", "disease", "nutrient_deficiency", "waterlogging"]


# ── Dummy data generátor ─────────────────────────────────────────────────────

class DummyFieldModel:
    """
    Fáze 1: Generuje syntetická pole zdraví + anomálie pomocí sinusových vln + šumu.

    Hodnoty jsou deterministické z hash(cell_id) — funguje pro libovolnou sadu buněk
    bez předalokace, takže grid se může měnit za běhu.

    Fáze 2 — NAHRADIT:
      - Načíst kamerový obraz z /camera/image_raw
      - Spustit CNN model pro detekci zdraví vegetace (NDVI, chlorofyl index)
      - Vrátit predikci per buňka
    """

    SEED = 42

    def __init__(self) -> None:
        pass  # žádná předalokace — vše deterministické z hash(cell_id)

    @staticmethod
    def _cell_params(cid: str) -> tuple[float, float]:
        """Deterministická bazální hodnota (0.3–1.0) a fázový offset z hash cell_id."""
        h = abs(hash(cid + str(DummyFieldModel.SEED)))
        base  = 0.3 + (h % 1000) / 1000.0 * 0.7        # 0.3–1.0
        phase = (h % 6284) / 1000.0                      # 0–2π
        return base, phase

    def cell_health_scores(self, cell_ids: List[str], t: float) -> Dict[str, float]:
        """
        Vrátí skóre zdraví 0–1 pro předané cell_ids v čase t.
        Přijímá aktuální seznam buněk — nezávisí na init.

        Fáze 2 — NAHRADIT ML modelem:
          scores = vision_model.predict(camera_frame, cell_positions)
        """
        rng = random.Random(int(t * 1000) & 0xFFFF)  # noise seed per tick, ne per cell
        scores: Dict[str, float] = {}
        for cid in cell_ids:
            base, phase = self._cell_params(cid)
            oscillation = 0.15 * math.sin(t / 120.0 * 2 * math.pi + phase)
            noise = rng.gauss(0.0, 0.03)
            score = max(0.0, min(1.0, base + oscillation + noise))
            scores[cid] = round(score, 3)
        return scores

    def anomalies(self, scores: Dict[str, float], threshold: float) -> List[dict]:
        """
        Vrátí seznam buněk kde skóre < threshold.

        Fáze 2 — NAHRADIT ML modelem:
          detections = anomaly_detector.detect(image_patches, cell_ids)
        """
        result = []
        for cid, score in scores.items():
            if score < threshold:
                # dummy: typ anomálie vybrán deterministicky z cell_id hash
                anomaly_type = ANOMALY_TYPES[hash(cid) % len(ANOMALY_TYPES)]
                confidence = round(1.0 - score / threshold, 3)
                result.append({
                    "cell_id": cid,
                    "type": anomaly_type,
                    "confidence": min(1.0, confidence),
                })
        return result


class DummySprayModel:
    """
    Fáze 1: Mapuje skóre zdraví na dávku postřiku (nízké zdraví = vyšší dávka).

    Fáze 2 — NAHRADIT:
      - Přijmout aktuální pozici dronu
      - Dotázat se agro modelu na optimální dávku pro aktuální buňku
      - Vrátit dávku v ml/m²
    """

    def drone_doses(
        self,
        scores: Dict[str, float],
        drone_count: int,
        max_dose: float,
    ) -> Dict[str, float]:
        """
        Vrátí průměrnou dávku pro každý dron na základě průměrného zdraví v jeho sektoru.

        Fáze 2 — NAHRADIT ML modelem:
          doses = spray_optimizer.compute(drone_positions, cell_health_map)
        """
        if not scores:
            return {f"drone_{i}": 0.0 for i in range(drone_count)}

        cell_list = list(scores.values())
        n = len(cell_list)
        sector_size = max(1, n // drone_count)

        doses: Dict[str, float] = {}
        for i in range(drone_count):
            start = i * sector_size
            end = start + sector_size if i < drone_count - 1 else n
            sector_scores = cell_list[start:end]
            avg_health = sum(sector_scores) / len(sector_scores)
            # inverzní vztah: nízké zdraví → vysoká dávka
            dose = round(max_dose * (1.0 - avg_health), 3)
            doses[f"drone_{i}"] = dose
        return doses


# ── Hlavní node ───────────────────────────────────────────────────────────────

class MLInterface(Node):
    """
    Tooling-only ML stub node — publikuje dummy ML výstupy.

    Fáze 1: Syntetická data pro vývoj a testování navazujících nodů.
    Fáze 2: Nahradit _field_model a _spray_model reálnými ML modely.
    """

    def __init__(self) -> None:
        super().__init__("ml_interface")

        # ── parametry ────────────────────────────────────────────────────────
        self.declare_parameter("publish_hz",        1.0)
        self.declare_parameter("drone_count",       2)
        self.declare_parameter("anomaly_threshold", 0.35)
        self.declare_parameter("max_spray_dose",    3.0)

        self._hz:        float = self.get_parameter("publish_hz").value
        self._n_drones:  int   = self.get_parameter("drone_count").value
        self._threshold: float = self.get_parameter("anomaly_threshold").value
        self._max_dose:  float = self.get_parameter("max_spray_dose").value

        # ── grid — sledujeme mtime, reload při změně ──────────────────────────
        self._cell_ids: List[str] = []
        self._grid_mtime: Optional[float] = None
        self._reload_grid()

        # ── ML modely (Fáze 1 = dummy) ────────────────────────────────────────
        self._field_model = DummyFieldModel()
        self._spray_model = DummySprayModel()

        # ── publishers ───────────────────────────────────────────────────────
        self._pub_anomaly = self.create_publisher(
            String, "/field/anomaly", QOS_VOLATILE
        )
        self._pub_health = self.create_publisher(
            String, "/field/cell_health", QOS_LATCHED
        )
        self._pub_dose = self.create_publisher(
            String, "/drone/spray_dose", QOS_VOLATILE
        )

        # ── timer ─────────────────────────────────────────────────────────────
        period = 1.0 / max(0.1, self._hz)
        self._timer = self.create_timer(period, self._publish)

        self.get_logger().info(
            f"[ml_interface] Start ({MODEL_MODE}) — {len(self._cell_ids)} cells, "
            f"{self._n_drones} drones, {self._hz:.1f} Hz, "
            f"anomaly_threshold={self._threshold}, max_dose={self._max_dose} ml/m²"
        )

    # ── helpers ───────────────────────────────────────────────────────────────

    def _reload_grid(self) -> None:
        """
        Načte cell_ids z field_grid.json pokud soubor existuje a změnil se (mtime).
        Pokud soubor neexistuje, použije fallback 4×4 dummy grid.
        """
        try:
            mtime = os.path.getmtime(GRID_FILE)
        except OSError:
            mtime = None

        if mtime == self._grid_mtime:
            return  # nic se nezměnilo

        if mtime is None:
            if not self._cell_ids:
                self._cell_ids = [f"x{x}_y{y}" for y in range(4) for x in range(4)]
                self.get_logger().warn(
                    "[ml_interface] field_grid.json nenalezen — fallback 4×4 dummy grid"
                )
            return

        try:
            with open(GRID_FILE, "r") as f:
                data = json.load(f)
            new_ids = [c["id"] for c in data.get("cells", [])]
        except (KeyError, json.JSONDecodeError) as e:
            self.get_logger().error(f"[ml_interface] Chyba čtení gridu: {e}")
            return

        old_count = len(self._cell_ids)
        self._cell_ids = new_ids
        self._grid_mtime = mtime
        self.get_logger().info(
            f"[ml_interface] Grid reload: {old_count} → {len(new_ids)} buněk"
        )

    # ── publish callback ─────────────────────────────────────────────────────

    def _publish(self) -> None:
        # reload gridu pokud se field_grid.json změnil (grid_generator přepsal soubor)
        self._reload_grid()

        stamp = self.get_clock().now().nanoseconds / 1e9

        # ── 1. cell health ────────────────────────────────────────────────────
        # Fáze 2: nahradit vision_model.predict(...)
        scores = self._field_model.cell_health_scores(self._cell_ids, stamp)

        health_msg = String()
        health_msg.data = json.dumps({
            "stamp": stamp,
            "model_mode": MODEL_MODE,
            "scores": scores,
        })
        self._pub_health.publish(health_msg)

        # ── 2. anomalie ───────────────────────────────────────────────────────
        # Fáze 2: nahradit anomaly_detector.detect(...)
        anomaly_list = self._field_model.anomalies(scores, self._threshold)

        anomaly_msg = String()
        anomaly_msg.data = json.dumps({
            "stamp": stamp,
            "model_mode": MODEL_MODE,
            "anomalies": anomaly_list,
        })
        self._pub_anomaly.publish(anomaly_msg)

        # ── 3. spray dose ─────────────────────────────────────────────────────
        # Fáze 2: nahradit spray_optimizer.compute(...)
        doses = self._spray_model.drone_doses(scores, self._n_drones, self._max_dose)

        dose_msg = String()
        dose_msg.data = json.dumps({
            "stamp": stamp,
            "model_mode": MODEL_MODE,
            "doses": doses,
        })
        self._pub_dose.publish(dose_msg)

        # log shrnutí (jednou za 10 s, ne každý tick)
        if int(stamp) % 10 == 0 and int(stamp * self._hz) % max(1, int(self._hz * 10)) == 0:
            n_anom = len(anomaly_list)
            avg_h  = sum(scores.values()) / len(scores) if scores else 0.0
            self.get_logger().info(
                f"[ml_interface] health avg={avg_h:.2f}  anomalies={n_anom}  "
                f"doses={doses}"
            )


# ── main ──────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = MLInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
