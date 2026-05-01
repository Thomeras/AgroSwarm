"""
Swarm Center — Ground Control Station for Scout Autonomous System
=================================================================

Samostatná PyQt6 aplikace pro ovládání roje dronů. Komunikuje s PX4 SITL
instancemi přes MAVLink (UDP). Pro Isaac Sim / Gazebo sim běží mimo
ROS2 workspace — je to externí GCS aplikace.

Architecture:
  Drone 0 (PX4 SITL) ──MAVLink UDP 14540──┐
  Drone 1 (PX4 SITL) ──MAVLink UDP 14541──┼──► Swarm Center
  Drone N (PX4 SITL) ──MAVLink UDP 1454N──┘      │
                                                  ├─► Grid awareness broadcast
                                                  ├─► Task allocation
                                                  └─► Aggregated scan data → AI

Milestone 1 (this file):
  [x] PyQt6 main window
  [x] Field view with grid overlay
  [x] MAVLink connection to one or more drones
  [x] Live drone positions on map
  [x] Drone list with grid-cell position

Later milestones:
  [ ] Multi-drone task allocation UI
  [ ] ROS2 bridge for camera/depth data
  [ ] RTAB-Map 3D visualisation
  [ ] Mode switcher (mapping / spraying / checking)

Usage:
  python3 main.py
  python3 main.py --drones 2 --base-port 14540

Requirements:
  pip install PyQt6 pymavlink
"""

import argparse
import sys
import signal

from PyQt6.QtWidgets import QApplication

from ui.main_window import MainWindow
from ui.style import DARK_QSS


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Scout Swarm Center — GCS")
    p.add_argument(
        "--drones", type=int, default=2,
        help="Number of drones in the swarm (default: 2)")
    p.add_argument(
        "--base-port", type=int, default=14540,
        help="UDP base port for MAVLink. Drone N listens on base+N (default: 14540)")
    p.add_argument(
        "--host", type=str, default="127.0.0.1",
        help="MAVLink bind host (default: 127.0.0.1)")
    p.add_argument(
        "--bridge-host", type=str, default="127.0.0.1",
        help="ROS2 bridge TCP host (default: 127.0.0.1)")
    p.add_argument(
        "--bridge-port", type=int, default=17845,
        help="ROS2 bridge TCP port (default: 17845)")
    p.add_argument(
        "--grid-file", type=str, default=None,
        help="Path to field_grid.json (optional — Swarm Center can load later)")
    p.add_argument(
        "--cell-size", type=float, default=5.0,
        help="Grid cell size in metres, used if no grid file loaded (default: 5.0)")
    p.add_argument(
        "--world-image", type=str, default=None,
        help="Path to overhead PNG (top-down satellite/aerial view). "
             "A sidecar .json with ned_x_min/max/y_min/max aligns it to NED coords. "
             "Example: ../worlds/agro_field_overhead.png")
    p.add_argument(
        "--origin-file", type=str, default=None,
        help="Optional per-drone PX4 local-origin file written by scout_launcher.")
    return p.parse_args()


def main() -> int:
    args = parse_args()

    # Allow Ctrl+C to kill the app cleanly
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QApplication(sys.argv)
    app.setApplicationName("Scout Swarm Center")
    app.setStyleSheet(DARK_QSS)

    window = MainWindow(
        drone_count=args.drones,
        base_port=args.base_port,
        host=args.host,
        grid_file=args.grid_file,
        default_cell_size=args.cell_size,
        bridge_host=args.bridge_host,
        bridge_port=args.bridge_port,
        world_image=args.world_image,
        origin_file=args.origin_file,
    )
    window.show()

    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
