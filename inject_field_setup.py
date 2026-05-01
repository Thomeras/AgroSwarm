#!/usr/bin/env python3
"""
inject_field_setup.py - Interactive field setup injection.

Publishes pad assignments and polygon boundary vertices directly to
field_setup_coordinator. By default it reuses the currently saved
perimeters/home_positions.json and perimeters/field_boundary.json if present,
then clears generated perimeter files before publishing a clean setup run.

Usage (with ROS2 sourced, simulation + all nodes running):
  source install/setup.bash
  python3 inject_field_setup.py --drone-count 4
  python3 inject_field_setup.py --preset swarm_field --drone-count 4
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# -- Built-in setup presets ----------------------------------------------------

RECORDED_2026_04_29_PADS = [
    {"pad_id": "pad_0", "drone_id": "drone_0", "x": 22.364, "y": -51.284, "z": -0.5},
    {"pad_id": "pad_1", "drone_id": "drone_1", "x": 26.815, "y": -53.298, "z": -0.5},
    {"pad_id": "pad_2", "drone_id": "drone_2", "x": 34.435, "y": -53.228, "z": -0.5},
    {"pad_id": "pad_3", "drone_id": "drone_3", "x": 43.192, "y": -53.360, "z": -0.5},
]

RECORDED_2026_04_29_BOUNDARY = [
    {"x": 51.099, "y": -48.738, "z": -5.059},
    {"x": 47.126, "y":  -9.578, "z": -5.089},
    {"x": 15.691, "y": -10.167, "z": -5.089},
    {"x": 18.440, "y": -42.828, "z": -5.124},
]

SWARM_FIELD_PADS = [
    {"pad_id": "pad_0", "drone_id": "drone_0", "x": -12.0, "y": -26.0, "z": -0.5},
    {"pad_id": "pad_1", "drone_id": "drone_1", "x":  -4.0, "y": -26.0, "z": -0.5},
    {"pad_id": "pad_2", "drone_id": "drone_2", "x":   4.0, "y": -26.0, "z": -0.5},
    {"pad_id": "pad_3", "drone_id": "drone_3", "x":  12.0, "y": -26.0, "z": -0.5},
]

SWARM_FIELD_BOUNDARY = [
    {"x": -20.0, "y": -20.0, "z": -5.0},
    {"x":  20.0, "y": -20.0, "z": -5.0},
    {"x":  20.0, "y":  20.0, "z": -5.0},
    {"x": -20.0, "y":  20.0, "z": -5.0},
]

# -- Files to clear for clean run ---------------------------------------------

WS_ROOT = os.path.dirname(os.path.abspath(__file__))
HOME_POS_FILE = os.path.join(WS_ROOT, "perimeters", "home_positions.json")
BOUNDARY_FILE = os.path.join(WS_ROOT, "perimeters", "field_boundary.json")
GRID_FILE = os.path.join(WS_ROOT, "perimeters", "field_grid.json")
CLEAR_FILES = [
    HOME_POS_FILE,
    BOUNDARY_FILE,
    GRID_FILE,
]

QOS_VOL = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Inject field setup pads and polygon boundary into ROS2."
    )
    parser.add_argument(
        "--preset",
        choices=("saved", "swarm_field", "recorded_2026_04_29"),
        default="saved" if os.path.exists(HOME_POS_FILE) and os.path.exists(BOUNDARY_FILE)
        else "swarm_field",
        help=(
            "Coordinate source. 'saved' reads perimeters/*.json before clearing; "
            "'swarm_field' uses the Gazebo world pad/table coordinates."
        ),
    )
    parser.add_argument(
        "--drone-count",
        type=int,
        default=int(os.environ.get("DRONE_COUNT", "4")),
        help="Number of required pads for the currently launched coordinator.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Publish all points without waiting for Enter at each step.",
    )
    parser.add_argument(
        "--no-clear",
        action="store_true",
        help="Do not delete old perimeter output files before publishing.",
    )
    parser.add_argument(
        "--repeats",
        type=int,
        default=3,
        help="Publish each setup message this many times to avoid BEST_EFFORT drops.",
    )
    parser.add_argument(
        "--discovery-timeout",
        type=float,
        default=6.0,
        help="Seconds to wait for matching ROS subscribers before publishing.",
    )
    return parser.parse_args()


def _sort_by_numeric_id(items: list[dict], key: str) -> list[dict]:
    def _num(item: dict) -> int:
        try:
            return int(str(item.get(key, "")).split("_")[-1])
        except (ValueError, IndexError):
            return 10_000

    return sorted(items, key=_num)


def load_saved_setup() -> tuple[list[dict], list[dict]]:
    with open(HOME_POS_FILE, encoding="utf-8") as f:
        home_payload = json.load(f)
    with open(BOUNDARY_FILE, encoding="utf-8") as f:
        boundary_payload = json.load(f)

    pads = []
    for item in home_payload.get("home_positions", []):
        ned = item.get("ned", {})
        pads.append({
            "pad_id": item.get("pad_id", ""),
            "drone_id": item.get("drone_id", ""),
            "x": float(ned.get("x", 0.0)),
            "y": float(ned.get("y", 0.0)),
            "z": float(ned.get("z", -0.5)),
        })

    boundary = []
    for item in boundary_payload.get("vertices_ned", []):
        boundary.append({
            "x": float(item.get("x", 0.0)),
            "y": float(item.get("y", 0.0)),
            "z": float(item.get("z", -5.0)),
        })

    if not pads or len(boundary) < 3:
        raise ValueError("saved preset needs home_positions and >=3 boundary vertices")
    return _sort_by_numeric_id(pads, "pad_id"), boundary


def select_setup(preset: str) -> tuple[list[dict], list[dict]]:
    if preset == "saved":
        return load_saved_setup()
    if preset == "swarm_field":
        return list(SWARM_FIELD_PADS), list(SWARM_FIELD_BOUNDARY)
    return list(RECORDED_2026_04_29_PADS), list(RECORDED_2026_04_29_BOUNDARY)


def wait_enter(prompt: str) -> None:
    try:
        input(f"\n  {prompt}\n  >>> Press Enter to publish... ")
    except (EOFError, KeyboardInterrupt):
        print("\nAborted.")
        sys.exit(0)


def maybe_wait(prompt: str, auto_prompt: str, auto_yes: bool) -> None:
    if auto_yes:
        print(f"\n  {auto_prompt}\n  >>> auto-publish")
        return
    wait_enter(prompt)


def wait_for_subscribers(node: Node, publisher, topic: str, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if publisher.get_subscription_count() > 0:
            print(f"  subscriber discovered on {topic}")
            return True
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f"  WARNING: no subscriber discovered on {topic}")
    return False


def pub(node: Node, publisher, payload: dict, repeats: int) -> None:
    msg = String()
    msg.data = json.dumps(payload)
    for _ in range(max(1, repeats)):
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)


def main() -> None:
    args = parse_args()
    try:
        pads, boundary_vertices = select_setup(args.preset)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"Cannot load preset '{args.preset}': {exc}", file=sys.stderr)
        sys.exit(2)

    if args.drone_count < 1:
        print("--drone-count must be >= 1", file=sys.stderr)
        sys.exit(2)
    pads = pads[:args.drone_count]
    for i, pad in enumerate(pads):
        pad["pad_id"] = f"pad_{i}"
        pad["drone_id"] = f"drone_{i}"

    if len(pads) < args.drone_count:
        print(
            f"Preset '{args.preset}' only has {len(pads)} pads, "
            f"but --drone-count={args.drone_count}",
            file=sys.stderr,
        )
        sys.exit(2)

    print("\n=== Field Setup Injection ===")
    print(f"Preset: {args.preset} | drone_count={args.drone_count}")
    print(f"Pads: {len(pads)} | boundary vertices: {len(boundary_vertices)}")
    if not args.no_clear:
        print("Clearing old perimeter files for clean run...")
        for path in CLEAR_FILES:
            if os.path.exists(path):
                os.remove(path)
                print(f"  removed {os.path.basename(path)}")

    rclpy.init()
    node = Node("field_setup_injector")

    pad_pub      = node.create_publisher(String, "/swarm/pad_assignment",   QOS_VOL)
    boundary_pub = node.create_publisher(String, "/field/boundary_point",   QOS_VOL)
    close_pub    = node.create_publisher(String, "/field/boundary_close",   QOS_VOL)

    wait_for_subscribers(node, pad_pub, "/swarm/pad_assignment", args.discovery_timeout)
    wait_for_subscribers(node, boundary_pub, "/field/boundary_point", args.discovery_timeout)
    wait_for_subscribers(node, close_pub, "/field/boundary_close", args.discovery_timeout)

    print("\nNodes ready. Coordinator should still be waiting for pad assignments.\n")

    # -- Phase 1: Pads ---------------------------------------------------------
    print("─── PHASE 1: Landing Pads ───────────────────────────────────────────")
    for pad in pads:
        maybe_wait(
            f"Fly drone_0 over {pad['pad_id']} ({pad['drone_id']})  "
            f"NED({pad['x']:.3f}, {pad['y']:.3f})",
            f"Inject {pad['pad_id']} ({pad['drone_id']})  "
            f"NED({pad['x']:.3f}, {pad['y']:.3f})",
            args.yes,
        )
        pub(node, pad_pub, {
            "drone_id": pad["drone_id"],
            "pad_id":   pad["pad_id"],
            "x": pad["x"],
            "y": pad["y"],
            "z": pad["z"],
        }, args.repeats)
        print(f"  ✓ {pad['pad_id']} published")

    print("\n  All pads sent. Coordinator should advance to CAPTURE_BOUNDARY.")
    time.sleep(0.5)
    rclpy.spin_once(node, timeout_sec=0.3)

    # -- Phase 2: Boundary vertices -------------------------------------------
    print("\n─── PHASE 2: Field Boundary ─────────────────────────────────────────")
    for i, v in enumerate(boundary_vertices):
        maybe_wait(
            f"Fly drone_0 to boundary vertex #{i+1}/{len(boundary_vertices)}  "
            f"NED({v['x']:.3f}, {v['y']:.3f})",
            f"Inject boundary vertex #{i+1}/{len(boundary_vertices)}  "
            f"NED({v['x']:.3f}, {v['y']:.3f})",
            args.yes,
        )
        pub(node, boundary_pub, {
            "index": i,
            "ned": {"x": v["x"], "y": v["y"], "z": v["z"]},
            "type": "vertex",
        }, args.repeats)
        print(f"  ✓ vertex #{i+1} published")

    # -- Phase 3: Close boundary ----------------------------------------------
    maybe_wait(
        "Close boundary and generate grid",
        "Inject boundary close and generate grid",
        args.yes,
    )
    pub(node, close_pub, {"source": "injection", "closed": True}, args.repeats)
    print("  ✓ boundary_close sent — grid generation triggered")

    print("\n=== Injection complete. Grid is being generated. ===\n")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
