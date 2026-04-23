"""
scan_cloud_viz.py — Simple viewer for obstacle scan point clouds.

Usage:
  ros2 run scout_control scan_cloud_viz -- /path/to/scan_cloud.ply
  ros2 run scout_control scan_cloud_viz -- /path/to/scan_cloud.ply --meta /path/to/scan_meta.json
  ros2 run scout_control scan_cloud_viz -- /path/to/scan_cloud.ply --out /tmp/scan.png
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize scan_cloud.ply point clouds.")
    parser.add_argument("ply_path", help="Path to ASCII .ply point cloud")
    parser.add_argument("--meta", help="Optional path to matching scan_meta.json")
    parser.add_argument(
        "--point-size",
        type=float,
        default=1.5,
        help="Scatter point size for 3D view",
    )
    parser.add_argument(
        "--stride",
        type=int,
        default=2,
        help="Render every Nth point to keep plotting responsive",
    )
    parser.add_argument(
        "--out",
        help="Optional path to save the figure instead of only showing it",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open an interactive window",
    )
    return parser.parse_args()


def _load_ascii_ply(path: Path) -> np.ndarray:
    with path.open("r", encoding="utf-8") as fh:
        lines = fh.readlines()

    if not lines or lines[0].strip() != "ply":
        raise ValueError(f"{path} is not an ASCII PLY file")

    vertex_count = None
    header_end = None
    for idx, line in enumerate(lines):
        stripped = line.strip()
        if stripped.startswith("element vertex "):
            vertex_count = int(stripped.split()[-1])
        if stripped == "end_header":
            header_end = idx + 1
            break

    if vertex_count is None or header_end is None:
        raise ValueError(f"{path} is missing a valid PLY header")

    points = []
    for line in lines[header_end:header_end + vertex_count]:
        parts = line.strip().split()
        if len(parts) < 3:
            continue
        points.append((float(parts[0]), float(parts[1]), float(parts[2])))

    if not points:
        raise ValueError(f"{path} contains no points")

    return np.asarray(points, dtype=np.float32)


def _infer_meta_path(ply_path: Path, explicit_meta: str | None) -> Path | None:
    if explicit_meta:
        return Path(explicit_meta)
    sibling = ply_path.with_name("scan_meta.json")
    return sibling if sibling.exists() else None


def _load_meta(path: Path | None) -> dict | None:
    if path is None or not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> None:
    args = _parse_args()

    ply_path = Path(args.ply_path).expanduser().resolve()
    meta_path = _infer_meta_path(ply_path, args.meta)
    meta = _load_meta(meta_path)

    points = _load_ascii_ply(ply_path)
    stride = max(1, int(args.stride))
    points_view = points[::stride]

    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(14, 6))
    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    ax_top = fig.add_subplot(1, 2, 2)

    colors = -points_view[:, 2]
    ax3d.scatter(
        points_view[:, 0],
        points_view[:, 1],
        -points_view[:, 2],
        c=colors,
        s=args.point_size,
        cmap="viridis",
        alpha=0.7,
        linewidths=0.0,
    )
    ax3d.set_xlabel("NED X")
    ax3d.set_ylabel("NED Y")
    ax3d.set_zlabel("Up (-Z)")
    ax3d.set_title("3D Point Cloud")

    ax_top.scatter(
        points_view[:, 1],
        points_view[:, 0],
        c=colors,
        s=max(args.point_size, 1.0),
        cmap="viridis",
        alpha=0.7,
        linewidths=0.0,
    )
    ax_top.set_xlabel("NED Y")
    ax_top.set_ylabel("NED X")
    ax_top.set_title("Top-Down")
    ax_top.axis("equal")
    ax_top.grid(True, alpha=0.25)

    title = ply_path.name
    if meta:
        drone = meta.get("drone_ned")
        target = meta.get("target_ned")
        sectors = meta.get("scan_best_sectors", {})
        reason = meta.get("reason", "")
        mission = meta.get("mission_name", "")
        title = (
            f"{mission} | {reason} | points={meta.get('points', len(points))}\n"
            f"sectors L/C/R={sectors.get('left', '?')}/{sectors.get('center', '?')}/{sectors.get('right', '?')}"
        )
        if drone and len(drone) >= 2:
            ax_top.scatter(
                [drone[1]], [drone[0]],
                c="red", s=90, marker="x", label="drone",
            )
            ax3d.scatter(
                [drone[0]], [drone[1]], [-drone[2] if len(drone) > 2 else 0.0],
                c="red", s=90, marker="x",
            )
        if target and len(target) >= 2:
            ax_top.scatter(
                [target[1]], [target[0]],
                c="orange", s=90, marker="*", label="target",
            )
            if drone and len(drone) >= 2:
                ax_top.plot(
                    [drone[1], target[1]],
                    [drone[0], target[0]],
                    linestyle="--",
                    color="orange",
                    alpha=0.8,
                )
        ax_top.legend(loc="best")

    fig.suptitle(title)
    fig.tight_layout()

    if args.out:
        out_path = Path(args.out).expanduser().resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out_path, dpi=180, bbox_inches="tight")
        print(out_path)

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
