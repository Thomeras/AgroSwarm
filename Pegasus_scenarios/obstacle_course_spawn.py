"""
obstacle_course_spawn.py — Spawn obstacle course for depth-camera avoidance testing.

Run from Isaac Sim Script Editor AFTER:
  1. World is loaded (agro_field.usd or obstacle_course.usda)
  2. Drone vehicle loaded via Pegasus UI
  3. simulation_cam.py already executed (camera publishers active)

Coordinate system: Isaac Sim Z-up ENU
  world X = East  = NED_y
  world Y = North = NED_x
  world Z = Up

Obstacles match obstacle_avoidance_mission.py exactly:
  wall_north   NED(12,  0)  → world( 0, 12) — 6×0.4×6 m concrete wall
  poles_east   NED( 0, 12)  → world(12,  0) — 3× steel pole Ø0.3×8 m
  building_ne  NED( 9,  9)  → world( 9,  9) — 4×4×10 m block
  fence_nnw    NED(12, -8)  → world(-8, 12) — 7×0.15×5 m wooden fence

Target markers (flat discs on ground) show mission end-points.
"""

import builtins
import carb
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics, Sdf

_SENTINEL = "_scout_obstacle_course_spawned"

# ── Coordinate helpers ────────────────────────────────────────────────────────

def _ned_to_world(ned_x: float, ned_y: float, z: float = 0.0) -> Gf.Vec3d:
    """NED (North, East) → Isaac Sim world (Z-up ENU: X=East, Y=North)."""
    return Gf.Vec3d(ned_y, ned_x, z)


# ── Primitive builders ────────────────────────────────────────────────────────

def _box(stage, path: str, center: Gf.Vec3d, dims_xyz,
         color=(0.6, 0.5, 0.4)) -> None:
    """Static box obstacle.  dims_xyz = (width_E, depth_N, height) in metres."""
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(center)
    # UsdGeom.Cube default size = 2  →  scale = dims / 2
    sx, sy, sz = dims_xyz[0] / 2.0, dims_xyz[1] / 2.0, dims_xyz[2] / 2.0
    xform.AddScaleOp().Set(Gf.Vec3f(sx, sy, sz))

    cube = UsdGeom.Cube.Define(stage, f"{path}/Shape")
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def _cylinder(stage, path: str, center: Gf.Vec3d,
              radius: float, height: float,
              color=(0.4, 0.4, 0.7)) -> None:
    """Static vertical cylinder obstacle."""
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(center)

    cyl = UsdGeom.Cylinder.Define(stage, f"{path}/Shape")
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr("Z")
    cyl.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())


def _target_disc(stage, path: str, center: Gf.Vec3d,
                 radius: float, color=(1.0, 0.8, 0.0)) -> None:
    """Flat disc marking a mission target position (no physics, visual only)."""
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(center)

    cyl = UsdGeom.Cylinder.Define(stage, f"{path}/Shape")
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(0.05)
    cyl.CreateAxisAttr("Z")
    cyl.CreateDisplayColorAttr([Gf.Vec3f(*color)])


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    if getattr(builtins, _SENTINEL, False):
        carb.log_warn("[obstacle_course] Already spawned in this session. Skipping.")
        return

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("[obstacle_course] No USD stage is open.")

    ROOT = "/World/ObstacleCourse"
    UsdGeom.Xform.Define(stage, ROOT)

    GROUND = 0.0   # ground plane Z in world coords

    # ── 1. wall_north — NED(12, 0) ───────────────────────────────────────────
    # 6 m wide (E-W) × 0.4 m thick (N-S) × 6 m tall
    # Center: halfway up = Z 3 m
    _box(stage, f"{ROOT}/wall_north",
         center=_ned_to_world(12.0, 0.0, GROUND + 3.0),
         dims_xyz=(6.0, 0.4, 6.0),
         color=(0.75, 0.70, 0.55))   # concrete beige
    carb.log_warn("[obstacle_course] wall_north   → world(0, 12, 3)")

    # ── 2. poles_east — NED(0, 12), 3 poles spaced ±1.5 m in North ──────────
    # Each pole: Ø 0.3 m, 8 m tall
    for i, dn in enumerate((-1.5, 0.0, 1.5)):
        _cylinder(stage, f"{ROOT}/poles_east/pole_{i}",
                  center=_ned_to_world(0.0 + dn, 12.0, GROUND + 4.0),
                  radius=0.15, height=8.0,
                  color=(0.35, 0.35, 0.75))   # steel blue
    carb.log_warn("[obstacle_course] poles_east   → world(12, -1.5/0/1.5, 4)")

    # ── 3. building_ne — NED(9, 9), 4×4×10 m block ───────────────────────────
    _box(stage, f"{ROOT}/building_ne",
         center=_ned_to_world(9.0, 9.0, GROUND + 5.0),
         dims_xyz=(4.0, 4.0, 10.0),
         color=(0.50, 0.50, 0.58))   # grey concrete
    carb.log_warn("[obstacle_course] building_ne  → world(9, 9, 5)")

    # ── 4. fence_nnw — NED(12, -8), 7 m fence along E-W axis ────────────────
    # 7 m wide (E-W) × 0.15 m thick (N-S) × 5 m tall
    _box(stage, f"{ROOT}/fence_nnw",
         center=_ned_to_world(12.0, -8.0, GROUND + 2.5),
         dims_xyz=(7.0, 0.15, 5.0),
         color=(0.55, 0.38, 0.22))   # wood brown
    carb.log_warn("[obstacle_course] fence_nnw    → world(-8, 12, 2.5)")

    # ── Target markers (ground discs showing where each mission flies to) ─────
    TARGETS = [
        ("t_north",   22.0,   0.0, (1.0, 0.3, 0.3)),   # red
        ("t_east",     0.0,  22.0, (0.3, 0.7, 1.0)),   # blue
        ("t_ne",      18.0,  18.0, (0.3, 1.0, 0.5)),   # green
        ("t_nnw",     22.0, -12.0, (1.0, 0.9, 0.2)),   # yellow
    ]
    for name, nx, ny, col in TARGETS:
        _target_disc(stage, f"{ROOT}/Targets/{name}",
                     center=_ned_to_world(nx, ny, GROUND + 0.03),
                     radius=1.5, color=col)

    setattr(builtins, _SENTINEL, True)
    carb.log_warn(
        "\n[obstacle_course] ✓ Obstacle course spawned under /World/ObstacleCourse\n"
        "  wall_north   NED(12,  0)  6×0.4×6 m  concrete\n"
        "  poles_east   NED( 0, 12)  Ø0.3×8 m   3× steel poles\n"
        "  building_ne  NED( 9,  9)  4×4×10 m   concrete block\n"
        "  fence_nnw    NED(12, -8)  7×0.15×5 m wood fence\n"
        "  + 4 target discs on ground\n"
        "Next: ros2 run scout_control obstacle_avoidance_mission"
    )


main()
