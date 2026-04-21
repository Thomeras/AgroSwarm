"""
simulation_cam.py — Script Editor snippet for attaching ROS2 camera publishers.

Run this INSIDE an already running Isaac Sim + Pegasus session:
  Window > Script Editor > paste:
    exec(open(".../Pegasus_scenarios/simulation_cam.py").read())

This script does NOT create a SimulationApp, does NOT load a world,
and does NOT spawn a vehicle. It only finds the existing camera prim
on the already-loaded drone and attaches Replicator ROS2 writers to it.

Resolution: 640×400 — matches obstacle_detector.py (OakD-Lite params).
"""

import builtins

import carb
import omni.usd
import omni.graph.core as og
import omni.replicator.core as rep

try:
    from isaacsim.ros2.bridge import read_camera_info
except ModuleNotFoundError:
    read_camera_info = None


DRONE_INDEX      = 0
DRONE_NAMESPACE  = f"drone_{DRONE_INDEX}"
RGB_TOPIC        = "camera/image_raw"
DEPTH_TOPIC      = "depth/image_raw"
CAMERA_INFO_TOPIC = "camera/camera_info"

# Resolution matching obstacle_detector.py cam_width/cam_height params
CAM_W = 640
CAM_H = 400

# Camera prim candidates — searched in order
CAMERA_CANDIDATES = [
    "/World/quadrotor/body/camera",
    "/World/drone_0/body/camera",
    "/World/drone/body/camera",
]

_SENTINEL = "_scout_camera_publishers_installed"


def _find_camera_prim(stage):
    for path in CAMERA_CANDIDATES:
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            return prim
    # Fallback: traverse stage looking for any /body/camera
    for prim in stage.Traverse():
        if str(prim.GetPath()).endswith("/body/camera"):
            return prim
    return None


def _set_gate_step(render_product_path: str, frequency_hz: float) -> None:
    """Throttle the Replicator pipeline to the requested Hz."""
    if not isinstance(render_product_path, str):
        render_product_path = getattr(render_product_path, "path",
                                      str(render_product_path))
    try:
        import omni.syntheticdata
        gate_path = omni.syntheticdata.SyntheticData._get_node_path(
            "PostProcessDispatch" + "IsaacSimulationGate",
            render_product_path,
        )
        og.Controller.attribute(gate_path + ".inputs:step").set(
            int(60 / frequency_hz)
        )
    except Exception as exc:
        carb.log_warn(f"[simulation_cam] Gate step not set: {exc}")


def main() -> None:
    if getattr(builtins, _SENTINEL, False):
        carb.log_warn(
            "[simulation_cam] Publishers already installed in this session. "
            "Skipping. Restart Isaac to re-attach."
        )
        return

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("[simulation_cam] No USD stage is open.")

    camera_prim = _find_camera_prim(stage)
    if camera_prim is None:
        raise RuntimeError(
            "[simulation_cam] Camera prim not found. "
            "Load the drone via Pegasus UI first, then run this script."
        )

    camera_path = str(camera_prim.GetPath())
    carb.log_warn(f"[simulation_cam] Using camera prim: {camera_path}")

    render_product = rep.create.render_product(
        camera_path, [CAM_W, CAM_H], name="scout_cam"
    )
    carb.log_warn(f"[simulation_cam] Render product {CAM_W}×{CAM_H}: {render_product}")

    # ── RGB writer ────────────────────────────────────────────────────────────
    rgb_writer = rep.writers.get("LdrColorSDROS2PublishImage")
    rgb_writer.initialize(
        nodeNamespace=DRONE_NAMESPACE,
        topicName=RGB_TOPIC,
        frameId="camera",
        queueSize=1,
    )
    rgb_writer.attach([render_product])

    # ── Depth writer ──────────────────────────────────────────────────────────
    depth_writer = rep.writers.get("DistanceToImagePlaneSDROS2PublishImage")
    depth_writer.initialize(
        nodeNamespace=DRONE_NAMESPACE,
        topicName=DEPTH_TOPIC,
        frameId="camera",
        queueSize=1,
    )
    depth_writer.attach([render_product])

    # ── Camera info writer (optional) ─────────────────────────────────────────
    if read_camera_info is not None:
        try:
            camera_info, _ = read_camera_info(render_product_path=render_product)
            info_writer = rep.writers.get("ROS2PublishCameraInfo")
            info_writer.initialize(
                nodeNamespace=DRONE_NAMESPACE,
                topicName=CAMERA_INFO_TOPIC,
                frameId="camera",
                queueSize=1,
                width=camera_info.width,
                height=camera_info.height,
                projectionType=camera_info.distortion_model,
                k=camera_info.k.reshape([1, 9]),
                r=camera_info.r.reshape([1, 9]),
                p=camera_info.p.reshape([1, 12]),
                physicalDistortionModel=camera_info.distortion_model,
                physicalDistortionCoefficients=camera_info.d,
            )
            info_writer.attach([render_product])
        except Exception as exc:
            carb.log_warn(f"[simulation_cam] camera_info publisher skipped: {exc}")
    else:
        carb.log_warn("[simulation_cam] read_camera_info unavailable, skipping camera_info.")

    # ── Throttle to 30 Hz ─────────────────────────────────────────────────────
    _set_gate_step(render_product, 30.0)

    setattr(builtins, _SENTINEL, True)
    carb.log_warn(
        f"[simulation_cam] ROS2 camera publishers active ({CAM_W}×{CAM_H} @ 30 Hz):\n"
        f"  /{DRONE_NAMESPACE}/{RGB_TOPIC}\n"
        f"  /{DRONE_NAMESPACE}/{DEPTH_TOPIC}"
    )


main()
