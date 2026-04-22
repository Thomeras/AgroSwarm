import math

import numpy as np

from scout_control.avoidance.depth_projector import DepthProjector
from scout_control.avoidance.peer_tracks import PeerTrackStore
from scout_control.avoidance.types import PointBatch, TargetCommand


def test_target_command_normalizes_runtime_payload() -> None:
    command = TargetCommand.from_payload(
        {
            "command": "goto",
            "target_id": "pad_1",
            "target_ned": [12.0, -3.5],
            "altitude_m": 6.0,
            "cruise_speed_mps": 3.0,
        }
    )

    assert command.target_id == "pad_1"
    assert command.cmd_id == "pad_1"
    assert command.target_ned == (12.0, -3.5)
    assert command.to_payload()["target_ned"] == [12.0, -3.5]


def test_depth_projector_filters_invalid_depth_samples() -> None:
    depth = np.array(
        [
            [np.nan, 0.1, 2.0],
            [25.0, 1.5, 0.4],
        ],
        dtype=np.float32,
    )
    projector = DepthProjector(min_range_m=0.3, max_range_m=20.0, default_stride=1)

    batch = projector.depth_to_body_points(depth, pixel_stride=1)

    assert batch.frame == "body_frd"
    assert batch.count == 3
    assert np.all(batch.points_xyz[:, 0] >= 0.3)
    assert np.all(batch.points_xyz[:, 0] <= 20.0)


def test_depth_projector_world_projection_drops_ground_points_outside_band() -> None:
    projector = DepthProjector(collision_band_m=(-0.3, 0.3))
    body_batch = PointBatch(
        source="test",
        frame="body_frd",
        stamp_s=1.0,
        points_xyz=np.array(
            [
                [2.0, 0.0, 0.0],
                [2.0, 0.0, 2.0],
                [2.0, 1.0, 0.1],
            ],
            dtype=np.float32,
        ),
    )

    world = projector.project_to_world_points(
        body_batch,
        origin_ned=(10.0, 20.0, -2.0),
        yaw_rad=math.pi / 2.0,
        ground_z_ned=0.0,
    )

    assert world.count == 2
    np.testing.assert_allclose(world.points_xyz[:, 0], np.array([10.0, 9.0]), atol=1e-5)
    np.testing.assert_allclose(world.points_xyz[:, 1], np.array([22.0, 22.0]), atol=1e-5)


def test_peer_track_store_builds_dynamic_safety_disks() -> None:
    store = PeerTrackStore(
        track_ttl_s=3.0,
        base_radius_m=1.5,
        soft_shell_m=2.0,
        lookahead_s=2.0,
        velocity_inflation_gain=1.0,
        age_inflation_gain=0.5,
    )
    store.update_from_json(
        """
        {
          "tracks": [
            {"drone_id": 1, "x": 0.0, "y": 1.0, "vx": 1.0, "vy": 0.0, "age_s": 0.5, "status": "active"},
            {"drone_id": 2, "x": 4.0, "y": 5.0, "vx": 0.0, "vy": 0.0, "age_s": 5.0, "status": "active"}
          ]
        }
        """,
        stamp_s=10.0,
    )

    zones = store.build_safety_disks(now_s=10.0)

    assert len(zones) == 1
    assert zones[0].zone_id == "peer_1"
    assert zones[0].center_ned == (2.0, 1.0)
    assert zones[0].radius_m > 2.0
    assert zones[0].soft_radius_m == zones[0].radius_m + 2.0


def test_peer_track_store_accepts_position_velocity_ned_payload() -> None:
    store = PeerTrackStore(track_ttl_s=5.0)
    updated = store.update_from_json(
        """
        {
          "tracks": [
            {"drone_id": 1, "position_ned": [1.0, 2.0, -5.0], "velocity_ned": [0.5, 0.0], "status": "active"},
            {"id": 2, "x": 4.0, "y": 5.0, "vx": 0.1, "vy": 0.2, "status": "active"},
            {"drone_id": "bad", "x": 0.0, "y": 0.0}
          ]
        }
        """,
        stamp_s=10.0,
    )

    assert len(updated) == 2
    assert updated[0].drone_id == 1
    assert updated[0].x == 1.0
    assert updated[0].y == 2.0
    assert updated[1].drone_id == 2


def test_peer_track_store_limits_speed_from_bad_jump() -> None:
    store = PeerTrackStore(max_track_speed_mps=5.0, velocity_smoothing=0.0)
    store.update_track(drone_id=1, x=0.0, y=0.0, stamp_s=10.0)
    track = store.update_track(drone_id=1, x=100.0, y=0.0, stamp_s=11.0)

    assert track.speed_mps <= 5.01
