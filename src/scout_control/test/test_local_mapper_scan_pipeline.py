from __future__ import annotations

import numpy as np

from scout_control.avoidance.local_mapper import LocalMapper, LocalMapperConfig
from scout_control.avoidance.local_planner import (
    LocalGridSnapshot,
    LocalPlanner,
    PlannerPose,
    PlannerResultStatus,
    PlannerTarget,
)
from scout_control.avoidance.scan_manager import ScanManager
from scout_control.avoidance.types import PointBatch


def test_local_mapper_dense_scan_enrichment_marks_occupied_cells() -> None:
    mapper = LocalMapper(
        LocalMapperConfig(
            resolution_m=0.5,
            span_x_m=12.0,
            span_y_m=12.0,
            warn_distance_m=1.0,
            critical_distance_m=0.5,
        )
    )
    mapper.update_pose(0.0, 0.0, 0.0, 0.0, 0.0)

    dense_batch = PointBatch(
        source="test_dense_scan",
        frame="map",
        stamp_s=1.0,
        points_xyz=np.array(
            [
                [2.0, 0.0, 0.0],
                [2.1, 0.1, 0.0],
                [2.2, -0.1, 0.0],
            ],
            dtype=np.float32,
        ),
        confidence=1.0,
        sensor_range_m=10.0,
        is_dense_scan=True,
    )
    inserted = mapper.ingest_point_batch(dense_batch)
    snapshot, _summary = mapper.update(now_s=1.1)

    assert inserted > 0
    assert bool(np.any(snapshot.occupied_mask))
    assert mapper.summary()["dense_scan_points"] > 0


def test_scan_manager_dense_capture_enriches_mapper(tmp_path) -> None:
    mapper = LocalMapper(
        LocalMapperConfig(
            resolution_m=0.5,
            span_x_m=16.0,
            span_y_m=16.0,
            warn_distance_m=2.0,
            critical_distance_m=1.0,
        )
    )
    mapper.update_pose(0.0, 0.0, 0.0, 0.0, 0.0)

    run_events: list[tuple[str, dict]] = []
    scan = ScanManager(
        mapper=mapper,
        assets_dir=tmp_path,
        hover_ticks=1,
        spin_ticks=2,
        point_stride=2,
        free_distance_m=3.0,
        cam_hfov_deg=72.0,
        camera_topic="/test/camera",
        depth_topic="/test/depth",
        log_cb=lambda _msg: None,
        run_log_cb=lambda event, **fields: run_events.append((event, fields)),
    )
    scan.start_scan(
        reason="unit_test_enrichment",
        pose_ned=(0.0, 0.0, 0.0),
        yaw=0.0,
        mission_target_ned=(8.0, 0.0),
        target_id="target_1",
        target_name="Target 1",
        phase_name="STOP_HOVER",
        closest_m=4.0,
        committed_side="none",
    )

    rgb = np.zeros((16, 16, 3), dtype=np.uint8)
    depth = np.full((16, 16), 3.0, dtype=np.float32)
    result = None
    for idx in range(24):
        result = scan.step(
            pose_ned=(0.0, 0.0, 0.0),
            yaw=0.0,
            rgb_frame=rgb,
            depth_frame=depth,
            depth_ts=10.0 + float(idx),
            obstacle_sectors={"left": 99.0, "center": 99.0, "right": 99.0},
        )
        if result.finished:
            break

    assert result is not None
    assert result.finished
    assert result.success
    assert result.point_batch is not None
    assert result.point_batch.is_dense_scan
    assert result.point_batch.point_count > 0
    assert mapper.summary()["dense_scan_points"] > 0
    assert any(event_name == "scan_complete" for event_name, _ in run_events)


def test_dense_scan_enrichment_changes_subsequent_planner_result() -> None:
    mapper = LocalMapper(
        LocalMapperConfig(
            resolution_m=0.5,
            span_x_m=20.0,
            span_y_m=20.0,
        )
    )
    mapper.update_pose(0.0, 0.0, 0.0, 0.0, 0.0)
    planner = LocalPlanner()

    base_snapshot, _ = mapper.update(now_s=1.0)
    base_grid = LocalGridSnapshot(
        occupancy=base_snapshot.occupied_mask | base_snapshot.dynamic_no_go_mask,
        resolution_m=base_snapshot.resolution_m,
        origin_x=base_snapshot.origin_ned[0],
        origin_y=base_snapshot.origin_ned[1],
        inflation_cost=base_snapshot.inflation_map,
        blocked_cost=base_snapshot.blocked_cost_layer,
        state=base_snapshot.state.name,
        stamp_s=base_snapshot.stamp_s,
    )
    base_result = planner.plan(
        grid=base_grid,
        start=PlannerPose(x=0.0, y=0.0),
        mission_target=PlannerTarget(x=6.0, y=0.0),
    )
    assert base_result.status == PlannerResultStatus.DIRECT

    wall_points = []
    for y in np.arange(-2.0, 2.1, 0.25):
        wall_points.append((2.0, float(y), 0.0))
    scan_batch = PointBatch(
        source="test_scan_wall",
        frame="map",
        stamp_s=2.0,
        points_xyz=np.asarray(wall_points, dtype=np.float32),
        confidence=1.0,
        sensor_range_m=12.0,
        is_dense_scan=True,
    )
    mapper.ingest_point_batch(scan_batch)
    enriched_snapshot, _ = mapper.update(now_s=2.1)
    enriched_grid = LocalGridSnapshot(
        occupancy=enriched_snapshot.occupied_mask | enriched_snapshot.dynamic_no_go_mask,
        resolution_m=enriched_snapshot.resolution_m,
        origin_x=enriched_snapshot.origin_ned[0],
        origin_y=enriched_snapshot.origin_ned[1],
        inflation_cost=enriched_snapshot.inflation_map,
        blocked_cost=enriched_snapshot.blocked_cost_layer,
        state=enriched_snapshot.state.name,
        stamp_s=enriched_snapshot.stamp_s,
    )
    enriched_result = planner.plan(
        grid=enriched_grid,
        start=PlannerPose(x=0.0, y=0.0),
        mission_target=PlannerTarget(x=6.0, y=0.0),
    )

    assert enriched_result.status in {PlannerResultStatus.DETOUR, PlannerResultStatus.NO_PATH}
    assert enriched_result.status != PlannerResultStatus.DIRECT
