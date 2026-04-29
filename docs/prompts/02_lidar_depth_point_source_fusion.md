# Prompt 02 — Lightweight LiDAR + Depth Point Source Fusion [DONE]

## Priority

Do this after the probabilistic occupancy grid. Do not start with a Kalman filter or factor graph.

## Context

The current architecture is depth-camera-first. LiDAR may be bridged as a raw ROS topic through simulation infrastructure, but `obstacle_avoidance_runtime` does not actively merge LiDAR observations into `LocalMapper`.

The desired direction is a simple source-aware point ingestion layer:

- depth camera points remain the primary near-field source
- scan-enriched depth points remain useful for local recovery
- LiDAR can add longer-range or complementary points
- each source can carry a confidence/update weight

Relevant files to inspect first:

- `src/scout_control/scout_control/core/obstacle_avoidance_runtime.py`
- `src/scout_control/scout_control/avoidance/local_mapper.py`
- `src/scout_control/scout_control/avoidance/depth_projector.py`
- `src/scout_control/scout_control/avoidance/scan_manager.py`
- `src/scout_control/scout_control/avoidance/types.py`
- `src/scout_control/scout_control/avoidance/telemetry_hub.py`
- `docs/topic_contract.md`

## Goal

Add a lightweight multi-source obstacle point ingestion path so LiDAR and depth observations can update the same local occupancy model with source-specific confidence.

This is not a full sensor-fusion research task. Keep it practical and compatible with the current mission-first swarm architecture.

## Requirements

1. Introduce a small internal representation for obstacle point batches, for example:
   - source name: `depth`, `scan_depth`, `lidar`
   - timestamp
   - frame or coordinate assumption
   - points in body/world/local coordinates as appropriate for the existing mapper
   - update weight or confidence
2. Preserve the existing depth pipeline.
3. Add LiDAR subscription/config only where the runtime can consume it safely.
4. If LiDAR topic names are uncertain, make them configurable and disabled by default.
5. Do not add Kalman filters, factor graphs, SLAM or heavy dependencies.
6. Do not make LiDAR mandatory for existing tests or launches.
7. Update `docs/topic_contract.md` if a new topic contract is introduced.

## Suggested Implementation Shape

- Add a source-aware mapper method such as `update_from_points(batch)` or similar.
- Let existing depth and scan paths call that shared method.
- Add a LiDAR callback in `obstacle_avoidance_runtime` only if a point representation can be normalized cleanly.
- For simulated LiDAR, support the existing message type used in the repo if present.
- Apply source weights through the log-odds update parameters from Prompt 01.

## Tests

Add focused unit tests that do not require a running simulator.

Cover at minimum:

- depth batch updates occupancy
- LiDAR batch updates occupancy through the same code path
- source weights affect occupancy strength
- disabled/missing LiDAR does not break runtime construction
- stale or malformed LiDAR data is ignored safely

Run:

```bash
PYTHONPATH=src/scout_control pytest \
  src/scout_control/test/test_local_mapper_scan_pipeline.py \
  src/scout_control/test/test_local_planner.py \
  src/scout_control/test/test_avoidance_helpers.py
```

## Acceptance Criteria

- Runtime still works with depth only.
- LiDAR is optional and disabled/configurable by default.
- The mapper has one clear ingestion path for depth, scan-depth and LiDAR points.
- No heavy fusion framework is introduced.
- Topic documentation is updated if new runtime topics are added.

