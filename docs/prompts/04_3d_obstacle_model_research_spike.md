# Prompt 04 — 3D Obstacle Model Research Spike

## Priority

Do this only after 2D/2.5D local avoidance, mapping, RTH and GCS operation are stable.

## Context

`LocalMapper` is currently a 2D/2.5D local grid. That is a good fit for field-grid missions and should remain the production default. A voxel map or OctoMap-style model may become useful later for trees, wires, tall structures, multi-height flight corridors or complex inspections.

This prompt is intentionally a research spike, not a production rewrite.

Relevant files to inspect first:

- `src/scout_control/scout_control/avoidance/local_mapper.py`
- `src/scout_control/scout_control/avoidance/local_planner.py`
- `src/scout_control/scout_control/avoidance/depth_projector.py`
- `src/scout_control/scout_control/avoidance/scan_manager.py`
- `src/scout_control/scout_control/core/obstacle_avoidance_runtime.py`
- `src/scout_control/test/test_local_planner.py`
- `src/scout_control/test/test_local_mapper_scan_pipeline.py`

## Goal

Evaluate and prototype a minimal 3D obstacle representation without replacing the current production mapper/planner.

The output should answer whether this project benefits from a voxel model, and what the smallest compatible integration would look like.

## Requirements

1. Do not replace the current `LocalMapper`.
2. Do not make a 3D dependency mandatory for normal launches.
3. Prototype behind an experimental flag or in a separate module.
4. Keep the current 2D/2.5D planner as the default production path.
5. Document memory, CPU and integration tradeoffs.
6. If using an external library, justify it and keep imports optional.
7. Provide a fallback path when the 3D model is unavailable.

## Suggested Implementation Shape

- Start with a small `VoxelObstacleMap` prototype in the avoidance package or a separate experimental module.
- Ingest the same point batches planned in Prompt 02.
- Provide projection helpers:
  - 3D occupancy summary to 2D cost map
  - height band query for flight altitude
  - obstacle column summary for planner compatibility
- Keep all production runtime behavior unchanged unless an explicit experimental parameter is enabled.

## Tests

Cover at minimum:

- voxel insert/query works for simple points
- projection to 2D preserves occupied columns
- empty map behaves safely
- optional dependency absence does not break normal tests
- current `LocalPlanner` tests still pass

Run:

```bash
PYTHONPATH=src/scout_control pytest \
  src/scout_control/test/test_local_planner.py \
  src/scout_control/test/test_local_mapper_scan_pipeline.py
```

## Acceptance Criteria

- A small experimental 3D model exists or a clear written decision says not to proceed.
- Production path remains 2D/2.5D and unchanged by default.
- The spike documents when 3D is worth activating.
- Tests prove the prototype does not break current avoidance behavior.

