# Prompt 01 — Probabilistic Occupancy Grid [DONE]

## Priority

Do this first. It improves the current avoidance runtime without changing the whole architecture.

## Context

This repository is `scout_ws`, a ROS2 Jazzy workspace for PX4 SITL agrodrone swarm missions. The current autonomy path is runtime-owned:

- `obstacle_avoidance_runtime` is the single flight-control owner.
- `swarm_agent` delegates high-level targets through `/{drone}/avoidance/target_cmd`.
- `LocalMapper` currently uses deterministic confidence/increment/threshold behavior for local obstacle memory.

Relevant files to inspect first:

- `src/scout_control/scout_control/core/obstacle_avoidance_runtime.py`
- `src/scout_control/scout_control/avoidance/local_mapper.py`
- `src/scout_control/scout_control/avoidance/local_planner.py`
- `src/scout_control/scout_control/avoidance/depth_projector.py`
- `src/scout_control/scout_control/avoidance/types.py`
- `src/scout_control/test/test_local_mapper_scan_pipeline.py`
- `src/scout_control/test/test_local_planner.py`
- `src/scout_control/test/test_avoidance_helpers.py`

## Goal

Replace or extend the current deterministic local occupancy confidence model with a Bayesian log-odds occupancy grid while preserving the existing public runtime behavior and tests.

The output should still be usable by `LocalPlanner` as a local obstacle/cost map, but the mapper should track uncertainty better across repeated depth/scan observations.

## Requirements

1. Keep the current 2D/2.5D local planning model. Do not introduce OctoMap or a 3D planner in this task.
2. Add a log-odds representation internally in `LocalMapper`.
3. Support configurable probabilities for:
   - occupied hit update
   - free/miss update
   - prior occupancy
   - min/max log-odds clamp
   - occupancy threshold
4. Keep existing runtime status and planner contracts backwards-compatible.
5. Preserve scan-enriched observations as longer-lived or higher-confidence input, matching the current intent of scan persistence.
6. Add source-aware update helpers if useful, but keep the implementation small and understandable.
7. Avoid touching launch files unless a new parameter is truly needed.

## Suggested Implementation Shape

- Add small conversion helpers:
  - `prob_to_logodds(p)`
  - `logodds_to_prob(l)`
- Store occupancy as log-odds values in the mapper grid.
- Convert to probability/confidence only at boundaries where existing code expects that shape.
- Treat repeated hits as stronger evidence, repeated free observations as clearing evidence, and clamp both.
- Keep unknown space distinguishable from confirmed free space.

## Tests

Add or update focused tests under `src/scout_control/test/`.

Cover at minimum:

- repeated obstacle hits increase occupancy probability
- repeated free/miss updates decrease occupancy probability
- values clamp at configured min/max
- unknown cells do not become free without evidence
- planner still rejects/avoids occupied cells
- scan-enriched observations survive the intended persistence behavior

Run:

```bash
PYTHONPATH=src/scout_control pytest \
  src/scout_control/test/test_local_mapper_scan_pipeline.py \
  src/scout_control/test/test_local_planner.py \
  src/scout_control/test/test_avoidance_helpers.py
```

## Acceptance Criteria

- Existing runtime ownership remains unchanged.
- Existing mission/high-level target command behavior remains unchanged.
- Local mapper uncertainty is represented with log-odds internally.
- Tests prove accumulation, clearing, clamping and planner compatibility.
- No broad refactor outside the avoidance mapper/planner boundary.

