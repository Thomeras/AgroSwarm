# Prompt 05 — VIO Positioning Backup

## Priority

Do this much later, when moving toward real hardware, GPS-weak environments or precision spraying.

## Context

The project currently depends on PX4 local position, usually coming from the PX4 estimator with IMU/GPS/simulated ground truth depending on environment. A visual-inertial odometry path could improve robustness in GPS-denied or GPS-degraded conditions, but it is a large integration task.

This should not be part of the first functional swarm MVP.

Relevant files to inspect first:

- `src/scout_control/scout_control/core/obstacle_avoidance_runtime.py`
- `src/scout_control/scout_control/avoidance/telemetry_hub.py`
- `src/scout_control/scout_control/avoidance/types.py`
- `docs/topic_contract.md`
- `src/scout_control/launch/isaac_e2e_mission.launch.py`
- `src/scout_control/launch/full_e2e_mission.launch.py`

## Goal

Design and prototype an optional VIO/local-position backup path without disrupting the existing PX4 local-position contract.

The first implementation should be advisory or selectable, not a forced replacement for PX4 local position.

## Requirements

1. Keep PX4 local position as the default source.
2. Add a clear abstraction for pose source selection or pose quality reporting.
3. Do not implement a dense VIO algorithm from scratch unless explicitly approved.
4. Prefer integrating a proven VIO source if one is available in the target environment.
5. Make the VIO path optional and disabled by default.
6. Add health/quality gates before using VIO for flight decisions.
7. Update `docs/topic_contract.md` for any new pose/quality topics.

## Suggested Implementation Shape

- Add a pose-source adapter boundary rather than wiring VIO directly into runtime logic.
- Support modes like:
  - `px4_local_position`
  - `vio_advisory`
  - `vio_primary_experimental`
- Add pose quality fields:
  - source
  - timestamp/age
  - covariance or quality score if available
  - reset/jump detection
- Let runtime refuse unsafe source switches during active flight unless explicitly configured.

## Tests

Cover at minimum:

- default PX4 local position path is unchanged
- stale VIO data is rejected
- low-quality VIO is rejected
- pose-source selection does not switch unexpectedly mid-mission
- runtime status exposes selected source and quality

Run:

```bash
PYTHONPATH=src/scout_control pytest \
  src/scout_control/test/test_avoidance_helpers.py \
  src/scout_control/test/test_typed_status_payloads.py
```

## Acceptance Criteria

- VIO is optional and off by default.
- Runtime can report pose source and quality.
- PX4 local position remains the production default.
- No hand-rolled dense VIO implementation is introduced as part of this task.
- The implementation is ready for real VIO integration later.

