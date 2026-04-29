# Prompt 03 — Semantic Terrain vs Obstacle Labels

## Priority

Do this later, after geometric mapping and local avoidance are stable.

## Context

The current mapping pipeline is geometric. `obstacle_extractor.py` classifies obstacles primarily from height/geometry. For an agricultural UAV, this is useful but incomplete: crops, canopy, trees, poles, people, buildings and machinery should not all be treated the same.

This should start as a field-model/mapping enhancement, not as a flight-critical neural-net dependency inside the low-level runtime loop.

Relevant files to inspect first:

- `src/scout_control/scout_control/mapping/field_model_builder.py`
- `src/scout_control/scout_control/mapping/heightmap.py`
- `src/scout_control/scout_control/mapping/obstacle_extractor.py`
- `src/scout_control/scout_control/mapping/grid_refiner.py`
- `src/scout_control/scout_control/mapping/mission_package_builder.py`
- `perimeters/field_model/manifest.json`
- `swarm_center/core/field_model_loader.py`
- `swarm_center/ui/field_view.py`

## Goal

Add optional semantic labels to the field model so mapping/refinement can distinguish crop/terrain from true obstacles and no-go objects.

Start with an interface that supports semantic labels. Do not require a trained neural network to make the system run.

## Requirements

1. Keep geometric obstacle extraction as the default behavior.
2. Add optional semantic metadata to field model outputs, for example labels like:
   - `crop`
   - `terrain`
   - `tree`
   - `person`
   - `vehicle`
   - `building`
   - `unknown_obstacle`
3. Make the semantic layer optional. If no semantic model/output exists, existing mapping still works.
4. Update `grid_refiner` so semantics can influence `available`, `caution` and `no_go` decisions.
5. Avoid putting neural inference in the flight-critical avoidance loop for this task.
6. If a model adapter is added, make it pluggable and disabled by default.
7. Update Swarm Center loading/rendering only enough to inspect semantic labels or overlays.

## Suggested Implementation Shape

- Define a simple `SemanticObstacle` or semantic annotation payload in the mapping package.
- Extend field model JSON with backwards-compatible optional fields.
- Add tests for old field model payloads without semantics.
- Add a placeholder/rule-based semantic provider first, such as labels loaded from JSON or assigned from simple classes.
- Let future neural segmentation plug into the same interface.

## Tests

Cover at minimum:

- old field model files load without semantic fields
- semantic labels are persisted and reloaded
- `grid_refiner` treats high-risk classes as `no_go`
- crop/terrain labels do not automatically become hard obstacles
- unknown labels degrade conservatively to `caution` or existing geometric behavior

Run:

```bash
PYTHONPATH=src/scout_control pytest \
  src/scout_control/test/test_grid_refiner.py \
  src/scout_control/test/test_grid_from_polygon.py
```

## Acceptance Criteria

- Semantics are optional and backwards-compatible.
- Geometry remains the default safety basis.
- Field model can carry semantic labels.
- Refined grid decisions can use labels without requiring ML at runtime.
- Swarm Center can still load existing field model artifacts.

