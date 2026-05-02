# Legacy Scout Control Nodes

This package contains historical controllers and experimental nodes retained for
reference, diagnostics, and migration review.

Current production flight ownership belongs to:

- `scout_control.core.obstacle_avoidance_runtime`

Production E2E launch files must not start legacy/manual PX4 setpoint
controllers. In the current architecture, `swarm_agent`, `swarm_coordinator`,
Swarm Center, and setup/manual tooling publish operator or mission intent only;
they do not own `/fmu/in/*` or `/px4_N/fmu/in/*` setpoints.

Keep these legacy modules only while they still help with comparison,
experiments, or migration. If a legacy node is needed again, move the useful
logic into a current module and cover it with tests before putting it in a
production launch path.
