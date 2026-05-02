# Legacy Scenarios

These launcher scenarios are archived/debug workflows from earlier project
phases.

They are useful for:

- isolating camera, lidar, GCS, ML, spray, and coordinator nodes,
- replaying old Phase 1-3 verification flows,
- debugging Isaac/Pegasus paths separately from the current Gazebo E2E flow.

They are not the current production path. The current final milestone scenario
is `../full_e2e_mission.yaml`.

Before promoting any legacy scenario back to production, re-test it against the
current topic contract and confirm that it does not start a second PX4 setpoint
owner.
