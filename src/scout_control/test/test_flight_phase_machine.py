from enum import Enum, auto

from scout_control.avoidance.flight_phase_machine import FlightPhaseMachine


class Phase(Enum):
    IDLE = auto()
    CRUISE = auto()


def test_flight_phase_machine_tracks_ticks_and_transition_metadata() -> None:
    now = [10.0]
    machine = FlightPhaseMachine(Phase.IDLE, clock=lambda: now[0])

    assert machine.phase == Phase.IDLE
    assert machine.entered_at_s == 10.0
    assert machine.tick() == 1
    assert machine.tick() == 2

    now[0] = 12.5
    transition = machine.transition_to(
        Phase.CRUISE,
        reason="target_active",
        target_id="cell_x1_y2",
    )

    assert transition.old_phase == Phase.IDLE
    assert transition.new_phase == Phase.CRUISE
    assert transition.reason == "target_active"
    assert transition.fields == {"target_id": "cell_x1_y2"}
    assert machine.phase == Phase.CRUISE
    assert machine.ticks == 0
    assert machine.entered_at_s == 12.5
