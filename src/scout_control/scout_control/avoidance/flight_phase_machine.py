"""Small phase-state boundary for obstacle avoidance runtime."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Generic, TypeVar


PhaseT = TypeVar("PhaseT", bound=Enum)


@dataclass(frozen=True, slots=True)
class PhaseTransition(Generic[PhaseT]):
    """Immutable record for one runtime phase transition."""

    old_phase: PhaseT
    new_phase: PhaseT
    reason: str
    entered_at_s: float
    fields: dict[str, Any] = field(default_factory=dict)


class FlightPhaseMachine(Generic[PhaseT]):
    """Own the current flight phase, phase ticks, and phase entry timestamp."""

    def __init__(
        self,
        initial_phase: PhaseT,
        *,
        clock: Callable[[], float] = time.time,
    ) -> None:
        self._clock = clock
        self._phase = initial_phase
        self._ticks = 0
        self._entered_at_s = float(clock())

    @property
    def phase(self) -> PhaseT:
        return self._phase

    @property
    def ticks(self) -> int:
        return self._ticks

    @property
    def entered_at_s(self) -> float:
        return self._entered_at_s

    def tick(self) -> int:
        self._ticks += 1
        return self._ticks

    def transition_to(
        self,
        new_phase: PhaseT,
        *,
        reason: str,
        **fields: Any,
    ) -> PhaseTransition[PhaseT]:
        old_phase = self._phase
        self._phase = new_phase
        self._ticks = 0
        self._entered_at_s = float(self._clock())
        return PhaseTransition(
            old_phase=old_phase,
            new_phase=new_phase,
            reason=reason,
            entered_at_s=self._entered_at_s,
            fields=dict(fields),
        )
