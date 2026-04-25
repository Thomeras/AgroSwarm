"""Unit tests for PadRegistry state machine and allocation."""

from __future__ import annotations

import pytest

from scout_control.core.home_manager import (
    PadRegistry,
    VALID_STATUSES,
    normalize_pad,
)


# ── normalize_pad / backward compatibility ───────────────────────────────────

def test_normalize_pad_fills_defaults_for_legacy_payload() -> None:
    legacy = {
        "pad_id": "pad_0",
        "drone_id": "drone_0",
        "ned": {"x": 1.0, "y": 2.0},
        "status": "available",
    }
    pad = normalize_pad(legacy)
    assert pad is not None
    assert pad["pad_id"] == "pad_0"
    assert pad["drone_id"] == "drone_0"
    assert pad["ned"] == {"x": 1.0, "y": 2.0, "z": -0.5}
    assert pad["status"] == "available"
    # New fields get sensible defaults
    assert pad["charging_capable"] is False
    assert pad["orientation_deg"] == 0.0
    assert pad["service_priority"] == 0
    assert pad["allowed_drone_classes"] == ["*"]


def test_normalize_pad_preserves_extended_metadata() -> None:
    raw = {
        "pad_id": "pad_1",
        "drone_id": "drone_1",
        "ned": {"x": 0.0, "y": 0.0, "z": -0.3},
        "status": "available",
        "charging_capable": True,
        "orientation_deg": 90.5,
        "service_priority": 2,
        "allowed_drone_classes": ["large", "heavy"],
    }
    pad = normalize_pad(raw)
    assert pad is not None
    assert pad["charging_capable"] is True
    assert pad["orientation_deg"] == pytest.approx(90.5)
    assert pad["service_priority"] == 2
    assert pad["allowed_drone_classes"] == ["large", "heavy"]


def test_normalize_pad_rejects_missing_required_fields() -> None:
    assert normalize_pad({"pad_id": "p", "drone_id": "d"}) is None
    assert normalize_pad({}) is None
    assert normalize_pad(None) is None  # type: ignore[arg-type]


def test_normalize_pad_coerces_invalid_status() -> None:
    pad = normalize_pad({
        "pad_id": "p", "drone_id": "d",
        "ned": {"x": 0, "y": 0}, "status": "bogus",
    })
    assert pad is not None
    assert pad["status"] == "available"


def test_normalize_pad_valid_statuses_cover_all_states() -> None:
    assert set(VALID_STATUSES) == {
        "available", "occupied", "charging", "maintenance"
    }


# ── State machine transitions ────────────────────────────────────────────────

def _mk_registry(charging_capable: bool = False) -> PadRegistry:
    return PadRegistry([{
        "pad_id": "pad_0",
        "drone_id": "drone_0",
        "ned": {"x": 0.0, "y": 0.0, "z": -0.5},
        "status": "available",
        "charging_capable": charging_capable,
    }])


def test_rth_request_transitions_available_to_occupied() -> None:
    reg = _mk_registry()
    pad = reg.request_rth("drone_0")
    assert pad is not None
    assert pad["status"] == "occupied"


def test_rth_request_returns_none_for_unknown_drone() -> None:
    reg = _mk_registry()
    assert reg.request_rth("drone_99") is None


def test_landed_transitions_to_charging_when_capable() -> None:
    reg = _mk_registry(charging_capable=True)
    reg.request_rth("drone_0")
    pad = reg.confirm_landed("drone_0")
    assert pad is not None
    assert pad["status"] == "charging"


def test_landed_transitions_to_available_when_not_charging_capable() -> None:
    reg = _mk_registry(charging_capable=False)
    reg.request_rth("drone_0")
    pad = reg.confirm_landed("drone_0")
    assert pad is not None
    assert pad["status"] == "available"


def test_release_moves_charging_back_to_available() -> None:
    reg = _mk_registry(charging_capable=True)
    reg.request_rth("drone_0")
    reg.confirm_landed("drone_0")
    pad = reg.release("drone_0")
    assert pad is not None
    assert pad["status"] == "available"


def test_maintenance_override_from_any_state() -> None:
    reg = _mk_registry(charging_capable=True)
    pad = reg.by_drone("drone_0")
    assert pad is not None

    reg.set_maintenance("pad_0", True)
    assert pad["status"] == "maintenance"

    # Clearing maintenance returns to available
    reg.set_maintenance("pad_0", False)
    assert pad["status"] == "available"


def test_transition_rejects_illegal_jumps() -> None:
    reg = _mk_registry()
    pad = reg.by_drone("drone_0")
    assert pad is not None
    # available -> charging is not allowed directly
    assert reg.transition(pad, "charging") is False
    assert pad["status"] == "available"


def test_confirm_landed_noop_when_not_occupied() -> None:
    reg = _mk_registry()
    pad = reg.confirm_landed("drone_0")
    # Still available (never transitioned to occupied)
    assert pad is not None
    assert pad["status"] == "available"


# ── Upsert / dynamic assignment ──────────────────────────────────────────────

def test_upsert_creates_new_pad_with_defaults() -> None:
    reg = PadRegistry()
    pad, created = reg.upsert_from_assignment(
        drone_id="drone_0", pad_id="pad_0", x=1.0, y=2.0,
    )
    assert created is True
    assert pad["status"] == "available"
    assert pad["charging_capable"] is False
    assert pad["allowed_drone_classes"] == ["*"]


def test_upsert_updates_existing_pad_coords() -> None:
    reg = _mk_registry()
    pad, created = reg.upsert_from_assignment(
        drone_id="drone_0", pad_id="pad_0", x=5.0, y=6.0, z=-0.2,
    )
    assert created is False
    assert pad["ned"] == {"x": 5.0, "y": 6.0, "z": -0.2}


def test_upsert_merges_metadata_without_clobbering_defaults() -> None:
    reg = PadRegistry()
    reg.upsert_from_assignment(
        drone_id="drone_1", pad_id="pad_1", x=0.0, y=0.0,
        charging_capable=True, service_priority=3,
    )
    pad = reg.by_drone("drone_1")
    assert pad is not None
    assert pad["charging_capable"] is True
    assert pad["service_priority"] == 3


# ── Allocation / pad query ───────────────────────────────────────────────────

def test_allocate_prefers_charging_pad_for_low_battery() -> None:
    reg = PadRegistry([
        {"pad_id": "pad_a", "drone_id": "d_a", "ned": {"x": 0, "y": 0},
         "status": "available", "charging_capable": False},
        {"pad_id": "pad_b", "drone_id": "d_b", "ned": {"x": 10, "y": 10},
         "status": "available", "charging_capable": True},
    ])
    pad = reg.allocate(drone_id="drone_0", reason="low_battery")
    assert pad is not None
    assert pad["pad_id"] == "pad_b"


def test_allocate_falls_back_to_any_pad_when_no_charging_available() -> None:
    reg = PadRegistry([
        {"pad_id": "pad_a", "drone_id": "d_a", "ned": {"x": 0, "y": 0},
         "status": "available", "charging_capable": False},
    ])
    pad = reg.allocate(drone_id="drone_0", reason="low_battery")
    assert pad is not None
    assert pad["pad_id"] == "pad_a"


def test_allocate_skips_non_available_pads() -> None:
    reg = PadRegistry([
        {"pad_id": "pad_a", "drone_id": "d_a", "ned": {"x": 0, "y": 0},
         "status": "occupied"},
        {"pad_id": "pad_b", "drone_id": "d_b", "ned": {"x": 5, "y": 5},
         "status": "maintenance"},
        {"pad_id": "pad_c", "drone_id": "d_c", "ned": {"x": 9, "y": 9},
         "status": "available"},
    ])
    pad = reg.allocate(drone_id="drone_0")
    assert pad is not None
    assert pad["pad_id"] == "pad_c"


def test_allocate_sorts_by_priority_then_distance() -> None:
    reg = PadRegistry([
        {"pad_id": "far_hi", "drone_id": "d1", "ned": {"x": 100, "y": 0},
         "status": "available", "service_priority": 0},
        {"pad_id": "near_lo", "drone_id": "d2", "ned": {"x": 1, "y": 0},
         "status": "available", "service_priority": 5},
    ])
    pad = reg.allocate(
        drone_id="drone_0",
        reference_ned={"x": 0, "y": 0},
    )
    assert pad is not None
    # far_hi wins by priority 0 < 5 regardless of distance
    assert pad["pad_id"] == "far_hi"


def test_allocate_respects_drone_class_filter() -> None:
    reg = PadRegistry([
        {"pad_id": "heavy_only", "drone_id": "d1", "ned": {"x": 0, "y": 0},
         "status": "available", "allowed_drone_classes": ["heavy"]},
        {"pad_id": "any", "drone_id": "d2", "ned": {"x": 5, "y": 5},
         "status": "available", "allowed_drone_classes": ["*"]},
    ])
    pad = reg.allocate(drone_id="drone_0", drone_class="light")
    assert pad is not None
    assert pad["pad_id"] == "any"


def test_allocate_returns_none_when_nothing_available() -> None:
    reg = PadRegistry([
        {"pad_id": "p", "drone_id": "d", "ned": {"x": 0, "y": 0},
         "status": "maintenance"},
    ])
    assert reg.allocate(drone_id="drone_0") is None
