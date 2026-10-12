"""Ordered slot selection and fail-closed state transitions."""

from __future__ import annotations
import re
from typing import Any, Mapping, Sequence
from .models import (PlacementSlotError, SlotCapacityError, SlotStateError, SlotStatus,
                     SLOTS_PER_ZONE, load_placement_slots, _MATRIX_NODE_KINDS)

def slot_for_occupied_count(
    points: Mapping[str, Any], zone: str, occupied_count: int
) -> PlacementSlot:
    """Return the next 1-based slot after a contiguous occupied prefix.

    ``occupied_count`` is the number of reserved or physically occupied slots,
    not merely the number of placements that passed strict region verification.
    Callers should use :func:`next_available_slot` when per-slot statuses exist.
    """
    if isinstance(occupied_count, bool) or not isinstance(occupied_count, int):
        raise PlacementSlotError("occupied_count must be an integer")
    if occupied_count < 0:
        raise PlacementSlotError("occupied_count cannot be negative")
    slots = load_placement_slots(points, zone)
    if not slots:
        raise PlacementSlotError(f"zone {zone} has no placement slots")
    if occupied_count >= len(slots):
        raise SlotCapacityError(f"all {len(slots)} slots in zone {zone} are occupied")
    return slots[occupied_count]

def _coerce_status(value: SlotStatus | str) -> SlotStatus:
    if isinstance(value, SlotStatus):
        return value
    try:
        return SlotStatus(str(value))
    except ValueError as exc:
        raise SlotStateError(f"unknown slot status: {value!r}") from exc

def occupied_slot_count(
    states: Mapping[str, SlotStatus | str] | Sequence[SlotStatus | str]
) -> int:
    """Count reserved, ambiguous, physically released, and verified slots."""
    values = states.values() if isinstance(states, Mapping) else states
    return sum(_coerce_status(value) is not SlotStatus.AVAILABLE for value in values)

def next_available_slot(
    points: Mapping[str, Any],
    zone: str,
    states: Mapping[str, SlotStatus | str] | None = None,
) -> PlacementSlot:
    """Select the first reusable slot or stop on an in-flight/unknown slot.

    A ``reserved`` slot blocks advancement until its release outcome is known.
    ``unknown`` also blocks safely. ``physically_released`` consumes the slot
    even when strict placement verification failed; ``verified`` consumes it.
    A pre-release failure is reusable after transitioning ``reserved`` back to
    ``available`` with :func:`transition_slot_state`.
    """
    slots = load_placement_slots(points, zone)
    if not slots:
        raise PlacementSlotError(f"zone {zone} has no placement slots")
    status_map = {str(key): _coerce_status(value) for key, value in (states or {}).items()}
    known_ids = {slot.slot_id for slot in slots}
    unknown_ids = set(status_map) - known_ids
    if unknown_ids:
        raise SlotStateError(f"slot state contains unknown IDs: {sorted(unknown_ids)}")

    first_available: PlacementSlot | None = None
    saw_available = False
    for slot in slots:
        status = status_map.get(slot.slot_id, SlotStatus.AVAILABLE)
        if status is SlotStatus.AVAILABLE:
            if first_available is None:
                first_available = slot
            saw_available = True
            continue
        if saw_available:
            raise SlotStateError("occupied slots must form a contiguous prefix")
        if status is SlotStatus.RESERVED:
            raise SlotStateError(f"slot {slot.slot_id} is still reserved")
        if status is SlotStatus.UNKNOWN:
            raise SlotStateError(f"slot {slot.slot_id} release outcome is unknown")
    if first_available is not None:
        return first_available
    raise SlotCapacityError(f"all {len(slots)} slots in zone {zone} are occupied")

_ALLOWED_TRANSITIONS = {
    SlotStatus.AVAILABLE: frozenset((SlotStatus.RESERVED,)),
    SlotStatus.RESERVED: frozenset((SlotStatus.AVAILABLE, SlotStatus.PHYSICALLY_RELEASED, SlotStatus.UNKNOWN)),
    SlotStatus.PHYSICALLY_RELEASED: frozenset((SlotStatus.VERIFIED,)),
    SlotStatus.UNKNOWN: frozenset((SlotStatus.PHYSICALLY_RELEASED, SlotStatus.VERIFIED)),
    SlotStatus.VERIFIED: frozenset(),
}

def transition_slot_state(
    states: Mapping[str, SlotStatus | str],
    slot_id: str,
    new_state: SlotStatus | str,
) -> dict[str, SlotStatus]:
    """Return a copied state map after checking a legal slot lifecycle step."""
    current_states = {str(key): _coerce_status(value) for key, value in states.items()}
    old_state = current_states.get(slot_id, SlotStatus.AVAILABLE)
    target_state = _coerce_status(new_state)
    if target_state not in _ALLOWED_TRANSITIONS[old_state]:
        raise SlotStateError(f"invalid slot transition for {slot_id}: {old_state.value} -> {target_state.value}")
    current_states[slot_id] = target_state
    return current_states

def matrix_node_name(zone: str, slot_number: int, kind: str) -> str:
    """Return stable route-matrix node names for staging, release, and exit."""
    normalized_zone = str(zone).upper()
    if not re.fullmatch(r"[A-Z][A-Z0-9_]*", normalized_zone):
        raise PlacementSlotError(f"invalid zone name: {zone!r}")
    if isinstance(slot_number, bool) or not isinstance(slot_number, int) or not 1 <= slot_number <= SLOTS_PER_ZONE:
        raise PlacementSlotError(f"slot_number must be between 1 and {SLOTS_PER_ZONE}")
    if kind not in _MATRIX_NODE_KINDS:
        raise PlacementSlotError(f"unsupported slot matrix node kind: {kind!r}")
    return f"zone_{normalized_zone}_slot_{slot_number}_{kind}"
