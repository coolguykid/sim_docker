"""Atomic server backtracking for mutually blocked, stationary aircraft.

Each input is an already selected current-region flight prefix. This module
selects no route and receives no maps. It can release a false wait cycle by
checking the members' simultaneous departures together. A true unsafe swap
remains rejected, with all original occupancy intact.
"""
from collections import defaultdict
from collections.abc import Mapping
from copy import copy
from dataclasses import dataclass, field
import itertools
import math

from .prefix_reservations import PrefixReservations


@dataclass(frozen=True)
class JointReservationResult:
    accepted: bool
    results: dict = field(default_factory=dict)
    attempted_orders: int = 0
    reason: str = ""
    blocked_by: tuple = ()

    def __bool__(self):
        return self.accepted


def _clone(registry):
    """Copy mutable indexes; immutable phase records and geometry are shared.

    Keeping the counter's exact state matters: retired heap entries can refer
    to record IDs larger than the maximum live ID, so restarting at max+1
    could let a stale heap entry expire an unrelated newly inserted phase.
    """
    trial = copy(registry)
    trial._records = dict(registry._records)
    trial._by_uid = defaultdict(
        set, ((uid, set(ids)) for uid, ids in registry._by_uid.items())
    )
    trial._cells = defaultdict(
        set, ((key, set(ids)) for key, ids in registry._cells.items())
    )
    trial._record_cells = dict(registry._record_cells)
    trial._large = set(registry._large)
    trial._ends = list(registry._ends)
    trial._next_id = copy(registry._next_id)
    trial._pending = dict(registry._pending)
    trial._counts = dict(registry._counts)
    return trial


def _orders(uids, limit):
    if len(uids) <= 4:
        yield from itertools.islice(itertools.permutations(uids), limit)
        return
    seen = set()
    for orientation in (uids, tuple(reversed(uids))):
        for index in range(len(uids)):
            order = orientation[index:] + orientation[:index]
            if order not in seen:
                seen.add(order)
                yield order
                if len(seen) >= limit:
                    return


def wait_cycles(waiting, eligible=None):
    """Return strongly connected wait groups using nonrecursive Kosaraju.

    Restrict ``eligible`` to stationary airborne UIDs before invoking this
    helper. Waiting on a moving outsider is ordinary congestion, not a cycle
    whose existing certificate can be withdrawn.
    """
    members = set(waiting) if eligible is None else set(waiting) & set(eligible)
    adjacency = {
        uid: tuple(other for other in waiting[uid] if other in members)
        for uid in members
    }
    reverse = {uid: [] for uid in members}
    for uid, neighbors in adjacency.items():
        for other in neighbors:
            reverse[other].append(uid)
    seen, order = set(), []
    for uid in sorted(members, key=repr):
        if uid in seen:
            continue
        seen.add(uid)
        stack = [(uid, iter(adjacency[uid]))]
        while stack:
            node, neighbors = stack[-1]
            try:
                other = next(neighbors)
            except StopIteration:
                order.append(node)
                stack.pop()
            else:
                if other not in seen:
                    seen.add(other)
                    stack.append((other, iter(adjacency[other])))
    groups, visited = [], set()
    for uid in reversed(order):
        if uid in visited:
            continue
        component, stack = [], [uid]
        visited.add(uid)
        while stack:
            node = stack.pop()
            component.append(node)
            for other in reverse[node]:
                if other not in visited:
                    visited.add(other)
                    stack.append(other)
        if len(component) > 1:
            groups.append(tuple(sorted(component, key=repr)))
    return tuple(sorted(groups, key=lambda values: repr(values[0])))


def reserve_joint(registry, requests, now, max_orders=8):
    """Reserve a stationary wait group atomically, or change nothing.

    ``requests`` maps UID to the keyword arguments accepted by
    ``PrefixReservations.reserve``, except UID and earliest_start. Times in
    each supplied prefix are relative to zero. Only genuinely stationary
    members with a currently effective hold may be replaced. Already committed
    moving or future-departure profiles cannot be canceled by this operation.

    Every trial still checks exact continuous separation, waiting intervals,
    endpoint holds and all outsiders through the normal reservation engine.
    No trial state is published until every member has a safe certificate.
    """
    if not isinstance(registry, PrefixReservations):
        raise TypeError("registry must be PrefixReservations")
    if not isinstance(requests, Mapping) or not requests:
        raise ValueError("joint requests must be a nonempty UID mapping")
    now = float(now)
    if not math.isfinite(now):
        raise ValueError("now must be finite")
    if (
        isinstance(max_orders, bool)
        or not isinstance(max_orders, int)
        or max_orders < 1
    ):
        raise ValueError("max_orders must be a positive integer")
    normalized = {}
    for uid, options in requests.items():
        options = dict(options)
        pieces = tuple(options.get("pieces", ()))
        options["pieces"] = pieces
        records = [registry._records[index] for index in registry._by_uid.get(uid, ())]
        holds = [record for record in records if record.is_hold and record.t0 <= now]
        if not holds or any(
            not record.is_hold and record.t1 > now for record in records
        ):
            raise ValueError(
                "joint member %r must be stationary at an effective hold" % (uid,)
            )
        hold = holds[0]
        source = pieces[0].position0 if pieces else options.get("start_position")
        if source is None or math.dist(source, hold.position) > 1e-5:
            raise ValueError(
                "joint member %r must start at its actual held position" % (uid,)
            )
        if pieces and math.hypot(*pieces[0].velocity0) > 1e-6:
            raise ValueError("stationary joint member %r must start at rest" % (uid,))
        if options.get("cold", False):
            raise ValueError("joint wait-cycle members must already be airborne")
        normalized[uid] = options
    members = tuple(sorted(normalized, key=repr))
    attempted, blockers, last_reason = 0, set(), ""
    for order in _orders(members, max_orders):
        attempted += 1
        trial = _clone(registry)
        trial.expire(now)
        for uid in members:
            trial.release(uid)
        results = {}
        for uid in order:
            result = trial.reserve(uid, earliest_start=now, **normalized[uid])
            if not result.accepted:
                blockers.update(result.blocked_by)
                last_reason = result.reason
                break
            results[uid] = result
        else:
            # The caller owns the single authoritative simulation thread.
            # Preserve object identity so its controller keeps this registry.
            registry.__dict__.clear()
            registry.__dict__.update(trial.__dict__)
            return JointReservationResult(True, results, attempted)
    return JointReservationResult(
        False,
        attempted_orders=attempted,
        reason=last_reason or "no safe joint departure order",
        blocked_by=tuple(sorted(blockers, key=repr)),
    )
