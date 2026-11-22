"""Reproducible goals on the original physical pipes, without routing state.

The default pool is sampled once from the 600 m layer and reused by all
batches. Returned dictionaries are copies so graph compilation and callers
cannot alter the pool used by later missions.
"""
import math
import random
from collections.abc import Mapping
from numbers import Integral, Real


class DestinationPool:
    """Sample interior lane points and assign missions in round-robin order.

    ``manual`` supplies an explicit fixed selection (its length determines the
    effective count). It may reference any physical layer for legacy runs.
    ``per_batch`` resamples generated selections with a local ``seed + index``
    RNG; requesting batches in a different order does not change their goals.
    """

    TOP_HEIGHT_M = 600.0
    INTERIOR_MARGIN_M = 1.e-6

    def __init__(self, env, count=10, seed=42, policy="fixed", manual=None):
        self.count = self._integer(count, "count", minimum=1)
        self.seed = self._integer(seed, "seed")
        if policy not in ("fixed", "per_batch"):
            raise ValueError("policy must be 'fixed' or 'per_batch'")
        self.policy = policy
        self._batches = {}
        self._pipes = {}
        for pipe in env.pipelines:
            pipe_id = self._integer(pipe.pipe_id, "pipe_id")
            if pipe_id in self._pipes:
                raise ValueError("duplicate original pipe_id: {}".format(pipe_id))
            self._pipes[pipe_id] = pipe

        self._manual = manual is not None
        if self._manual:
            self._manual_specs = self._validate_manual(manual)
            self.count = len(self._manual_specs)
        else:
            self._top_pipes = tuple(
                pipe for _, pipe in sorted(self._pipes.items())
                if float(pipe.height) == self.TOP_HEIGHT_M
            )
            if not self._top_pipes:
                raise ValueError("destination pool requires original pipes at height 600 m")
            for pipe in self._top_pipes:
                self._validate_pipe(pipe)
        # Fixed sampling occurs once, even if the first request is batch 99.
        if self.policy == "fixed" or self._manual:
            self._batches[0] = self._manual_specs if self._manual else self._generate(0)

    @staticmethod
    def _integer(value, name, minimum=None):
        if isinstance(value, bool) or not isinstance(value, Integral):
            raise ValueError("{} must be an integer".format(name))
        value = int(value)
        if minimum is not None and value < minimum:
            raise ValueError("{} must be at least {}".format(name, minimum))
        return value

    @classmethod
    def _validate_pipe(cls, pipe):
        try:
            length = float(pipe.length)
        except (TypeError, ValueError):
            raise ValueError("pipe length must be finite and positive") from None
        if not math.isfinite(length) or length <= 2.0 * cls.INTERIOR_MARGIN_M:
            raise ValueError("pipe length is invalid or too short for an interior destination")
        cls._integer(pipe.num_lanes, "lane count", minimum=1)
        return length

    def _validate_manual(self, manual):
        if isinstance(manual, (str, bytes, Mapping)):
            raise ValueError("manual destinations must be a nonempty sequence of records")
        try:
            records = list(manual)
        except TypeError:
            raise ValueError("manual destinations must be a nonempty sequence of records") from None
        if not records:
            raise ValueError("manual destinations must not be empty")
        result, node_ids, points = [], set(), set()
        for index, record in enumerate(records):
            if not isinstance(record, Mapping):
                raise ValueError("manual destination must be a record")
            if not {"pipe_id", "lane_id", "distance_m"}.issubset(record):
                raise ValueError("manual destination requires pipe_id, lane_id and distance_m")
            pipe_id = self._integer(record["pipe_id"], "pipe_id")
            if pipe_id not in self._pipes:
                raise ValueError("manual destination references an unknown physical pipe")
            pipe = self._pipes[pipe_id]
            length = self._validate_pipe(pipe)
            lane_id = self._integer(record["lane_id"], "lane_id", minimum=0)
            if lane_id >= pipe.num_lanes:
                raise ValueError("manual destination lane lies outside its physical pipe")
            distance = record["distance_m"]
            if isinstance(distance, bool) or not isinstance(distance, Real):
                raise ValueError("manual distance_m must be a finite number")
            distance = float(distance)
            if not math.isfinite(distance) or not 0.0 <= distance <= length:
                raise ValueError("manual distance_m must lie on its physical pipe")
            node_id = record.get("node_id", "GOAL_MANUAL_{}".format(index))
            if not isinstance(node_id, str) or not node_id.strip():
                raise ValueError("manual node_id must be a nonempty string")
            if node_id in node_ids:
                raise ValueError("duplicate manual destination node_id")
            point = (pipe_id, lane_id, distance)
            if point in points:
                raise ValueError("duplicate manual destination coordinates")
            node_ids.add(node_id)
            points.add(point)
            result.append(dict(node_id=node_id, pipe_id=pipe_id,
                               lane_id=lane_id, distance_m=distance))
        return tuple(result)

    def _generate(self, batch_index):
        value = self.seed + batch_index
        # Random treats negative integer seeds as positive; a signed string
        # prevents -n and +n batches from silently sharing the same points.
        rng = random.Random(value if value >= 0 else str(value))
        result, points = [], set()
        candidates = []
        for index in range(self.count):
            if not candidates:
                candidates = rng.sample(self._top_pipes, len(self._top_pipes))
            pipe = candidates.pop()
            length = float(pipe.length)
            while True:
                lane_id = rng.randrange(int(pipe.num_lanes))
                distance = rng.uniform(self.INTERIOR_MARGIN_M,
                                       length - self.INTERIOR_MARGIN_M)
                point = (int(pipe.pipe_id), lane_id, distance)
                if point not in points:
                    break
            points.add(point)
            prefix = ("GOAL_FIXED" if self.policy == "fixed" else
                      "GOAL_BATCH_{}".format(batch_index))
            result.append(dict(node_id="{}_{}".format(prefix, index),
                               pipe_id=int(pipe.pipe_id), lane_id=lane_id,
                               distance_m=distance))
        return tuple(result)

    @staticmethod
    def _copies(specs):
        return tuple(dict(spec) for spec in specs)

    def specs(self, batch_index=0):
        """Return fresh records for a zero-based batch index."""
        batch_index = self._integer(batch_index, "batch_index", minimum=0)
        key = 0 if self.policy == "fixed" or self._manual else batch_index
        if key not in self._batches:
            self._batches[key] = self._generate(key)
        return self._copies(self._batches[key])

    def all_specs(self, batch_count):
        """Return unique goals needed by batches 0 through ``batch_count - 1``."""
        batch_count = self._integer(batch_count, "batch_count", minimum=0)
        unique = {}
        for batch_index in range(batch_count):
            for spec in self.specs(batch_index):
                unique.setdefault(spec["node_id"], spec)
        return tuple(unique.values())

    def for_mission(self, uav_id, batch_index=0):
        """Balance zero-based mission IDs over the selected batch's goals."""
        uav_id = self._integer(uav_id, "uav_id", minimum=0)
        specs = self.specs(batch_index)
        return specs[uav_id % self.count]

    def manifest(self):
        """Return a JSON-ready snapshot of the generated pools and settings."""
        return dict(policy=self.policy, seed=self.seed, count=self.count,
                    manual=self._manual,
                    batches=[dict(batch_index=index, specs=list(self._copies(specs)))
                             for index, specs in sorted(self._batches.items())])
