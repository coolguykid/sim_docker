"""Physical-time replenishment, independent of the selected timestep."""
import math


class PopulationScheduler:
    def __init__(
        self, target=10000, batch_limit=1000, period_seconds=100.0, enabled=True
    ):
        if (
            any(
                isinstance(x, bool) or not isinstance(x, int) or x < 1
                for x in (target, batch_limit)
            )
            or batch_limit > 1000
        ):
            raise ValueError(
                "Population must be positive; each batch is limited to 1000"
            )
        if (
            isinstance(period_seconds, bool)
            or not math.isfinite(period_seconds)
            or period_seconds <= 0
            or abs(round(period_seconds * 10) - period_seconds * 10) > 1e-8
        ):
            raise ValueError("Population period must be positive whole deciseconds")
        self.target = target
        self.batch_limit = batch_limit
        self.period_ticks = round(period_seconds * 10)
        self.enabled = enabled
        self.next_tick = 0
        self.batch_index = 0

    def due(self, time_seconds, active, pending):
        if (
            not math.isfinite(time_seconds)
            or time_seconds < 0
            or min(active, pending) < 0
        ):
            raise ValueError("Population counts and time must be nonnegative")
        tick = round(time_seconds * 10)
        if tick < self.next_tick or (not self.enabled and self.batch_index):
            return 0
        self.next_tick = (tick // self.period_ticks + 1) * self.period_ticks
        self.batch_index += 1
        return max(0, min(self.batch_limit, self.target - active - pending))
