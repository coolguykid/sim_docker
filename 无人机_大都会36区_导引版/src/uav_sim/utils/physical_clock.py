"""An integer-decisecond clock shared by original serial and Ray loops."""
import math


class PhysicalClock:
    def __init__(self, dt=1.0):
        if (
            isinstance(dt, bool)
            or not isinstance(dt, (int, float))
            or not math.isfinite(dt)
            or dt not in (1.0, 0.1)
        ):
            raise ValueError("Simulation dt must be 1 or 0.1 seconds")
        self.dt_ticks = 10 if dt == 1 else 1
        self.step_index = 0

    @property
    def dt(self):
        return self.dt_ticks / 10

    @property
    def time_seconds(self):
        return self.step_index * self.dt_ticks / 10

    def advance(self):
        self.step_index += 1
        return self.time_seconds

    def steps_for(self, seconds):
        if isinstance(seconds, bool) or not math.isfinite(seconds) or seconds < 0:
            raise ValueError("Duration must be finite nonnegative seconds")
        n = round(seconds / self.dt)
        if abs(n * self.dt - seconds) > 1e-8:
            raise ValueError("Duration must contain whole simulation steps")
        return n

    def context(self, version=0, time_seconds=None):
        return dict(
            step_index=self.step_index,
            sim_time_seconds=self.time_seconds
            if time_seconds is None
            else time_seconds,
            sim_dt_seconds=self.dt,
            topology_version=version,
        )
