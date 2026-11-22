"""Low-overhead per-turn wall-clock timing output."""
from dataclasses import asdict, dataclass
from pathlib import Path
import time


@dataclass(frozen=True)
class SimulationTimingSummary:
    completed_turns: int
    average_turn_seconds: float
    simulation_loop_seconds: float
    initial_route_planning_seconds: float
    algorithm_total_seconds: float
    real_time_factor: float

    def to_dict(self):
        return asdict(self)


class SimulationTimingRecorder:
    HEADER = (
        "turn\tsim_time_s\tcompute_s\texport_s\ttotal_s\t"
        "active_uavs\tarrived_uavs\n"
    )

    def __init__(self, path, sim_dt=1.0, flush_interval=50, clock=None):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.sim_dt = float(sim_dt)
        self.flush_interval = max(1, int(flush_interval))
        self.clock = clock or time.perf_counter
        self.started_at = self.clock()
        self._fh = self.path.open("w", encoding="utf-8", buffering=64 * 1024)
        self._fh.write(self.HEADER)
        self._turn_totals = []
        self._closed = False

    def record_turn(self, turn, compute_seconds, export_seconds,
                    total_seconds, active_uavs, arrived_uavs):
        if self._closed:
            raise RuntimeError("timing recorder is closed")
        total_seconds = float(total_seconds)
        self._turn_totals.append(total_seconds)
        self._fh.write(
            f"{int(turn)}\t{int(turn) * self.sim_dt:.3f}\t"
            f"{float(compute_seconds):.6f}\t{float(export_seconds):.6f}\t"
            f"{total_seconds:.6f}\t{int(active_uavs)}\t{int(arrived_uavs)}\n"
        )
        if len(self._turn_totals) % self.flush_interval == 0:
            self._fh.flush()

    def close(self, initial_route_planning_seconds=0.0,
              algorithm_total_seconds=None):
        if not self._closed:
            self._fh.flush()
            self._fh.close()
            self._closed = True
        completed = len(self._turn_totals)
        loop_seconds = float(sum(self._turn_totals))
        average = loop_seconds / completed if completed else 0.0
        if algorithm_total_seconds is None:
            algorithm_total_seconds = self.clock() - self.started_at
        simulated_seconds = completed * self.sim_dt
        real_time_factor = (
            simulated_seconds / loop_seconds if loop_seconds > 0.0 else 0.0
        )
        return SimulationTimingSummary(
            completed_turns=completed,
            average_turn_seconds=average,
            simulation_loop_seconds=loop_seconds,
            initial_route_planning_seconds=float(initial_route_planning_seconds),
            algorithm_total_seconds=float(algorithm_total_seconds),
            real_time_factor=real_time_factor,
        )

