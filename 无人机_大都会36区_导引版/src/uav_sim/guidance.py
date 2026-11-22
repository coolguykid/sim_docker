"""Region/layer digital-twin model and guidance evaluation."""
from dataclasses import asdict, dataclass, fields
import math
from numbers import Integral, Real

import numpy as np

from uav_sim.algorithms.avoidance_features import collision_threshold_for_height


# label, minimum, maximum, decimals. Shared by GUI, command line and validation.
PARAMETERS = {
    "sample_interval_seconds": ("采样与导引周期（秒）", 1, 3600, 0),
    "density_limit": ("密度上限 ρmax", .000001, 1., 6),
    "capacity_scale": ("容量换算系数", .000001, 100., 6),
    "gamma": ("CBF 衰减率 γ", .000001, 1., 6),
    "balance_weight": ("流量均衡权重 λ1", 0., 1000000., 4),
    "intention_weight": ("飞行意愿权重 λ2", 0., 1000000., 4),
    "max_probability_change": ("单次概率最大变化 ΔP", 0., 1., 4),
    "deviation_rate": ("执行偏移率 α", 0., 1., 4),
    "rollout_steps": ("马尔可夫推演步数", 1, 100, 0),
    "refinement_rounds": ("候选求解轮数上限", 1, 20, 0),
    "overload_penalty": ("超载单元格惩罚倍率 Γ", 1.0001, 100., 4),
    "edge_penalty_seconds": ("Dijkstra 导引权重（秒）", 0., 10000., 4),
}


@dataclass(frozen=True)
class GuidanceConfig:
    enabled: bool = False
    evaluate: bool = False
    sample_interval_seconds: int = 30
    density_limit: float = 1.
    capacity_scale: float = 1.
    gamma: float = .5
    balance_weight: float = 10.
    intention_weight: float = 1.
    max_probability_change: float = .25
    deviation_rate: float = 0.
    rollout_steps: int = 3
    refinement_rounds: int = 3
    overload_penalty: float = 2.
    edge_penalty_seconds: float = 15.

    def __post_init__(self):
        if type(self.enabled) is not bool or type(self.evaluate) is not bool:
            raise ValueError("guidance.enabled and guidance.evaluate must be booleans")
        for key, (_, low, high, decimals) in PARAMETERS.items():
            value = getattr(self, key)
            expected = Real if decimals else Integral
            if isinstance(value, bool) or not isinstance(value, expected) or not low <= value <= high:
                raise ValueError("guidance.{} must be {} in [{}, {}]".format(
                    key, "a finite number" if decimals else "an integer", low, high))
        if self.balance_weight == self.intention_weight == 0.:
            raise ValueError("at least one guidance objective weight must be positive")

    @classmethod
    def from_mapping(cls, mapping):
        if not isinstance(mapping, dict):
            raise ValueError("guidance must be a JSON object")
        unknown = set(mapping) - {field.name for field in fields(cls)}
        if unknown:
            raise ValueError("unknown guidance parameters: {}".format(", ".join(sorted(unknown))))
        return cls(**mapping)


class GuidanceNetwork:
    def __init__(self, graph, capacity_scale):
        region_ids = sorted(graph.regions)
        layers = sorted({float(p["height"]) for p in graph.metadata.get("pipes", {}).values()}
                        or {n.position[2] for n in graph.nodes.values()})
        self.layers = np.asarray(layers, dtype=float)
        self.regions = {rid: index for index, rid in enumerate(region_ids)}
        self.cells = tuple((rid, layer) for rid in region_ids for layer in layers)
        self.index = {cell: index for index, cell in enumerate(self.cells)}
        self.cell_regions = np.array([self.regions[rid] for rid, _ in self.cells])
        self.centers = {
            rid: np.array([(r["x_min"] + r["x_max"]) / 2, (r["y_min"] + r["y_max"]) / 2])
            for rid, r in graph.regions.items()
        }
        self.legal = np.eye(len(self.cells), dtype=bool)
        self.edge_transfers = {}
        capacity = np.zeros(len(self.cells))
        spacing = {height: collision_threshold_for_height(int(height)) for height in layers}
        for edge in graph.edges.values():
            if edge.ramp_id is not None and not graph.ramp_states.get(edge.ramp_id, True):
                continue
            a, b = graph.nodes[edge.source], graph.nodes[edge.target]
            source = self.index[self.cell(a.region_id, a.position[2])]
            target = self.index[self.cell(b.region_id, b.position[2])]
            if source != target:
                self.legal[source, target] = True
                self.edge_transfers[edge.edge_id] = (source, target)
            if edge.kind == "FLY" and edge.pipe_id is not None:
                capacity[source] += edge.lane_count * edge.length_m / spacing[self.cells[source][1]]
        # A ramp may traverse a region/layer without a through pipe.
        self.capacities = np.maximum(1., capacity * capacity_scale)
        self._directions = {}

    def cell(self, region, height):
        return (region, float(self.layers[np.abs(self.layers - height).argmin()]))

    def _preferred_regions(self, source, destination):
        key = (source, destination)
        if key not in self._directions:
            region = self.cells[source][0]
            legal_regions = sorted({self.cells[i][0] for i in np.flatnonzero(self.legal[source])} - {region})
            if region == destination or not legal_regions:
                best = [region]
            else:
                direction = self.centers[destination] - self.centers[region]
                local = np.array([self.centers[rid] - self.centers[region] for rid in legal_regions])
                scores = (local @ direction) / (np.linalg.norm(local, axis=1) * np.linalg.norm(direction))
                best = [rid for rid, ok in zip(legal_regions, np.isclose(scores, scores.max(), atol=1.e-12,
                                                                        rtol=0.)) if ok]
            self._directions[key] = [self.regions[rid] for rid in best]
        return self._directions[key]

    def intentions(self, observations):
        counts = np.zeros(len(self.cells), dtype=int)
        q = np.zeros((len(self.cells), len(self.regions)))
        for cell, destination in observations:
            source = self.index[cell]
            best = self._preferred_regions(source, destination)
            q[source, best] += 1. / len(best)
            counts[source] += 1
        np.divide(q, counts[:, None], out=q, where=counts[:, None] != 0)
        return counts, q

    def lift(self, q, counts):
        """Split each region-level intention across its legal target layers."""
        distribution = self.legal / self.legal.sum(axis=1, keepdims=True)
        for source in np.flatnonzero(counts):
            candidates = np.flatnonzero(self.legal[source])
            regions = self.cell_regions[candidates]
            multiplicity = np.bincount(regions, minlength=q.shape[1])
            distribution[source, candidates] = q[source, regions] / multiplicity[regions]
        return distribution

    def edge_penalties(self, policy, guided_sources, strength):
        # The probability floor preserves reachability even for a zero quota.
        return {
            edge: -strength * math.log(max(min(policy[a, b], 1.), 1.e-6))
            for edge, (a, b) in self.edge_transfers.items()
            if guided_sources[a] and policy[a, b] < 1. and strength > 0.
        }


def actual_probabilities(policy, legal, alpha):
    """Spread the deviating mass uniformly over other legal next cells."""
    alternatives = legal.sum(axis=1, keepdims=True) - 1
    overflow = np.divide(alpha * (policy.sum(axis=1, keepdims=True) - policy),
                         alternatives, out=np.zeros_like(policy), where=alternatives > 0)
    return np.where(legal, np.where(alternatives > 0, (1. - alpha) * policy + overflow, policy), 0.)


class GuidanceModel:
    """Solve the CBF-triggered policy and expose a report for comparison runs."""
    def __init__(self, network, config):
        self.network, self.config = network, config
        self.active_policy = None
        self.edge_penalties = {}
        self.revision = 0
        self.transfer_feedback = None
        self.on_sample = None
        self._totals = dict(samples=0, trigger_samples=0,
                            rejected_updates=0, overloaded_cell_samples=0,
                            peak_actual_density=0., peak_predicted_density=0.,
                            density_variance_sum=0., guided_missions=0)

    def _predicted(self, density, counts, policy):
        incoming = counts @ policy - counts * policy.diagonal()
        return density + incoming / self.network.capacities

    def _rollout_risk(self, counts, effective):
        max_density = np.zeros(len(counts))
        population = counts.astype(float)
        for _ in range(self.config.rollout_steps):
            population = population @ effective
            max_density = np.maximum(max_density, population / self.network.capacities)
        return max_density

    def evaluate(self, counts, q, *, unexpected_departures=None, predicted_density=None, transfer_feedback=None):
        """Return the current/candidate policy and immutable-to-callers diagnostics.

        Departure counts and externally predicted densities use network.cells
        order. An optional feedback callback receives a policy copy and
        returns {(source_cell, target_cell): maximum aircraft per second}.
        """
        from uav_sim.guidance_solver import solve_policy

        cfg, net = self.config, self.network
        density = counts / net.capacities
        departing = (np.zeros_like(counts) if unexpected_departures is None
                     else np.asarray(unexpected_departures, dtype=float))
        if departing.shape != counts.shape or np.any(~np.isfinite(departing)) or np.any(departing < 0.):
            raise ValueError("unexpected departures must be finite nonnegative aircraft counts")
        retained = np.maximum(0., counts - departing) / net.capacities
        initial = net.lift(q, counts)
        old = self.active_policy
        previous = initial if old is None else old
        effective = actual_probabilities(previous, net.legal, cfg.deviation_rate)
        estimated = (self._predicted(retained, counts, effective) if predicted_density is None
                     else np.asarray(predicted_density, dtype=float))
        if estimated.shape != density.shape or np.any(~np.isfinite(estimated)) or np.any(estimated < 0.):
            raise ValueError("predicted density must be a finite nonnegative value per cell")
        threshold = (1. - cfg.gamma) * density + cfg.gamma * cfg.density_limit
        triggered = bool(np.any(estimated >= threshold))
        stats = dict(triggered=triggered, status="disabled" if not cfg.enabled else "idle",
                     actual_density=density, predicted_density=estimated, trigger_density=threshold,
                     maximum_rollout_density=None, refinement_rounds=0)
        if not cfg.enabled or not triggered or not np.any(counts):
            return previous.copy(), stats

        penalty = np.ones(len(counts))
        limits = np.full(net.legal.shape, np.inf)
        for _ in range(cfg.refinement_rounds):
            stats["refinement_rounds"] += 1
            policy, status = solve_policy(net, counts, q, previous, retained, penalty, old, limits, cfg)
            if policy is None:
                stats["status"] = status
                break
            effective = actual_probabilities(policy, net.legal, cfg.deviation_rate)
            rollout = np.maximum(self._rollout_risk(np.maximum(0., counts - departing), effective),
                                 self._predicted(retained, counts, effective))
            risk = rollout > cfg.density_limit + 1.e-6
            stats["maximum_rollout_density"] = float(rollout.max())
            exceeds_transfer = False
            if transfer_feedback is not None:
                for (source, target), rate in transfer_feedback(policy.copy()).items():
                    a, b = net.index[source], net.index[target]
                    if a == b or not net.legal[a, b] or not math.isfinite(rate) or rate < 0.:
                        raise ValueError("transfer feedback requires a legal movement and a finite nonnegative rate")
                    limits[a, b] = min(counts[a], cfg.sample_interval_seconds * rate)
                    exceeds_transfer |= counts[a] * policy[a, b] > limits[a, b] + 1.e-6
            if not np.any(risk) and not exceeds_transfer:
                stats["status"] = "published"
                self.active_policy = policy
                self.revision += 1
                self.edge_penalties = net.edge_penalties(effective, counts > 0, cfg.edge_penalty_seconds)
                return policy.copy(), stats
            stats["status"] = "rollout overload" if np.any(risk) else "transfer feedback requires another solve"
            penalty[risk] *= cfg.overload_penalty
        return previous.copy(), stats

    def observe(self, time_seconds, observations):
        counts, q = self.network.intentions(observations)
        policy, stats = self.evaluate(counts, q, transfer_feedback=self.transfer_feedback)
        totals = self._totals
        totals["samples"] += 1
        totals["trigger_samples"] += int(stats["triggered"])
        totals["rejected_updates"] += int(stats["status"] not in ("published", "idle", "disabled"))
        totals["overloaded_cell_samples"] += int(np.count_nonzero(stats["actual_density"] > self.config.density_limit))
        totals["peak_actual_density"] = max(totals["peak_actual_density"], float(stats["actual_density"].max()))
        totals["peak_predicted_density"] = max(totals["peak_predicted_density"], float(stats["predicted_density"].max()))
        totals["density_variance_sum"] += float(np.var(stats["actual_density"]))
        if self.on_sample is not None:
            self.on_sample(dict(stats, time_seconds=time_seconds, cells=self.network.cells,
                                region_ids=tuple(self.network.regions), counts=counts, intention=q,
                                policy=policy, revision=self.revision))

    def record_guided_missions(self, count):
        self._totals["guided_missions"] += count

    def report(self):
        stats = dict(self._totals)
        total = stats.pop("density_variance_sum")
        samples, cells = stats["samples"], len(self.network.cells)
        stats["published_policies"] = self.revision
        stats["mean_spatial_density_variance"] = total / samples if samples else None
        stats["overloaded_cell_sample_fraction"] = (
            stats["overloaded_cell_samples"] / (samples * cells) if samples else None)
        return dict(config=asdict(self.config), cell_count=cells, **stats)


def compare_guidance_runs(baseline, guided):
    """Metrics and guided-minus-baseline differences from two session summaries."""
    first, second = baseline["guidance"], guided["guidance"]
    if first["config"]["enabled"] or not second["config"]["enabled"]:
        raise ValueError("comparison requires a disabled baseline and an enabled guidance run")
    if not first["samples"] or not second["samples"]:
        raise ValueError("comparison requires sampled guidance metrics in both runs")
    for key in ("routing_mode", "sim_dt_seconds", "destinations"):
        if baseline[key] != guided[key]:
            raise ValueError("comparison requires matching {}".format(key))
    for key in ("sample_interval_seconds", "capacity_scale", "density_limit"):
        if first["config"][key] != second["config"][key]:
            raise ValueError("comparison requires matching guidance.{}".format(key))
    if first["cell_count"] != second["cell_count"]:
        raise ValueError("comparison requires matching cell counts")
    result = {}
    keys = ("generated", "arrived", "mean_travel_seconds", "mean_ground_wait_seconds", "safety_violations")
    density_keys = ("peak_actual_density", "mean_spatial_density_variance",
                    "overloaded_cell_samples", "overloaded_cell_sample_fraction")
    for key, a, b in ([(key, baseline[key], guided[key]) for key in keys]
                      + [(key, first[key], second[key]) for key in density_keys]):
        result[key] = dict(baseline=a, guided=b, delta=b - a if a is not None and b is not None else None)
    return result
