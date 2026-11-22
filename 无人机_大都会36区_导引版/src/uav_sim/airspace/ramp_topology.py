"""Ramp topology profiles and deterministic sparse candidate selection."""
from dataclasses import dataclass
from typing import Iterable, List, Mapping


@dataclass(frozen=True)
class RampTopologyProfile:
    name: str
    adjacent_layer_links_per_pair: int
    express_layer_links_per_pair: int
    cross_region_portals_per_layer: int
    legacy_degree_builder: bool = False
    reference_total_ramps: int = 0
    directed_inter_layer: bool = True


LEGACY_DENSE_BIDIRECTIONAL_V1 = RampTopologyProfile(
    name="legacy_dense_bidirectional_v1",
    adjacent_layer_links_per_pair=8,
    express_layer_links_per_pair=2,
    cross_region_portals_per_layer=10,
    legacy_degree_builder=True,
    reference_total_ramps=31744,
    directed_inter_layer=False,
)

LEGACY_DENSE_DIRECTED_V2 = RampTopologyProfile(
    name="legacy_dense_directed_v2",
    adjacent_layer_links_per_pair=8,
    express_layer_links_per_pair=2,
    cross_region_portals_per_layer=10,
    legacy_degree_builder=True,
)

BALANCED_SPARSE_V1 = RampTopologyProfile(
    name="balanced_sparse_v1",
    adjacent_layer_links_per_pair=4,
    express_layer_links_per_pair=1,
    cross_region_portals_per_layer=3,
)

# One physical metropolis, with O(pipes) links rather than all pipe pairs.
METROPOLIS_PARTITION_V1 = RampTopologyProfile(
    name="metropolis_partition_v1",
    adjacent_layer_links_per_pair=50,
    express_layer_links_per_pair=10,
    cross_region_portals_per_layer=0,
)

TOPOLOGY_PROFILES: Mapping[str, RampTopologyProfile] = {
    "legacy_dense": LEGACY_DENSE_DIRECTED_V2,
    LEGACY_DENSE_BIDIRECTIONAL_V1.name: LEGACY_DENSE_BIDIRECTIONAL_V1,
    LEGACY_DENSE_DIRECTED_V2.name: LEGACY_DENSE_DIRECTED_V2,
    BALANCED_SPARSE_V1.name: BALANCED_SPARSE_V1,
    METROPOLIS_PARTITION_V1.name: METROPOLIS_PARTITION_V1,
}


def get_ramp_topology_profile(value=None, airspace_mode="standard"):
    if isinstance(value, RampTopologyProfile):
        return value
    if value is None:
        value = {"regional_6x6": "balanced_sparse_v1", "metropolis": "metropolis_partition_v1"}.get(airspace_mode, "legacy_dense")
    key = str(value).strip().lower()
    if key not in TOPOLOGY_PROFILES:
        raise ValueError(f"未知匝道拓扑配置: {value}")
    return TOPOLOGY_PROFILES[key]


def evenly_spaced_indexes(total: int, count: int) -> List[int]:
    """Return deterministic, unique indexes spanning the full range."""
    total = int(total)
    count = max(0, min(int(count), total))
    if count == 0:
        return []
    if count == 1:
        return [total // 2]
    indexes = []
    for idx in range(count):
        value = round(idx * (total - 1) / (count - 1))
        if value not in indexes:
            indexes.append(value)
    for value in range(total):
        if len(indexes) >= count:
            break
        if value not in indexes:
            indexes.append(value)
    return sorted(indexes[:count])


def select_spatially_diverse_candidates(candidates: Iterable[dict], count: int) -> List[dict]:
    """Pick short but spatially dispersed inter-layer candidates.

    Candidate features use both pipe indexes and normalized positions. The
    first candidate is the shortest; subsequent candidates maximize their
    minimum feature-space distance from already selected candidates.
    """
    candidates = list(candidates)
    count = max(0, min(int(count), len(candidates)))
    if count == 0:
        return []
    candidates.sort(key=lambda item: (
        float(item["xy_dist"]),
        int(item["pipe_a"].pipe_id),
        int(item["pipe_b"].pipe_id),
    ))
    index_a_max = max(1., max(float(getattr(item["pipe_a"], "local_layer_index", 0)) for item in candidates))
    index_b_max = max(1., max(float(getattr(item["pipe_b"], "local_layer_index", 0)) for item in candidates))
    selected = [candidates.pop(0)]

    def feature(item):
        pipe_a = item["pipe_a"]
        pipe_b = item["pipe_b"]
        return (
            float(getattr(pipe_a, "local_layer_index", 0)) / index_a_max,
            float(getattr(pipe_b, "local_layer_index", 0)) / index_b_max,
            float(item["sa"]) / max(float(pipe_a.length), 1.0),
            float(item["sb"]) / max(float(pipe_b.length), 1.0),
        )

    selected_features = [feature(selected[0])]
    while candidates and len(selected) < count:
        best_index = 0
        best_key = None
        for idx, item in enumerate(candidates):
            item_feature = feature(item)
            min_distance = min(
                sum((left - right) ** 2 for left, right in zip(item_feature, chosen))
                for chosen in selected_features
            )
            key = (
                min_distance,
                -float(item["xy_dist"]),
                -int(item["pipe_a"].pipe_id),
                -int(item["pipe_b"].pipe_id),
            )
            if best_key is None or key > best_key:
                best_key = key
                best_index = idx
        chosen = candidates.pop(best_index)
        selected.append(chosen)
        selected_features.append(feature(chosen))
    return selected
