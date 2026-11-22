"""Focused real-runtime checks, including real Ray and independent oracles."""
import argparse
from collections import Counter, defaultdict
import hashlib
import heapq
import json
import math
from pathlib import Path
import random
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
import numpy as np
from scipy.spatial import cKDTree
from uav_sim.app import _build_airspace_from_config
from uav_sim.airspace.partition_router import RegionalDijkstra
from uav_sim.simulation.metropolis_runtime import MetropolisSession


def graph_fingerprint(graph):
    digest = hashlib.sha256()
    for group in (graph.nodes, graph.edges):
        for key in sorted(group):
            digest.update(repr(group[key]).encode('utf-8'))
    return digest.hexdigest()


def check_shortest_costs(session):
    # Independent whole-world reverse Dijkstra, without the regional router.
    reverse = defaultdict(list)
    for edge in session.graph.edges.values():
        if edge.ramp_id is not None and not session.graph.ramp_states.get(edge.ramp_id, True):
            continue
        reverse[edge.target].append((edge.source, edge.length_m / edge.speed_limit_mps))
    checked, maximum = 0, 0.
    for target in session.graph.metadata['goal_specs']:
        distances, queue = {target: 0.}, [(0., target)]
        while queue:
            distance, node = heapq.heappop(queue)
            if distance != distances[node]:
                continue
            for before, cost in reverse.get(node, ()):
                value = distance + cost
                if value < distances.get(before, math.inf):
                    distances[before] = value
                    heapq.heappush(queue, (value, before))
        goal = session.maps.destination(target)
        for pipe in range(50):
            start = 'P{}_START'.format(pipe)
            region = session.maps.region(session.graph.nodes[start].region_id)
            plan = RegionalDijkstra(session.maps.summary(), region).plan(start, goal)
            assert plan.status == 'OK', (start, target, plan)
            assert all(edge in region.edges for edge in plan.edge_ids)
            error = abs(plan.cost - distances[start])
            assert math.isclose(plan.cost, distances[start], rel_tol=1e-10, abs_tol=1e-8)
            maximum = max(maximum, error)
            checked += 1
    return dict(checked_pairs=checked, max_cost_error_seconds=maximum)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    results, records, fingerprint = [], {}, None
    for mode in ('centralized_full', 'centralized_regional', 'onboard_regional'):
        for dt in (1., .1):
            name = mode + '_' + ('1' if dt == 1 else '0.1') + 's'
            directory = args.output / name
            cfg = dict(airspace_mode='metropolis', topology_profile='metropolis_partition_v1',
                num_pipes_per_layer=50, num_lanes_per_pipe=20, include_dynamic=True,
                inter_dynamic_level='low', intra_dynamic_level='low', routing_mode=mode,
                sim_dt_seconds=dt, num_uavs=120, mission_limit=120, max_batch_size=30,
                replenishment_period_seconds=100., max_sim_time_seconds=400.,
                drain_timeout_seconds=20000., destination_count=10, destination_policy='fixed',
                random_seed=42, num_pickup_points=0, use_ray=mode == 'onboard_regional',
                parallelism_level=30, run_output_dir=str(directory), stop_at_region_boundaries=True)
            random.seed(42)
            np.random.seed(42)
            started = time.perf_counter()
            env = _build_airspace_from_config(cfg)
            sampled = dict(samples=0, unsafe_pairs=0, body_contacts=0, min_distance_m=None)
            scopes = dict(checks=0, max_private_planners=0)
            next_sample, next_scope = [0.], [0.]
            with MetropolisSession(env, cfg) as session:
                current_fingerprint = graph_fingerprint(session.graph)
                if fingerprint is None:
                    fingerprint = current_fingerprint
                    oracle = check_shortest_costs(session)
                    (args.output/'whole_graph_oracle.json').write_text(json.dumps(oracle, indent=2), encoding='utf-8')
                assert fingerprint == current_fingerprint, 'Topologies differ across comparison cases'
                assert len(env.pipelines) == 300 and len(session.slots) == 1000
                assert len(session.graph.regions) == 36

                def inspect(progress):
                    now = progress['sim_time_seconds']
                    if now >= next_sample[0]:
                        states = [u.sample_partition_state(now) for u in session.active.values()]
                        if len(states) > 1:
                            points = np.array([state.position for state in states])
                            tree = cKDTree(points)
                            nearest = float(tree.query(points, k=2)[0][:, 1].min())
                            sampled['min_distance_m'] = nearest if sampled['min_distance_m'] is None else min(nearest, sampled['min_distance_m'])
                            for a, b in tree.query_pairs(30.001):
                                distance = float(np.linalg.norm(points[a]-points[b]))
                                required = max(session._threshold(states[a]), session._threshold(states[b]))
                                sampled['unsafe_pairs'] += int(distance < required-1e-6)
                                sampled['body_contacts'] += int(distance < 2.)
                        sampled['samples'] += 1
                        next_sample[0] += 10.
                    if mode == 'onboard_regional' and now >= next_scope[0]:
                        snapshots = session.backend.snapshots()
                        scopes['max_private_planners'] = max(scopes['max_private_planners'], len(snapshots))
                        for uid, state in snapshots.items():
                            uav = session.active.get(uid) or session.pending_uavs.get(uid)
                            assert uav is not None, 'Completed planner was not discarded'
                            assert tuple(state['detailed_region_ids']) == (uav._road_state.region_id,), (uid, state, uav._road_state.region_id)
                            scopes['checks'] += 1
                        next_scope[0] += 100.

                summary = session.run(inspect)
                rows = {row['uav_id']: dict(row) for row in session.completed}
                assert summary['success'] and summary['generated'] == summary['departed'] == summary['arrived'] == 120
                assert summary['unfinished'] == summary['safety_violations'] == summary['hard_collisions'] == 0
                assert summary['max_goal_position_error_m'] < 1e-5
                assert sampled['unsafe_pairs'] == sampled['body_contacts'] == 0
                counts = Counter(row['requested_time_seconds'] for row in rows.values())
                assert counts == {0.:30, 100.:30, 200.:30, 300.:30}, counts
                if mode == 'onboard_regional':
                    assert summary['routing']['use_ray']
                    assert len(summary['routing']['worker_pids']) == 4
                    assert scopes['checks'] > 120 and scopes['max_private_planners'] > 0
                summary.update(config=cfg, topology_sha256=fingerprint,
                    independent_sampled_safety=sampled, private_scope_audit=scopes,
                    requested_batches=dict(counts), wall_seconds_including_setup=time.perf_counter()-started)
                (directory/'verification_result.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
                records[name] = rows
                results.append(dict(case=name, arrived=120, peak_active=summary['peak_active'],
                    safety_violations=summary['safety_violations'], hard_collisions=summary['hard_collisions'],
                    wall_seconds=summary['wall_seconds_including_setup'], scope_checks=scopes['checks'],
                    worker_pids=summary['routing']['worker_pids'],
                    regional_plans=summary['journey_routing']['verified_regional_entry_plans']))
                print(json.dumps(results[-1], ensure_ascii=False), flush=True)
    baseline = records['centralized_full_1s']
    parity = []
    for name, rows in records.items():
        assert rows.keys() == baseline.keys()
        delta = max(abs(row['arrival_time_seconds']-baseline[uid]['arrival_time_seconds']) for uid,row in rows.items())
        spawn_delta = max(abs(row['spawn_time_seconds']-baseline[uid]['spawn_time_seconds']) for uid,row in rows.items())
        assert delta < 1e-8 and spawn_delta < 1e-8, (name, delta, spawn_delta)
        assert all(row['visited_regions'] == baseline[uid]['visited_regions'] for uid,row in rows.items())
        parity.append(dict(case=name, max_arrival_delta_seconds=delta, max_spawn_delta_seconds=spawn_delta))
    (args.output/'matrix_summary.json').write_text(json.dumps(dict(cases=results, parity=parity,
        topology_sha256=fingerprint, oracle=oracle), ensure_ascii=False, indent=2), encoding='utf-8')
    print('PASS all six small-runtime cases and per-UAV physical-time comparisons', flush=True)


if __name__ == '__main__':
    main()
