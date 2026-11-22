"""
main.py — UAV 空域仿真 算法对比实验平台（主入口）
V6.0: 支持串行模式 (V5.0 兼容) 和分布式 Actor 模式 (Ray)
"""
import sys
import os
import math
import random
import time
import uuid
import json
from datetime import datetime
import numpy as np
from uav_sim.airspace import Airspace, DatabaseTopologyAdapter, PickupPoint
from uav_sim.integrations.postgres_airspace import PostgresAirspaceRepository
from uav_sim.integrations.postgres_ramp_state import PostgresRampStateProvider
from uav_sim.entities.uav import UAV, get_uav_3d_pos
from uav_sim.algorithms.registry import ALGORITHM_REGISTRY, ALGO_DISTRIBUTED_META


def generate_capacity_sequence(num_uavs, capacity_weights):
    total_weight = sum(capacity_weights.values())
    if total_weight == 0:
        return [0] * num_uavs
    capacities = []
    for cap_level in sorted(capacity_weights.keys()):
        w = capacity_weights[cap_level]
        count = round(num_uavs * w / total_weight)
        capacities.extend([cap_level] * count)
    while len(capacities) < num_uavs:
        capacities.append(max(capacity_weights, key=capacity_weights.get))
    capacities = capacities[:num_uavs]
    random.shuffle(capacities)
    return capacities


def parse_trajectory_ids(raw_input, num_uavs):
    raw = raw_input.strip()
    if not raw:
        return [
            int(num_uavs * r)
            for r in [0.3, 0.35, 0.4, 0.45]
            if int(num_uavs * r) < num_uavs
        ] or [0]
    if raw.lower().startswith("random"):
        parts = raw.split()
        try:
            n = int(parts[1]) if len(parts) > 1 else 4
        except ValueError:
            n = 4
        n = min(n, num_uavs)
        return random.sample(range(num_uavs), n)
    ids = []
    for token in raw.replace(",", " ").split():
        try:
            uid = int(token)
            if 0 <= uid < num_uavs:
                ids.append(uid)
            else:
                print(f"  警告: UAV ID {uid} 超出范围 [0, {num_uavs - 1}]，已跳过")
        except ValueError:
            pass
    return ids if ids else [0]


def parse_optional_uav_ids(raw_input, num_uavs):
    raw = str(raw_input or "").strip()
    if not raw:
        return []
    ids = []
    seen = set()
    for token in raw.replace(",", " ").split():
        try:
            uid = int(token)
        except ValueError:
            continue
        if 0 <= uid < num_uavs and uid not in seen:
            ids.append(uid)
            seen.add(uid)
        elif uid < 0 or uid >= num_uavs:
            print(f"  警告: UAV ID {uid} 超出范围 [0, {num_uavs - 1}]，已跳过")
    return ids


def generate_uavs(env, config):
    num_uavs = config["num_uavs"]
    capacity_weights = config["capacity_weights"]
    dest_pipe_ids = config["dest_pipe_ids"]
    if not dest_pipe_ids:
        raise ValueError("旧空域 / 数据库流程请至少勾选一个终点管道。")
    available = {pipe.pipe_id for pipe in env.pipelines}
    invalid = [pid for pid in dest_pipe_ids if pid not in available]
    if invalid:
        raise ValueError("选定终点管道不属于当前空域：{}。请在当前空域或数据库地图中重新选择。".format(invalid))
    batch_interval = config.get("batch_interval", 50)
    capacities = generate_capacity_sequence(num_uavs, capacity_weights)
    bottom_pipes = [p for p in env.pipelines if p.height == 100]
    max_lanes = max((pipe.num_lanes for pipe in bottom_pipes))
    spawn_slots = [
        (pipe.pipe_id, lane_id)
        for lane_id in range(max_lanes)
        for pipe in bottom_pipes
        if lane_id < pipe.num_lanes
    ]
    if not spawn_slots:
        raise RuntimeError("没有可用的底层起飞槽位。")
    batch_size = min(num_uavs, len(spawn_slots))
    num_batches = math.ceil(num_uavs / batch_size)
    uavs = []
    uav_counter = 0
    for batch_idx in range(num_batches):
        spawn_t = batch_idx * batch_interval
        batch_count = min(batch_size, num_uavs - uav_counter)
        for i in range(batch_count):
            pipe_id, lane_id = spawn_slots[i % len(spawn_slots)]
            dest_pipe_id = dest_pipe_ids[uav_counter % len(dest_pipe_ids)]
            cap = capacities[uav_counter]
            uav = UAV(
                uav_id=uav_counter,
                start_pipe_id=pipe_id,
                lane_id=lane_id,
                dest_pipe_id=dest_pipe_id,
                env=env,
                capacity=cap,
            )
            uav.final_dest_pipe_id = dest_pipe_id
            uav.spawn_turn = spawn_t
            uavs.append(uav)
            uav_counter += 1
    return uavs


def create_pickup_points_and_assign(config, uavs, env):
    from sklearn.cluster import KMeans
    from scipy.optimize import linear_sum_assignment

    if config["num_pickup_points"] == 0:
        return []
    uavs_with_cap = [u for u in uavs if u.capacity > 0]
    if not uavs_with_cap:
        return []
    N = config["num_pickup_points"]
    locations = config["pickup_locations"]
    positions = np.array([get_uav_3d_pos(u, env) for u in uavs_with_cap])
    actual_N = min(N, len(uavs_with_cap))
    kmeans = KMeans(n_clusters=actual_N, random_state=42, n_init="auto").fit(positions)
    labels = kmeans.labels_
    pt_3d_coords = []
    for loc in locations[:actual_N]:
        pipe = env.pipelines[loc["pipe_id"]]
        pt_3d_coords.append(pipe.start_pt + pipe.direction * loc["pos_1d"])
    pt_3d_coords = np.array(pt_3d_coords)
    cluster_centers = kmeans.cluster_centers_
    cost_matrix = np.zeros((actual_N, actual_N))
    for i in range(actual_N):
        for j in range(actual_N):
            cost_matrix[i][j] = np.linalg.norm(cluster_centers[i] - pt_3d_coords[j])
    row_ind, col_ind = linear_sum_assignment(cost_matrix)
    cluster_to_pt = {row_ind[k]: col_ind[k] for k in range(actual_N)}
    cluster_caps = {i: 0 for i in range(actual_N)}
    for idx, u in enumerate(uavs_with_cap):
        cluster_caps[labels[idx]] += u.capacity
    pickup_points = []
    for cluster_id, pt_idx in cluster_to_pt.items():
        loc = locations[pt_idx]
        cap = cluster_caps[cluster_id]
        pickup_points.append(
            PickupPoint(
                pt_idx,
                loc["pipe_id"],
                loc["pos_1d"],
                capacity=cap,
                region_id=getattr(env.pipelines[loc["pipe_id"]], "region_id", None),
            )
        )
    pt_map = {pt.pt_id: pt for pt in pickup_points}
    for idx, u in enumerate(uavs_with_cap):
        pt_idx = cluster_to_pt[labels[idx]]
        pt = pt_map[pt_idx]
        u.pickup_pt = pt
        u.phase = "TO_PICKUP"
        u.dest_pipe_id = pt.pipe_id
        u.dest_region_id = getattr(env.pipelines[pt.pipe_id], "region_id", 0)
    return pickup_points


def _build_uav_configs(uavs, pickup_points):
    """将 UAV 对象列表转换为可序列化的 config 字典列表"""
    pt_map = {pt.pt_id: pt for pt in pickup_points} if pickup_points else {}
    configs = []
    for u in uavs:
        cfg = {
            "uav_id": u.uav_id,
            "start_pipe_id": u.start_pipe_id,
            "lane_id": u.start_lane_id,
            "dest_pipe_id": u.dest_pipe_id,
            "start_region_id": getattr(u, "start_region_id", 0),
            "current_region_id": getattr(u, "current_region_id", 0),
            "dest_region_id": getattr(u, "dest_region_id", 0),
            "capacity": u.capacity,
            "spawn_turn": u.spawn_turn,
            "final_dest_pipe_id": u.final_dest_pipe_id,
            "max_accel": getattr(u, "max_accel", 10.0),
            "max_decel": getattr(u, "max_decel", 10.0),
            "pickup_info": None,
        }
        if u.pickup_pt is not None:
            cfg["pickup_info"] = {
                "pt_id": u.pickup_pt.pt_id,
                "pipe_id": u.pickup_pt.pipe_id,
                "pos_1d": u.pickup_pt.pos_1d,
                "capacity": u.pickup_pt.capacity,
                "region_id": getattr(u.pickup_pt, "region_id", None),
            }
        configs.append(cfg)
    return configs


def _create_uav_group_actors(
    uav_configs,
    parallelism_level,
    env_ref,
    routing_mode,
    control_mode="legacy_local",
    route_server=None,
    actor_num_cpus=1.0,
):
    """按负载均衡策略创建 UAVGroupActor 分组"""
    import ray
    from uav_sim.distributed.uav_actor import UAVGroupActor

    n = len(uav_configs)
    n_groups = math.ceil(n / parallelism_level)
    groups_configs = [[] for _ in range(n_groups)]
    for i, cfg in enumerate(uav_configs):
        gid = i % n_groups
        groups_configs[gid].append(cfg)
    actors = []
    for gid, group_cfgs in enumerate(groups_configs):
        actor = UAVGroupActor.options(num_cpus=actor_num_cpus).remote(
            group_id=gid,
            uav_configs=group_cfgs,
            env_snapshot_ref=env_ref,
            routing_mode=routing_mode,
            control_mode=control_mode,
            route_server=route_server,
        )
        actors.append(actor)
    print(
        f"[V6.0] 已创建 {len(actors)} 个 UAVGroupActor (并行级别: {parallelism_level} 架/进程, 控制模式: {control_mode}, CPU预约: {actor_num_cpus:.3f}/Actor)"
    )
    return actors


def _estimate_uav_actor_num_cpus(
    num_uavs,
    parallelism_level,
    total_cpus=None,
    service_actor_cpus=3.0,
    driver_reserve_cpus=1.0,
    min_actor_cpu=0.001,
):
    total_cpus = total_cpus or (os.cpu_count() or 4)
    num_groups = max(1, math.ceil(num_uavs / max(1, parallelism_level)))
    available_for_uav_actors = max(
        min_actor_cpu,
        float(total_cpus) - float(service_actor_cpus) - float(driver_reserve_cpus),
    )
    return min(1.0, max(min_actor_cpu, available_for_uav_actors / num_groups))


def _is_centralized_control(control_mode):
    return control_mode in ("centralized_route", "centralized_full")


def _is_server_authoritative_control(control_mode):
    return control_mode == "server_authoritative"


def _build_airspace_from_config(config):
    source = str(config.get("airspace_source", "generated")).strip().lower()

    def build_generated():
        enhanced={key:config[key] for key in ('num_pipes_per_layer','num_lanes_per_pipe',
            'ramp_same_layer_degree','ramp_adjacent_layer_degree','ramp_express_layer_degree') if key in config}
        if config.get('airspace_mode')=='metropolis':
            enhanced['include_dynamic']=bool(config.get('include_dynamic', True))
        return Airspace(
            mode=config.get("airspace_mode", "standard"),
            topology_profile=config.get("topology_profile"),
            inter_dynamic_level=config.get("inter_dynamic_level", "low"),
            intra_dynamic_level=config.get("intra_dynamic_level", "low"),
            inter_dynamic_pct=config.get("inter_dynamic_pct"),
            intra_dynamic_pct=config.get("intra_dynamic_pct"),
            **enhanced,
        )

    if source == "generated":
        return build_generated()
    if source != "postgres":
        raise ValueError("未知空域数据源: {}".format(source))
    database_config = dict(config.get("database") or {})
    try:
        repository = PostgresAirspaceRepository.from_config(database_config)
        rows = repository.fetch_topology_rows()
        topology = DatabaseTopologyAdapter(
            strict=database_config.get("strict_validation", True),
            derive_lane_width_from_pipe_width=database_config.get(
                "derive_lane_width_from_pipe_width", True
            ),
        ).adapt_from_repository_result(rows)
        return Airspace.from_adapted_topology(topology)
    except Exception:
        if not database_config.get("fallback_to_generated", False):
            raise
        print("[WARNING] PostgreSQL 空域加载失败；配置显式允许回退，本次将使用程序生成拓扑。", flush=True)
        return build_generated()


def _build_ramp_state_provider(env, config):
    if getattr(env, "topology_source", "generated") != "postgres":
        return None
    repository = PostgresAirspaceRepository.from_config(
        dict(config.get("database") or {})
    )
    return PostgresRampStateProvider(
        repository,
        external_to_internal=env.external_id_maps.ramp_external_to_internal
        if env.external_id_maps is not None
        else {},
    )


def _snapshot_for_uav_group_actors(env, control_mode):
    if _is_centralized_control(control_mode) or _is_server_authoritative_control(
        control_mode
    ):
        from uav_sim.distributed.actor_airspace_snapshot import (
            build_actor_airspace_snapshot,
        )

        return build_actor_airspace_snapshot(env)
    snapshot = dict(env.get_readonly_snapshot())
    snapshot["route_graph_available"] = True
    return snapshot


def _server_avoidance_policy_for_algorithm(algo_id, required_models, config):
    if str(algo_id) != "1":
        raise ValueError("Only algorithm 1 is available")
    return "rule"


def _server_avoidance_policy_request(config):
    return "rule"


def _flatten_route_request_lists(request_lists):
    requests = []
    for group_requests in request_lists:
        requests.extend(group_requests or [])
    return requests


def _group_route_updates_by_actor(route_updates, n_groups):
    grouped = {gid: [] for gid in range(n_groups)}
    for update in route_updates or []:
        uid = update.get("uav_id") if isinstance(update, dict) else update.uav_id
        grouped[uid % n_groups].append(update)
    return grouped


def _route_context_from_registry(ray_module, registry, config, turn=0):
    return ray_module.get(registry.get_shared_context.remote(current_turn=turn))


def _apply_route_updates_to_actors(ray, actors, route_updates):
    if not route_updates:
        return
    grouped = _group_route_updates_by_actor(route_updates, len(actors))
    futures = [
        actors[gid].apply_route_updates.remote(updates)
        for gid, updates in grouped.items()
        if updates
    ]
    if futures:
        ray.get(futures)


def _run_distributed(
    env, uavs, pickup_points, config, selected_algos, TARGET_UAVS, results_dict
):
    """V6.0 分布式 Actor 模式运行"""
    import ray
    from uav_sim.distributed.engine import DistributedEngine
    from uav_sim.airspace.registry import AirspaceRegistry
    from uav_sim.distributed.orchestrator import SimulationOrchestrator
    from uav_sim.distributed.route_planning_server import RoutePlanningServer
    from uav_sim.distributed.authoritative_control_server import (
        AuthoritativeControlServer,
    )
    from uav_sim.distributed.engine import DEFAULT_UAVS_PER_ACTOR

    parallelism_level = config.get("parallelism_level", DEFAULT_UAVS_PER_ACTOR)
    control_mode = config.get("distributed_control_mode", "legacy_local")
    centralized_control = _is_centralized_control(control_mode)
    server_authoritative = _is_server_authoritative_control(control_mode)
    actor_num_cpus = config.get("uav_actor_num_cpus")
    ramp_state_provider = _build_ramp_state_provider(env, config)
    if actor_num_cpus is None:
        service_actor_cpus = 3.0 if centralized_control or server_authoritative else 2.0
        actor_num_cpus = _estimate_uav_actor_num_cpus(
            num_uavs=len(uavs),
            parallelism_level=parallelism_level,
            service_actor_cpus=service_actor_cpus,
        )
    engine = DistributedEngine(parallelism_level=parallelism_level, use_ray=True)
    engine.start()
    try:
        registry = AirspaceRegistry.remote(len(env.pipelines))
        route_server = None
        if centralized_control:
            route_server = RoutePlanningServer.remote(
                env.get_readonly_snapshot(), routing_mode="DIJKSTRA"
            )
            print(
                f"[V6.0] RoutePlanningServer 已启用: {control_mode} (宏观路线集中计算，规则避障在 UAVGroupActor 执行)"
            )
        authoritative_control_server = None
        if server_authoritative:
            authoritative_control_server = AuthoritativeControlServer.remote(
                env.get_readonly_snapshot(),
                routing_mode="DIJKSTRA",
                avoidance_policy="rule",
                safety_substep=config.get("safety_substep", 0.2),
            )
            print(
                f"[V6.0] AuthoritativeControlServer 已启用: {control_mode} (寻路+避障集中, 策略按算法自动选择)"
            )
        uav_configs = _build_uav_configs(uavs, pickup_points)
        if getattr(env, "topology_source", "generated") == "postgres":
            static_only_ramp_states = {r.ramp_id: bool(r.is_active) for r in env.ramps}
        else:
            static_only_ramp_states = {
                r.ramp_id: not getattr(r, "is_dynamic", False) for r in env.ramps
            }
        ramp_regions = {
            r.ramp_id: tuple(
                sorted(
                    {
                        int(getattr(r, "region_a_id", 0)),
                        int(getattr(r, "region_b_id", 0)),
                    }
                )
            )
            for r in env.ramps
        }
        ray.get(
            registry.init_ramp_states.remote(
                static_only_ramp_states, ramp_regions=ramp_regions
            )
        )
        for algo_id in selected_algos:
            name_cn, name_en, _, required = ALGORITHM_REGISTRY[algo_id]
            meta = ALGO_DISTRIBUTED_META[algo_id]
            print(f"\n{'=' * 60}")
            print(f"  正在运行算法 {algo_id}: {name_cn} [分布式模式]")
            print(f"{'=' * 60}")
            ray.get(registry.full_reset.remote())
            dynamic_enabled = False
            if getattr(env, "topology_source", "generated") == "postgres":
                ramp_initial_states = {r.ramp_id: bool(r.is_active) for r in env.ramps}
            else:
                ramp_initial_states = {
                    r.ramp_id: not getattr(r, "is_dynamic", False) for r in env.ramps
                }
            for r in env.ramps:
                r.is_active = ramp_initial_states[r.ramp_id]
            print(
                f"[Airspace] mode={getattr(env, 'airspace_mode', 'standard')}, topology={getattr(getattr(env, 'topology_profile', None), 'name', 'unknown')}, pipelines={len(env.pipelines)}, ramps={len(env.ramps)}, graph_nodes={len(env.node_info)}"
            )
            env_ref = ray.put(_snapshot_for_uav_group_actors(env, control_mode))
            ray.get(
                registry.init_ramp_states.remote(
                    ramp_initial_states, ramp_regions=ramp_regions
                )
            )
            server_policy_for_algo = _server_avoidance_policy_for_algorithm(
                algo_id, required, config
            )
            if server_authoritative:
                ray.get(
                    authoritative_control_server.set_avoidance_policy.remote(
                        server_policy_for_algo
                    )
                )
                requested_policy = _server_avoidance_policy_request(config)
                print(
                    f"[V6.0] 服务器权威避障策略: {server_policy_for_algo} (配置={requested_policy}, 算法需求={required})"
                )
            if centralized_control:
                ray.get(route_server.set_routing_mode.remote(meta["mode"]))
            elif server_authoritative:
                ray.get(
                    authoritative_control_server.set_routing_mode.remote(meta["mode"])
                )
            actors = _create_uav_group_actors(
                uav_configs,
                parallelism_level,
                env_ref,
                meta["mode"],
                control_mode=control_mode,
                route_server=route_server,
                actor_num_cpus=actor_num_cpus,
            )
            ray.get([a.reset_all.remote() for a in actors])
            ray.get([a.set_routing_mode.remote(meta["mode"]) for a in actors])
            initial_route_planning_seconds = 0.0
            if server_authoritative:
                observation_parts = ray.get(
                    [
                        actor.report_control_observations.remote(turn=0)
                        for actor in actors
                    ]
                )
                nonempty_parts = [part for part in observation_parts if len(part) > 0]
                if nonempty_parts:
                    observations = np.concatenate(nonempty_parts)
                    initial_context = ray.get(
                        registry.get_shared_context.remote(current_turn=0)
                    )
                    initial_route_started_at = time.perf_counter()
                    initial_route_updates = ray.get(
                        authoritative_control_server.prepare_initial_routes.remote(
                            0, observations, initial_context
                        )
                    )
                    initial_route_planning_seconds = (
                        time.perf_counter() - initial_route_started_at
                    )
                    _apply_route_updates_to_actors(ray, actors, initial_route_updates)
                    print(
                        f"[Timing] 服务器权威初始路线规划: {initial_route_planning_seconds:.3f}s | 更新 {len(initial_route_updates)} 架",
                        flush=True,
                    )
                    del observations, initial_context, initial_route_updates
                del observation_parts, nonempty_parts
            if centralized_control:
                request_lists = ray.get(
                    [
                        a.build_route_requests.remote(turn=0, force_all=True)
                        for a in actors
                    ]
                )
                route_requests = _flatten_route_request_lists(request_lists)
                route_context = _route_context_from_registry(
                    ray, registry, config, turn=0
                )
                route_updates = ray.get(
                    route_server.plan_routes.remote(route_requests, route_context)
                )
                _apply_route_updates_to_actors(ray, actors, route_updates)
                del request_lists, route_requests, route_updates, route_context
            else:
                ray.get([a.plan_all.remote() for a in actors])
            initial_states = {}
            states_list = ray.get([a.get_uav_states.remote() for a in actors])
            for s in states_list:
                initial_states.update(s)
            ray.get(registry.batch_register_uavs.remote(initial_states))
            orchestrator = SimulationOrchestrator(
                env=env,
                uav_groups=actors,
                registry=registry,
                route_server=route_server,
                total_uav_count=len(uavs),
                mode=meta["mode"],
                control_mode=control_mode,
                authoritative_control_server=authoritative_control_server,
                server_avoidance_policy=server_policy_for_algo,
                safety_substep=config.get("safety_substep", 0.2),
                max_turns=config.get("max_turns", 3500),
                track_uav_ids=TARGET_UAVS,
                pickup_points=pickup_points,
                trace_uav_ids=config.get("trace_uav_ids", []),
                trace_log_path=config.get("trace_log_path"),
                trace_algo_id=algo_id,
                enable_kl_divergence=config.get("enable_kl_divergence", False),
                enable_channel_gain=config.get("enable_channel_gain", False),
                run_output_dir=os.path.join(
                    config.get("run_output_root", "artifacts/runs"),
                    config.get("run_id", "run"),
                    f"algorithm_{algo_id}",
                ),
                frame_export_enabled=config.get("frame_export_enabled", False),
                run_id=config.get("run_id"),
                algorithm_id=algo_id,
                initial_route_planning_seconds=initial_route_planning_seconds,
                ramp_state_provider=ramp_state_provider,
                ramp_poll_interval_turns=config.get("database", {}).get(
                    "ramp_poll_interval_turns", 1
                ),
                ramp_poll_region_id=config.get("database", {}).get(
                    "ramp_poll_region_id"
                ),
            )
            orch_result = orchestrator.run()
            if len(orch_result) == 8:
                t, v, k, m, a, g, trajs, metrics = orch_result
            else:
                t, v, k, m, a, g, trajs = orch_result
                metrics = {}
            results_dict[algo_id] = {
                "name_cn": name_cn,
                "name_en": name_en,
                "time": t,
                "violations": v,
                "kl_history": k,
                "missed": m,
                "avg_kl": a,
                "gain_history": g,
                "trajectories": trajs,
                "metrics": metrics,
            }
            for actor in actors:
                ray.kill(actor, no_restart=True)
            del actors
            del env_ref
    finally:
        engine.shutdown()


def _run_sequential(
    env, uavs, pickup_points, config, selected_algos, TARGET_UAVS, results_dict
):
    """V5.0 串行模式运行（完整保留原始逻辑）"""
    for algo_id in selected_algos:
        name_cn, name_en, func, required = ALGORITHM_REGISTRY[algo_id]
        print(f"\n{'=' * 60}")
        print(f"  正在运行算法 {algo_id}: {name_cn} [串行模式]")
        print(f"{'=' * 60}")
        result = func(
            env,
            uavs,
            pickup_points,
            TARGET_UAVS,
            max_turns=config.get("max_turns", env.scale_config.default_max_turns),
            trace_uav_ids=config.get("trace_uav_ids", []),
            trace_log_path=config.get("trace_log_path"),
            trace_algo_id=algo_id,
            enable_kl_divergence=config.get("enable_kl_divergence", False),
            enable_channel_gain=config.get("enable_channel_gain", False),
            run_output_dir=os.path.join(
                config.get("run_output_root", "artifacts/runs"),
                config.get("run_id", "run"),
                f"algorithm_{algo_id}",
            ),
            frame_export_enabled=config.get("frame_export_enabled", False),
            run_id=config.get("run_id"),
        )
        t, v, k, m, a, g, trajs, metrics = result
        results_dict[algo_id] = {
            "name_cn": name_cn,
            "name_en": name_en,
            "time": t,
            "violations": v,
            "kl_history": k,
            "missed": m,
            "avg_kl": a,
            "gain_history": g,
            "trajectories": trajs,
            "metrics": metrics,
        }


def _is_enhanced_metropolis(env, config):
    return (config.get('airspace_source', 'generated') == 'generated'
            and config.get('airspace_mode', 'metropolis') == 'metropolis'
            and getattr(env, 'topology_source', 'generated') == 'generated'
            and getattr(env, 'airspace_mode', 'metropolis') == 'metropolis')


def run_config(config,env=None):
    """Headless access to the same first-version runner/coordinator as the GUI."""
    from uav_sim.simulation.runner import run_simulation
    cfg=dict(config)
    cfg.setdefault('airspace_mode','metropolis')
    if cfg.get('airspace_source', 'generated') != 'generated' or cfg['airspace_mode'] != 'metropolis':
        raise ValueError('run_config supports generated metropolis only; use the original GUI flow for legacy/database airspaces')
    cfg.setdefault('destination_policy','fixed');cfg.setdefault('destination_count',10)
    cfg.setdefault('random_seed',42);cfg.setdefault('sim_dt_seconds',1.)
    cfg.setdefault('max_sim_time_seconds',float(cfg.get('max_turns',18000))*cfg['sim_dt_seconds'])
    cfg.setdefault('run_id',datetime.now().strftime('%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:8])
    cfg.setdefault('run_output_dir',os.path.join(cfg.get('run_output_root','artifacts/runs'),cfg['run_id'],'algorithm_1'))
    cfg.setdefault('track_uav_ids',parse_trajectory_ids(cfg.get('trajectory_ids_raw',''),cfg.get('num_uavs',10000)))
    cfg.setdefault('trace_uav_ids',parse_optional_uav_ids(cfg.get('trace_ids_raw',''),cfg.get('num_uavs',10000)))
    if cfg['trace_uav_ids']:
        cfg['trace_log_path']=cfg.get('trace_log_path') or os.path.join(cfg['run_output_dir'],'uav_turn_trace.txt')
    env=env or _build_airspace_from_config(cfg)
    if not _is_enhanced_metropolis(env, cfg):
        raise ValueError('run_config requires an actual generated metropolis topology')
    result=run_simulation(env,[],[],mode='DIJKSTRA',metropolis_config=cfg)
    summary=result[7]['metropolis_summary'];summary['output_dir']=os.path.abspath(cfg['run_output_dir'])
    print(json.dumps(summary,ensure_ascii=False,indent=2))
    if cfg.get('show_plots',False):
        from uav_sim.simulation.plotting import plot_3d_trajectories
        results={'1':dict(name_cn='双层Dijkstra',name_en=cfg.get('routing_mode','centralized_full'),
            time=result[0],violations=result[1],kl_history=result[2],missed=result[3],avg_kl=result[4],gain_history=result[5],trajectories=result[6],metrics=result[7])}
        tracked=result[7].get('_tracked_uav_objects',{})
        visible=[uid for uid in cfg['track_uav_ids'] if uid in tracked and result[6].get(uid)]
        plot_3d_trajectories(results,env,tracked,visible,plot_multi_uav_same_figure=cfg.get('plot_multi_uav_same_figure',False))
        if cfg.get('enable_kl_divergence'):
            from uav_sim.simulation.plotting import plot_kl_divergence
            plot_kl_divergence(results)
        if cfg.get('enable_channel_gain'):
            from uav_sim.simulation.plotting import plot_beamforming_gain
            plot_beamforming_gain(results)
    return summary


def main():
    print("正在构建6层空域环境...")
    env = Airspace(mode="metropolis", include_dynamic=True)
    from PyQt5.QtWidgets import QApplication

    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    from uav_sim.ui.gui_config import ConfigGUI

    gui = ConfigGUI(env)
    config = gui.run()
    if config is None:
        print("用户取消，退出程序。")
        return
    selected_algos = config["selected_algorithms"]
    use_ray = config.get("use_ray", False)
    config["run_id"] = (
        config.get("run_id")
        or datetime.now().strftime("%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8]
    )
    print(
        f"\n配置确认: 无人机={config['num_uavs']}架 | 空域={config.get('airspace_mode', 'standard')} | 拓扑={config.get('topology_profile', 'balanced_sparse_v1')} | 最大回合={config.get('max_turns', 3500)} | 中间节点={config['num_pickup_points']}个 | 终点={len(config['dest_pipe_ids'])}个 | 算法={selected_algos} | 模式={('分布式(Ray)' if use_ray else '串行(V5.0)')} | 跨层动态={config.get('inter_dynamic_level', 'low')} | 同层动态={config.get('intra_dynamic_level', 'low')}"
    )
    if use_ray:
        from uav_sim.distributed.engine import DEFAULT_UAVS_PER_ACTOR

        configured_parallelism = config.get("parallelism_level", DEFAULT_UAVS_PER_ACTOR)
        print(
            f"并行级别: {configured_parallelism} 架/进程 | 预计进程数: {math.ceil(config['num_uavs'] / configured_parallelism)} | 分布式控制: {config.get('distributed_control_mode', 'legacy_local')}"
        )
    print("正在根据配置重建空域环境（含动态匝道参数）...")
    preview_env = getattr(gui, 'env', None)
    if (config.get('airspace_source') == 'postgres'
            and getattr(preview_env, 'topology_source', None) == 'postgres'):
        env = preview_env
    else:
        env = _build_airspace_from_config(config)
    if _is_enhanced_metropolis(env, config):
        config.setdefault('show_plots',True)
        run_config(config,env=env)
        return
    print("正在生成无人机...")
    uavs = generate_uavs(env, config)
    print(f"成功生成 {len(uavs)} 架无人机")
    pickup_points = create_pickup_points_and_assign(config, uavs, env)
    if pickup_points:
        total_pt_cap = sum((pt.capacity for pt in pickup_points))
        print(f"创建了 {len(pickup_points)} 个中间节点，总运力需求 = {total_pt_cap}")
        for pt in pickup_points:
            print(
                f"  节点{pt.pt_id}: 管道{pt.pipe_id}, 位置{pt.pos_1d:.0f}, 运力={pt.capacity}"
            )
    else:
        print("无中间节点（直飞模式）")
    n = config["num_uavs"]
    TARGET_UAVS = parse_trajectory_ids(config.get("trajectory_ids_raw", ""), n)
    print(f"航迹追踪目标: {TARGET_UAVS}")
    trace_uav_ids = parse_optional_uav_ids(config.get("trace_ids_raw", ""), n)
    config["trace_uav_ids"] = trace_uav_ids
    if trace_uav_ids:
        trace_path = config.get("trace_log_path") or "uav_turn_trace.txt"
        config["trace_log_path"] = trace_path
        print(f"逐回合日志目标: {trace_uav_ids} | 输出文件: {trace_path}")
    results_dict = {}
    if use_ray:
        _run_distributed(
            env, uavs, pickup_points, config, selected_algos, TARGET_UAVS, results_dict
        )
    else:
        _run_sequential(
            env, uavs, pickup_points, config, selected_algos, TARGET_UAVS, results_dict
        )
    from uav_sim.simulation.plotting import (
        print_comparison_report,
        plot_kl_divergence,
        plot_beamforming_gain,
        plot_3d_trajectories,
    )

    print_comparison_report(results_dict)
    if config.get("enable_kl_divergence", False):
        plot_kl_divergence(results_dict)
    else:
        print("KL 散度计算已关闭，跳过 KL 曲线。")
    if config.get("enable_channel_gain", False):
        plot_beamforming_gain(results_dict)
    else:
        print("信道/波束增益计算已关闭，跳过增益曲线。")
    plot_3d_trajectories(
        results_dict,
        env,
        uavs,
        TARGET_UAVS,
        plot_multi_uav_same_figure=config.get("plot_multi_uav_same_figure", False),
    )


if __name__ == "__main__":
    main()
