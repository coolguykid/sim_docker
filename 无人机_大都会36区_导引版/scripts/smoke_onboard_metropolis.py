"""Small real-Ray, real-metropolis check before long acceptance runs."""
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from uav_sim.airspace.environment import Airspace
from uav_sim.simulation.metropolis_runtime import MetropolisSession


def main():
    results=[]
    for dt in (1.,.1):
        env=Airspace(mode='metropolis',num_pipes_per_layer=2,include_dynamic=False)
        cfg=dict(sim_dt_seconds=dt,routing_mode='onboard_regional',use_ray=True,parallelism_level=1,
            num_uavs=2,max_batch_size=2,mission_limit=2,max_sim_time_seconds=1,
            drain_timeout_seconds=10000,destination_count=10,random_seed=42,destination_policy='fixed')
        with MetropolisSession(env,cfg) as session:
            next_check=[0.]
            checked=[0]
            def inspect(progress):
                if progress['sim_time_seconds'] < next_check[0]:
                    return
                for uid,state in session.backend.snapshots().items():
                    u=session.active.get(uid) or session.pending_uavs.get(uid)
                    assert u is not None
                    assert tuple(state['detailed_region_ids']) == (u._road_state.region_id,)
                    checked[0]+=1
                next_check[0]+=100.
            result=session.run(inspect)
            assert result['success'] and result['arrived']==2
            assert result['safety_violations']==result['hard_collisions']==0
            assert result['max_goal_position_error_m'] < 1e-5
            assert result['routing']['use_ray'] and len(result['routing']['worker_pids'])==2
            results.append(dict(dt=dt,arrived=result['arrived'],last_arrival=result['last_arrival_time_seconds'],
                pids=result['routing']['worker_pids'],private_scope_checks=checked[0],
                actual_region_plans=result['journey_routing']['verified_regional_entry_plans'],
                goal_error_m=result['max_goal_position_error_m']))
    assert abs(results[0]['last_arrival']-results[1]['last_arrival']) < 1e-8
    path=ROOT/'docs'/'verification_metropolis36'/'real_ray_metropolis_smoke.json'
    path.write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(results))


if __name__=='__main__':
    main()
