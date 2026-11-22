import pytest

from tests.enhanced.test_certified_runtime import make_session


@pytest.mark.parametrize('mode', ['centralized_full', 'centralized_regional'])
@pytest.mark.parametrize('dt', [1., .1])
def test_finite_server_safety_forecast_uses_actual_regional_planner_and_arrives(mode, dt):
    with make_session(dt, mode, execution_engine='certified_journey') as session:
        result = session.run()
        assert result['execution_engine'] == 'certified_journey'
        assert result['generated'] == result['arrived'] == 2 and result['success']
        assert result['certified_prefixes'] == 2
        assert result['max_goal_position_error_m'] < 1e-6
        if mode == 'centralized_regional':
            assert result['journey_routing']['verified_regional_entry_plans'] >= 2
            assert result['routing']['requests'] >= 4
        assert result['reservations']['active_holds'] == 0


def test_journey_reserves_a_finite_cohort_without_infinite_portal_holds():
    with make_session(1., execution_engine='certified_journey', num_uavs=30,
                      max_batch_size=30, mission_limit=30, drain_timeout_seconds=300) as session:
        result = session.run()
        assert result['execution_engine'] == 'certified_journey'
        assert result['success'] and result['arrived'] == 30
        assert result['partial_queue_prefixes'] == 0
        assert result['ground_serialization_fallbacks'] >= 0


def test_new_default_is_finite_journey_and_live_stepping_can_continue_with_run():
    with make_session(.1) as session:
        session.config.pop('execution_engine', None)
        assert session.advance_one_step()['sim_time_seconds'] == .1
        assert session.current_frame()['uavs']
        result = session.run()
        assert result['execution_engine'] == 'certified_journey'
        assert result['success']


def test_optional_pickup_and_final_goal_share_one_finite_physical_certificate():
    from uav_sim.airspace.environment import Airspace
    from uav_sim.simulation.metropolis_runtime import MetropolisSession
    env = Airspace(mode='metropolis', num_pipes_per_layer=2, include_dynamic=False)
    config = dict(routing_mode='centralized_regional',num_uavs=1,max_batch_size=1,
        mission_limit=1,max_sim_time_seconds=1,drain_timeout_seconds=10000,
        destination_mode='manual',destination_count=1,
        manual_destinations=[dict(pipe_id=10,lane_id=0,distance_m=env.pipelines[10].length*.7)],
        capacity_weights={2:100},num_pickup_points=1,
        pickup_locations=[dict(pipe_id=2,pos_1d=env.pipelines[2].length*.5)])
    with MetropolisSession(env,config) as session:
        result = session.run()
        assert result['success'] and result['arrived'] == 1
        assert result['certified_prefixes'] == 1
        assert session.features.pickup_points[0].remaining == 0
        assert not session.completed[0]['actual_goal_node_id'].startswith('PICKUP_')


def test_bounded_conflict_search_can_fall_back_to_a_provably_empty_future_slot():
    with make_session(1.,execution_engine='certified_journey',num_uavs=30,
                      max_batch_size=30,mission_limit=30,drain_timeout_seconds=300,
                      max_admission_search_chunks=1) as session:
        session.control.prefix_reservations.max_attempts = 1
        result = session.run()
        assert result['success'] and result['arrived'] == 30
        assert result['ground_serialization_fallbacks'] > 0


def test_search_continuations_do_not_wait_for_the_entire_world_to_empty():
    with make_session(1.,execution_engine='certified_journey',num_uavs=5,
                      max_batch_size=5,mission_limit=5,drain_timeout_seconds=100) as session:
        session.control.prefix_reservations.max_attempts = 1
        result = session.run()
        assert result['success'] and result['arrived'] == 5
        assert result['ground_serialization_fallbacks'] == 0
        assert result['ground_search_continuations'] > 0


@pytest.mark.parametrize('dt',[1.,.1])
def test_straight_virtual_boundary_can_be_crossed_continuously_with_real_local_replanning(dt):
    with make_session(dt,execution_engine='certified_journey',stop_at_region_boundaries=False) as session:
        result=session.run()
        first=next(record for record in session.completed if record['uav_id']==0)
        assert result['success'] and first['arrival_time_seconds'] == pytest.approx(2.5,abs=1e-9)
        assert result['journey_routing']['verified_regional_entry_plans'] >= 2
        assert result['region_boundary_stop'] is False
        assert result['safety_violations'] == 0


def test_optional_admission_progress_does_not_advance_physical_time():
    with make_session(1.,execution_engine='certified_journey',
                      report_admission_progress=True,admission_progress_interval=1) as session:
        notices = []
        result = session.run(notices.append)
        admission = [r for r in notices if r.get('stage') == 'admission']
        assert admission and admission[0]['sim_time_seconds'] == 0.
        assert admission[-1]['admission_completed'] == 2
        assert result['success']
