from dataclasses import replace
from types import SimpleNamespace
import pytest

from tests.enhanced.test_uav_partition_motion import fixture
from uav_sim.airspace.partition_graph import PartitionMaps
from uav_sim.distributed.regional_backend import RegionalRoutingBackend
from uav_sim.entities.uav import UAV
from uav_sim.utils.physical_clock import PhysicalClock


def setup(mode):
    env, graph, goal = fixture()
    graph.metadata.update(goal_specs={'f':dict(pipe_id=1, lane_id=0, distance_m=19.5)})
    maps = PartitionMaps(graph)
    backend = RegionalRoutingBackend(maps,mode,use_ray=False,target_uavs=1)
    clock = PhysicalClock(.1)
    u = UAV(0,0,0,1,env,capacity=0)
    u.configure_partition_motion(graph,'a',goal)
    plan = backend.plan_many({0:dict(start_node_id='a',region_id=0,goal_node_id='f')},clock.context(0))[0]
    u.set_partition_route(plan.edge_ids)
    session = SimpleNamespace(maps=maps, graph=graph, backend=backend, clock=clock, routing_mode=mode)
    return session,u


@pytest.mark.parametrize('mode',['centralized_full','centralized_regional','onboard_regional'])
def test_safety_forecast_does_not_install_future_maps_in_regional_planner(mode):
    from uav_sim.simulation.journey_routes import JourneyRoutes
    s,u=setup(mode)
    routes=JourneyRoutes(s)
    predicted=routes.forecast_many({0:u},0.)[0]
    assert predicted == ('ab','bc','cd','de','ef')
    assert u._road_state.route == (predicted if mode=='centralized_full' else ('ab','bc'))
    if mode=='onboard_regional':
        assert s.backend.snapshots()[0]['detailed_region_ids'] == (0,)
    at_next=replace(u._road_state,node_id='c',region_id=1)
    routes.verify_many({0:(at_next,('cd','de','ef'))},1.37)
    if mode=='onboard_regional':
        assert s.backend.snapshots()[0]['detailed_region_ids'] == (1,)
    s.backend.close()


def test_actual_regional_plan_must_match_safety_certificate():
    from uav_sim.simulation.journey_routes import JourneyRoutes, ReferenceMismatch
    s,u=setup('centralized_regional')
    routes=JourneyRoutes(s)
    with pytest.raises(ReferenceMismatch):
        routes.verify_many({0:(replace(u._road_state,node_id='c',region_id=1),('ef',))},1.)
    s.backend.close()
