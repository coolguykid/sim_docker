from types import SimpleNamespace

from uav_sim.distributed import orchestrator as orchestrator_module
from uav_sim.distributed.orchestrator import SimulationOrchestrator


class _RemoteMethod:
    def __init__(self, result=None):
        self.result = result
        self.calls = []

    def remote(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        return self.result


def test_service_route_promises_forwards_scalar_routing_versions(monkeypatch):
    route_server = SimpleNamespace(
        plan_routes=_RemoteMethod(result="pending-ref"),
    )
    group = SimpleNamespace(
        build_route_requests=_RemoteMethod(result=[]),
    )
    orchestrator = SimulationOrchestrator.__new__(SimulationOrchestrator)
    orchestrator._pending_route_ref = None
    orchestrator.uav_groups = [group]
    orchestrator.mode = "DIJKSTRA"
    orchestrator.route_server = route_server
    monkeypatch.setattr(
        orchestrator_module,
        "ray",
        SimpleNamespace(get=lambda value: [
            [{
                "uav_id": 1,
                "current_pipe_id": 0,
                "pos_1d": 0.0,
                "dest_pipe_id": 1,
                "dest_pos": 10.0,
                "phase": "TO_DEST",
                "spawn_turn": 0,
                "current_turn": 0,
                "speed": 20.0,
            }]
        ]),
    )

    orchestrator._service_route_promises(0, {
        "collisions_multiplier": {0: 1.0},
        "traffic_load": {0: 1},
        "ramp_states": {3: False},
        "topology_version": 4,
        "ramp_state_version": 5,
        "traffic_cost_version": 6,
    })

    _, route_context = route_server.plan_routes.calls[0][0]
    assert route_context["topology_version"] == 4
    assert route_context["ramp_state_version"] == 5
    assert route_context["traffic_cost_version"] == 6
