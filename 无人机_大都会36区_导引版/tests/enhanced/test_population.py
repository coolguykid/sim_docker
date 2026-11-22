import pytest
from uav_sim.utils.physical_clock import PhysicalClock
from uav_sim.simulation.population import PopulationScheduler


@pytest.mark.parametrize('dt',[1.,.1])
def test_population_uses_physical_period_and_reserves_pending_budget(dt):
    clock=PhysicalClock(dt)
    population=PopulationScheduler(target=10000,batch_limit=1000,period_seconds=100)
    assert population.due(clock.time_seconds,active=0,pending=0)==1000
    for _ in range(round(100/dt)-1):clock.advance()
    assert population.due(clock.time_seconds,active=1000,pending=0)==0
    clock.advance()
    assert population.due(clock.time_seconds,active=1000,pending=0)==1000
    assert population.due(clock.time_seconds,active=1000,pending=1000)==0
    assert population.due(200,active=9600,pending=300)==100
    assert population.due(300,active=9999,pending=1)==0


def test_clock_and_scheduler_reject_invalid_units():
    for dt in [0,.2,float('nan'),True]:
        with pytest.raises(ValueError):PhysicalClock(dt)
    with pytest.raises(ValueError):PopulationScheduler(10000,1001,100)
    with pytest.raises(ValueError):PopulationScheduler(10000,1000,float('inf'))


def test_disabled_replenishment_creates_only_first_batch():
    p=PopulationScheduler(10000,1000,100,enabled=False)
    assert p.due(0,0,0)==1000
    assert p.due(100,0,0)==0
