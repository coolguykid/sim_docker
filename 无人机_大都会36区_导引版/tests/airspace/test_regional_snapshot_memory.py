import pickle

import numpy as np

from uav_sim.airspace import Airspace


def _pickle_size_mb(value):
    return len(pickle.dumps(value, protocol=pickle.HIGHEST_PROTOCOL)) / 1024 / 1024


def test_regional_dynamic_snapshot_uses_compact_serialization():
    env = Airspace(mode="regional_6x6", include_dynamic=True)
    snapshot = env.get_readonly_snapshot()

    assert _pickle_size_mb(snapshot) < 25.0

    restored = pickle.loads(pickle.dumps(snapshot, protocol=pickle.HIGHEST_PROTOCOL))
    pipe = restored["pipelines"][0]
    ramp = restored["ramps"][0]

    assert len(pipe.lanes) == pipe.num_lanes == 30
    assert np.allclose(pipe.lanes[0].end_pt - pipe.lanes[0].start_pt, pipe.end_pt - pipe.start_pt)
    assert isinstance(ramp.pos_a, np.ndarray)
    assert isinstance(ramp.pos_b, np.ndarray)
