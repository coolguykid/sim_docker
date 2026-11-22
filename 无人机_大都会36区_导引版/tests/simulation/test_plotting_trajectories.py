import numpy as np

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

from uav_sim.simulation.plotting import plot_3d_trajectories


class DummyPipe:
    def __init__(self, pipe_id, start_pt, end_pt, height):
        self.pipe_id = pipe_id
        self.start_pt = np.array(start_pt, dtype=float)
        self.end_pt = np.array(end_pt, dtype=float)
        self.height = height
        vector = self.end_pt - self.start_pt
        self.length = float(np.linalg.norm(vector))
        self.direction = vector / self.length


class DummyPickup:
    def __init__(self, pipe_id, pos_1d):
        self.pipe_id = pipe_id
        self.pos_1d = pos_1d


class DummyUAV:
    def __init__(self, uid, start_pipe_id, pickup_pipe_id, dest_pipe_id):
        self.uav_id = uid
        self.start_pipe_id = start_pipe_id
        self.pickup_pt = DummyPickup(pickup_pipe_id, 5.0)
        self.final_dest_pipe_id = dest_pipe_id


class DummyEnv:
    def __init__(self):
        self.pipelines = [
            DummyPipe(0, (0, 0, 100), (10, 0, 100), 100),
            DummyPipe(1, (0, 10, 200), (10, 10, 200), 200),
            DummyPipe(2, (0, 20, 300), (10, 20, 300), 300),
        ]
        self.x_min = 0
        self.x_max = 10
        self.y_min = 0
        self.y_max = 20
        self.z_min = 100
        self.z_max = 300


def test_plot_3d_trajectories_can_add_combined_multi_uav_figure(monkeypatch):
    plt.close("all")
    shown = []
    monkeypatch.setattr(plt, "show", lambda: shown.append(plt.gcf()))

    env = DummyEnv()
    uavs = [
        DummyUAV(0, start_pipe_id=0, pickup_pipe_id=1, dest_pipe_id=2),
        DummyUAV(1, start_pipe_id=1, pickup_pipe_id=2, dest_pipe_id=0),
    ]
    results_dict = {
        "1": {
            "trajectories": {
                0: [(0, 0, 100), (5, 0, 100), (10, 0, 100)],
                1: [(0, 10, 200), (5, 10, 200), (10, 10, 200)],
            },
        }
    }

    plot_3d_trajectories(
        results_dict,
        env,
        uavs,
        [0, 1],
        plot_multi_uav_same_figure=True,
    )

    assert len(shown) == 3
    combined_ax = shown[-1].axes[0]
    trajectory_lines = [
        line for line in combined_ax.lines
        if line.get_label().startswith("UAV ")
    ]

    assert len(trajectory_lines) == 2
    assert {line.get_color() for line in trajectory_lines} == {"#1f77b4", "#ff7f0e"}
    assert {line.get_marker() for line in trajectory_lines} == {"o", "s"}
    assert {line.get_linestyle() for line in trajectory_lines} == {"-", "--"}

    plt.close("all")
