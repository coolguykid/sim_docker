from collections import namedtuple
import pytest
from uav_sim.airspace.environment import Airspace
from uav_sim.visualization.frame_exporter import FrameJsonExporter


def test_capacity_guard_covers_the_actual_size_of_realistic_enhanced_rows(tmp_path, monkeypatch):
    env=Airspace(mode='standard',include_dynamic=False)
    exporter=FrameJsonExporter(tmp_path,env,'size-check','1')
    updates={uid:dict(pipe_id=0,lane_id=0,pos_1d=1000.,speed=19.,state='FLYING',phase='TO_DEST',
                     goal_node_id='GOAL_FIXED_0',goal_position_m=[10000.,20000.,600.]) for uid in range(1000)}
    exporter.write_frame(0,updates)
    one_frame=exporter._frame_path(0).stat().st_size
    exporter.close()
    Usage=namedtuple('Usage','total used free')
    # A disk smaller than the actual 1000-frame stream must be rejected,
    # even when the former 320-byte-per-UAV estimate would have accepted it.
    free=one_frame*1000-1
    monkeypatch.setattr('uav_sim.visualization.frame_exporter.shutil.disk_usage',lambda path:Usage(free,0,free))
    with pytest.raises(OSError,match='磁盘空间不足'):
        FrameJsonExporter.check_disk_capacity(tmp_path,1000,1000)
