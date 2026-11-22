"""
airspace_config.py — 空域尺度模式配置
"""
from dataclasses import dataclass
from enum import Enum


class AirspaceMode(str, Enum):
    STANDARD = "standard"
    METROPOLIS = "metropolis"
    REGIONAL_6X6 = "regional_6x6"


@dataclass(frozen=True)
class AirspaceScaleConfig:
    mode: AirspaceMode
    label_cn: str
    side_length: float
    z_max: float
    num_pipes_per_layer: int
    default_num_uavs: int
    max_num_uavs: int
    default_max_turns: int
    default_batch_interval: int
    num_lanes_per_pipe: int = 10
    lane_width: float = 10.0
    region_rows: int = 1
    region_cols: int = 1
    region_side_length: float = 0.0

    @property
    def x_min(self):
        return 0.0

    @property
    def y_min(self):
        return 0.0

    @property
    def x_max(self):
        return self.side_length

    @property
    def y_max(self):
        return self.side_length

    @property
    def center_xy(self):
        half = self.side_length / 2.0
        return (half, half)

    @property
    def spacing_extent(self):
        return self.side_length * 0.4


STANDARD_CONFIG = AirspaceScaleConfig(
    mode=AirspaceMode.STANDARD,
    label_cn="标准模式 5km x 5km",
    side_length=5000.0,
    z_max=600.0,
    num_pipes_per_layer=10,
    default_num_uavs=1000,
    max_num_uavs=10000,
    default_max_turns=3500,
    default_batch_interval=50,
)
METROPOLIS_CONFIG = AirspaceScaleConfig(
    mode=AirspaceMode.METROPOLIS,
    label_cn="大都会模式 30km x 30km",
    side_length=30000.0,
    z_max=600.0,
    num_pipes_per_layer=50,
    default_num_uavs=10000,
    max_num_uavs=50000,
    default_max_turns=18000,
    default_batch_interval=50,
    num_lanes_per_pipe=20,
)
REGIONAL_6X6_CONFIG = AirspaceScaleConfig(
    mode=AirspaceMode.REGIONAL_6X6,
    label_cn="36区域矩阵模式 30km x 30km",
    side_length=30000.0,
    z_max=600.0,
    num_pipes_per_layer=10,
    default_num_uavs=10000,
    max_num_uavs=50000,
    default_max_turns=18000,
    default_batch_interval=50,
    num_lanes_per_pipe=30,
    lane_width=10.0,
    region_rows=6,
    region_cols=6,
    region_side_length=5000.0,
)


def get_airspace_config(mode="standard"):
    if isinstance(mode, AirspaceScaleConfig):
        return mode
    if isinstance(mode, AirspaceMode):
        normalized = mode
    else:
        normalized = AirspaceMode(str(mode or "standard").strip().lower())
    if normalized == AirspaceMode.STANDARD:
        return STANDARD_CONFIG
    if normalized == AirspaceMode.METROPOLIS:
        return METROPOLIS_CONFIG
    if normalized == AirspaceMode.REGIONAL_6X6:
        return REGIONAL_6X6_CONFIG
    raise ValueError(f"Unsupported airspace mode: {mode}")
