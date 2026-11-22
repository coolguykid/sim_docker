"""
engine.py — V6.0 分布式引擎管理（Ray 生命周期 & 并行配置）
"""
import os
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")
os.environ.setdefault("RAY_ACCEL_ENV_VAR_OVERRIDE_ON_ZERO", "0")

DEFAULT_UAVS_PER_ACTOR = 300


def _project_src_dir():
    return Path(__file__).resolve().parents[2]


def _prepend_path(existing, path):
    path = str(Path(path).resolve())
    parts = [p for p in str(existing or "").split(os.pathsep) if p]
    target = os.path.normcase(os.path.abspath(path))
    normalized_parts = []
    target_added = False
    for part in parts:
        if os.path.normcase(os.path.abspath(part)) == target:
            if not target_added:
                normalized_parts.append(path)
                target_added = True
            continue
        normalized_parts.append(part)
    if not target_added:
        normalized_parts.insert(0, path)
    return os.pathsep.join(normalized_parts)


def _build_runtime_env():
    src_dir = str(_project_src_dir())
    return {
        "env_vars": {
            "PYTHONPATH": _prepend_path(os.environ.get("PYTHONPATH"), src_dir),
        }
    }


def _ensure_driver_pythonpath():
    os.environ["PYTHONPATH"] = _build_runtime_env()["env_vars"]["PYTHONPATH"]


class DistributedEngine:
    """管理 Ray 集群的生命周期和并行配置"""

    def __init__(self, parallelism_level=DEFAULT_UAVS_PER_ACTOR, use_ray=True):
        """
        Args:
            parallelism_level: 每个 UAVGroupActor 管理的 UAV 数量
                              1 = 每架无人机一个独立进程
                              N = N架无人机共享一个进程
            use_ray: False 时回退到 V5.0 串行模式
        """
        self.parallelism_level = max(1, parallelism_level)
        self.use_ray = use_ray
        self._initialized = False

    def start(self):
        if not self.use_ray:
            return
        import ray
        if not ray.is_initialized():
            _ensure_driver_pythonpath()
            num_cpus = os.cpu_count() or 4
            ray.init(
                ignore_reinit_error=True,
                num_cpus=num_cpus,
                include_dashboard=False,
                runtime_env=_build_runtime_env(),
            )
            self._initialized = True
            print(f"[Engine] Ray 已启动 — CPU 核心数: {num_cpus}, "
                  f"并行级别: {self.parallelism_level} 架/进程")

    def shutdown(self):
        if not self.use_ray:
            return
        import ray
        if self._initialized and ray.is_initialized():
            ray.shutdown()
            self._initialized = False
            print("[Engine] Ray 已关闭")

    def compute_num_actors(self, num_uavs):
        import math
        return math.ceil(num_uavs / self.parallelism_level)
