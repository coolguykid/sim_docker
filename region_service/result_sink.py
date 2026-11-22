from __future__ import annotations

from repository import store_outbox


class RepositoryOutputSink:
    """Persistence boundary used by a future streaming simulation engine."""

    def __init__(self, request: dict, round_index: int):
        """初始化某次仿真尝试及调整轮次的输出接收器。

        输入
            request：区域任务请求及运行配置。
            round_index：该任务内从零开始的调整轮次。
        输出
            None。
        """
        self.request = request
        self.round_index = int(round_index)
        self.frame_count = 0
        self.statistics_batches = 0
        self.handoff_count = 0
        self._handoff_event_ids: set[str] = set()

    def on_frame(self, frame: dict) -> None:
        # Frame storage/streaming is intentionally a port.  The final engine can
        # write CSV, TimescaleDB, or frontend streams without changing workflow.
        """接收一帧无人机状态并交给具体输出实现。

        输入
            frame：当前帧的无人机状态记录。
        输出
            None。
        """
        self.frame_count += 1

    def on_statistics(self, statistics: dict) -> None:
        """接收一批道路统计并交给具体输出实现。

        输入
            statistics：道路统计记录或历史批次。
        输出
            None。
        """
        self.statistics_batches += 1

    def on_handoff(self, handoff: dict) -> None:
        """接收一次移交并交给具体输出实现。

        输入
            handoff：待记录的跨区移交载荷。
        输出
            None。
        """
        event = dict(handoff)
        event.setdefault("run_id", self.request["run_id"])
        event.setdefault("attempt_id", self.request["attempt_id"])
        event.setdefault("from_region", self.request["region_id"])
        event_id = str(event["event_id"])
        if event_id in self._handoff_event_ids:
            return
        store_outbox(event)
        self._handoff_event_ids.add(event_id)
        self.handoff_count += 1

    def summary(self) -> dict:
        """汇总已接收的输出和统计。

        输入
            无显式参数。
        输出
            结果摘要字典。
        """
        return {
            "frame_count": self.frame_count,
            "statistics_batches": self.statistics_batches,
            "handoff_count": self.handoff_count,
        }
