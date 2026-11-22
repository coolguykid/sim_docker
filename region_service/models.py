from __future__ import annotations

from datetime import datetime
from typing import Any

from pydantic import BaseModel, Field, model_validator


class GuidanceTransfer(BaseModel):
    source_layer: int = Field(ge=0)
    target_region_id: int = Field(ge=1)
    target_layer: int = Field(ge=0)
    proportion: float = Field(ge=0.0, le=1.0)


class GlobalGuidanceTransfer(GuidanceTransfer):
    source_region_id: int = Field(ge=1)


class GuidanceRequest(BaseModel):
    run_id: str = Field(min_length=1, max_length=128, pattern=r"^[A-Za-z0-9_.:-]+$")
    attempt_id: int = Field(ge=0)
    region_id: int = Field(ge=1)
    snapshot_time: datetime
    topology_version: int = Field(ge=1)
    horizon_seconds: int = Field(gt=0, le=86400)
    step_seconds: float = Field(default=1.0, gt=0.0)
    random_seed: int = 42
    transfers: list[GuidanceTransfer] = Field(default_factory=list)
    # Test bootstrap until center DT publishes the same global routing graph.
    global_routing_graph: dict[str, Any] | None = None
    global_guidance_transfers: list[GlobalGuidanceTransfer] = Field(default_factory=list)
    guidance_revision: int = Field(default=1, ge=1)
    participating_region_ids: list[int] = Field(default_factory=lambda: list(range(1, 36)))
    coordinator_url: str = "http://simulation-coordinator:8080"
    simulation_uavs: list[dict[str, Any]] = Field(default_factory=list)
    congestion_policy: dict[str, Any] = Field(default_factory=dict)
    topology_adjustments: dict[str, Any] = Field(default_factory=dict)
    max_adjustment_rounds: int = Field(default=3, ge=0, le=20)

    @model_validator(mode="after")
    def validate_transfer_proportions(self):
        """校验区域参与方及各源区域层比例归一化。

        输入
            无显式参数。
        输出
            通过校验的请求实例。
        """
        totals: dict[int, float] = {}
        for transfer in self.transfers:
            totals[transfer.source_layer] = (
                totals.get(transfer.source_layer, 0.0) + transfer.proportion
            )
        invalid = {layer: total for layer, total in totals.items() if abs(total - 1.0) > 1e-6}
        if invalid:
            raise ValueError(f"transfer proportions must sum to 1 per source layer: {invalid}")
        if self.region_id not in self.participating_region_ids:
            raise ValueError("participating_region_ids must include this region")
        if len(set(self.participating_region_ids)) != len(self.participating_region_ids):
            raise ValueError("participating_region_ids must not contain duplicates")
        global_totals: dict[tuple[int, int], float] = {}
        for transfer in self.global_guidance_transfers:
            key = (transfer.source_region_id, transfer.source_layer)
            global_totals[key] = global_totals.get(key, 0.0) + transfer.proportion
        invalid_global = {key: total for key, total in global_totals.items()
                          if abs(total - 1.0) > 1e-6}
        if invalid_global:
            raise ValueError(
                f"global guidance proportions must sum to 1 per source cell: {invalid_global}")
        return self


class HandoffInput(BaseModel):
    run_id: str = Field(min_length=1, max_length=128)
    attempt_id: int = Field(ge=0)
    event_id: str = Field(min_length=1, max_length=160)
    uav_id: int
    handoff_epoch: int = Field(ge=1)
    from_region: int = Field(ge=1)
    to_region: int = Field(ge=1)
    boundary_node_id: int
    crossing_time: float = Field(ge=0.0)
    remaining_step_time: float = Field(ge=0.0)
    payload: dict[str, Any] = Field(default_factory=dict)
