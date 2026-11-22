"""Persistent HTTP client used by a region to join distributed attempts."""
from __future__ import annotations

from threading import Lock
import time

import httpx


HTTP_TIMEOUT_SECONDS = 120.0
HTTP_KEEPALIVE_SECONDS = 300.0

_CLIENT: httpx.Client | None = None
_CLIENT_LOCK = Lock()


def _get_client():
    """按需创建并复用协调器 HTTP 客户端。

    输入
        无显式参数。
    输出
        HTTP 客户端。

    原有说明
        Lazily create one thread-safe connection pool per region process.
    """
    global _CLIENT
    if _CLIENT is None:
        with _CLIENT_LOCK:
            if _CLIENT is None:
                _CLIENT = httpx.Client(
                    timeout=httpx.Timeout(HTTP_TIMEOUT_SECONDS),
                    limits=httpx.Limits(
                        max_connections=4,
                        max_keepalive_connections=4,
                        keepalive_expiry=HTTP_KEEPALIVE_SECONDS,
                    ),
                    headers={"Content-Type": "application/json; charset=utf-8"},
                )
    return _CLIENT


def close_client():
    """关闭并移除共享 HTTP 客户端。

    输入
        无显式参数。
    输出
        None。

    原有说明
        Close pooled sockets during the region service shutdown.
    """
    global _CLIENT
    with _CLIENT_LOCK:
        client = _CLIENT
        _CLIENT = None
    if client is not None:
        client.close()


def _request(method, url, payload=None):
    """向协调器发送 HTTP 请求并检查响应状态。

    输入
        method：HTTP 方法。
        url：目标 HTTP URL。
        payload：请求、登记信息或阶段数据字典，字段由对应接口约定。
    输出
        解析后的 JSON 响应。
    """
    try:
        response = _get_client().request(method, url, json=payload)
        response.raise_for_status()
        return response.json()
    except httpx.HTTPStatusError as exc:
        raise RuntimeError(
            "coordinator HTTP {}: {}".format(
                exc.response.status_code, exc.response.text)) from exc
    except httpx.HTTPError as exc:
        raise RuntimeError("coordinator HTTP request failed: {}".format(exc)) from exc


def register_attempt(base_url, payload):
    """将本区域的参与信息登记到协调器。

    输入
        base_url：协调器 HTTP 根地址。
        payload：请求、登记信息或阶段数据字典，字段由对应接口约定。
    输出
        协调器的 JSON 响应。
    """
    url = (
        base_url.rstrip("/") +
        "/api/v1/runs/{}/attempts/{}/regions/{}/register".format(
            payload["run_id"], payload["attempt_id"], payload["region_id"]))
    return _request("POST", url, payload)


def get_attempt(base_url, run_id, attempt_id):
    """向协调器查询指定运行和尝试的状态。

    输入
        base_url：协调器 HTTP 根地址。
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
    输出
        协调器返回的尝试状态字典。
    """
    return _request(
        "GET", base_url.rstrip("/") +
        "/api/v1/runs/{}/attempts/{}".format(run_id, attempt_id))


def wait_for_status(base_url, run_id, attempt_id, terminal, poll_seconds=0.2):
    """轮询协调器直到进入给定终态。

    输入
        base_url：协调器 HTTP 根地址。
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
        terminal：允许结束轮询的状态集合。
        poll_seconds：相邻轮询间隔，单位秒。
    输出
        终态的尝试记录。
    """
    while True:
        state = get_attempt(base_url, run_id, attempt_id)
        if state["status"] in terminal:
            return state
        time.sleep(float(poll_seconds))


def report_decision(base_url, payload):
    """向协调器发送本区评估决策。

    输入
        base_url：协调器 HTTP 根地址。
        payload：请求、登记信息或阶段数据字典，字段由对应接口约定。
    输出
        协调器返回的 JSON 响应。
    """
    url = (
        base_url.rstrip("/") +
        "/api/v1/runs/{}/attempts/{}/regions/{}/decision".format(
            payload["run_id"], payload["attempt_id"], payload["region_id"]))
    return _request("POST", url, payload)
