'''区域 PostgreSQL 操作接口，负责连接、查询和持久化。'''
from __future__ import annotations

import json
import os
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from itertools import groupby

import psycopg
from psycopg import sql
from psycopg.rows import dict_row


REGION_ID = int(os.environ.get("REGION_ID", "1"))


def dsn() -> str:
    """从环境配置组装 PostgreSQL 连接参数。

    输入
        无显式参数。
    输出
        数据库连接串。
    """
    configured = os.environ.get("REGION_DATABASE_URL")
    if configured:
        return configured
    return (
        f"host=127.0.0.1 port=5432 dbname={os.environ['POSTGRES_DB']} "
        f"user={os.environ['POSTGRES_USER']} password={os.environ['POSTGRES_PASSWORD']}"
    )


@contextmanager
def connection(*, row_factory=None):
    """打开数据库连接并管理其事务和生命周期。

    输入
        row_factory：数据库记录到 Python 对象的转换器。
    输出
        数据库连接上下文。
    """
    with psycopg.connect(dsn(), row_factory=row_factory) as conn:
        yield conn


def initialize_control_schema() -> None:
    """按需初始化区域控制表和支持结构。

    输入
        无显式参数。
    输出
        None。
    """
    with connection() as conn:
        conn.execute(
            """
            CREATE TABLE IF NOT EXISTS region_dt_simulation (
                run_id TEXT NOT NULL,
                attempt_id INTEGER NOT NULL,
                region_id INTEGER NOT NULL,
                status TEXT NOT NULL,
                request JSONB NOT NULL,
                snapshot_summary JSONB,
                result_summary JSONB,
                error_message TEXT,
                created_at TIMESTAMPTZ NOT NULL DEFAULT now(),
                started_at TIMESTAMPTZ,
                finished_at TIMESTAMPTZ,
                PRIMARY KEY (run_id, attempt_id, region_id)
            );
            CREATE TABLE IF NOT EXISTS region_dt_workflow_event (
                event_seq BIGSERIAL PRIMARY KEY,
                run_id TEXT NOT NULL,
                attempt_id INTEGER NOT NULL,
                region_id INTEGER NOT NULL,
                event_type TEXT NOT NULL,
                payload JSONB NOT NULL DEFAULT '{}'::jsonb,
                created_at TIMESTAMPTZ NOT NULL DEFAULT now()
            );
            """
        )
        conn.execute(
            """
            UPDATE region_dt_simulation
            SET status='failed', error_message='service restarted while task was active',
                finished_at=now()
            WHERE region_id=%s AND status IN (
                'snapshotting','snapshot_ready','simulating','evaluating',
                'adjusting_network','dispatching_guidance'
            )
            """,
            (REGION_ID,),
        )


def database_ready() -> bool:
    """探测本区数据库能否响应。

    输入
        无显式参数。
    输出
        布尔值。
    """
    try:
        with connection() as conn:
            value = conn.execute("SELECT 1").fetchone()
            return bool(value and value[0] == 1)
    except Exception:
        return False


def has_active_task() -> bool:
    """检查区域是否有活跃仿真任务。

    输入
        无显式参数。
    输出
        布尔值。
    """
    try:
        with connection() as conn:
            row = conn.execute(
                "SELECT EXISTS(SELECT 1 FROM region_dt_simulation "
                "WHERE region_id=%s AND status IN ("
                "'queued','snapshotting','snapshot_ready','simulating','evaluating',"
                "'adjusting_network','dispatching_guidance'))",
                (REGION_ID,),
            ).fetchone()
            return bool(row and row[0])
    except Exception:
        return False


def waiting_integration_status() -> str | None:
    """查询正在等待外部集成的任务状态。

    输入
        无显式参数。
    输出
        状态字符串或 None。
    """
    with connection() as conn:
        row = conn.execute(
            "SELECT status FROM region_dt_simulation "
            "WHERE region_id=%s AND status LIKE 'waiting_%%' "
            "ORDER BY created_at DESC LIMIT 1",
            (REGION_ID,),
        ).fetchone()
        return str(row[0]) if row else None


def enqueue_request(payload: dict) -> tuple[bool, dict]:
    """持久化导引请求，并识别重复提交。

    输入
        payload：请求、登记信息或阶段数据字典，字段由对应接口约定。
    输出
        是否新建及对应任务记录。
    """
    key = (payload["run_id"], payload["attempt_id"], payload["region_id"])
    canonical = json.dumps(payload, ensure_ascii=False, sort_keys=True, default=str)
    with connection(row_factory=dict_row) as conn:
        existing = conn.execute(
            "SELECT status,request FROM region_dt_simulation "
            "WHERE run_id=%s AND attempt_id=%s AND region_id=%s",
            key,
        ).fetchone()
        if existing:
            existing_canonical = json.dumps(existing["request"], ensure_ascii=False, sort_keys=True, default=str)
            if existing_canonical != canonical:
                raise ValueError("run_id/attempt_id already exists with a different request")
            return False, existing
        conn.execute(
            "INSERT INTO region_dt_simulation(run_id,attempt_id,region_id,status,request) "
            "VALUES(%s,%s,%s,'queued',%s::jsonb)",
            (*key, canonical),
        )
    return True, {"status": "queued"}


def list_queued_keys() -> list[tuple[str, int]]:
    """取出可由工作队列恢复的数据库任务键。

    输入
        无显式参数。
    输出
        运行 ID 和尝试编号列表。
    """
    with connection() as conn:
        return [
            (row[0], int(row[1]))
            for row in conn.execute(
                "SELECT run_id,attempt_id FROM region_dt_simulation "
                "WHERE region_id=%s AND status='queued' ORDER BY created_at",
                (REGION_ID,),
            ).fetchall()
        ]


def append_workflow_event(
    run_id: str, attempt_id: int, event_type: str, payload: dict
) -> None:
    """持久化一次区域工作流事件。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
        event_type：工作流事件类型。
        payload：请求、登记信息或阶段数据字典，字段由对应接口约定。
    输出
        None。
    """
    with connection() as conn:
        conn.execute(
            "INSERT INTO region_dt_workflow_event(" 
            "run_id,attempt_id,region_id,event_type,payload) "
            "VALUES(%s,%s,%s,%s,%s::jsonb)",
            (
                run_id, attempt_id, REGION_ID, event_type,
                json.dumps(payload, ensure_ascii=False, default=str),
            ),
        )


def list_workflow_events(run_id: str, attempt_id: int) -> list[dict]:
    """按时间顺序读取指定尝试的工作流事件。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
    输出
        事件记录列表。
    """
    with connection(row_factory=dict_row) as conn:
        rows = conn.execute(
            "SELECT event_seq,event_type,payload,created_at "
            "FROM region_dt_workflow_event "
            "WHERE run_id=%s AND attempt_id=%s AND region_id=%s "
            "ORDER BY event_seq",
            (run_id, attempt_id, REGION_ID),
        ).fetchall()
        return [dict(row) for row in rows]


def requeue_task(run_id: str, attempt_id: int) -> bool:
    """将符合恢复条件的任务标为可再次执行。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
    输出
        是否更新成功。
    """
    with connection() as conn:
        result = conn.execute(
            "UPDATE region_dt_simulation SET status='queued',error_message=NULL," 
            "started_at=NULL,finished_at=NULL "
            "WHERE run_id=%s AND attempt_id=%s AND region_id=%s "
            "AND (status LIKE 'waiting_%%' OR status='failed')",
            (run_id, attempt_id, REGION_ID),
        )
        return result.rowcount == 1


def get_task(run_id: str, attempt_id: int) -> dict | None:
    """读取指定运行和尝试的任务记录。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
    输出
        任务字典或 None。
    """
    with connection(row_factory=dict_row) as conn:
        row = conn.execute(
            "SELECT * FROM region_dt_simulation "
            "WHERE run_id=%s AND attempt_id=%s AND region_id=%s",
            (run_id, attempt_id, REGION_ID),
        ).fetchone()
        return dict(row) if row else None


def mark_status(run_id: str, attempt_id: int, status: str, **fields) -> None:
    """写入区域任务状态及附加字段。

    输入
        run_id：仿真运行标识。
        attempt_id：同一运行的尝试编号。
        status：要记录的工作流状态。
        fields：随状态一并写入的补充字段。
    输出
        None。
    """
    assignments = ["status=%s"]
    values = [status]
    for name in ("snapshot_summary", "result_summary"):
        if name in fields:
            assignments.append(f"{name}=%s::jsonb")
            values.append(json.dumps(fields[name], ensure_ascii=False, default=str))
    if "error_message" in fields:
        assignments.append("error_message=%s")
        values.append(fields["error_message"])
    if status == "snapshotting":
        assignments.append("started_at=COALESCE(started_at,now())")
    if status in ("completed", "failed"):
        assignments.append("finished_at=now()")
    values.extend([run_id, attempt_id, REGION_ID])
    with connection() as conn:
        conn.execute(
            f"UPDATE region_dt_simulation SET {','.join(assignments)} "
            "WHERE run_id=%s AND attempt_id=%s AND region_id=%s",
            values,
        )


def fetch_topology_rows(version_id: int | None, at: datetime) -> tuple:
    """在同一个只读事务中查询区域拓扑及设施状态。

    输入
        version_id：指定拓扑版本。为 None 时按查询时间选择版本。
        at：拓扑有效性和设施状态的查询时刻。
    输出
        依次返回区域、版本、层、节点、航道、坡道和车道记录。
    """
    with connection(row_factory=dict_row) as conn:
        conn.execute("SET TRANSACTION ISOLATION LEVEL REPEATABLE READ, READ ONLY")
        region = conn.execute(
            "SELECT * FROM region_info WHERE region_id=%s", (REGION_ID,)
        ).fetchone()
        if not region:
            raise LookupError(f"region {REGION_ID} does not exist")

        if version_id is None:
            version = conn.execute(
                """
                SELECT * FROM topology_version
                WHERE region_id=%s AND valid_from<=%s
                  AND (valid_to IS NULL OR valid_to>%s)
                ORDER BY valid_from DESC,version_id DESC LIMIT 1
                """,
                (REGION_ID, at, at),
            ).fetchone()
            if not version:
                version = conn.execute(
                    "SELECT * FROM topology_version WHERE region_id=%s "
                    "ORDER BY version_id DESC LIMIT 1",
                    (REGION_ID,),
                ).fetchone()
        else:
            version = conn.execute(
                "SELECT * FROM topology_version WHERE region_id=%s AND version_id=%s",
                (REGION_ID, version_id),
            ).fetchone()
        if not version:
            raise LookupError(
                f"topology version {version_id!r} does not exist in region {REGION_ID}"
            )
        selected_version = int(version["version_id"])

        layers = conn.execute(
            "SELECT * FROM layer_info WHERE region_id=%s AND version_id=%s ORDER BY layer_id",
            (REGION_ID, selected_version),
        ).fetchall()
        nodes = conn.execute(
            "SELECT * FROM node_info WHERE region_id=%s AND version_id=%s ORDER BY node_id",
            (REGION_ID, selected_version),
        ).fetchall()
        pipes = conn.execute(
            """
            SELECT p.*,s.time AS status_time,COALESCE(s.is_open,TRUE) AS is_open,
                   COALESCE(s.speed_limit,15.0) AS speed_limit,
                   COALESCE(s.d_safe,20.0) AS d_safe,
                   COALESCE(s.max_capacity,50) AS max_capacity
            FROM pipe_info p
            LEFT JOIN LATERAL(
                SELECT * FROM pipe_status s
                WHERE s.region_id=p.region_id AND s.version_id=p.version_id
                  AND s.pipe_id=p.pipe_id AND s.time<=%s
                ORDER BY s.time DESC LIMIT 1
            ) s ON TRUE
            WHERE p.region_id=%s AND p.version_id=%s ORDER BY p.pipe_id
            """,
            (at, REGION_ID, selected_version),
        ).fetchall()
        ramps = conn.execute(
            """
            SELECT r.*,s.time AS status_time,COALESCE(s.is_open,TRUE) AS is_open
            FROM ramp_info r
            LEFT JOIN LATERAL(
                SELECT * FROM ramp_status s
                WHERE s.region_id=r.region_id AND s.version_id=r.version_id
                  AND s.ramp_id=r.ramp_id AND s.time<=%s
                ORDER BY s.time DESC LIMIT 1
            ) s ON TRUE
            WHERE r.region_id=%s AND r.version_id=%s ORDER BY r.ramp_id
            """,
            (at, REGION_ID, selected_version),
        ).fetchall()
        lanes = conn.execute(
            "SELECT * FROM lane_info WHERE region_id=%s AND version_id=%s ORDER BY lane_id",
            (REGION_ID, selected_version),
        ).fetchall()

    return region, version, layers, nodes, pipes, ramps, lanes


def _fetch_all(conn, query, params):
    """以参数化 SQL 读取数据库记录。

    输入
        conn：已打开的数据库连接。
        query：待执行的 SQL。
        params：绑定到 SQL 的参数。
    输出
        查询结果列表。
    """
    return [dict(row) for row in conn.execute(query, params).fetchall()]


def load_snapshot(request: dict) -> dict:
    """从区域数据库装载满足请求版本及状态时刻的仿真快照。

    输入
        request：区域任务请求及运行配置。
    输出
        包含网络和设施状态的快照。

    原有说明
        Load one immutable, repeatable-read network and UAV snapshot.
        
        This function only maps database records into the engine-input DTO.  It does
        not build a routing graph, allocate guidance, or advance any UAV.
    """
    version = int(request["topology_version"])
    snapshot_time = request["snapshot_time"]
    if isinstance(snapshot_time, str):
        snapshot_time = datetime.fromisoformat(snapshot_time)

    with connection(row_factory=dict_row) as conn:
        conn.execute("SET TRANSACTION ISOLATION LEVEL REPEATABLE READ, READ ONLY")
        exists = conn.execute(
            "SELECT 1 FROM topology_version WHERE region_id=%s AND version_id=%s",
            (REGION_ID, version),
        ).fetchone()
        if not exists:
            raise ValueError(f"topology version {version} does not exist in region {REGION_ID}")

        nodes = _fetch_all(
            conn,
            "SELECT * FROM node_info WHERE region_id=%s AND version_id=%s ORDER BY node_id",
            (REGION_ID, version),
        )
        pipes = _fetch_all(conn, """
            SELECT p.*,COALESCE(s.is_open,TRUE) AS is_open,
                   COALESCE(s.speed_limit,15.0) AS speed_limit,
                   COALESCE(s.d_safe,20.0) AS d_safe,
                   COALESCE(s.max_capacity,50) AS max_capacity
            FROM pipe_info p
            LEFT JOIN LATERAL(
                SELECT * FROM pipe_status s
                WHERE s.region_id=p.region_id AND s.version_id=p.version_id
                  AND s.pipe_id=p.pipe_id AND s.time<=%s
                ORDER BY s.time DESC LIMIT 1
            ) s ON TRUE
            WHERE p.region_id=%s AND p.version_id=%s ORDER BY p.pipe_id
            """, (snapshot_time, REGION_ID, version))
        ramps = _fetch_all(conn, """
            SELECT r.*,COALESCE(s.is_open,TRUE) AS is_open
            FROM ramp_info r
            LEFT JOIN LATERAL(
                SELECT * FROM ramp_status s
                WHERE s.region_id=r.region_id AND s.version_id=r.version_id
                  AND s.ramp_id=r.ramp_id AND s.time<=%s
                ORDER BY s.time DESC LIMIT 1
            ) s ON TRUE
            WHERE r.region_id=%s AND r.version_id=%s ORDER BY r.ramp_id
            """, (snapshot_time, REGION_ID, version))
        lanes = _fetch_all(
            conn,
            "SELECT * FROM lane_info WHERE region_id=%s AND version_id=%s ORDER BY lane_id",
            (REGION_ID, version),
        )
        uavs = _fetch_all(conn, """
            SELECT DISTINCT ON(d.uav_id) d.*,i.source,i.destination
            FROM uav_data d JOIN uav_info i USING(uav_id)
            WHERE d.region_id=%s AND d.version_id=%s AND d.time<=%s
            ORDER BY d.uav_id,d.time DESC
            """, (REGION_ID, version, snapshot_time))

    return {
        "region_id": REGION_ID,
        "topology_version": version,
        "snapshot_time": snapshot_time,
        "network": {
            "nodes": nodes,
            "pipes": pipes,
            "ramps": ramps,
            "lanes": lanes,
        },
        "uavs": uavs,
    }


def commit_to_database(topology_version, effective_snapshot, adjustments):
    """持久化已接受的网络调整。

    输入
        topology_version：有效拓扑的版本号。
        effective_snapshot：当前尝试使用的有效快照。
        adjustments：待应用或持久化的设施调整。
    输出
        数据库提交回执。

    原有说明
        Persist only accepted dynamic road-state changes in one transaction.
    """
    if not (adjustments or {}).get("pipes") and not (adjustments or {}).get("ramps"):
        return {"topology_changed": False, "pipes": 0, "ramps": 0}
    pipes = {int(row["pipe_id"]): row for row in effective_snapshot["network"]["pipes"]}
    ramps = {int(row["ramp_id"]): row for row in effective_snapshot["network"]["ramps"]}
    changed_at = datetime.now(timezone.utc)
    with connection() as conn:
        for change in adjustments.get("pipes", ()):
            row = pipes[int(change["pipe_id"])]
            conn.execute(
                "INSERT INTO pipe_status(time,region_id,version_id,pipe_id,is_open,"
                "d_safe,speed_limit,max_capacity) VALUES(%s,%s,%s,%s,%s,%s,%s,%s)",
                (changed_at, REGION_ID, topology_version, int(change["pipe_id"]),
                 bool(row.get("is_open", True)), float(row.get("d_safe", 20.0)),
                 float(row.get("speed_limit", 15.0)), int(row.get("max_capacity", 50))))
        for change in adjustments.get("ramps", ()):
            row = ramps[int(change["ramp_id"])]
            conn.execute(
                "INSERT INTO ramp_status(time,region_id,version_id,ramp_id,is_open) "
                "VALUES(%s,%s,%s,%s,%s)",
                (changed_at, REGION_ID, topology_version, int(change["ramp_id"]),
                 bool(row.get("is_open", True))))
    return {
        "topology_changed": True,
        "changed_at": changed_at.isoformat(),
        "pipes": len(adjustments.get("pipes", ())),
        "ramps": len(adjustments.get("ramps", ())),
    }


# 以下接口由 database/database.h 和 database/database.cpp 的 C++ 版迁移。
# 数据库记录使用 dict，时间使用 datetime，PostgreSQL NULL 使用 None。
# 路网投影保留 C++ 接口语义，导引版推演继续通过自身适配器建图。

_TOPOLOGY_KEYS = {
    "topology_version": ("region_id", "version_id"),
    "layer_info": ("region_id", "version_id", "layer_id"),
    "node_info": ("region_id", "version_id", "node_id"),
    "pipe_info": ("region_id", "version_id", "pipe_id"),
    "ramp_info": ("region_id", "version_id", "ramp_id"),
    "lane_info": ("region_id", "version_id", "lane_id"),
}
_FLOW_IDS = {"pipe_flow": "pipe_id", "ramp_flow": "ramp_id", "lane_flow": "lane_id"}
_FLOW_FIELDS = ("uav_number", "flow", "density", "average_speed")
FLOW_FEATURE_COLUMNS = (
    *_FLOW_FIELDS, "is_open", "length", "type_pipe",
    "type_same_layer_ramp", "type_cross_layer_ramp", "type_funnel_ramp",
)


def _write_rows(table, rows, *, conflict_keys=()):
    """在一个事务中批量写记录。指定冲突键时更新传入的非键字段，返回写入条数。"""
    count = 0
    with connection() as conn:
        # 相同字段集合使用同一条参数化语句，允许记录省略有数据库默认值的列。
        for columns, group in groupby(rows, key=lambda row: tuple(sorted(row))):
            values = [tuple(row[column] for column in columns) for row in group]
            statement = sql.SQL("INSERT INTO {} ({}) VALUES ({})").format(
                sql.Identifier(table),
                sql.SQL(", ").join(map(sql.Identifier, columns)),
                sql.SQL(", ").join(sql.Placeholder() for _ in columns),
            )
            if conflict_keys:
                keys = sql.SQL(", ").join(map(sql.Identifier, conflict_keys))
                updates = [
                    sql.SQL("{} = EXCLUDED.{}").format(sql.Identifier(c), sql.Identifier(c))
                    for c in columns if c not in conflict_keys
                ]
                action = (
                    sql.SQL("DO UPDATE SET {}").format(sql.SQL(", ").join(updates))
                    if updates else sql.SQL("DO NOTHING")
                )
                statement += sql.SQL(" ON CONFLICT ({}) {}").format(keys, action)
            with conn.cursor() as cursor:
                cursor.executemany(statement, values)
                count += cursor.rowcount
    return count


def upsert_region_info(row):
    """接收一区域字段字典，按 region_id 写入或更新，返回影响条数。"""
    return _write_rows("region_info", [row], conflict_keys=("region_id",))


def insert_topology_versions(rows):
    """接收拓扑版本记录，按区域和版本导入或更新，返回影响条数。"""
    return _write_rows("topology_version", rows, conflict_keys=_TOPOLOGY_KEYS["topology_version"])


def insert_layer_info(rows):
    """接收层记录，按区域、版本和层编号导入或更新，返回影响条数。"""
    return _write_rows("layer_info", rows, conflict_keys=_TOPOLOGY_KEYS["layer_info"])


def insert_node_info(rows):
    """接收节点记录，按区域、版本和节点编号导入或更新，返回影响条数。"""
    return _write_rows("node_info", rows, conflict_keys=_TOPOLOGY_KEYS["node_info"])


def insert_pipe_info(rows):
    """导入或更新航道记录。新记录须提供 main_pipe_pair_id，返回影响条数。"""
    return _write_rows("pipe_info", rows, conflict_keys=_TOPOLOGY_KEYS["pipe_info"])


def insert_ramp_info(rows):
    """接收引路及路径记录，按区域、版本和引路编号导入或更新，返回影响条数。"""
    return _write_rows("ramp_info", rows, conflict_keys=_TOPOLOGY_KEYS["ramp_info"])


def insert_lane_info(rows):
    """接收车道几何记录，按区域、版本和车道编号导入或更新，返回影响条数。"""
    return _write_rows("lane_info", rows, conflict_keys=_TOPOLOGY_KEYS["lane_info"])


def insert_pipe_status(rows):
    """接收带时间戳的航道动态状态并追加，返回写入条数。"""
    return _write_rows("pipe_status", rows)


def insert_ramp_status(rows):
    """接收带时间戳的引路开闭状态并追加，返回写入条数。"""
    return _write_rows("ramp_status", rows)


def insert_pipe_flow(rows):
    """接收航道流量、密度等时序记录并追加，返回写入条数。"""
    return _write_rows("pipe_flow", rows)


def insert_ramp_flow(rows):
    """接收引路交通流时序记录并追加，返回写入条数。"""
    return _write_rows("ramp_flow", rows)


def insert_lane_flow(rows):
    """接收含所属航道 ID 的车道交通流记录并追加，返回写入条数。"""
    return _write_rows("lane_flow", rows)


def upsert_uav_info(rows):
    """接收 UAV 起终点坐标记录，按 uav_id 写入或更新，返回影响条数。"""
    return _write_rows("uav_info", rows, conflict_keys=("uav_id",))


def insert_uav_states(rows):
    """追加 UAV 时序状态。可空字段以 None 表示，返回写入条数。"""
    return _write_rows("uav_data", rows)


def upsert_flow_prediction(row):
    """接收区域及预测矩阵，覆盖该区上一份预测，返回影响条数。"""
    return _write_rows("flow_prediction", [row], conflict_keys=("region_id",))


def upsert_flow_guidance(row):
    """接收区域及导引矩阵，覆盖该区上一份导引，返回影响条数。"""
    return _write_rows("flow_guidance", [row], conflict_keys=("region_id",))


def get_region_info(region_id=REGION_ID):
    """按区域 ID 查询基本信息，返回记录或 None。"""
    with connection(row_factory=dict_row) as conn:
        return conn.execute("SELECT * FROM region_info WHERE region_id=%s", (region_id,)).fetchone()


def get_region_resources(region_id=REGION_ID):
    """按区域 ID 查询带宽、算力和负载，返回记录或 None。"""
    with connection(row_factory=dict_row) as conn:
        return conn.execute(
            "SELECT region_id, bandwidth, computing_resource, load "
            "FROM region_info WHERE region_id=%s", (region_id,),
        ).fetchone()


def update_region_resources(row):
    """按记录中的区域 ID 更新带宽、算力和负载，返回是否找到该区域。"""
    with connection() as conn:
        result = conn.execute(
            "UPDATE region_info SET bandwidth=%(bandwidth)s, "
            "computing_resource=%(computing_resource)s, load=%(load)s "
            "WHERE region_id=%(region_id)s", row,
        )
        return result.rowcount == 1


def get_current_topology_version(region_id=REGION_ID):
    """按区域 ID 查询 valid_to 为空的版本，返回版本记录或 None。"""
    with connection(row_factory=dict_row) as conn:
        return conn.execute(
            "SELECT * FROM topology_version WHERE region_id=%s AND valid_to IS NULL",
            (region_id,),
        ).fetchone()


def _version_at(conn, region_id, at):
    """在现有连接中查询指定时刻有效的版本，返回记录或 None。"""
    return conn.execute(
        "SELECT * FROM topology_version WHERE region_id=%s AND valid_from<=%s "
        "AND (valid_to IS NULL OR valid_to>%s) "
        "ORDER BY valid_from DESC, version_id DESC LIMIT 1",
        (region_id, at, at),
    ).fetchone()


def get_topology_version_at(region_id=REGION_ID, observation_time=None):
    """按区域和观测时刻查询有效拓扑，默认当前时间，返回记录或 None。"""
    with connection(row_factory=dict_row) as conn:
        return _version_at(conn, region_id, observation_time or datetime.now(timezone.utc))


def _topology_rows(conn, table, region_id, version_id):
    """用复合主键前缀读取拓扑记录，按设施 ID 排序返回列表。"""
    order = _TOPOLOGY_KEYS[table][-1]
    return conn.execute(
        sql.SQL("SELECT * FROM {} WHERE region_id=%s AND version_id=%s ORDER BY {}")
        .format(sql.Identifier(table), sql.Identifier(order)),
        (region_id, version_id),
    ).fetchall()


def _read_topology(table, region_id, version_id):
    """打开连接并读取指定区域版本的一类静态记录，返回列表。"""
    with connection(row_factory=dict_row) as conn:
        return _topology_rows(conn, table, region_id, version_id)


def get_layer_info(region_id, version_id):
    """按区域和版本读取层记录，返回列表。"""
    return _read_topology("layer_info", region_id, version_id)


def get_node_info(region_id, version_id):
    """按区域和版本读取节点记录，返回列表。"""
    return _read_topology("node_info", region_id, version_id)


def get_pipe_info(region_id, version_id):
    """按区域和版本读取航道记录，包含主干配对 ID，返回列表。"""
    return _read_topology("pipe_info", region_id, version_id)


def get_ramp_info(region_id, version_id):
    """按区域和版本读取引路记录，原样保留可空宽度和路径数组，返回列表。"""
    return _read_topology("ramp_info", region_id, version_id)


def get_lane_info(region_id, version_id):
    """按区域和版本读取车道记录，按航道及车道编号排序返回列表。"""
    return sorted(_read_topology("lane_info", region_id, version_id),
                  key=lambda row: (row["pipe_id"], row["lane_id"]))


def _latest_rows_at(conn, table, id_field, region_id, version_id, at):
    """在现有连接中读取各设施不晚于时刻 at 的最新记录，返回按设施 ID 索引的字典。"""
    ident = sql.Identifier(id_field)
    rows = conn.execute(
        sql.SQL("SELECT DISTINCT ON ({id}) * FROM {table} "
                "WHERE region_id=%s AND version_id=%s AND time<=%s "
                "ORDER BY {id}, time DESC").format(id=ident, table=sql.Identifier(table)),
        (region_id, version_id, at),
    ).fetchall()
    return {row[id_field]: row for row in rows}


def get_pipe_status_at(region_id, version_id, observation_time=None):
    """查询每条航道在指定时刻有效的状态，返回按 pipe_id 索引的字典。"""
    with connection(row_factory=dict_row) as conn:
        return _latest_rows_at(conn, "pipe_status", "pipe_id", region_id, version_id,
                               observation_time or datetime.now(timezone.utc))


def get_ramp_status_at(region_id, version_id, observation_time=None):
    """查询每条引路在指定时刻有效的状态，返回按 ramp_id 索引的字典。"""
    with connection(row_factory=dict_row) as conn:
        return _latest_rows_at(conn, "ramp_status", "ramp_id", region_id, version_id,
                               observation_time or datetime.now(timezone.utc))


def _recent_rows(conn, table, id_field, region_id, top_n, version_id=None):
    """按设施或 UAV 分组读取各自最近 N 条记录，返回分组字典。"""
    version = sql.SQL(" AND version_id=%s") if version_id is not None else sql.SQL("")
    params = [region_id] if version_id is None else [region_id, version_id]
    ident = sql.Identifier(id_field)
    rows = conn.execute(
        sql.SQL("SELECT * FROM (SELECT *, ROW_NUMBER() OVER "
                "(PARTITION BY {id} ORDER BY time DESC) AS row_number FROM {table} "
                "WHERE region_id=%s{version}) ranked WHERE row_number<=%s "
                "ORDER BY {id}, time DESC").format(
                    id=ident, table=sql.Identifier(table), version=version),
        (*params, top_n),
    ).fetchall()
    grouped = {}
    for row in rows:
        row.pop("row_number")
        grouped.setdefault(row[id_field], []).append(row)
    return grouped


def _recent_flow(table, top_n, region_id, version_id):
    """按指定或当前有效版本读取流量时序，返回按设施索引的历史记录。"""
    with connection(row_factory=dict_row) as conn:
        if version_id is None:
            version = _version_at(conn, region_id, datetime.now(timezone.utc))
            if version is None:
                raise LookupError(f"region {region_id} has no active topology version")
            version_id = version["version_id"]
        return _recent_rows(conn, table, _FLOW_IDS[table], region_id, top_n, version_id)


def get_recent_pipe_flow(top_n=12, *, region_id=REGION_ID, version_id=None):
    """返回各航道最近 top_n 条流量记录，可指定区域和版本。"""
    return _recent_flow("pipe_flow", top_n, region_id, version_id)


def get_recent_ramp_flow(top_n=12, *, region_id=REGION_ID, version_id=None):
    """返回各引路最近 top_n 条流量记录，可指定区域和版本。"""
    return _recent_flow("ramp_flow", top_n, region_id, version_id)


def get_recent_lane_flow(top_n=12, *, region_id=REGION_ID, version_id=None):
    """返回各车道最近 top_n 条流量记录，可指定区域和版本。"""
    return _recent_flow("lane_flow", top_n, region_id, version_id)


def get_uav_info():
    """读取本库所有 UAV 基本任务信息，按 UAV ID 排序返回。"""
    with connection(row_factory=dict_row) as conn:
        return conn.execute("SELECT * FROM uav_info ORDER BY uav_id").fetchall()


def get_recent_uav_states(top_n=12, *, region_id=REGION_ID, version_id=None):
    """返回本区各 UAV 最近 top_n 条记录；可选限制拓扑版本。"""
    with connection(row_factory=dict_row) as conn:
        return _recent_rows(conn, "uav_data", "uav_id", region_id, top_n, version_id)


def get_flight_intents(region_id=REGION_ID):
    """接收区域 ID，返回每架 UAV 的最新位置、任务起终点和时间。"""
    with connection(row_factory=dict_row) as conn:
        return conn.execute(
            "SELECT DISTINCT ON (d.uav_id) d.time,d.uav_id,d.position,i.source,i.destination "
            "FROM uav_data d JOIN uav_info i USING (uav_id) "
            "WHERE d.region_id=%s ORDER BY d.uav_id,d.time DESC", (region_id,),
        ).fetchall()


def get_flow_guidance(region_id=REGION_ID):
    """读取该区保存的导引矩阵记录，缺失时返回 None。"""
    with connection(row_factory=dict_row) as conn:
        return conn.execute("SELECT * FROM flow_guidance WHERE region_id=%s", (region_id,)).fetchone()


def get_flow_prediction(region_id=REGION_ID):
    """读取该区保存的预测矩阵记录，缺失时返回 None。"""
    with connection(row_factory=dict_row) as conn:
        return conn.execute("SELECT * FROM flow_prediction WHERE region_id=%s", (region_id,)).fetchone()


def append_pipe_safety_distance(changes, time=None, *, region_id=REGION_ID):
    """按时刻继承航道其他有效状态并写入新安全距离，返回影响条数。

    changes 接收 (pipe_id, d_safe) 序列。同时间的状态主键冲突时只更新 d_safe。
    """
    at = time or datetime.now(timezone.utc)
    with connection(row_factory=dict_row) as conn:
        version = _version_at(conn, region_id, at)
        if version is None:
            raise LookupError(f"region {region_id} has no topology version at {at}")
        version_id = version["version_id"]
        values = [(at, region_id, version_id, pid, distance, region_id, version_id, pid, at)
                  for pid, distance in changes]
        if not values:
            return 0
        with conn.cursor() as cursor:
            cursor.executemany(
                """INSERT INTO pipe_status
                       (time,region_id,version_id,pipe_id,is_open,d_safe,speed_limit,max_capacity)
                   SELECT %s,%s,%s,%s,COALESCE(s.is_open,TRUE),%s,
                          COALESCE(s.speed_limit,15),COALESCE(s.max_capacity,50)
                   FROM (SELECT 1) seed
                   LEFT JOIN LATERAL (
                       SELECT is_open,speed_limit,max_capacity FROM pipe_status
                       WHERE region_id=%s AND version_id=%s AND pipe_id=%s AND time<=%s
                       ORDER BY time DESC LIMIT 1
                   ) s ON TRUE
                   ON CONFLICT (time,region_id,version_id,pipe_id)
                       DO UPDATE SET d_safe=EXCLUDED.d_safe""",
                values,
            )
            return cursor.rowcount


def get_latest_pipe_safety_and_speed(region_id=REGION_ID):
    """返回当前有效拓扑中各航道的 (pipe_id, d_safe, speed_limit) 元组列表。"""
    at = datetime.now(timezone.utc)
    with _read_transaction() as conn:
        version = _require_version(conn, region_id, at)
        states = _latest_rows_at(conn, "pipe_status", "pipe_id", region_id, version, at)
    return [(pid, row["d_safe"], row["speed_limit"]) for pid, row in states.items()]


@contextmanager
def _read_transaction():
    """提供同一数据库快照下的只读连接，供多表查询与历史取样使用。"""
    with connection(row_factory=dict_row) as conn:
        conn.execute("SET TRANSACTION ISOLATION LEVEL REPEATABLE READ, READ ONLY")
        yield conn


def _require_version(conn, region_id, at):
    """返回观测时刻的有效版本编号，找不到版本时抛出 LookupError。"""
    version = _version_at(conn, region_id, at)
    if version is None:
        raise LookupError(f"region {region_id} has no topology version at {at}")
    return version["version_id"]


def _graph_edges_at(conn, region_id, version_id, at):
    """按航道、引路及编号顺序，返回指定时刻的路段和动态状态。"""
    edges = []
    for kind, id_field, info, status in (
        (0, "pipe_id", "pipe_info", "pipe_status"),
        (1, "ramp_id", "ramp_info", "ramp_status"),
    ):
        rows = _topology_rows(conn, info, region_id, version_id)
        states = _latest_rows_at(conn, status, id_field, region_id, version_id, at)
        for row in rows:
            state = states.get(row[id_field], {})
            edge = {**row, "kind": kind, "edge_id": row[id_field],
                    "is_open": state.get("is_open", False)}
            if kind == 0:
                edge.update({name: state.get(name, default)
                             for name, default in (("d_safe", 20), ("speed_limit", 15),
                                                   ("max_capacity", 50))})
            edges.append(edge)
    return edges


def get_local_topology_adj(observation_time=None, *, region_id=REGION_ID):
    """返回指定时刻的 C++ 兼容本区邻接视图，包含道路状态、车道与车道流量。"""
    at = observation_time or datetime.now(timezone.utc)
    with _read_transaction() as conn:
        version = _require_version(conn, region_id, at)
        nodes = _topology_rows(conn, "node_info", region_id, version)
        edges = _graph_edges_at(conn, region_id, version, at)
        lanes = _topology_rows(conn, "lane_info", region_id, version)
        flows = _latest_rows_at(conn, "lane_flow", "lane_id", region_id, version, at)
    grouped_lanes = {}
    for lane in lanes:
        group = grouped_lanes.setdefault(lane["pipe_id"], [])
        flow = flows.get(lane["lane_id"])
        group.append({**lane, "lane_index": len(group), "latest_flow": flow,
                      "has_latest_flow": flow is not None})
    graph = {}
    for edge in edges:
        if edge["kind"] == 0:
            edge["lanes"] = grouped_lanes.get(edge["pipe_id"], [])
        graph.setdefault(edge["start_node_id"], []).append(edge)
    return {"observation_time": at, "region_id": region_id, "version_id": version,
            "nodes": {node["node_id"]: node for node in nodes}, "graph": graph}


def get_global_topology_adj(observation_time=None):
    """查询连接到的数据库内各区域的有效拓扑，返回按全局节点 ID 合并的 C++ 兼容图。"""
    at = observation_time or datetime.now(timezone.utc)
    nodes, graph = {}, {}
    with _read_transaction() as conn:
        versions = conn.execute(
            "SELECT DISTINCT ON (region_id) region_id, version_id FROM topology_version "
            "WHERE valid_from<=%s AND (valid_to IS NULL OR valid_to>%s) "
            "ORDER BY region_id, valid_from DESC, version_id DESC", (at, at),
        ).fetchall()
        for version in versions:
            rid, vid = version["region_id"], version["version_id"]
            for node in _topology_rows(conn, "node_info", rid, vid):
                merged = nodes.setdefault(node["node_id"], node)
                merged["region_ids"] = list(dict.fromkeys(merged["region_ids"] + node["region_ids"]))
            for edge in _graph_edges_at(conn, rid, vid, at):
                graph.setdefault(edge["start_node_id"], []).append(edge)
    return {"observation_time": at, "nodes": nodes, "graph": graph}


@dataclass(frozen=True)
class TimeSeriesQuery:
    """预测与回放的固定采样网格，由结束时刻、步数和相邻秒数定义。"""

    end_time: datetime | str | None = None
    steps: int = 1000
    step_seconds: int = 60


def _series_times(conn, region_id, query):
    """按最近观测或固定网格取样，返回升序时间列表。"""
    if isinstance(query, TimeSeriesQuery):
        if query.steps <= 0:
            return []
        if query.step_seconds <= 0:
            raise ValueError("step_seconds must be positive")
        end = query.end_time or datetime.now(timezone.utc)
        interval = timedelta(seconds=query.step_seconds)
        if isinstance(end, str):
            end = datetime.fromisoformat(end)
        return [end - i * interval for i in range(query.steps - 1, -1, -1)]
    if query <= 0:
        return []
    rows = conn.execute(
        """SELECT time FROM (
               SELECT time FROM pipe_flow WHERE region_id=%s UNION
               SELECT time FROM ramp_flow WHERE region_id=%s UNION
               SELECT time FROM lane_flow WHERE region_id=%s UNION
               SELECT time FROM pipe_status WHERE region_id=%s UNION
               SELECT time FROM ramp_status WHERE region_id=%s
           ) observed ORDER BY time DESC LIMIT %s""",
        (*[region_id] * 5, query),
    ).fetchall()
    return [row["time"] for row in reversed(rows)]


def _edge_adjacency(edges):
    """按路段顺序构造开放路段之间的有向邻接矩阵，返回 uint8 ndarray。"""
    import numpy as np

    adjacency = np.zeros((len(edges), len(edges)), dtype=np.uint8)
    outgoing = {}
    for index, edge in enumerate(edges):
        if edge["is_open"]:
            outgoing.setdefault(edge["start_node_id"], []).append(index)
    for index, edge in enumerate(edges):
        if edge["is_open"]:
            for target in outgoing.get(edge["end_node_id"], ()):
                adjacency[index, target] = 1
    return adjacency


def _flow_features(conn, edges, region_id, version_id, at):
    """按路段顺序读取时点流量并生成十列特征，缺失流量补零。"""
    flow = (
        _latest_rows_at(conn, "pipe_flow", "pipe_id", region_id, version_id, at),
        _latest_rows_at(conn, "ramp_flow", "ramp_id", region_id, version_id, at),
    )
    rows = []
    for edge in edges:
        kind = edge["kind"]
        values = flow[kind].get(edge["edge_id"], {})
        rows.append([
            *(float(values.get(field, 0)) for field in _FLOW_FIELDS),
            float(edge["is_open"]), float(edge["length"]), float(kind == 0),
            *(float(kind == 1 and edge["type"] == t) for t in range(3)),
        ])
    return rows


def _iter_series(query, region_id, *, topology, flow):
    """逐个产生指定区域的邻接、特征或组合样本，统一两者的路段顺序。"""
    with _read_transaction() as conn:
        for at in _series_times(conn, region_id, query):
            version = _version_at(conn, region_id, at)
            if version is None:
                if isinstance(query, TimeSeriesQuery):
                    continue
                raise LookupError(f"region {region_id} has no topology version at {at}")
            vid = version["version_id"]
            edges = _graph_edges_at(conn, region_id, vid, at)
            metadata = {"observation_time": at, "region_id": region_id, "version_id": vid,
                        "edge_order": [(edge["kind"], edge["edge_id"]) for edge in edges]}
            sample = {}
            if topology:
                sample["topology"] = {**metadata, "adjacency": _edge_adjacency(edges)}
            if flow:
                sample["flow"] = {**metadata, "features": _flow_features(conn, edges, region_id, vid, at)}
            yield sample


def iter_training_series(query=1000, *, region_id=REGION_ID):
    """按最近观测数或固定网格逐个返回对齐的邻接矩阵与流量特征样本。"""
    return _iter_series(query, region_id, topology=True, flow=True)


def get_training_series(query=1000, *, region_id=REGION_ID):
    """按最近观测数或固定网格返回邻接矩阵与流量特征样本列表。"""
    return list(iter_training_series(query, region_id=region_id))


def get_local_topology_matrix_series(query=1000, *, region_id=REGION_ID):
    """按最近观测数或固定网格返回本区开放路段的邻接矩阵序列。"""
    return [sample["topology"] for sample in _iter_series(query, region_id, topology=True, flow=False)]


def get_flow_feature_series(query=1000, *, region_id=REGION_ID):
    """按最近观测数或固定网格返回本区路段的十列流量特征序列。"""
    return [sample["flow"] for sample in _iter_series(query, region_id, topology=False, flow=True)]

