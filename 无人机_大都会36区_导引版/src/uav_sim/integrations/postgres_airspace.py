"""Read-only PostgreSQL repository for airspace topology and ramp states."""
import hashlib
import json
import os
import re


class PostgresTopologyReadError(RuntimeError):
    pass


class PostgresAirspaceRepository:
    TOPOLOGY_TABLES = (
        "region_info",
        "layer_info",
        "pipe_info",
        "lane_info",
        "ramp_info",
    )
    _IDENTIFIER = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")

    def __init__(self, connect, schema="public"):
        if not self._IDENTIFIER.match(str(schema)):
            raise ValueError("非法 PostgreSQL schema: {}".format(schema))
        self.connect = connect
        self.schema = str(schema)

    @classmethod
    def from_config(cls, config):
        config = dict(config or {})
        try:
            import psycopg
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "PostgreSQL 模式需要安装 psycopg[binary]"
            ) from exc

        def env_value(name_key, direct_key, default=None):
            if config.get(direct_key) is not None:
                return config[direct_key]
            env_name = config.get(name_key)
            return os.environ.get(env_name, default) if env_name else default

        kwargs = {
            "host": env_value("host_env", "host", "localhost"),
            "port": int(config.get("port", 5432)),
            "dbname": env_value("database_env", "database"),
            "user": env_value("user_env", "user"),
            "password": env_value("password_env", "password"),
        }
        connect_timeout = config.get("connect_timeout")
        if connect_timeout is not None:
            kwargs["connect_timeout"] = int(connect_timeout)
        return cls(
            connect=lambda: psycopg.connect(**kwargs),
            schema=config.get("schema", "public"),
        )

    def fetch_topology_rows(self):
        table_results = {}
        with self.connect() as connection:
            connection.execute(
                "SET TRANSACTION ISOLATION LEVEL REPEATABLE READ READ ONLY"
            )
            for table in self.TOPOLOGY_TABLES:
                table_results[table] = self._fetch_rows(connection, table)
        version = self._derive_version(table_results)
        return {
            "regions": table_results["region_info"],
            "layers": table_results["layer_info"],
            "pipes": table_results["pipe_info"],
            "lanes": table_results["lane_info"],
            "ramps": table_results["ramp_info"],
            "derived_topology_version": version,
        }

    def fetch_ramp_states(self, since_source_version, region_id=None):
        with self.connect() as connection:
            connection.execute(
                "SET TRANSACTION ISOLATION LEVEL REPEATABLE READ READ ONLY"
            )
            sql = (
                'SELECT ramp_id, is_open, update_time, region_id '
                'FROM "{}"."ramp_info"'.format(self.schema)
            )
            params = None
            if region_id is not None:
                sql += ' WHERE region_id = %s'
                params = (int(region_id),)
            try:
                cursor = connection.execute(sql, params)
                rows = self._rows_from_cursor(cursor)
            except Exception as exc:
                raise PostgresTopologyReadError(
                    "读取 PostgreSQL 表 ramp_info 失败: {}".format(exc)
                ) from exc
        return {
            "rows": rows,
            "derived_source_version": self._derive_version({
                "ramp_info": rows,
            }),
        }

    def _fetch_rows(self, connection, table):
        if table not in self.TOPOLOGY_TABLES:
            raise ValueError("不允许读取 PostgreSQL 表: {}".format(table))
        sql = 'SELECT * FROM "{}"."{}"'.format(self.schema, table)
        try:
            return self._rows_from_cursor(connection.execute(sql))
        except Exception as exc:
            raise PostgresTopologyReadError(
                "读取 PostgreSQL 表 {} 失败: {}".format(table, exc)
            ) from exc

    @staticmethod
    def _rows_from_cursor(cursor):
        columns = []
        for item in cursor.description or ():
            name = item.name if hasattr(item, "name") else item[0]
            columns.append(str(name).strip().lower())
        return [
            dict(zip(columns, values))
            for values in cursor.fetchall()
        ]

    @staticmethod
    def _derive_version(table_results):
        canonical = []
        for table in sorted(table_results):
            rows = list(table_results[table] or ())
            canonical.append((
                table,
                len(rows),
                sorted(
                    json.dumps(row, sort_keys=True, default=str, ensure_ascii=False)
                    for row in rows
                ),
            ))
        payload = json.dumps(
            canonical, sort_keys=True, ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        return int.from_bytes(hashlib.sha256(payload).digest()[:8], "big")
