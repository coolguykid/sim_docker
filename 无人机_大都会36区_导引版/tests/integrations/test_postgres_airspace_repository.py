from datetime import datetime, timezone

import pytest

from uav_sim.integrations.postgres_airspace import (
    PostgresAirspaceRepository,
    PostgresTopologyReadError,
)


class FakeCursor:
    def __init__(self, columns, rows):
        self.description = [(name,) for name in columns]
        self._rows = rows

    def fetchall(self):
        return list(self._rows)


class FakeConnection:
    def __init__(self, tables, fail_table=None):
        self.tables = tables
        self.fail_table = fail_table
        self.statements = []

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        return False

    def execute(self, sql, params=None):
        self.statements.append((sql, params))
        if sql.startswith("SET TRANSACTION"):
            return FakeCursor([], [])
        table = next(name for name in self.tables if '"{}"'.format(name) in sql)
        if table == self.fail_table:
            raise RuntimeError("database unavailable")
        columns, rows = self.tables[table]
        return FakeCursor(columns, rows)


def _tables():
    now = datetime(2026, 7, 21, tzinfo=timezone.utc)
    return {
        "region_info": (["Region_id", "Update_time"], [(0, now)]),
        "layer_info": (["Layer_id", "Update_time"], [(11, now)]),
        "pipe_info": (["Pipe_id", "Update_time"], [(101, now)]),
        "lane_info": (["Lane_id", "Pipe_id", "Update_time"], [(1, 101, now)]),
        "ramp_info": (["Ramp_id", "Is_open", "Update_time"], [(9001, 1, now)]),
    }


def test_repository_reads_all_topology_tables_in_one_read_only_transaction():
    connection = FakeConnection(_tables())
    repository = PostgresAirspaceRepository(
        connect=lambda: connection,
        schema="public",
    )

    result = repository.fetch_topology_rows()

    sql = [statement for statement, _params in connection.statements]
    assert sql[0] == "SET TRANSACTION ISOLATION LEVEL REPEATABLE READ READ ONLY"
    assert {name for name in _tables() if any('"{}"'.format(name) in item for item in sql)} == set(_tables())
    assert all(item.startswith(("SET TRANSACTION", "SELECT")) for item in sql)
    assert result["pipes"] == [{"pipe_id": 101, "update_time": result["pipes"][0]["update_time"]}]
    assert isinstance(result["derived_topology_version"], int)
    assert result["derived_topology_version"] > 0


def test_repository_wraps_read_error_with_table_name():
    connection = FakeConnection(_tables(), fail_table="pipe_info")
    repository = PostgresAirspaceRepository(connect=lambda: connection)

    with pytest.raises(PostgresTopologyReadError, match="pipe_info"):
        repository.fetch_topology_rows()


def test_repository_rejects_non_whitelisted_table_name():
    repository = PostgresAirspaceRepository(
        connect=lambda: FakeConnection(_tables())
    )

    with pytest.raises(ValueError, match="不允许"):
        repository._fetch_rows(FakeConnection(_tables()), "uav_data")
