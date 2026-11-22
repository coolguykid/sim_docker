from pathlib import Path
from uav_sim.app import parse_optional_uav_ids
from uav_sim.simulation.plotting import print_comparison_report
from uav_sim.utils.trace_logger import TurnTraceLogger


def test_parse_optional_uav_ids_blank_disables_trace_logging():
    assert parse_optional_uav_ids("", 100) == []
    assert parse_optional_uav_ids("1, 5 9", 10) == [1, 5, 9]
    assert parse_optional_uav_ids("1, 99, bad", 10) == [1]


def test_turn_trace_logger_writes_position_speed_and_action(tmp_path):
    path = tmp_path / "trace.txt"
    logger = TurnTraceLogger([7], str(path))
    logger.log_turn(
        "1",
        "DIJKSTRA",
        12,
        {
            7: {
                "pipe_id": 3,
                "lane_id": 2,
                "pos_1d": 123.5,
                "speed": 18.0,
                "max_accel": 10.0,
                "max_decel": 10.0,
                "last_acceleration": -2.5,
                "state": "FLYING",
                "last_action": 4,
                "pos_3d": [1.0, 2.0, 100.0],
            }
        },
    )
    logger.close()
    text = Path(path).read_text(encoding="utf-8")
    assert "algo_id\tmode\tturn\tuav_id" in text
    assert (
        "1\tDIJKSTRA\t12\t7\tFLYING\t3\t2\t123.500\t18.000\t10.000\t10.000\t-2.500\t4\t1.000\t2.000\t100.000"
        in text
    )


def test_turn_trace_logger_writes_external_and_ramp_timing_fields(tmp_path):
    path = tmp_path / "trace-v2.txt"
    logger = TurnTraceLogger([7], str(path))
    logger.log_turn(
        "1",
        "DIJKSTRA",
        5,
        {
            7: {
                "pipe_id": 3,
                "external_pipe_id": 101,
                "lane_id": 0,
                "pos_1d": 12.0,
                "speed": 20.0,
                "state": "SWITCHING",
                "current_ramp_id": 4,
                "external_ramp_id": 9001,
                "ramp_length_m": 141.4,
                "ramp_remaining_seconds": 5.0,
                "pos_3d": [1.0, 2.0, 100.0],
            }
        },
    )
    logger.close()
    text = Path(path).read_text(encoding="utf-8")
    assert "external_pipe_id" in text.splitlines()[0]
    assert "external_ramp_id" in text.splitlines()[0]
    assert "\t101\t" in text.splitlines()[1]
    assert "\t9001\t141.400\t5.000" in text.splitlines()[1]


def test_comparison_report_prints_hard_collision_count(capsys):
    print_comparison_report(
        {
            "1": {
                "name_cn": "纯 Dijkstra + 规则避障",
                "time": 10,
                "avg_kl": 0.5,
                "violations": 2,
                "missed": 0,
                "metrics": {"hard_collisions": 0},
            }
        }
    )
    out = capsys.readouterr().out
    assert "碰撞次数" in out
    assert "0" in out
