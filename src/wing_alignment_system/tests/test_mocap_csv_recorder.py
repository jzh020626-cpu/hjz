from pathlib import Path
from types import SimpleNamespace
import csv
import math
import re
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wing_alignment_system.common_async_csv import AsyncCsvLogger
from wing_alignment_system.mocap_csv_recorder import CSV_FIELDS, build_csv_row, default_csv_path


def _pose_msg(
    *,
    stamp_sec=12,
    stamp_nanosec=345000000,
    x_mm=14540.0,
    y_raw=180.0,
    z_mm=3320.0,
    qx=0.0,
    qy=180.0,
    qz=0.0,
    qw=1.0,
):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp_sec, nanosec=stamp_nanosec)),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x_mm, y=y_raw, z=z_mm),
            orientation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
        ),
    )


def test_default_csv_path_uses_timestamped_tmp_location():
    path = default_csv_path("20260515_154500")

    assert path == "/tmp/three_tracer_mocap_20260515_154500.csv"


def test_build_csv_row_contains_raw_and_projected_fields():
    msg = _pose_msg()

    row = build_csv_row(
        robot_name="tracer1",
        topic="/Rigid17/pose",
        msg=msg,
        seq=7,
        recv_wall_time_sec=123.456,
    )

    assert list(row.keys()) == CSV_FIELDS
    assert row["robot_name"] == "tracer1"
    assert row["topic"] == "/Rigid17/pose"
    assert row["seq"] == 7
    assert row["ros_time_sec"] == 12.345
    assert row["recv_wall_time_sec"] == 123.456
    assert row["raw_x_mm"] == 14540.0
    assert row["raw_y_mm"] == 180.0
    assert row["raw_z_mm"] == 3320.0
    assert math.isclose(row["world_x_m"], 14.54, rel_tol=0.0, abs_tol=1e-9)
    assert math.isclose(row["world_y_m"], -3.32, rel_tol=0.0, abs_tol=1e-9)
    assert math.isclose(row["yaw_rad"], -math.pi, rel_tol=0.0, abs_tol=1e-9)
    assert row["yaw_deg"] == -180.0


def test_build_csv_row_supports_quaternion_yaw_mode():
    half = math.pi / 4.0
    msg = _pose_msg(
        y_raw=0.0,
        qx=0.0,
        qy=0.0,
        qz=math.sin(half),
        qw=math.cos(half),
    )

    row = build_csv_row(
        robot_name="tracer2",
        topic="/Rigid14/pose",
        msg=msg,
        seq=3,
        recv_wall_time_sec=1.0,
        mocap_yaw_mode="quaternion",
    )

    assert math.isclose(row["yaw_deg"], 90.0, rel_tol=0.0, abs_tol=1e-6)


def test_async_csv_logger_flushes_queued_rows_on_close():
    with tempfile.TemporaryDirectory() as tmpdir:
        csv_path = str(Path(tmpdir) / "mocap.csv")
        logger = AsyncCsvLogger(csv_path, CSV_FIELDS)
        logger.log(
            {
                "ros_time_sec": 1.0,
                "recv_wall_time_sec": 2.0,
                "robot_name": "tracer3",
                "topic": "/Rigid15/pose",
                "seq": 1,
                "raw_x_mm": 1.0,
                "raw_y_mm": 2.0,
                "raw_z_mm": 3.0,
                "raw_qx": 0.0,
                "raw_qy": 0.0,
                "raw_qz": 0.0,
                "raw_qw": 1.0,
                "world_x_m": 0.001,
                "world_y_m": -0.003,
                "yaw_rad": 0.0,
                "yaw_deg": 0.0,
            }
        )
        logger.close()

        with open(csv_path, newline="", encoding="utf-8") as fp:
            rows = list(csv.DictReader(fp))

    assert len(rows) == 1
    assert rows[0]["robot_name"] == "tracer3"
    assert rows[0]["topic"] == "/Rigid15/pose"
