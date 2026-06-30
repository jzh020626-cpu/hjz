import csv
import json
import os
import py_compile
import stat
import subprocess
import sys
import types

import pytest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = REPO_ROOT / "src/wing_alignment_system/scripts"
SRC_ROOT = REPO_ROOT / "src"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

import create_bench_run_manifest as manifest_script
import validate_bench_run_artifacts as validator
from test_hardware_preliminary_validation import _write_csv


RECORDER_FILE = REPO_ROOT / "src/wing_alignment_system/wing_alignment_system/passive_measurement_recorder.py"
SETUP_FILE = REPO_ROOT / "src/wing_alignment_system/setup.py"
CLOCK_SCRIPT = REPO_ROOT / "src/wing_alignment_system/scripts/hardware_clock_sync_monitor.sh"
NETWORK_SCRIPT = REPO_ROOT / "src/wing_alignment_system/scripts/hardware_network_monitor.sh"
SYSTEM_BRINGUP_FILE = REPO_ROOT / "src/wing_alignment_system/launch/system_bringup.launch.py"


def _read_rows(path: Path) -> list[dict]:
    with path.open("r", encoding="utf-8", newline="") as fp:
        return list(csv.DictReader(fp))


def _make_executable(path: Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")
    path.chmod(path.stat().st_mode | stat.S_IXUSR)


def _is_numeric_or_placeholder(value: str) -> bool:
    text = str(value).strip().lower()
    if text in {'', 'n/a'}:
        return True
    try:
        float(text)
    except ValueError:
        return False
    return True


def _write_recorder_h0_files(capture_dir: Path, run_id: str) -> None:
    _write_csv(
        capture_dir / "recorder_health.csv",
        [
            "run_id",
            "logger_name",
            "stream_name",
            "rows_written",
            "rows_dropped",
            "queue_depth",
            "max_queue_depth",
            "last_write_error",
            "control_publishers_count",
            "user_defined_publishers_count",
            "service_clients_count",
            "user_defined_services_count",
            "ros_infrastructure_endpoints",
            "t_wall",
            "t_ros",
        ],
        [
            {
                "run_id": run_id,
                "logger_name": "passive_measurement_recorder",
                "stream_name": "startup",
                "rows_written": "1",
                "rows_dropped": "0",
                "queue_depth": "0",
                "max_queue_depth": "0",
                "last_write_error": "",
                "control_publishers_count": "0",
                "user_defined_publishers_count": "0",
                "service_clients_count": "0",
                "user_defined_services_count": "0",
                "ros_infrastructure_endpoints": "[]",
                "t_wall": "1.0",
                "t_ros": "1.0",
            }
        ],
    )
    _write_csv(
        capture_dir / "recorder_topic_status.csv",
        [
            "run_id",
            "topic",
            "message_type",
            "configured",
            "observed",
            "row_count",
            "last_observed_wall",
            "last_observed_ros",
            "source_stamp_observed",
            "qos_note",
            "classification",
            "required_gate",
        ],
        [
            {
                "run_id": run_id,
                "topic": "/tracer1/object_position",
                "message_type": "geometry_msgs/msg/PoseStamped",
                "configured": "true",
                "observed": "false",
                "row_count": "0",
                "last_observed_wall": "",
                "last_observed_ros": "",
                "source_stamp_observed": "false",
                "qos_note": "best_effort",
                "classification": "feedback",
                "required_gate": "H1",
            }
        ],
    )


def test_passive_measurement_recorder_file_exists_and_stays_read_only():
    text = RECORDER_FILE.read_text(encoding="utf-8")

    assert RECORDER_FILE.exists()
    assert "create_publisher(" not in text
    assert "create_client(" not in text
    assert "create_service(" not in text
    assert "enable_rosout=False" in text
    assert "start_parameter_services=False" in text


def test_passive_measurement_recorder_compiles():
    py_compile.compile(str(RECORDER_FILE), doraise=True)


def test_setup_registers_passive_measurement_recorder_entry_point():
    text = SETUP_FILE.read_text(encoding="utf-8")

    assert "passive_measurement_recorder = wing_alignment_system.passive_measurement_recorder:main" in text


def test_system_bringup_declares_opt_in_passive_recorder_args_with_safe_defaults():
    text = SYSTEM_BRINGUP_FILE.read_text(encoding="utf-8")

    assert "'start_passive_recorder'" in text
    assert "default_value='false'" in text
    assert "'measurement_log_dir'" in text
    assert "default_value='~/.ros/hardware_preliminary_logs'" in text
    assert "'measurement_run_id'" in text
    assert "'measurement_robots'" in text
    assert "'measurement_slides'" in text


def test_system_bringup_uses_conditional_passive_recorder_node_with_expected_arguments():
    text = SYSTEM_BRINGUP_FILE.read_text(encoding="utf-8")

    assert "Node(" in text
    assert "package='wing_alignment_system'" in text
    assert "executable='passive_measurement_recorder'" in text
    assert "condition=IfCondition(start_passive_recorder)" in text
    assert "'--run-id'" in text
    assert "'--require-run-id'" in text
    assert "'--out-dir'" in text
    assert "'--config-file'" in text
    assert "'--robots'" in text
    assert "'--slides'" in text
    assert "'--duration-sec'" not in text


def test_system_bringup_does_not_launch_host_monitor_scripts():
    text = SYSTEM_BRINGUP_FILE.read_text(encoding="utf-8")

    assert "hardware_clock_sync_monitor.sh" not in text
    assert "hardware_network_monitor.sh" not in text


def test_clock_monitor_writes_unavailable_row_without_working_clock_tools(tmp_path):
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    for name in ("chronyc", "pmc", "ptp4l", "timedatectl", "ntpq"):
        _make_executable(fake_bin / name, "#!/bin/bash\nexit 127\n")

    out_dir = tmp_path / "clock"
    env = dict(os.environ)
    env["PATH"] = f"{fake_bin}:{env.get('PATH', '')}"
    subprocess.run(
        [
            "/bin/bash",
            str(CLOCK_SCRIPT),
            "--run-id",
            "clock_run",
            "--host-id",
            "host-a",
            "--host-role",
            "control_host",
            "--out-dir",
            str(out_dir),
            "--once",
        ],
        check=True,
        env=env,
    )

    rows = _read_rows(out_dir / "clock_sync_status.csv")
    assert len(rows) >= 1
    assert rows[0]["run_id"] == "clock_run"
    assert rows[0]["one_way_delay_allowed"].lower() == "false"


def test_network_monitor_writes_files_when_ping_target_is_unreachable(tmp_path):
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    _make_executable(
        fake_bin / "ping",
        "#!/bin/bash\n"
        "echo 'PING $2 ($2): 56 data bytes'\n"
        "echo ''\n"
        "echo '--- $2 ping statistics ---'\n"
        "echo '1 packets transmitted, 0 received, 100% packet loss, time 0ms'\n"
        "exit 1\n",
    )

    out_dir = tmp_path / "network"
    env = dict(os.environ)
    env["PATH"] = f"{fake_bin}:{env.get('PATH', '')}"
    subprocess.run(
        [
            "/bin/bash",
            str(NETWORK_SCRIPT),
            "--run-id",
            "net_run",
            "--host-id",
            "host-a",
            "--iface",
            "lo",
            "--ping-target",
            "198.51.100.1",
            "--peer-role",
            "gateway",
            "--out-dir",
            str(out_dir),
            "--duration-sec",
            "1",
        ],
        check=True,
        env=env,
    )

    ping_rows = _read_rows(out_dir / "network_ping_samples.csv")
    iface_rows = _read_rows(out_dir / "interface_counters.csv")
    assert len(ping_rows) >= 1
    assert len(iface_rows) >= 1
    assert ping_rows[0]["run_id"] == "net_run"



def test_network_monitor_parses_linux_ping_success_fields(tmp_path):
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    _make_executable(
        fake_bin / "ping",
        "#!/bin/bash\n"
        "host=\"${@: -1}\"\n"
        "echo 'PING ${host} (${host}) 56(84) bytes of data.'\n"
        "echo '64 bytes from ${host}: icmp_seq=1 ttl=64 time=2.610 ms'\n"
        "echo ''\n"
        "echo '--- ${host} ping statistics ---'\n"
        "echo '1 packets transmitted, 1 received, 0% packet loss, time 0ms'\n"
        "echo 'rtt min/avg/max/mdev = 2.610/2.610/2.610/0.000 ms'\n",
    )

    out_dir = tmp_path / "network_success"
    env = dict(os.environ)
    env["PATH"] = f"{fake_bin}:{env.get('PATH', '')}"
    subprocess.run(
        [
            "/bin/bash",
            str(NETWORK_SCRIPT),
            "--run-id",
            "net_success",
            "--host-id",
            "host-a",
            "--iface",
            "lo",
            "--ping-target",
            "127.0.0.1",
            "--peer-role",
            "gateway",
            "--out-dir",
            str(out_dir),
            "--duration-sec",
            "1",
        ],
        check=True,
        env=env,
    )

    row = _read_rows(out_dir / "network_ping_samples.csv")[0]
    assert float(row["packet_loss_percent"]) == 0.0
    assert float(row["rtt_min_ms"]) == 2.61
    assert float(row["rtt_avg_ms"]) == 2.61
    assert float(row["rtt_max_ms"]) == 2.61
    assert _is_numeric_or_placeholder(row["jitter_ms"])
    assert _is_numeric_or_placeholder(row["rtt_mdev_ms"])
    assert row["packet_loss_percent"] != "loss"
    assert not row["jitter_ms"].startswith("mdev")
    assert row["status"] == "ok"



def test_network_monitor_parses_100_percent_packet_loss_without_literal_words(tmp_path):
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    _make_executable(
        fake_bin / "ping",
        "#!/bin/bash\n"
        "host=\"${@: -1}\"\n"
        "echo 'PING ${host} (${host}) 56(84) bytes of data.'\n"
        "echo ''\n"
        "echo '--- ${host} ping statistics ---'\n"
        "echo '1 packets transmitted, 0 received, 100% packet loss, time 0ms'\n"
        "exit 1\n",
    )

    out_dir = tmp_path / "network_loss"
    env = dict(os.environ)
    env["PATH"] = f"{fake_bin}:{env.get('PATH', '')}"
    subprocess.run(
        [
            "/bin/bash",
            str(NETWORK_SCRIPT),
            "--run-id",
            "net_loss",
            "--host-id",
            "host-a",
            "--iface",
            "lo",
            "--ping-target",
            "198.51.100.1",
            "--peer-role",
            "gateway",
            "--out-dir",
            str(out_dir),
            "--duration-sec",
            "1",
        ],
        check=True,
        env=env,
    )

    row = _read_rows(out_dir / "network_ping_samples.csv")[0]
    assert float(row["packet_loss_percent"]) == 100.0
    for key in ("rtt_ms", "rtt_min_ms", "rtt_avg_ms", "rtt_max_ms", "jitter_ms", "rtt_mdev_ms"):
        assert _is_numeric_or_placeholder(row[key])
        assert row[key] not in {"loss", "mdev = 2.605"}
    assert row["status"] != ""
    assert row["status"] != "ok"



def test_network_monitor_records_ping_failure_row_with_status(tmp_path):
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    _make_executable(
        fake_bin / "ping",
        "#!/bin/bash\n"
        "echo 'ping: unknown host example.invalid' >&2\n"
        "exit 2\n",
    )

    out_dir = tmp_path / "network_failed"
    env = dict(os.environ)
    env["PATH"] = f"{fake_bin}:{env.get('PATH', '')}"
    subprocess.run(
        [
            "/bin/bash",
            str(NETWORK_SCRIPT),
            "--run-id",
            "net_failed",
            "--host-id",
            "host-a",
            "--iface",
            "lo",
            "--ping-target",
            "example.invalid",
            "--peer-role",
            "gateway",
            "--out-dir",
            str(out_dir),
            "--duration-sec",
            "1",
        ],
        check=True,
        env=env,
    )

    row = _read_rows(out_dir / "network_ping_samples.csv")[0]
    for key in ("packet_loss_percent", "rtt_ms", "rtt_min_ms", "rtt_avg_ms", "rtt_max_ms", "jitter_ms", "rtt_mdev_ms"):
        assert _is_numeric_or_placeholder(row[key])
    assert row["status"] != ""
    assert "unknown host" in row["note"]


def test_network_monitor_aligns_two_targets_with_two_peer_roles(tmp_path):
    out_dir = tmp_path / "network_multi"
    subprocess.run(
        [
            "/bin/bash",
            str(NETWORK_SCRIPT),
            "--run-id",
            "net_multi",
            "--host-id",
            "host-a",
            "--iface",
            "lo",
            "--ping-target",
            "127.0.0.1",
            "--peer-role",
            "gateway",
            "--ping-target",
            "127.0.0.1",
            "--peer-role",
            "robot_control",
            "--out-dir",
            str(out_dir),
            "--duration-sec",
            "1",
        ],
        check=True,
    )

    ping_rows = _read_rows(out_dir / "network_ping_samples.csv")
    assert [row["peer_role"] for row in ping_rows[:2]] == ["gateway", "robot_control"]


def test_write_minimal_h0_artifacts_creates_recorder_filesystem_artifacts(tmp_path):
    import wing_alignment_system.passive_measurement_recorder as recorder_module

    out_dir = tmp_path / "recorder"

    paths = recorder_module.write_minimal_h0_artifacts(
        run_id="recorder_run",
        out_dir=str(out_dir),
        robots=["tracer1"],
        slides=["huatai1"],
        config_file="",
    )

    assert Path(paths["recorder_health.csv"]).exists()
    assert Path(paths["recorder_topic_status.csv"]).exists()
    assert Path(paths["recorder_callback_timing.csv"]).exists()
    health_rows = _read_rows(out_dir / "recorder_health.csv")
    topic_rows = _read_rows(out_dir / "recorder_topic_status.csv")
    assert len(health_rows) >= 1
    assert len(topic_rows) >= 1
    assert all(row["run_id"] == "recorder_run" for row in health_rows)
    assert all(row["run_id"] == "recorder_run" for row in topic_rows)
    assert all(row["observed"] == "false" for row in topic_rows)


def test_duration_mode_uses_bounded_executor_loop_in_source():
    text = RECORDER_FILE.read_text(encoding="utf-8")

    assert "SingleThreadedExecutor" in text
    assert "spin_once(" in text
    assert "time.monotonic()" in text


def test_passive_measurement_recorder_duration_arg_and_endpoint_policy_helper():
    import wing_alignment_system.passive_measurement_recorder as recorder_module

    parsed = recorder_module._parse_args(
        ["--run-id", "run", "--out-dir", "/tmp/out", "--duration-sec", "10"]
    )
    endpoint_status = recorder_module.build_endpoint_policy_status(7)

    assert parsed.duration_sec == 10
    assert endpoint_status["control_publishers_count"] == 0
    assert endpoint_status["user_defined_publishers_count"] == 0
    assert endpoint_status["service_clients_count"] == 0
    assert endpoint_status["user_defined_services_count"] == 0
    assert endpoint_status["configured_subscriptions_count"] == 7


def test_passive_measurement_recorder_requires_non_empty_run_id_when_guard_enabled():
    import wing_alignment_system.passive_measurement_recorder as recorder_module

    with pytest.raises(SystemExit):
        recorder_module._parse_args(
            ["--run-id", "", "--require-run-id", "--out-dir", "/tmp/out"]
        )



def test_non_blocking_csv_writer_close_is_idempotent(tmp_path):
    import wing_alignment_system.passive_measurement_recorder as recorder_module

    writer = recorder_module._NonBlockingCsvWriter(str(tmp_path / "rows.csv"), ["run_id", "value"])
    writer.log({"run_id": "writer_run", "value": "1"})
    writer.close()
    writer.close()

    rows = _read_rows(tmp_path / "rows.csv")
    assert rows[0]["run_id"] == "writer_run"
    assert rows[0]["value"] == "1"



def test_shutdown_passive_measurement_recorder_is_idempotent_on_interrupted_cleanup(monkeypatch):
    import wing_alignment_system.passive_measurement_recorder as recorder_module

    shutdown_calls = []

    class FakeNode:
        def __init__(self):
            self.close_calls = 0
            self.destroy_calls = 0

        def close(self):
            self.close_calls += 1

        def destroy_node(self):
            self.destroy_calls += 1
            raise ValueError("list.remove(x): x not in list")

    node = FakeNode()
    monkeypatch.setattr(
        recorder_module,
        "rclpy",
        types.SimpleNamespace(ok=lambda: True, shutdown=lambda: shutdown_calls.append("shutdown")),
    )

    recorder_module.shutdown_passive_measurement_recorder(node)
    recorder_module.shutdown_passive_measurement_recorder(node)

    assert node.close_calls == 1
    assert node.destroy_calls == 1
    assert shutdown_calls == ["shutdown"]


def test_h0_passes_with_monitor_scripts_and_recorder_fixture_files(tmp_path):
    manifest = manifest_script.create_manifest(
        out_dir=str(tmp_path / "manifest"),
        profile="nominal",
        evidence_class="hardware_preliminary",
        ros2_available=False,
        tc_available=False,
    )
    manifest_path = tmp_path / "manifest" / "run_manifest.json"
    capture_dir = Path(manifest["capture_artifacts"][0]["path"]).parent
    capture_dir.mkdir(parents=True, exist_ok=True)

    subprocess.run(
        [
            "/bin/bash",
            str(CLOCK_SCRIPT),
            "--run-id",
            manifest["run_id"],
            "--host-id",
            "host-a",
            "--host-role",
            "control_host",
            "--out-dir",
            str(capture_dir),
            "--once",
        ],
        check=True,
    )
    subprocess.run(
        [
            "/bin/bash",
            str(NETWORK_SCRIPT),
            "--run-id",
            manifest["run_id"],
            "--host-id",
            "host-a",
            "--iface",
            "lo",
            "--ping-target",
            "127.0.0.1",
            "--peer-role",
            "gateway",
            "--out-dir",
            str(capture_dir),
            "--duration-sec",
            "1",
        ],
        check=True,
    )
    _write_recorder_h0_files(capture_dir, manifest["run_id"])

    report = validator.validate_manifest(str(manifest_path), gate="H0")

    assert report["status"] == "passed_with_warnings"
    assert any("non-startup capture rows" in warning for warning in report["warnings"])
    assert all("external sensor rows" not in warning for warning in report["warnings"])


def test_warning_wording_uses_non_startup_capture_rows(tmp_path):
    manifest = manifest_script.create_manifest(
        out_dir=str(tmp_path / "manifest"),
        profile="nominal",
        evidence_class="hardware_preliminary",
        ros2_available=False,
        tc_available=False,
    )
    manifest_path = tmp_path / "manifest" / "run_manifest.json"
    capture_dir = Path(manifest["capture_artifacts"][0]["path"]).parent
    capture_dir.mkdir(parents=True, exist_ok=True)
    _write_csv(
        capture_dir / "clock_sync_status.csv",
        ["run_id", "host_id", "host_role", "monitor_source", "sync_available", "sync_verified", "offset_ms", "reference_clock", "time_base", "one_way_delay_allowed", "t_wall"],
        [{"run_id": manifest["run_id"], "host_id": "host-a", "host_role": "control_host", "monitor_source": "unavailable", "sync_available": "false", "sync_verified": "false", "offset_ms": "", "reference_clock": "", "time_base": "system_wall", "one_way_delay_allowed": "false", "t_wall": "1.0"}],
    )
    _write_csv(
        capture_dir / "network_ping_samples.csv",
        ["run_id", "host_id", "peer_host", "peer_role", "iface", "rtt_ms", "jitter_ms", "packet_loss_percent", "t_wall"],
        [{"run_id": manifest["run_id"], "host_id": "host-a", "peer_host": "127.0.0.1", "peer_role": "gateway", "iface": "lo", "rtt_ms": "0.1", "jitter_ms": "0.0", "packet_loss_percent": "0.0", "t_wall": "1.0"}],
    )
    _write_csv(
        capture_dir / "interface_counters.csv",
        ["run_id", "host_id", "iface", "rx_bytes", "tx_bytes", "rx_dropped", "tx_dropped", "rx_errors", "tx_errors", "t_wall"],
        [{"run_id": manifest["run_id"], "host_id": "host-a", "iface": "lo", "rx_bytes": "1", "tx_bytes": "1", "rx_dropped": "0", "tx_dropped": "0", "rx_errors": "0", "tx_errors": "0", "t_wall": "1.0"}],
    )
    _write_recorder_h0_files(capture_dir, manifest["run_id"])

    report = validator.validate_manifest(str(manifest_path), gate="H0")

    assert any("non-startup capture rows" in warning for warning in report["warnings"])
    assert all("external sensor rows" not in warning for warning in report["warnings"])


def test_passive_measurement_recorder_source_avoids_raw_executor_spin():
    text = RECORDER_FILE.read_text(encoding="utf-8")

    assert "run_passive_measurement_recorder(node, executor)" in text
    assert "executor.spin()" not in text


def test_run_passive_measurement_recorder_treats_wait_set_context_invalidation_as_normal_shutdown(monkeypatch):
    import wing_alignment_system.passive_measurement_recorder as recorder_module

    class FakeRCLError(RuntimeError):
        pass

    class FakeExecutor:
        def __init__(self):
            self.calls = 0

        def spin_once(self, timeout_sec=0.1):
            self.calls += 1
            raise FakeRCLError("failed to initialize wait set: the given context is not valid, either rcl_init() was not called or rcl_shutdown() was called.")

    node = types.SimpleNamespace(duration_sec=0.0)
    executor = FakeExecutor()

    monkeypatch.setattr(recorder_module, "RCLError", FakeRCLError)
    monkeypatch.setattr(recorder_module.rclpy, "ok", lambda: True)

    recorder_module.run_passive_measurement_recorder(node, executor)

    assert executor.calls == 1


def test_run_passive_measurement_recorder_reraises_unexpected_rcl_error(monkeypatch):
    import wing_alignment_system.passive_measurement_recorder as recorder_module

    class FakeRCLError(RuntimeError):
        pass

    class FakeExecutor:
        def spin_once(self, timeout_sec=0.1):
            raise FakeRCLError("unexpected wait set failure")

    node = types.SimpleNamespace(duration_sec=0.0)
    executor = FakeExecutor()

    monkeypatch.setattr(recorder_module, "RCLError", FakeRCLError)
    monkeypatch.setattr(recorder_module.rclpy, "ok", lambda: True)

    with pytest.raises(FakeRCLError, match="unexpected wait set failure"):
        recorder_module.run_passive_measurement_recorder(node, executor)
