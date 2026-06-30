import csv
import json
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = REPO_ROOT / "src/wing_alignment_system/scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import create_bench_run_manifest as manifest_script
import derive_hardware_preliminary_artifacts as derive_script


def _write_csv(path: Path, fieldnames: list[str], rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _read_csv(path: Path) -> list[dict]:
    with path.open("r", encoding="utf-8", newline="") as fp:
        return list(csv.DictReader(fp))


def _load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _update_manifest(manifest_path: Path, mutate_fn) -> dict:
    manifest = _load_json(manifest_path)
    mutate_fn(manifest)
    manifest["manifest_sha256"] = manifest_script._canonical_manifest_sha256(manifest)
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return manifest


def _build_layout(tmp_path: Path, run_id: str = "hw_run") -> tuple[dict, Path, Path, Path, Path, Path, Path]:
    manifest = manifest_script.create_manifest(
        out_dir=str(tmp_path / "manifest"),
        profile="nominal",
        baseline_mode="current_safe_default",
        evidence_class="hardware_preliminary",
        run_id=run_id,
        ros2_available=False,
        tc_available=False,
    )
    manifest_path = tmp_path / "manifest" / "run_manifest.json"
    capture_dir = Path(manifest["capture_artifacts"][0]["path"]).parent
    derived_dir = Path(manifest["derived_artifacts"][0]["path"]).parent
    scheduler_dir = Path(manifest["csv_paths"]["scheduler"][-1]).parent
    watchdog_dir = Path(manifest["csv_paths"]["watchdog"][0]).parent
    mission_dir = Path(manifest["csv_paths"]["mission"][0]).parent
    capture_dir.mkdir(parents=True, exist_ok=True)
    derived_dir.mkdir(parents=True, exist_ok=True)
    scheduler_dir.mkdir(parents=True, exist_ok=True)
    watchdog_dir.mkdir(parents=True, exist_ok=True)
    mission_dir.mkdir(parents=True, exist_ok=True)
    return manifest, manifest_path, capture_dir, derived_dir, scheduler_dir, watchdog_dir, mission_dir


def _write_scheduler(path: Path, rows: list[dict]) -> None:
    _write_csv(
        path,
        [
            "run_id",
            "robot_id",
            "task_phase",
            "t_tx",
            "robot",
            "seq",
            "command_id",
            "command_type",
            "v",
            "w",
            "reason",
            "scheduler_decision",
            "precision_mode",
        ],
        rows,
    )


def _write_rx(path: Path, rows: list[dict]) -> None:
    _write_csv(
        path,
        [
            "run_id",
            "robot_id",
            "command_id",
            "command_type",
            "t_source",
            "t_rx",
            "delta_net_proxy_ms",
        ],
        rows,
    )


def _write_ts(path: Path, rows: list[dict]) -> None:
    _write_csv(
        path,
        [
            "run_id",
            "robot_id",
            "command_id",
            "command_type",
            "t_source",
            "t_rx",
            "t_watchdog",
            "t",
            "age",
            "age_ms",
            "delta_net_proxy_ms",
            "delta_exec_proxy_ms",
            "delta_eff_proxy_ms",
            "queue_delay_proxy_ms",
            "v",
            "w",
            "state",
            "watchdog_action",
            "stale_reason",
            "emg",
        ],
        rows,
    )


def _write_mission(path: Path, rows: list[dict]) -> None:
    _write_csv(
        path,
        [
            "run_id",
            "timestamp",
            "mission_state",
            "task_phase",
            "precision_mode",
            "robot_id",
            "team_scope",
            "Delta_eff_proxy_ms",
            "S_eff",
            "F_eff",
            "base_authority_weight",
            "slide_authority_weight",
            "authority_policy_mode",
            "freeze_state",
            "watchdog_or_safe_state",
            "docking_residual_proxy",
            "slide_residual_proxy",
            "support_residual_proxy",
            "safe_abort_reason",
            "event_type",
            "event_note",
        ],
        rows,
    )


def _write_capture_minimum(capture_dir: Path, run_id: str) -> None:
    _write_csv(
        capture_dir / "clock_sync_status.csv",
        [
            "run_id",
            "host_id",
            "host_role",
            "monitor_source",
            "sync_available",
            "sync_verified",
            "offset_ms",
            "reference_clock",
            "time_base",
            "one_way_delay_allowed",
            "t_wall",
        ],
        [
            {
                "run_id": run_id,
                "host_id": "host-a",
                "host_role": "control_host",
                "monitor_source": "chrony",
                "sync_available": "true",
                "sync_verified": "false",
                "offset_ms": "",
                "reference_clock": "",
                "time_base": "system_wall",
                "one_way_delay_allowed": "false",
                "t_wall": "1.0",
            }
        ],
    )
    for name, header in (
        ("recorder_callback_timing.csv", ["run_id", "callback_owner", "stream_name", "topic", "t_callback_start_wall", "t_callback_end_wall", "duration_ms", "classification", "interpretation_note"]),
        ("chassis_command_samples.csv", ["run_id", "stream_name", "topic", "robot_id", "slide_id", "classification", "t_receive_wall", "t_receive_ros", "msg_source_stamp", "source_stamp_valid", "stamp_origin", "source_time_base", "frame_id", "command_kind", "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"]),
        ("slide_command_samples.csv", ["run_id", "stream_name", "topic", "robot_id", "slide_id", "classification", "t_receive_wall", "t_receive_ros", "msg_source_stamp", "source_stamp_valid", "stamp_origin", "source_time_base", "frame_id", "command_type", "x", "y", "z", "time", "is_relative", "vx", "vy", "vz", "can_id"]),
        ("delta_samples.csv", ["run_id", "stream_name", "topic", "robot_id", "slide_id", "classification", "t_receive_wall", "t_receive_ros", "msg_source_stamp", "source_stamp_valid", "stamp_origin", "source_time_base", "frame_id", "x", "y", "z"]),
    ):
        _write_csv(capture_dir / name, header, [])


def test_command_residence_primary_join_by_command_id(tmp_path):
    manifest, manifest_path, capture_dir, derived_dir, scheduler_dir, watchdog_dir, mission_dir = _build_layout(tmp_path, "join_primary")
    _write_capture_minimum(capture_dir, manifest["run_id"])
    _write_scheduler(
        scheduler_dir / "events.csv",
        [{"run_id": manifest["run_id"], "robot_id": "tracer1", "task_phase": "unknown", "t_tx": "1.000000", "robot": "tracer1", "seq": "11", "command_id": "11", "command_type": "cmd_vel", "v": "0.1", "w": "0.0", "reason": "periodic", "scheduler_decision": "periodic", "precision_mode": "0"}],
    )
    _write_rx(
        watchdog_dir / "rx_tracer1.csv",
        [{"run_id": manifest["run_id"], "robot_id": "tracer1", "command_id": "11", "command_type": "cmd_vel", "t_source": "1.000000", "t_rx": "1.040000", "delta_net_proxy_ms": "40.0"}],
    )
    _write_ts(
        watchdog_dir / "ts_tracer1.csv",
        [
            {"run_id": manifest["run_id"], "robot_id": "tracer1", "command_id": "11", "command_type": "cmd_vel", "t_source": "1.000000", "t_rx": "1.040000", "t_watchdog": "1.060000", "t": "1.060000", "age": "0.060", "age_ms": "60.0", "delta_net_proxy_ms": "40.0", "delta_exec_proxy_ms": "20.0", "delta_eff_proxy_ms": "60.0", "queue_delay_proxy_ms": "20.0", "v": "0.1", "w": "0.0", "state": "FRESH", "watchdog_action": "FRESH", "stale_reason": "", "emg": "0"},
            {"run_id": manifest["run_id"], "robot_id": "tracer1", "command_id": "11", "command_type": "cmd_vel", "t_source": "1.000000", "t_rx": "1.040000", "t_watchdog": "1.090000", "t": "1.090000", "age": "0.090", "age_ms": "90.0", "delta_net_proxy_ms": "40.0", "delta_exec_proxy_ms": "50.0", "delta_eff_proxy_ms": "90.0", "queue_delay_proxy_ms": "50.0", "v": "0.1", "w": "0.0", "state": "FRESH", "watchdog_action": "FRESH", "stale_reason": "", "emg": "0"},
        ],
    )

    report = derive_script.derive_artifacts(str(manifest_path), out_dir=str(derived_dir))
    rows = _read_csv(derived_dir / "command_residence_events.csv")

    assert report["join_success_counts"]["command_residence"] == 1
    assert rows[0]["join_key_type"] == "run_id+robot_id+command_id"
    assert rows[0]["join_quality"] == "exact_command_id"
    assert rows[0]["net_receive_proxy_ms"] == "40.000000"
    assert rows[0]["residence_apply_window_ms"] == "30.000000"
    assert rows[0]["classification"] == "proxy"


def test_command_residence_fallback_join_by_nearest_timestamp(tmp_path):
    manifest, manifest_path, capture_dir, derived_dir, scheduler_dir, watchdog_dir, _ = _build_layout(tmp_path, "join_nearest")
    _write_capture_minimum(capture_dir, manifest["run_id"])
    _write_scheduler(
        scheduler_dir / "events.csv",
        [{"run_id": manifest["run_id"], "robot_id": "tracer1", "task_phase": "unknown", "t_tx": "2.000000", "robot": "tracer1", "seq": "20", "command_id": "20", "command_type": "cmd_vel", "v": "0.1", "w": "0.0", "reason": "periodic", "scheduler_decision": "periodic", "precision_mode": "0"}],
    )
    _write_rx(
        watchdog_dir / "rx_tracer1.csv",
        [{"run_id": manifest["run_id"], "robot_id": "tracer1", "command_id": "99", "command_type": "cmd_vel", "t_source": "2.000000", "t_rx": "2.070000", "delta_net_proxy_ms": "70.0"}],
    )
    _write_ts(
        watchdog_dir / "ts_tracer1.csv",
        [{"run_id": manifest["run_id"], "robot_id": "tracer1", "command_id": "99", "command_type": "cmd_vel", "t_source": "2.000000", "t_rx": "2.070000", "t_watchdog": "2.090000", "t": "2.090000", "age": "0.090", "age_ms": "90.0", "delta_net_proxy_ms": "70.0", "delta_exec_proxy_ms": "20.0", "delta_eff_proxy_ms": "90.0", "queue_delay_proxy_ms": "20.0", "v": "0.1", "w": "0.0", "state": "FRESH", "watchdog_action": "FRESH", "stale_reason": "", "emg": "0"}],
    )

    derive_script.derive_artifacts(str(manifest_path), out_dir=str(derived_dir))
    rows = _read_csv(derived_dir / "command_residence_events.csv")

    assert rows[0]["join_key_type"] == "run_id+robot_id+nearest_timestamp"
    assert rows[0]["join_quality"] == "nearest_timestamp"


def test_phase_attribution_by_timestamp_join(tmp_path):
    manifest, manifest_path, capture_dir, derived_dir, scheduler_dir, _, mission_dir = _build_layout(tmp_path, "phase_join")
    _write_capture_minimum(capture_dir, manifest["run_id"])
    _write_scheduler(
        scheduler_dir / "events.csv",
        [{"run_id": manifest["run_id"], "robot_id": "tracer1", "task_phase": "unknown", "t_tx": "5.000000", "robot": "tracer1", "seq": "5", "command_id": "5", "command_type": "cmd_vel", "v": "0.1", "w": "0.0", "reason": "periodic", "scheduler_decision": "periodic", "precision_mode": "0"}],
    )
    _write_mission(
        mission_dir / "mission_runtime_events.csv",
        [{"run_id": manifest["run_id"], "timestamp": "4.950000", "mission_state": "ALIGN", "task_phase": "dock_align", "precision_mode": "1", "robot_id": "tracer1", "team_scope": "robot", "Delta_eff_proxy_ms": "", "S_eff": "", "F_eff": "", "base_authority_weight": "0.4", "slide_authority_weight": "0.6", "authority_policy_mode": "proxy", "freeze_state": "", "watchdog_or_safe_state": "", "docking_residual_proxy": "0.02", "slide_residual_proxy": "", "support_residual_proxy": "0.01", "safe_abort_reason": "", "event_type": "TASK_SNAPSHOT", "event_note": ""}],
    )

    derive_script.derive_artifacts(str(manifest_path), out_dir=str(derived_dir))
    rows = _read_csv(derived_dir / "phase_attributed_scheduler_events.csv")

    assert rows[0]["attributed_phase"] == "dock_align"
    assert rows[0]["attributed_local_state"] == "ALIGN"
    assert rows[0]["classification"] == "estimated"


def test_missing_mission_runtime_produces_partial_report_not_crash(tmp_path):
    manifest, manifest_path, capture_dir, derived_dir, scheduler_dir, _, _ = _build_layout(tmp_path, "phase_missing")
    _write_capture_minimum(capture_dir, manifest["run_id"])
    _write_scheduler(
        scheduler_dir / "events.csv",
        [{"run_id": manifest["run_id"], "robot_id": "tracer1", "task_phase": "unknown", "t_tx": "5.000000", "robot": "tracer1", "seq": "5", "command_id": "5", "command_type": "cmd_vel", "v": "0.1", "w": "0.0", "reason": "periodic", "scheduler_decision": "periodic", "precision_mode": "0"}],
    )

    report = derive_script.derive_artifacts(str(manifest_path), out_dir=str(derived_dir))
    rows = _read_csv(derived_dir / "phase_attributed_scheduler_events.csv")

    assert "phase_attribution:mission_runtime_events_missing" in report["unavailable_fields"]
    assert rows[0]["phase_join_quality"] == "unavailable_mission"


def test_control_loop_timing_computes_interarrival_proxy(tmp_path):
    manifest, manifest_path, capture_dir, derived_dir, _, _, _ = _build_layout(tmp_path, "loop_timing")
    _write_capture_minimum(capture_dir, manifest["run_id"])
    _write_csv(
        capture_dir / "chassis_command_samples.csv",
        ["run_id", "stream_name", "topic", "robot_id", "slide_id", "classification", "t_receive_wall", "t_receive_ros", "msg_source_stamp", "source_stamp_valid", "stamp_origin", "source_time_base", "frame_id", "command_kind", "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"],
        [
            {"run_id": manifest["run_id"], "stream_name": "cmd_vel", "topic": "/tracer1/cmd_vel", "robot_id": "tracer1", "slide_id": "", "classification": "feedback", "t_receive_wall": "1.0", "t_receive_ros": "1.000000", "msg_source_stamp": "", "source_stamp_valid": "false", "stamp_origin": "none", "source_time_base": "unknown", "frame_id": "", "command_kind": "cmd_vel", "linear_x": "0.1", "linear_y": "0.0", "linear_z": "0.0", "angular_x": "0.0", "angular_y": "0.0", "angular_z": "0.0"},
            {"run_id": manifest["run_id"], "stream_name": "cmd_vel", "topic": "/tracer1/cmd_vel", "robot_id": "tracer1", "slide_id": "", "classification": "feedback", "t_receive_wall": "1.05", "t_receive_ros": "1.050000", "msg_source_stamp": "", "source_stamp_valid": "false", "stamp_origin": "none", "source_time_base": "unknown", "frame_id": "", "command_kind": "cmd_vel", "linear_x": "0.1", "linear_y": "0.0", "linear_z": "0.0", "angular_x": "0.0", "angular_y": "0.0", "angular_z": "0.0"},
            {"run_id": manifest["run_id"], "stream_name": "cmd_vel", "topic": "/tracer1/cmd_vel", "robot_id": "tracer1", "slide_id": "", "classification": "feedback", "t_receive_wall": "1.15", "t_receive_ros": "1.150000", "msg_source_stamp": "", "source_stamp_valid": "false", "stamp_origin": "none", "source_time_base": "unknown", "frame_id": "", "command_kind": "cmd_vel", "linear_x": "0.1", "linear_y": "0.0", "linear_z": "0.0", "angular_x": "0.0", "angular_y": "0.0", "angular_z": "0.0"},
        ],
    )

    derive_script.derive_artifacts(str(manifest_path), out_dir=str(derived_dir))
    rows = _read_csv(derived_dir / "control_loop_timing.csv")

    assert rows[0]["sample_count"] == "3"
    assert rows[0]["max_interarrival_ms"] == "100.000000"
    assert rows[0]["classification"] == "proxy"


def test_recorder_callback_timing_is_labeled_proxy_not_controller_truth(tmp_path):
    manifest, manifest_path, capture_dir, derived_dir, scheduler_dir, watchdog_dir, _ = _build_layout(tmp_path, "callback_proxy")
    _write_capture_minimum(capture_dir, manifest["run_id"])
    _write_csv(
        capture_dir / "recorder_callback_timing.csv",
        ["run_id", "callback_owner", "stream_name", "topic", "t_callback_start_wall", "t_callback_end_wall", "duration_ms", "classification", "interpretation_note"],
        [
            {"run_id": manifest["run_id"], "callback_owner": "passive_recorder", "stream_name": "cmd_vel", "topic": "/tracer1/cmd_vel", "t_callback_start_wall": "1.0", "t_callback_end_wall": "1.01", "duration_ms": "10.0", "classification": "proxy", "interpretation_note": "not_controller_callback_timing"}
        ],
    )
    _write_ts(
        watchdog_dir / "ts_tracer1.csv",
        [{"run_id": manifest["run_id"], "robot_id": "tracer1", "command_id": "1", "command_type": "cmd_vel", "t_source": "1.0", "t_rx": "1.01", "t_watchdog": "1.02", "t": "1.02", "age": "0.02", "age_ms": "20.0", "delta_net_proxy_ms": "10.0", "delta_exec_proxy_ms": "10.0", "delta_eff_proxy_ms": "20.0", "queue_delay_proxy_ms": "10.0", "v": "0.1", "w": "0.0", "state": "FRESH", "watchdog_action": "FRESH", "stale_reason": "", "emg": "0"}],
    )

    derive_script.derive_artifacts(str(manifest_path), out_dir=str(derived_dir))
    rows = _read_csv(derived_dir / "executor_backlog_proxy.csv")

    assert rows[0]["classification"] == "proxy"
    assert rows[0]["interpretation_note"] == "not internal executor queue-depth truth"


def test_authority_proxy_leaves_ratio_blank_when_limits_unavailable(tmp_path):
    manifest, manifest_path, capture_dir, derived_dir, _, _, mission_dir = _build_layout(tmp_path, "authority_missing")
    _write_capture_minimum(capture_dir, manifest["run_id"])
    _write_csv(
        capture_dir / "chassis_command_samples.csv",
        ["run_id", "stream_name", "topic", "robot_id", "slide_id", "classification", "t_receive_wall", "t_receive_ros", "msg_source_stamp", "source_stamp_valid", "stamp_origin", "source_time_base", "frame_id", "command_kind", "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"],
        [{"run_id": manifest["run_id"], "stream_name": "cmd_vel", "topic": "/tracer1/cmd_vel", "robot_id": "tracer1", "slide_id": "", "classification": "feedback", "t_receive_wall": "1.0", "t_receive_ros": "1.000000", "msg_source_stamp": "", "source_stamp_valid": "false", "stamp_origin": "none", "source_time_base": "unknown", "frame_id": "", "command_kind": "cmd_vel", "linear_x": "1.0", "linear_y": "0.0", "linear_z": "0.0", "angular_x": "0.0", "angular_y": "0.0", "angular_z": "0.5"}],
    )
    _write_csv(
        capture_dir / "slide_command_samples.csv",
        ["run_id", "stream_name", "topic", "robot_id", "slide_id", "classification", "t_receive_wall", "t_receive_ros", "msg_source_stamp", "source_stamp_valid", "stamp_origin", "source_time_base", "frame_id", "command_type", "x", "y", "z", "time", "is_relative", "vx", "vy", "vz", "can_id"],
        [{"run_id": manifest["run_id"], "stream_name": "slide_compensation_ref", "topic": "/huatai1_compensation_ref", "robot_id": "", "slide_id": "huatai1", "classification": "feedback", "t_receive_wall": "1.0", "t_receive_ros": "1.000000", "msg_source_stamp": "", "source_stamp_valid": "false", "stamp_origin": "none", "source_time_base": "unknown", "frame_id": "", "command_type": "compensation", "x": "", "y": "", "z": "", "time": "", "is_relative": "", "vx": "0.2", "vy": "0.0", "vz": "0.0", "can_id": "1"}],
    )
    _write_mission(
        mission_dir / "mission_runtime_events.csv",
        [{"run_id": manifest["run_id"], "timestamp": "1.000000", "mission_state": "ALIGN", "task_phase": "dock_align", "precision_mode": "1", "robot_id": "tracer1", "team_scope": "robot", "Delta_eff_proxy_ms": "", "S_eff": "", "F_eff": "", "base_authority_weight": "0.4", "slide_authority_weight": "0.6", "authority_policy_mode": "proxy", "freeze_state": "", "watchdog_or_safe_state": "", "docking_residual_proxy": "", "slide_residual_proxy": "", "support_residual_proxy": "", "safe_abort_reason": "", "event_type": "TASK_SNAPSHOT", "event_note": ""}],
    )

    derive_script.derive_artifacts(str(manifest_path), out_dir=str(derived_dir))
    rows = _read_csv(derived_dir / "authority_proxy_timeseries.csv")

    assert rows[0]["authority_proxy_chassis_ratio"] == ""
    assert rows[0]["authority_proxy_slide_ratio"] == ""
    assert "unavailable" in rows[0]["unavailable_reason"]


def test_authority_proxy_computes_ratio_when_limits_are_provided(tmp_path):
    manifest, manifest_path, capture_dir, derived_dir, _, _, mission_dir = _build_layout(tmp_path, "authority_present")
    _write_capture_minimum(capture_dir, manifest["run_id"])
    _update_manifest(
        manifest_path,
        lambda data: data.update({"authority_proxy_limits": {"v_max": 2.0, "w_max": 1.0, "vx_lim": 0.5, "vy_lim": 0.5, "vz_lim": 0.5}}),
    )
    _write_csv(
        capture_dir / "chassis_command_samples.csv",
        ["run_id", "stream_name", "topic", "robot_id", "slide_id", "classification", "t_receive_wall", "t_receive_ros", "msg_source_stamp", "source_stamp_valid", "stamp_origin", "source_time_base", "frame_id", "command_kind", "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"],
        [{"run_id": manifest["run_id"], "stream_name": "cmd_vel", "topic": "/tracer1/cmd_vel", "robot_id": "tracer1", "slide_id": "", "classification": "feedback", "t_receive_wall": "1.0", "t_receive_ros": "1.000000", "msg_source_stamp": "", "source_stamp_valid": "false", "stamp_origin": "none", "source_time_base": "unknown", "frame_id": "", "command_kind": "cmd_vel", "linear_x": "1.0", "linear_y": "0.0", "linear_z": "0.0", "angular_x": "0.0", "angular_y": "0.0", "angular_z": "0.0"}],
    )
    _write_csv(
        capture_dir / "slide_command_samples.csv",
        ["run_id", "stream_name", "topic", "robot_id", "slide_id", "classification", "t_receive_wall", "t_receive_ros", "msg_source_stamp", "source_stamp_valid", "stamp_origin", "source_time_base", "frame_id", "command_type", "x", "y", "z", "time", "is_relative", "vx", "vy", "vz", "can_id"],
        [{"run_id": manifest["run_id"], "stream_name": "slide_compensation_ref", "topic": "/huatai1_compensation_ref", "robot_id": "", "slide_id": "huatai1", "classification": "feedback", "t_receive_wall": "1.0", "t_receive_ros": "1.000000", "msg_source_stamp": "", "source_stamp_valid": "false", "stamp_origin": "none", "source_time_base": "unknown", "frame_id": "", "command_type": "compensation", "x": "", "y": "", "z": "", "time": "", "is_relative": "", "vx": "0.25", "vy": "0.0", "vz": "0.0", "can_id": "1"}],
    )
    _write_mission(
        mission_dir / "mission_runtime_events.csv",
        [{"run_id": manifest["run_id"], "timestamp": "1.000000", "mission_state": "ALIGN", "task_phase": "dock_align", "precision_mode": "1", "robot_id": "tracer1", "team_scope": "robot", "Delta_eff_proxy_ms": "", "S_eff": "", "F_eff": "", "base_authority_weight": "0.4", "slide_authority_weight": "0.6", "authority_policy_mode": "proxy", "freeze_state": "", "watchdog_or_safe_state": "", "docking_residual_proxy": "", "slide_residual_proxy": "", "support_residual_proxy": "", "safe_abort_reason": "", "event_type": "TASK_SNAPSHOT", "event_note": ""}],
    )

    derive_script.derive_artifacts(str(manifest_path), out_dir=str(derived_dir))
    rows = _read_csv(derived_dir / "authority_proxy_timeseries.csv")

    assert rows[0]["authority_proxy_chassis_ratio"] == "0.500000"
    assert rows[0]["authority_proxy_slide_ratio"] == "0.500000"


def test_terminal_residual_proxy_uses_delta_fallback_when_mission_residual_absent(tmp_path):
    manifest, manifest_path, capture_dir, derived_dir, _, _, _ = _build_layout(tmp_path, "delta_fallback")
    _write_capture_minimum(capture_dir, manifest["run_id"])
    _write_csv(
        capture_dir / "delta_samples.csv",
        ["run_id", "stream_name", "topic", "robot_id", "slide_id", "classification", "t_receive_wall", "t_receive_ros", "msg_source_stamp", "source_stamp_valid", "stamp_origin", "source_time_base", "frame_id", "x", "y", "z"],
        [
            {"run_id": manifest["run_id"], "stream_name": "qr_delta", "topic": "/tracer1/wing_alignment/delta", "robot_id": "tracer1", "slide_id": "", "classification": "feedback", "t_receive_wall": "1.0", "t_receive_ros": "1.0", "msg_source_stamp": "", "source_stamp_valid": "false", "stamp_origin": "none", "source_time_base": "unknown", "frame_id": "", "x": "0.03", "y": "0.04", "z": "0.00"},
            {"run_id": manifest["run_id"], "stream_name": "qr_delta", "topic": "/tracer1/wing_alignment/delta", "robot_id": "tracer1", "slide_id": "", "classification": "feedback", "t_receive_wall": "1.1", "t_receive_ros": "1.1", "msg_source_stamp": "", "source_stamp_valid": "false", "stamp_origin": "none", "source_time_base": "unknown", "frame_id": "", "x": "0.00", "y": "0.03", "z": "0.04"},
        ],
    )

    derive_script.derive_artifacts(str(manifest_path), out_dir=str(derived_dir))
    rows = _read_csv(derived_dir / "terminal_residual_proxy.csv")

    assert rows[0]["source_used"] == "delta_samples.csv:last_window"
    assert rows[0]["terminal_residual_proxy_max"] == "0.050000"
    assert rows[0]["classification"] == "proxy"


def test_one_way_delay_is_blocked_when_clock_sync_is_unverified(tmp_path):
    manifest, manifest_path, capture_dir, derived_dir, _, _, _ = _build_layout(tmp_path, "clock_block")
    _write_capture_minimum(capture_dir, manifest["run_id"])

    report = derive_script.derive_artifacts(str(manifest_path), out_dir=str(derived_dir))

    assert report["one_way_delay_reported"] is False
    assert report["clock_sync_policy_result"]["sync_verified_any"] is False


def test_all_output_csvs_have_headers_even_when_inputs_are_missing(tmp_path):
    manifest, manifest_path, _, derived_dir, _, _, _ = _build_layout(tmp_path, "headers_only")

    derive_script.derive_artifacts(str(manifest_path), out_dir=str(derived_dir))

    for name in (
        "command_residence_events.csv",
        "control_loop_timing.csv",
        "executor_backlog_proxy.csv",
        "authority_proxy_timeseries.csv",
        "terminal_residual_proxy.csv",
        "phase_attributed_scheduler_events.csv",
    ):
        path = derived_dir / name
        assert path.exists()
        assert path.read_text(encoding="utf-8").splitlines()[0]



def test_derivation_report_marks_own_artifact_exists_and_no_measured_labels(tmp_path):
    manifest, manifest_path, capture_dir, derived_dir, _, _, _ = _build_layout(tmp_path, "self_summary")
    _write_capture_minimum(capture_dir, manifest["run_id"])
    _write_csv(
        capture_dir / "recorder_callback_timing.csv",
        ["run_id", "callback_owner", "stream_name", "topic", "t_callback_start_wall", "t_callback_end_wall", "duration_ms", "classification", "interpretation_note"],
        [
            {
                "run_id": manifest["run_id"],
                "callback_owner": "passive_recorder",
                "stream_name": "raw_qr",
                "topic": "/tracer1/object_position",
                "t_callback_start_wall": "1.0",
                "t_callback_end_wall": "1.01",
                "duration_ms": "10.0",
                "classification": "proxy",
                "interpretation_note": "not_controller_callback_timing",
            }
        ],
    )

    derive_script.derive_artifacts(str(manifest_path), out_dir=str(derived_dir))
    report_path = derived_dir / "derivation_report.json"
    report = _load_json(report_path)

    assert report_path.exists()
    assert report["output_artifacts"]["derivation_report.json"]["exists"] is True
    for name in (
        "command_residence_events.csv",
        "control_loop_timing.csv",
        "executor_backlog_proxy.csv",
        "authority_proxy_timeseries.csv",
        "terminal_residual_proxy.csv",
        "phase_attributed_scheduler_events.csv",
    ):
        rows = _read_csv(derived_dir / name)
        for row in rows:
            if "classification" in row and row["classification"]:
                assert row["classification"] != "measured"


def test_derivation_report_contains_required_boundary_statements(tmp_path):
    manifest, manifest_path, capture_dir, derived_dir, _, _, _ = _build_layout(tmp_path, "boundary_report")
    _write_capture_minimum(capture_dir, manifest["run_id"])

    derive_script.derive_artifacts(str(manifest_path), out_dir=str(derived_dir))
    report = _load_json(derived_dir / "derivation_report.json")

    assert report["evidence_class"] == "hardware_preliminary"
    assert report["derived_artifact_boundary"]["all_residence_fields_are_proxy"] is True
    assert report["derived_artifact_boundary"]["callback_timing_is_passive_recorder_proxy"] is True
    assert report["derived_artifact_boundary"]["bandwidth_throughput_are_context_only"] is True
