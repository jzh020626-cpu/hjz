import csv
import importlib.util
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_PATH = REPO_ROOT / "src/wing_alignment_system/scripts/summarize_run_kpi.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("summarize_run_kpi", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _write_csv(path: Path, fieldnames: list[str], rows: list[dict]):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def test_summarize_run_collects_minimal_fr_tac_p1_metrics(tmp_path: Path):
    module = _load_module()
    _write_csv(
        tmp_path / "mission_runtime_events.csv",
        [
            "run_id", "timestamp", "mission_state", "task_phase", "precision_mode", "robot_id", "team_scope",
            "Delta_eff_proxy_ms", "S_eff", "F_eff", "base_authority_weight", "slide_authority_weight",
            "authority_policy_mode", "freeze_state", "watchdog_or_safe_state", "docking_residual_proxy",
            "slide_residual_proxy", "support_residual_proxy", "safe_abort_reason", "event_type", "event_note",
        ],
        [
            {"run_id": "r1", "timestamp": "1.0", "mission_state": "WAIT_WING", "task_phase": "approach", "robot_id": "fleet", "event_type": "STATE_TRANSITION", "docking_residual_proxy": "", "slide_residual_proxy": "", "support_residual_proxy": ""},
            {"run_id": "r1", "timestamp": "6.0", "mission_state": "DONE", "task_phase": "transport", "robot_id": "fleet", "event_type": "TASK_OUTCOME", "docking_residual_proxy": "0.02", "slide_residual_proxy": "0.01", "support_residual_proxy": "0.03"},
        ],
    )
    _write_csv(
        tmp_path / "wrapper" / "r1" / "tracer1_cmd_channel_meta.csv",
        [
            "seq_id", "seq_monotonic", "payload_type", "payload_bytes", "source_send_timestamp", "source_clock_type",
            "wrapper_receive_timestamp", "receiver_clock_type", "receiver_node_time_ns", "wrapper_emit_timestamp",
            "inter_arrival_ms", "receiver_side_aoi_proxy_ms", "true_one_way_delay_ms", "delay_semantics",
            "deadline_met", "retry_count", "transmission_mode", "execution_mode", "aoi_ms", "effective_freshness",
            "phase", "task_progress", "scenario_id", "method_id", "robot_id", "source_mode", "delivery_expected", "wrapper_mode",
        ],
        [
            {"seq_id": "1", "seq_monotonic": "True", "payload_type": "geometry_msgs/msg/TwistStamped", "payload_bytes": "128", "source_send_timestamp": "1", "source_clock_type": "message_header_stamp", "wrapper_receive_timestamp": "1", "receiver_clock_type": "node", "receiver_node_time_ns": "1", "wrapper_emit_timestamp": "1", "inter_arrival_ms": "0", "receiver_side_aoi_proxy_ms": "100", "true_one_way_delay_ms": "n/a", "delay_semantics": "receiver_side_proxy_only", "deadline_met": "n/a", "retry_count": "0", "transmission_mode": "compact_update", "execution_mode": "degraded", "aoi_ms": "100", "effective_freshness": "0.9", "phase": "transport", "task_progress": "0.1", "scenario_id": "real-main", "method_id": "FR-TPO", "robot_id": "tracer1", "source_mode": "wrapped", "delivery_expected": "True", "wrapper_mode": "active"},
            {"seq_id": "3", "seq_monotonic": "True", "payload_type": "geometry_msgs/msg/TwistStamped", "payload_bytes": "0", "source_send_timestamp": "2", "source_clock_type": "message_header_stamp", "wrapper_receive_timestamp": "2", "receiver_clock_type": "node", "receiver_node_time_ns": "2", "wrapper_emit_timestamp": "2", "inter_arrival_ms": "50", "receiver_side_aoi_proxy_ms": "250", "true_one_way_delay_ms": "n/a", "delay_semantics": "receiver_side_proxy_only", "deadline_met": "n/a", "retry_count": "1", "transmission_mode": "skip_update", "execution_mode": "hold", "aoi_ms": "250", "effective_freshness": "0.5", "phase": "transport", "task_progress": "0.2", "scenario_id": "real-main", "method_id": "FR-TPO", "robot_id": "tracer1", "source_mode": "wrapped", "delivery_expected": "False", "wrapper_mode": "active"},
        ],
    )
    _write_csv(
        tmp_path / "cmd_safety" / "r1" / "mode_timeline_tracer1.csv",
        [
            "run_id", "timestamp", "robot_id", "seq", "transmission_mode", "execution_mode", "AoI_ms",
            "effective_freshness", "output_scale", "stop_reason", "watchdog_state", "cmd_v_in", "cmd_w_in",
            "cmd_v_out", "cmd_w_out", "t_source", "t_rx", "t_watchdog",
        ],
        [
            {"run_id": "r1", "timestamp": "1.0", "robot_id": "tracer1", "seq": "1", "transmission_mode": "compact_update", "execution_mode": "degraded", "AoI_ms": "100", "effective_freshness": "0.9", "output_scale": "0.5", "stop_reason": "execution_mode_degraded", "watchdog_state": "NORMAL", "cmd_v_in": "0.2", "cmd_w_in": "0.1", "cmd_v_out": "0.1", "cmd_w_out": "0.05", "t_source": "1.0", "t_rx": "1.0", "t_watchdog": "1.0"},
            {"run_id": "r1", "timestamp": "2.0", "robot_id": "tracer1", "seq": "3", "transmission_mode": "skip_update", "execution_mode": "safe_stop", "AoI_ms": "250", "effective_freshness": "0.5", "output_scale": "0.0", "stop_reason": "execution_mode_safe_stop", "watchdog_state": "MODE_SAFE_STOP", "cmd_v_in": "0.2", "cmd_w_in": "0.1", "cmd_v_out": "0.0", "cmd_w_out": "0.0", "t_source": "2.0", "t_rx": "2.0", "t_watchdog": "2.0"},
        ],
    )

    summary = module.summarize_run(tmp_path)

    assert summary["mission_success"] == 1
    assert summary["outcome"] == "success"
    assert summary["makespan_s"] == 5.0
    assert summary["tx_count"] == 2
    assert summary["payload_bytes_total"] == 128
    assert summary["retry_count"] == 1
    assert summary["packet_loss_proxy"] == 1
    assert summary["avg_AoI_ms"] == 175.0
    assert summary["max_AoI_ms"] == 250.0
    assert summary["degraded_time_ratio"] == 0.5
    assert summary["hold_time_ratio"] == 0.0
    assert summary["safe_stop_count"] == 1
    assert summary["safety_override_count"] == 2
    assert summary["cmd_stop_count"] == 0
    assert summary["emergency_stop_count"] == 0
    assert summary["communication_saving_ratio"] == 0.75
    assert summary["control_error_proxy_max"] == 0.03
    assert summary["final_docking_residual_proxy"] == 0.02
    assert summary["final_slide_residual_proxy"] == 0.01
    assert summary["final_support_residual_proxy"] == 0.03
    assert summary["final_control_residual_proxy"] == 0.03
    assert summary["phase_transport_tx_count"] == 2
    assert summary["phase_transport_payload_bytes"] == 128
    assert summary["phase_transport_avg_AoI_ms"] == 175.0
    assert summary["phase_transport_max_AoI_ms"] == 250.0
    assert summary["phase_transport_execution_mode_ratio_degraded"] == 0.5
    assert summary["phase_transport_execution_mode_ratio_hold"] == 0.5
    assert summary["phase_transport_execution_mode_ratio_normal"] == 0.0
    assert summary["phase_transport_execution_mode_ratio_safe_stop"] == 0.0
    assert summary["phase_transport_communication_saving_ratio"] == 0.75
