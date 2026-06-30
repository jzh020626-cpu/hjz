import json
import sys
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = REPO_ROOT / "src/wing_alignment_system/scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import create_bench_run_manifest as manifest_script


def _load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _force_monitor_params(merged_yaml: dict, slide: str) -> dict:
    return merged_yaml[f"force_monitor_{slide}"]["ros__parameters"]


def test_hardware_preliminary_manifest_generation_writes_merged_params_file(tmp_path):
    cmd_root = tmp_path / "cmd_logs"
    mission_root = tmp_path / "mission_logs"
    manifest = manifest_script.create_manifest(
        out_dir=str(tmp_path),
        profile="nominal",
        evidence_class="hardware_preliminary",
        run_id="hw_manifest_test",
        cmd_safety_log_root=str(cmd_root),
        mission_log_root=str(mission_root),
        ros2_available=False,
        tc_available=False,
    )

    manifest_path = tmp_path / "run_manifest.json"
    manifest_on_disk = json.loads(manifest_path.read_text(encoding="utf-8"))
    overlay_path = Path(manifest_on_disk["overlay_params_path"])
    merged_path = Path(manifest_on_disk["merged_params_path"])
    merged_text = merged_path.read_text(encoding="utf-8")
    merged_yaml = _load_yaml(merged_path)

    assert overlay_path.exists()
    assert merged_path.exists()
    assert manifest["merged_params_path"] == str(merged_path)
    assert manifest["overlay_params_path"] == str(overlay_path)
    assert manifest_on_disk["merged_params_path"] == str(merged_path)
    assert manifest_on_disk["overlay_params_path"] == str(overlay_path)
    assert any(f"config_file:={merged_path}" in command for command in manifest_on_disk["ros2_commands"])
    assert all(f"config_file:={overlay_path}" not in command for command in manifest_on_disk["ros2_commands"])

    assert merged_yaml["tracer1"]["goto_pose_node"]["ros__parameters"]["robot_name"] == "tracer1"
    assert merged_yaml["tracer2"]["goto_pose_node"]["ros__parameters"]["robot_name"] == "tracer2"
    assert merged_yaml["tracer3"]["goto_pose_node"]["ros__parameters"]["robot_name"] == "tracer3"
    assert merged_yaml["tracer1"]["cmd_watchdog"]["ros__parameters"]["run_id"] == manifest["run_id"]
    assert merged_yaml["tracer2"]["cmd_watchdog"]["ros__parameters"]["log_dir"] == str(cmd_root)
    assert merged_yaml["tracer3"]["cmd_watchdog"]["ros__parameters"]["baseline_mode"] == manifest["baseline_mode"]
    assert merged_yaml["cmd_scheduler"]["ros__parameters"]["run_id"] == manifest["run_id"]
    assert merged_yaml["cmd_scheduler"]["ros__parameters"]["log_dir"] == str(cmd_root)
    assert merged_yaml["mission_coordinator"]["ros__parameters"]["bench_log_dir"] == str(mission_root)
    assert merged_yaml["mission_coordinator"]["ros__parameters"]["mission_log_dir"] == str(mission_root)
    assert merged_yaml["mission_coordinator"]["ros__parameters"]["terminal_rack_close_exception_enable"] is True
    assert merged_yaml["mission_coordinator"]["ros__parameters"]["terminal_rack_close_exception_exempt_pairs"] == [
        "tracer1,tracer2",
        "tracer2,tracer3",
        "tracer1,tracer3",
    ]
    assert isinstance(
        merged_yaml["mission_coordinator"]["ros__parameters"]["terminal_rack_close_exception_exempt_pairs"],
        list,
    )
    assert all(
        isinstance(value, str)
        for value in merged_yaml["mission_coordinator"]["ros__parameters"]["terminal_rack_close_exception_exempt_pairs"]
    )

    force1 = _force_monitor_params(merged_yaml, "huatai1")
    force2 = _force_monitor_params(merged_yaml, "huatai2")
    force3 = _force_monitor_params(merged_yaml, "huatai3")
    assert force1["topic_force_in"] == "/huatai1_force"
    assert force2["topic_force_in"] == "/huatai2_force"
    assert force3["topic_force_in"] == "/huatai3_force"
    assert force1["topic_force_filtered_out"] == "/huatai1_force_filtered"
    assert force2["topic_force_filtered_out"] == "/huatai2_force_filtered"
    assert force3["topic_force_filtered_out"] == "/huatai3_force_filtered"
    assert force1["topic_stop_out"] == "/huatai1/force_contact"
    assert force2["topic_stop_out"] == "/huatai2/force_contact"
    assert force3["topic_stop_out"] == "/huatai3/force_contact"
    assert force1["force_invalid_streak_warn"] == 3
    assert force2["force_invalid_streak_warn"] == 3
    assert force3["force_invalid_streak_warn"] == 3
    assert force1["force_invalid_streak_unavailable"] == 5
    assert force2["force_invalid_streak_unavailable"] == 5
    assert force3["force_invalid_streak_unavailable"] == 5
    assert force1["hold_last_valid_filtered"] is False
    assert force2["hold_last_valid_filtered"] is False
    assert force3["hold_last_valid_filtered"] is False
    assert force1["force_timeout_enable"] is False
    assert force2["force_timeout_enable"] is False
    assert force3["force_timeout_enable"] is False

    assert "huatai1" in merged_text
    assert "huatai2" in merged_text
    assert "huatai3" in merged_text
    assert "/Rigid17/pose" in merged_text
    assert "/Rigid14/pose" in merged_text
    assert "/Rigid15/pose" in merged_text
    assert manifest_script._canonical_manifest_sha256(manifest_on_disk) == manifest_on_disk["manifest_sha256"]


def test_non_hardware_manifest_generation_keeps_overlay_only_behavior(tmp_path):
    manifest = manifest_script.create_manifest(
        out_dir=str(tmp_path),
        profile="nominal",
        evidence_class="ros2_bench",
        run_id="bench_manifest_test",
        ros2_available=False,
        tc_available=False,
    )

    assert "merged_params_path" not in manifest
    assert "overlay_params_path" not in manifest
    assert (tmp_path / "bench_params_overlay.yaml").exists()
    assert not (tmp_path / "hardware_preliminary_merged_params.yaml").exists()
    assert any("bench_params_overlay.yaml" in command for command in manifest["ros2_commands"])
