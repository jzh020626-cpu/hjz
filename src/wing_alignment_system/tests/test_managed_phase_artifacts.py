from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
SETUP_PY = REPO_ROOT / "src/wing_alignment_system/setup.py"
SYSTEM_BRINGUP = REPO_ROOT / "src/wing_alignment_system/launch/system_bringup.launch.py"
RUN_ALL = REPO_ROOT / "src/wing_alignment_system/launch/run_all.launch.py"
MISSION_BRINGUP = REPO_ROOT / "src/wing_alignment_system/launch/mission_bringup.launch.py"
LAUNCH_BUILDERS = REPO_ROOT / "src/wing_alignment_system/wing_alignment_system/launch_builders.py"
MISSION_PARAMS = REPO_ROOT / "src/wing_alignment_system/config/mission_params.yaml"


def test_setup_registers_mission_phase_client_entry_point():
    text = SETUP_PY.read_text(encoding="utf-8")

    assert "mission_phase_client = wing_alignment_system.mission_phase_client:main" in text


def test_setup_registers_mocap_csv_recorder_entry_point():
    text = SETUP_PY.read_text(encoding="utf-8")

    assert "mocap_csv_recorder = wing_alignment_system.mocap_csv_recorder:main" in text


def test_system_bringup_launch_exists_and_enables_managed_phase_mode():
    text = SYSTEM_BRINGUP.read_text(encoding="utf-8")

    assert "managed_phase_mode" in text


def test_launch_artifacts_expose_resume_phase_parameter():
    system_text = SYSTEM_BRINGUP.read_text(encoding="utf-8")
    run_all_text = RUN_ALL.read_text(encoding="utf-8")
    builders_text = LAUNCH_BUILDERS.read_text(encoding="utf-8")

    assert "resume_phase" in system_text
    assert "resume_phase" in run_all_text
    assert "resume_phase_cfg" in builders_text


def test_system_bringup_exposes_log_controls():
    system_text = SYSTEM_BRINGUP.read_text(encoding="utf-8")
    builders_text = LAUNCH_BUILDERS.read_text(encoding="utf-8")

    assert "node_output" in system_text
    assert "driver_log_level" in system_text
    assert "coordinator_log_level" in system_text
    assert "driver_log_level_cfg" in builders_text
    assert "coordinator_log_level_cfg" in builders_text


def test_mission_params_expose_final_entry_controls():
    text = MISSION_PARAMS.read_text(encoding="utf-8")

    assert "final_entry_mode" in text
    assert "final_entry_gate_mode" in text
    assert "final_entry_profile_code" in text


def test_mission_params_lower_final_entry_speed_defaults():
    text = MISSION_PARAMS.read_text(encoding="utf-8")

    assert text.count("final_entry_v_nominal: 0.18") == 3
    assert text.count("final_entry_v_max: 0.18") == 3
    assert text.count("final_entry_v_slow_max: 0.08") == 3
    assert text.count("final_entry_v_min_far: 0.03") == 3


def test_mission_params_expose_motion_avoidance_controls():
    text = MISSION_PARAMS.read_text(encoding="utf-8")

    assert "avoidance_enable" in text
    assert "avoidance_min_center_dist_m" in text
    assert "avoidance_resume_center_dist_m" in text


def test_mission_bringup_launch_uses_staggered_timer_startup():
    text = MISSION_BRINGUP.read_text(encoding="utf-8")

    assert "TimerAction" in text
    assert "period=4.0" in text
    assert "period=8.0" in text


def test_mission_params_expose_return_home_startup_hold_controls():
    text = MISSION_PARAMS.read_text(encoding="utf-8")

    assert "startup_hold_sec" in text
