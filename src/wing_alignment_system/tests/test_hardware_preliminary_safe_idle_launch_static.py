import py_compile
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
SAFE_IDLE_LAUNCH = REPO_ROOT / "src/wing_alignment_system/launch/hardware_preliminary_safe_idle.launch.py"
SYSTEM_BRINGUP_FILE = REPO_ROOT / "src/wing_alignment_system/launch/system_bringup.launch.py"
RUN_ALL_FILE = REPO_ROOT / "src/wing_alignment_system/launch/run_all.launch.py"
MISSION_BRINGUP_FILE = REPO_ROOT / "src/wing_alignment_system/launch/mission_bringup.launch.py"
SCHEDULER_FILE = REPO_ROOT / "src/wing_alignment_system/wing_alignment_system/cmd_scheduler_node.py"
WATCHDOG_FILE = REPO_ROOT / "src/wing_alignment_system/wing_alignment_system/cmd_watchdog_node.py"
COORDINATOR_FILE = REPO_ROOT / "src/wing_alignment_system/wing_alignment_system/mission_coordinator.py"
VALIDATOR_FILE = REPO_ROOT / "src/wing_alignment_system/scripts/validate_bench_run_artifacts.py"


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_safe_idle_launch_file_exists_and_compiles():
    assert SAFE_IDLE_LAUNCH.exists()
    py_compile.compile(str(SAFE_IDLE_LAUNCH), doraise=True)


def test_safe_idle_launch_does_not_start_goto_pose_driver_or_force_monitor():
    text = _text(SAFE_IDLE_LAUNCH)

    assert "goto_pose_driver" not in text
    assert "force_monitor" not in text


def test_safe_idle_launch_starts_mission_chain_nodes_with_no_publish_params():
    text = _text(SAFE_IDLE_LAUNCH)

    assert 'executable="cmd_scheduler"' in text
    assert 'executable="cmd_watchdog"' in text
    assert 'executable="mission_coordinator"' in text
    assert '"safe_idle_no_publish": True' in text
    assert '"enable_execution_mode_output": enable_execution_mode_output' in text


def test_safe_idle_launch_can_attach_passive_recorder():
    text = _text(SAFE_IDLE_LAUNCH)

    assert 'start_passive_recorder' in text
    assert 'executable="passive_measurement_recorder"' in text
    assert 'measurement_log_dir' in text
    assert 'measurement_run_id' in text
    assert '"--run-id", measurement_run_id' in text
    assert '"--require-run-id"' in text


def test_safe_idle_no_publish_defaults_remain_false_in_nodes():
    assert 'declare_parameter("safe_idle_no_publish", False)' in _text(SCHEDULER_FILE)
    assert 'declare_parameter("safe_idle_no_publish", False)' in _text(WATCHDOG_FILE)
    assert "declare_parameter('safe_idle_no_publish', False)" in _text(COORDINATOR_FILE)


def test_existing_launches_do_not_reference_safe_idle_no_publish():
    assert 'safe_idle_no_publish' not in _text(SYSTEM_BRINGUP_FILE)
    assert 'safe_idle_no_publish' not in _text(RUN_ALL_FILE)
    assert 'safe_idle_no_publish' not in _text(MISSION_BRINGUP_FILE)


def test_safe_idle_does_not_change_h2_h3_validation_logic():
    text = _text(VALIDATOR_FILE)

    assert 'safe_idle' not in text
    assert 'choices=["", "H0", "H1", "H2", "H3", "H4", "H5"]' in text
