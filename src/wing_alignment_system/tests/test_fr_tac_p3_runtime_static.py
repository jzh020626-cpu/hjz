import py_compile
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
RUNTIME_LAUNCH = REPO_ROOT / "src/wing_alignment_system/launch/fr_tac_p3_single_robot_runtime.launch.py"
PRELIGHT_SCRIPT = REPO_ROOT / "src/wing_alignment_system/scripts/run_fr_tac_p3_single_robot_preflight.sh"
ESTOP_PUBLISHER = REPO_ROOT / "src/wing_alignment_system/wing_alignment_system/p3c_emergency_stop_publisher.py"
SETUP_FILE = REPO_ROOT / "src/wing_alignment_system/setup.py"


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_runtime_launch_exists_and_compiles():
    assert RUNTIME_LAUNCH.exists()
    py_compile.compile(str(RUNTIME_LAUNCH), doraise=True)


def test_runtime_launch_is_tracer1_only_and_starts_runtime_chain():
    text = _text(RUNTIME_LAUNCH)

    assert 'namespace="tracer1"' in text
    assert 'executable="cmd_watchdog"' in text
    assert 'executable="p3c_emergency_stop_publisher"' in text
    assert '"safe_idle_no_publish": ParameterValue(safe_idle_no_publish, value_type=bool)' in text
    assert '"enable_execution_mode_output": ParameterValue(enable_execution_mode_output, value_type=bool)' in text
    assert '"topic": "/wing_alignment/emergency_stop"' in text
    assert '"shutdown_publish_true": True' in text


def test_preflight_sections_and_remediation_match_runtime_gates():
    text = _text(PRELIGHT_SCRIPT)

    assert 'header "base_robot_online"' in text
    assert 'header "mission_coordinator_absent"' in text
    assert 'header "watchdog_chain_online"' in text
    assert 'header "emergency_stop_online"' in text
    assert 'TOPIC_CMD_IN="/${ROBOT}/cmd_vel_stamped"' in text
    assert 'TOPIC_ES="/wing_alignment/emergency_stop"' in text
    assert 'RUNTIME_RUN_ID="p3c_runtime"' in text
    assert 'start ${WATCHDOG_NODE} or a runtime publisher/subscriber chain that exposes ${TOPIC_CMD_IN}' in text
    assert 'start P3-C runtime so ${WATCHDOG_NODE} runs with run_id=${RUNTIME_RUN_ID}' in text
    assert 'start ${ESTOP_NODE} so ${TOPIC_ES} exists before real motion' in text
    assert 'publisher is not ${ESTOP_NODE}' in text
    assert 'stop dummy publishers and start ${ESTOP_NODE} for the P3-C runtime gate' in text
    assert 'grep -E \'mission_coordinator|mission_dispatcher|mission_gate\'' in text
    assert 'grep -q "Publisher count: 0"' in text


def test_estop_publisher_entrypoint_is_registered():
    text = _text(SETUP_FILE)

    assert "p3c_emergency_stop_publisher = wing_alignment_system.p3c_emergency_stop_publisher:main" in text


def test_estop_publisher_defaults_to_false_and_publishes_true_on_shutdown():
    assert ESTOP_PUBLISHER.exists()
    py_compile.compile(str(ESTOP_PUBLISHER), doraise=True)

    text = _text(ESTOP_PUBLISHER)
    assert 'declare_parameter("topic", "/wing_alignment/emergency_stop")' in text
    assert 'declare_parameter("default_state", False)' in text
    assert 'declare_parameter("shutdown_publish_true", True)' in text
    assert 'declare_parameter("stop_file", "/tmp/p3c_emergency_stop.flag")' in text

CONTROLLED_PY = REPO_ROOT / "src/wing_alignment_system/scripts/run_fr_tac_p3_single_robot_controlled.py"


def test_controlled_runner_c1_split_into_c1a_c1b():
    text = _text(CONTROLLED_PY)

    assert 'C1a_normal_first_ramp' in text
    assert 'C1b_normal_steady_state' in text
    assert 'output_scale_first' in text
    assert 'output_scale_last_window_mean' in text
    assert 'samples_count' in text
    assert '_steady_state_stats' in text


def test_controlled_runner_has_artifact_collision_avoidance():
    text = _text(CONTROLLED_PY)

    assert 'import datetime' in text
    assert '--force' in text
    assert 'run_dir.exists()' in text
