from pathlib import Path
from types import SimpleNamespace
import sys
import types

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

base_interfaces_demo = types.ModuleType("base_interfaces_demo")
base_interfaces_demo_msg = types.ModuleType("base_interfaces_demo.msg")
base_interfaces_demo_msg.MotorCommand = type("MotorCommand", (), {})
base_interfaces_demo_msg.MotorStatus = type("MotorStatus", (), {})
base_interfaces_demo.msg = base_interfaces_demo_msg
sys.modules.setdefault("base_interfaces_demo", base_interfaces_demo)
sys.modules.setdefault("base_interfaces_demo.msg", base_interfaces_demo_msg)

from wing_alignment_system.mission_coordinator import MissionCoordinator


class _BoolMsg:
    def __init__(self, data=False):
        self.data = data


def _make_helper(state="RUN_ALIGNMENT", local_a="WAIT_QR", local_b="WAIT_ALL_QR_LOCK", local_c="WAIT_ALL_QR_LOCK", pairs=None, emergency=False):
    stop_calls = []
    log_msgs = []
    clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(10.0 * 1e9)))
    helper = SimpleNamespace(
        avoidance_enable=True,
        avoidance_min_center_dist_m=0.80,
        avoidance_resume_center_dist_m=0.90,
        avoidance_use_mocap_center_distance=True,
        avoidance_final_static_exempt=True,
        gate_enable=True,
        emergency=emergency,
        state=state,
        final_entry_gate_mode="team_hold",
        gate_hold_sec=0.50,
        gate_keep_one_moving=True,
        robots=["tracer1", "tracer2", "tracer3"],
        dispatch_order=["tracer1", "tracer2", "tracer3"],
        robot_xy={"tracer1": (0.0, 0.0), "tracer2": (0.72, 0.0), "tracer3": (2.0, 0.0)},
        terminal_rack_close_exception_enable=True,
        terminal_rack_close_exception_mission_states={"RUN_ALIGNMENT", "SYNC_SLIDE_ALIGN", "ALL_READY_HOLD"},
        terminal_rack_close_exception_robot_states={"NAV_TO_FINAL", "WAIT_QR", "WAIT_ALL_QR_LOCK", "QR_PARKED"},
        terminal_rack_close_exception_exempt_pairs=set(pairs or [("tracer1", "tracer2"), ("tracer2", "tracer3"), ("tracer1", "tracer3")]),
        terminal_rack_close_exception_require_final_direct_or_qr_context=True,
        terminal_rack_close_exception_log_every_sec=1.0,
        _terminal_rack_exception_last_log={},
        rt={
            "tracer1": SimpleNamespace(faulted=False, finished=False, confirmed=False, gate_stopped=False, gate_hold_until=0.0, dwell_start=1.0, fine_active=False, first_qr_locked=True, staged=True, entered=True, ready_to_lift=False, direct_final_entry_active=False, local_state=local_a, goal_kind="FINAL", sync_wait_qr=False),
            "tracer2": SimpleNamespace(faulted=False, finished=False, confirmed=False, gate_stopped=False, gate_hold_until=0.0, dwell_start=0.0, fine_active=False, first_qr_locked=False, staged=True, entered=True, ready_to_lift=False, direct_final_entry_active=False, local_state=local_b, goal_kind="FINAL", sync_wait_qr=False),
            "tracer3": SimpleNamespace(faulted=False, finished=False, confirmed=False, gate_stopped=False, gate_hold_until=0.0, dwell_start=0.0, fine_active=False, first_qr_locked=True, staged=True, entered=True, ready_to_lift=False, direct_final_entry_active=False, local_state=local_c, goal_kind="FINAL", sync_wait_qr=False),
        },
        get_clock=lambda: clock,
        _near_wing=lambda rn: False,
        _robot_stage=lambda rn: getattr(helper.rt[rn], "local_state", ""),
        _leader_robot=lambda: "tracer1",
        stop_pub={rn: SimpleNamespace(publish=lambda msg, rn=rn: stop_calls.append((rn, msg.data))) for rn in ["tracer1", "tracer2", "tracer3"]},
        stop_slide_comp=lambda rn: stop_calls.append((rn, "slide")),
        resume_one=lambda rn: stop_calls.append((rn, "resume")),
        get_logger=lambda: SimpleNamespace(warn=lambda msg: log_msgs.append(msg)),
    )
    return helper, stop_calls, log_msgs


def _logger_with_msgs(messages):
    return SimpleNamespace(warn=lambda msg: messages.append(msg))


def test_sync_approach_staging_still_triggers_collision_gate():
    helper, stop_calls, log_msgs = _make_helper(state="SYNC_APPROACH_X", local_a="NAV_TO_STAGING", local_b="NAV_TO_STAGING")

    original_bool = MissionCoordinator.apply_collision_gate.__globals__["Bool"]
    MissionCoordinator.apply_collision_gate.__globals__["Bool"] = _BoolMsg
    try:
        MissionCoordinator.apply_collision_gate(helper)
    finally:
        MissionCoordinator.apply_collision_gate.__globals__["Bool"] = original_bool

    assert helper.rt["tracer2"].gate_stopped is True
    assert ("tracer2", True) in stop_calls


def test_terminal_rack_exception_skips_pairwise_stop_for_configured_pair():
    helper, stop_calls, log_msgs = _make_helper()

    original_bool = MissionCoordinator.apply_collision_gate.__globals__["Bool"]
    MissionCoordinator.apply_collision_gate.__globals__["Bool"] = _BoolMsg
    try:
        MissionCoordinator.apply_collision_gate(helper)
    finally:
        MissionCoordinator.apply_collision_gate.__globals__["Bool"] = original_bool

    assert helper.rt["tracer2"].gate_stopped is False
    assert stop_calls == []
    assert any("TERMINAL_RACK_EXCEPTION" in msg for msg in log_msgs)


def test_run_alignment_without_allowed_robot_state_still_triggers_gate():
    helper, stop_calls, log_msgs = _make_helper(local_a="NAV_TO_STAGING", local_b="WAIT_QR")

    original_bool = MissionCoordinator.apply_collision_gate.__globals__["Bool"]
    MissionCoordinator.apply_collision_gate.__globals__["Bool"] = _BoolMsg
    try:
        MissionCoordinator.apply_collision_gate(helper)
    finally:
        MissionCoordinator.apply_collision_gate.__globals__["Bool"] = original_bool

    assert helper.rt["tracer2"].gate_stopped is True
    assert any("MOTION_AVOIDANCE" in msg for msg in log_msgs)


def test_unconfigured_pair_still_triggers_gate():
    helper, stop_calls, log_msgs = _make_helper(pairs=[("tracer2", "tracer3")])

    original_bool = MissionCoordinator.apply_collision_gate.__globals__["Bool"]
    MissionCoordinator.apply_collision_gate.__globals__["Bool"] = _BoolMsg
    try:
        MissionCoordinator.apply_collision_gate(helper)
    finally:
        MissionCoordinator.apply_collision_gate.__globals__["Bool"] = original_bool

    assert helper.rt["tracer2"].gate_stopped is True
    assert any("MOTION_AVOIDANCE" in msg for msg in log_msgs)


def test_emergency_stop_path_is_not_overridden_by_terminal_exception():
    helper, stop_calls, log_msgs = _make_helper(emergency=True)

    original_bool = MissionCoordinator.apply_collision_gate.__globals__["Bool"]
    MissionCoordinator.apply_collision_gate.__globals__["Bool"] = _BoolMsg
    try:
        MissionCoordinator.apply_collision_gate(helper)
    finally:
        MissionCoordinator.apply_collision_gate.__globals__["Bool"] = original_bool

    assert stop_calls == []
    assert log_msgs == []


def test_config_missing_terminal_rack_exception_keeps_legacy_behavior():
    helper, stop_calls, log_msgs = _make_helper()
    helper.terminal_rack_close_exception_enable = False

    original_bool = MissionCoordinator.apply_collision_gate.__globals__["Bool"]
    MissionCoordinator.apply_collision_gate.__globals__["Bool"] = _BoolMsg
    try:
        MissionCoordinator.apply_collision_gate(helper)
    finally:
        MissionCoordinator.apply_collision_gate.__globals__["Bool"] = original_bool

    assert helper.rt["tracer2"].gate_stopped is True
    assert any("MOTION_AVOIDANCE" in msg for msg in log_msgs)


def test_terminal_rack_pair_parser_accepts_supported_string_delimiters():
    log_msgs = []
    helper = SimpleNamespace(get_logger=lambda: _logger_with_msgs(log_msgs))

    pairs = MissionCoordinator._parse_terminal_rack_exception_exempt_pairs(
        helper,
        ["tracer1,tracer2", "tracer2:tracer3", " tracer1 | tracer3 "],
    )

    assert pairs == {
        ("tracer1", "tracer2"),
        ("tracer2", "tracer3"),
        ("tracer1", "tracer3"),
    }
    assert log_msgs == []


def test_terminal_rack_pair_parser_rejects_malformed_pairs():
    log_msgs = []
    helper = SimpleNamespace(get_logger=lambda: _logger_with_msgs(log_msgs))

    pairs = MissionCoordinator._parse_terminal_rack_exception_exempt_pairs(
        helper,
        ["", "tracer1", "tracer1,", "tracer1,tracer2,tracer3", "tracer1|"],
    )

    assert pairs == set()
    assert len(log_msgs) == 5
