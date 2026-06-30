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

from wing_alignment_system.mission_gate_manager import MissionGateManagerMixin


class _BoolMsg:
    def __init__(self, data=False):
        self.data = data


def _make_pub(stop_calls, rn):
    return SimpleNamespace(publish=lambda msg: stop_calls.append((rn, msg.data)))


def test_sync_approach_uses_avoidance_min_center_distance_to_stop_lower_priority_robot():
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
        emergency=False,
        state="SYNC_APPROACH_X",
        gate_hold_sec=0.5,
        gate_keep_one_moving=True,
        final_entry_gate_mode="team_hold",
        robots=["tracer1", "tracer2", "tracer3"],
        dispatch_order=["tracer1", "tracer2", "tracer3"],
        robot_xy={"tracer1": (0.0, 0.0), "tracer2": (0.75, 0.0), "tracer3": (2.0, 0.0)},
        rt={
            "tracer1": SimpleNamespace(faulted=False, finished=False, confirmed=False, gate_stopped=False, gate_hold_until=0.0, dwell_start=0.0, fine_active=False, first_qr_locked=False, staged=False, entered=False, ready_to_lift=False, direct_final_entry_active=False),
            "tracer2": SimpleNamespace(faulted=False, finished=False, confirmed=False, gate_stopped=False, gate_hold_until=0.0, dwell_start=0.0, fine_active=False, first_qr_locked=False, staged=False, entered=False, ready_to_lift=False, direct_final_entry_active=False),
            "tracer3": SimpleNamespace(faulted=False, finished=False, confirmed=False, gate_stopped=False, gate_hold_until=0.0, dwell_start=0.0, fine_active=False, first_qr_locked=False, staged=False, entered=False, ready_to_lift=False, direct_final_entry_active=False),
        },
        get_clock=lambda: clock,
        _near_wing=lambda rn: False,
        _robot_stage=lambda rn: "NAV_APPROACH",
        _leader_robot=lambda: "tracer1",
        stop_pub={rn: _make_pub(stop_calls, rn) for rn in ["tracer1", "tracer2", "tracer3"]},
        stop_slide_comp=lambda rn: stop_calls.append((rn, "slide")),
        resume_one=lambda rn: stop_calls.append((rn, "resume")),
        get_logger=lambda: SimpleNamespace(warn=lambda msg: log_msgs.append(msg)),
    )

    original_bool = MissionGateManagerMixin.apply_collision_gate.__globals__["Bool"]
    MissionGateManagerMixin.apply_collision_gate.__globals__["Bool"] = _BoolMsg
    try:
        MissionGateManagerMixin.apply_collision_gate(helper)
    finally:
        MissionGateManagerMixin.apply_collision_gate.__globals__["Bool"] = original_bool

    assert helper.rt["tracer2"].gate_stopped is True
    assert helper.rt["tracer1"].gate_stopped is False
    assert ("tracer2", True) in stop_calls
    assert any("0.75" in msg or "0.80" in msg for msg in log_msgs)


def test_final_direct_does_not_apply_motion_avoidance_inside_wing_model():
    stop_calls = []
    log_msgs = []
    now_box = {"t": 10.0}
    clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(now_box["t"] * 1e9)))
    helper = SimpleNamespace(
        avoidance_enable=True,
        avoidance_min_center_dist_m=0.80,
        avoidance_resume_center_dist_m=0.90,
        avoidance_use_mocap_center_distance=True,
        avoidance_final_static_exempt=True,
        gate_enable=True,
        emergency=False,
        state="RUN_ALIGNMENT",
        final_entry_gate_mode="team_hold",
        gate_hold_sec=0.50,
        gate_keep_one_moving=True,
        robots=["tracer1", "tracer2", "tracer3"],
        dispatch_order=["tracer1", "tracer2", "tracer3"],
        robot_xy={"tracer1": (0.0, 0.0), "tracer2": (0.79, 0.0), "tracer3": (2.0, 0.0)},
        rt={
            rn: SimpleNamespace(
                faulted=False, finished=False, confirmed=False, gate_stopped=False, gate_hold_until=0.0,
                dwell_start=0.0, fine_active=False, first_qr_locked=False, staged=True, entered=True,
                ready_to_lift=False, direct_final_entry_active=True
            )
            for rn in ["tracer1", "tracer2", "tracer3"]
        },
        get_clock=lambda: clock,
        _near_wing=lambda rn: False,
        _robot_stage=lambda rn: "NAV_FINAL",
        _leader_robot=lambda: "tracer1",
        stop_pub={rn: _make_pub(stop_calls, rn) for rn in ["tracer1", "tracer2", "tracer3"]},
        stop_slide_comp=lambda rn: stop_calls.append((rn, "slide")),
        resume_one=lambda rn: stop_calls.append((rn, "resume")),
        get_logger=lambda: SimpleNamespace(warn=lambda msg: log_msgs.append(msg)),
    )

    original_bool = MissionGateManagerMixin.apply_collision_gate.__globals__["Bool"]
    MissionGateManagerMixin.apply_collision_gate.__globals__["Bool"] = _BoolMsg
    try:
        MissionGateManagerMixin.apply_collision_gate(helper)
    finally:
        MissionGateManagerMixin.apply_collision_gate.__globals__["Bool"] = original_bool

    assert helper.rt["tracer1"].gate_stopped is False
    assert helper.rt["tracer2"].gate_stopped is False
    assert helper.rt["tracer3"].gate_stopped is False
    assert stop_calls == []
    assert log_msgs == []


def test_final_direct_static_robot_is_exempt_from_motion_avoidance_blocking():
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
        emergency=False,
        state="RUN_ALIGNMENT",
        final_entry_gate_mode="team_hold",
        gate_hold_sec=0.50,
        gate_keep_one_moving=True,
        robots=["tracer1", "tracer2", "tracer3"],
        dispatch_order=["tracer1", "tracer2", "tracer3"],
        robot_xy={"tracer1": (0.0, 0.0), "tracer2": (0.75, 0.0), "tracer3": (2.0, 0.0)},
        rt={
            "tracer1": SimpleNamespace(faulted=False, finished=False, confirmed=False, gate_stopped=False, gate_hold_until=0.0, dwell_start=1.0, fine_active=False, first_qr_locked=False, staged=True, entered=True, ready_to_lift=False, direct_final_entry_active=False),
            "tracer2": SimpleNamespace(faulted=False, finished=False, confirmed=False, gate_stopped=False, gate_hold_until=0.0, dwell_start=0.0, fine_active=False, first_qr_locked=False, staged=True, entered=True, ready_to_lift=False, direct_final_entry_active=True),
            "tracer3": SimpleNamespace(faulted=False, finished=False, confirmed=False, gate_stopped=False, gate_hold_until=0.0, dwell_start=0.0, fine_active=False, first_qr_locked=False, staged=True, entered=True, ready_to_lift=False, direct_final_entry_active=True),
        },
        get_clock=lambda: clock,
        _near_wing=lambda rn: False,
        _robot_stage=lambda rn: "NAV_FINAL",
        _leader_robot=lambda: "tracer1",
        stop_pub={rn: _make_pub(stop_calls, rn) for rn in ["tracer1", "tracer2", "tracer3"]},
        stop_slide_comp=lambda rn: stop_calls.append((rn, "slide")),
        resume_one=lambda rn: stop_calls.append((rn, "resume")),
        get_logger=lambda: SimpleNamespace(warn=lambda msg: log_msgs.append(msg)),
    )

    original_bool = MissionGateManagerMixin.apply_collision_gate.__globals__["Bool"]
    MissionGateManagerMixin.apply_collision_gate.__globals__["Bool"] = _BoolMsg
    try:
        MissionGateManagerMixin.apply_collision_gate(helper)
    finally:
        MissionGateManagerMixin.apply_collision_gate.__globals__["Bool"] = original_bool

    assert helper.rt["tracer2"].gate_stopped is False
    assert helper.rt["tracer3"].gate_stopped is False
    assert stop_calls == []


def test_sync_transport_uses_team_hold_with_motion_avoidance():
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
        emergency=False,
        state="SYNC_TRANSPORT",
        final_entry_gate_mode="team_hold",
        gate_hold_sec=0.50,
        gate_keep_one_moving=True,
        robots=["tracer1", "tracer2", "tracer3"],
        dispatch_order=["tracer1", "tracer2", "tracer3"],
        robot_xy={"tracer1": (0.0, 0.0), "tracer2": (0.78, 0.0), "tracer3": (2.0, 0.0)},
        rt={
            rn: SimpleNamespace(
                faulted=False, finished=False, confirmed=False, gate_stopped=False, gate_hold_until=0.0,
                dwell_start=0.0, fine_active=False, first_qr_locked=False, staged=False, entered=True,
                ready_to_lift=False, direct_final_entry_active=False, transporting=True, transport_arrived=False
            )
            for rn in ["tracer1", "tracer2", "tracer3"]
        },
        get_clock=lambda: clock,
        _near_wing=lambda rn: False,
        _robot_stage=lambda rn: "TRANSPORTING",
        _leader_robot=lambda: "tracer1",
        stop_pub={rn: _make_pub(stop_calls, rn) for rn in ["tracer1", "tracer2", "tracer3"]},
        stop_slide_comp=lambda rn: stop_calls.append((rn, "slide")),
        resume_one=lambda rn: stop_calls.append((rn, "resume")),
        get_logger=lambda: SimpleNamespace(warn=lambda msg: log_msgs.append(msg)),
    )

    original_bool = MissionGateManagerMixin.apply_collision_gate.__globals__["Bool"]
    MissionGateManagerMixin.apply_collision_gate.__globals__["Bool"] = _BoolMsg
    try:
        MissionGateManagerMixin.apply_collision_gate(helper)
    finally:
        MissionGateManagerMixin.apply_collision_gate.__globals__["Bool"] = original_bool

    assert helper.rt["tracer1"].gate_stopped is True
    assert helper.rt["tracer2"].gate_stopped is True
    assert helper.rt["tracer3"].gate_stopped is True
    assert any("TEAM_HOLD" in msg for msg in log_msgs)
