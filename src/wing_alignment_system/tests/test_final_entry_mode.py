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
from wing_alignment_system.mission_gate_manager import MissionGateManagerMixin


class _BoolMsg:
    def __init__(self, data=False):
        self.data = data


def test_wait_entry_release_dispatches_all_robots_in_simultaneous_direct_mode():
    actions = []
    helper = SimpleNamespace(
        final_entry_mode="simultaneous_direct",
        state="WAIT_ENTRY_RELEASE",
        robots=["tracer1", "tracer2", "tracer3"],
        rt={
            rn: SimpleNamespace(
                faulted=False,
                finished=False,
                staged=True,
                entered=False,
                direct_final_entry_active=False,
            )
            for rn in ["tracer1", "tracer2", "tracer3"]
        },
        _dispatch_simultaneous_final_entry=lambda: actions.append("dispatch_all") or True,
        update_entry_owner=lambda: actions.append("update_entry_owner"),
        _set_global_state=lambda state, reason="": actions.append((state, reason)),
    )

    handled = MissionCoordinator._handle_wait_entry_release(helper)

    assert handled is True
    assert actions == [
        "dispatch_all",
        ("RUN_ALIGNMENT", "cooperative approach complete; start simultaneous direct final entry"),
    ]


def test_collision_gate_is_bypassed_during_direct_final_entry_inside_wing_model():
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
        gate_dmin_far=0.85,
        gate_dresume_far=1.10,
        gate_dmin_near=0.65,
        gate_dresume_near=0.80,
        gate_hold_sec=0.50,
        robots=["tracer1", "tracer2", "tracer3"],
        dispatch_order=["tracer1", "tracer2", "tracer3"],
        robot_xy={
            "tracer1": (0.0, 0.0),
            "tracer2": (0.4, 0.0),
            "tracer3": (2.0, 0.0),
        },
        rt={
            "tracer1": SimpleNamespace(
                faulted=False, finished=False, confirmed=False, gate_stopped=False, gate_hold_until=0.0,
                dwell_start=0.0, fine_active=False, first_qr_locked=False, staged=True, entered=True,
                ready_to_lift=False, direct_final_entry_active=True
            ),
            "tracer2": SimpleNamespace(
                faulted=False, finished=False, confirmed=False, gate_stopped=False, gate_hold_until=0.0,
                dwell_start=0.0, fine_active=False, first_qr_locked=False, staged=True, entered=True,
                ready_to_lift=False, direct_final_entry_active=True
            ),
            "tracer3": SimpleNamespace(
                faulted=False, finished=False, confirmed=False, gate_stopped=False, gate_hold_until=0.0,
                dwell_start=0.0, fine_active=False, first_qr_locked=False, staged=True, entered=True,
                ready_to_lift=False, direct_final_entry_active=True
            ),
        },
        get_clock=lambda: clock,
        _near_wing=lambda rn: False,
        _robot_stage=lambda rn: "NAV_FINAL",
        _leader_robot=lambda: "tracer1",
        stop_pub={rn: SimpleNamespace(publish=lambda msg, rn=rn: stop_calls.append((rn, msg.data))) for rn in ["tracer1", "tracer2", "tracer3"]},
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
