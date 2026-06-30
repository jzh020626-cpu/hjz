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
from wing_alignment_system.mission_dispatcher import MissionDispatcherMixin


def test_cooperative_common_x_uses_wing_center_for_all_robots():
    helper = SimpleNamespace(
        wing_x=12.5,
        cooperative_common_x_backoff_m=0.0,
    )

    common_x = MissionCoordinator._cooperative_common_x(helper)

    assert common_x == 12.5


def test_cooperative_wait_targets_follow_wing_center_y_and_fixed_line_offsets():
    helper = SimpleNamespace(
        robots=["tracer1", "tracer2", "tracer3"],
        wing_y=-5.4,
        cooperative_wait_tracer1_offset_y_m=1.5,
        cooperative_line_offsets_m={
            "tracer1": 0.0,
            "tracer2": 1.0,
            "tracer3": 2.0,
        },
        _cooperative_common_x=lambda: 9.2,
        _cooperative_wait_side_sign=lambda: -1.0,
    )

    targets = MissionCoordinator._cooperative_wait_targets(helper)

    assert targets == {
        "tracer1": (9.2, -6.9),
        "tracer2": (9.2, -5.9),
        "tracer3": (9.2, -4.9),
    }


def test_cooperative_wait_side_sign_prefers_live_tracer1_side():
    helper = SimpleNamespace(
        wing_y=-4.0,
        cooperative_anchor_robot="tracer1",
        cooperative_start_line_y_map={"tracer1": -3.320, "tracer2": -2.320, "tracer3": -1.320},
        robot_xy={"tracer1": (10.0, -6.2)},
    )

    side_sign = MissionCoordinator._cooperative_wait_side_sign(helper)

    assert side_sign == -1.0


def test_cooperative_wait_side_sign_falls_back_to_start_line_when_live_pose_is_near_center():
    helper = SimpleNamespace(
        wing_y=-4.0,
        cooperative_anchor_robot="tracer1",
        cooperative_start_line_y_map={"tracer1": -5.0, "tracer2": -4.0, "tracer3": -3.0},
        robot_xy={"tracer1": (10.0, -4.0002)},
    )

    side_sign = MissionCoordinator._cooperative_wait_side_sign(helper)

    assert side_sign == -1.0


def test_final_dispatch_uses_configured_x_first_path_mode():
    helper = SimpleNamespace(
        staging_path_mode="x_first",
        transport_path_mode="x_first",
        final_path_mode="x_first",
    )
    helper._path_mode_for_goal_kind = lambda kind: MissionDispatcherMixin._path_mode_for_goal_kind(helper, kind)

    mode = MissionDispatcherMixin._path_mode_for_dispatch(helper, "tracer1", "FINAL", tag="FINAL")

    assert mode == "x_first"


def test_cooperative_leg_barrier_requires_all_robots_to_finish_current_leg():
    helper = SimpleNamespace(
        robots=["tracer1", "tracer2", "tracer3"],
        rt={
            "tracer1": SimpleNamespace(faulted=False, goal_kind="APPROACH_X", segs=None),
            "tracer2": SimpleNamespace(faulted=False, goal_kind="APPROACH_X", segs=None),
            "tracer3": SimpleNamespace(faulted=False, goal_kind="APPROACH_X", segs=[(1.0, 2.0)]),
        },
    )

    assert MissionCoordinator._all_cooperative_leg_complete(helper, "APPROACH_X") is False

    helper.rt["tracer3"].segs = None

    assert MissionCoordinator._all_cooperative_leg_complete(helper, "APPROACH_X") is True


def test_predict_target_world_uses_swapped_final_slots_for_tracer1_and_tracer2():
    helper = SimpleNamespace(
        wing_x=10.0,
        wing_y=-2.0,
        wing_yaw=0.0,
        tool_offsets={
            "tracer1": SimpleNamespace(x=0.6070, y=-0.4753, yaw_deg=5.0),
            "tracer2": SimpleNamespace(x=0.4388, y=0.2423, yaw_deg=6.0),
            "tracer3": SimpleNamespace(x=-0.8521, y=0.1302, yaw_deg=0.8),
        },
    )

    tracer1 = MissionDispatcherMixin.predict_target_world(helper, "tracer1", 0.0, 0.0)
    tracer2 = MissionDispatcherMixin.predict_target_world(helper, "tracer2", 0.0, 0.0)

    assert tracer1[:2] == (10.607, -2.4753)
    assert tracer2[:2] == (10.4388, -1.7577)


def test_final_dispatch_uses_single_direct_segment_in_simultaneous_direct_mode():
    actions = []
    helper = SimpleNamespace(
        rt={"tracer1": SimpleNamespace()},
        final_entry_mode="simultaneous_direct",
        final_entry_profile_mode="final_entry",
        _reset_runtime_for_new_mission_leg=lambda rn, clear_alignment: actions.append(("reset", rn, clear_alignment)),
        predict_target_world=lambda rn, micro_dx, micro_dy: (11.2, -5.6, 1.2),
        staging_path_mode="x_first",
        transport_path_mode="x_first",
        final_path_mode="x_first",
        build_L_segments=lambda rn, xt, yt, goal_kind="FINAL", path_mode=None: [("bad", "bad"), (xt, yt)],
        _set_local_state=lambda rn, state, reason: actions.append(("state", rn, state, reason)),
        precision_on=lambda rn, on: actions.append(("precision", rn, on)),
        resume_one=lambda rn: actions.append(("resume", rn)),
        _send_current_segment=lambda rn, tag="PATH": actions.append(("send", rn, tag)),
    )
    helper._path_mode_for_goal_kind = lambda kind: MissionDispatcherMixin._path_mode_for_goal_kind(helper, kind)
    helper._path_mode_for_dispatch = lambda rn, goal_kind, tag="": MissionDispatcherMixin._path_mode_for_dispatch(
        helper, rn, goal_kind, tag=tag
    )

    MissionDispatcherMixin.dispatch_to_final_one(helper, "tracer1", tag="FINAL_DIRECT")

    ctx = helper.rt["tracer1"]
    assert ctx.segs == [(11.2, -5.6)]
    assert ctx.goal_kind == "FINAL"
    assert ctx.direct_final_entry_active is True
    assert ("send", "tracer1", "FINAL_DIRECT") in actions


def test_path_mode_for_dispatch_uses_final_entry_override_in_simultaneous_direct_mode():
    helper = SimpleNamespace(
        staging_path_mode="x_first",
        transport_path_mode="x_first",
        final_path_mode="x_first",
        final_entry_mode="simultaneous_direct",
    )
    helper._path_mode_for_goal_kind = lambda kind: MissionDispatcherMixin._path_mode_for_goal_kind(helper, kind)

    mode = MissionDispatcherMixin._path_mode_for_dispatch(helper, "tracer1", "FINAL", tag="FINAL_DIRECT")

    assert mode == "direct"
