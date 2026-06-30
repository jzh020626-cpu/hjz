from pathlib import Path
import math
from types import SimpleNamespace
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wing_alignment_system.mission_robot_step import MissionRobotStepMixin


def test_wait_qr_auto_micro_search_triggers_when_no_qr_pose_seen():
    helper = SimpleNamespace(
        base_micro_search_enable=False,
        wait_qr_auto_micro_search_enable=True,
        wait_qr_auto_micro_search_delay_sec=2.0,
    )
    ctx = SimpleNamespace(
        raw_qr_pose=None,
        raw_qr_seen_stamp=0.0,
        raw_qr_receive_stamp=0.0,
    )

    assert MissionRobotStepMixin._should_start_wait_qr_micro_search(helper, ctx, 2.5) is True


def test_wait_qr_auto_micro_search_waits_for_delay():
    helper = SimpleNamespace(
        base_micro_search_enable=False,
        wait_qr_auto_micro_search_enable=True,
        wait_qr_auto_micro_search_delay_sec=2.0,
    )
    ctx = SimpleNamespace(
        raw_qr_pose=None,
        raw_qr_seen_stamp=0.0,
        raw_qr_receive_stamp=0.0,
    )

    assert MissionRobotStepMixin._should_start_wait_qr_micro_search(helper, ctx, 1.0) is False


def test_wait_qr_auto_micro_search_does_not_trigger_after_qr_seen():
    helper = SimpleNamespace(
        base_micro_search_enable=False,
        wait_qr_auto_micro_search_enable=True,
        wait_qr_auto_micro_search_delay_sec=2.0,
    )
    ctx = SimpleNamespace(
        raw_qr_pose=(0.01, 0.0, 0.2),
        raw_qr_seen_stamp=10.0,
        raw_qr_receive_stamp=10.0,
    )

    assert MissionRobotStepMixin._should_start_wait_qr_micro_search(helper, ctx, 3.0) is False


def test_wait_qr_forward_probe_triggers_only_after_delay_without_any_qr_pose():
    helper = SimpleNamespace(
        wait_qr_forward_probe_enable=True,
        wait_qr_forward_probe_delay_sec=2.0,
        wait_qr_forward_probe_max_attempts=2,
    )
    ctx = SimpleNamespace(
        raw_qr_pose=None,
        raw_qr_seen_stamp=0.0,
        raw_qr_receive_stamp=0.0,
        wait_qr_forward_probe_attempts=0,
    )

    assert MissionRobotStepMixin._should_start_wait_qr_forward_probe(helper, ctx, 1.5) is False
    assert MissionRobotStepMixin._should_start_wait_qr_forward_probe(helper, ctx, 2.5) is True


def test_wait_qr_forward_probe_stops_after_pose_seen_or_attempt_budget_used():
    helper = SimpleNamespace(
        wait_qr_forward_probe_enable=True,
        wait_qr_forward_probe_delay_sec=2.0,
        wait_qr_forward_probe_max_attempts=2,
    )
    seen_ctx = SimpleNamespace(
        raw_qr_pose=(0.01, 0.0, 0.2),
        raw_qr_seen_stamp=10.0,
        raw_qr_receive_stamp=10.0,
        wait_qr_forward_probe_attempts=0,
    )
    exhausted_ctx = SimpleNamespace(
        raw_qr_pose=None,
        raw_qr_seen_stamp=0.0,
        raw_qr_receive_stamp=0.0,
        wait_qr_forward_probe_attempts=2,
    )

    assert MissionRobotStepMixin._should_start_wait_qr_forward_probe(helper, seen_ctx, 3.0) is False
    assert MissionRobotStepMixin._should_start_wait_qr_forward_probe(helper, exhausted_ctx, 3.0) is False


def test_wait_qr_forward_probe_offset_advances_along_locked_yaw_in_small_steps():
    helper = SimpleNamespace(wait_qr_forward_probe_step_m=0.015)
    ctx = SimpleNamespace(
        locked_yaw=math.pi / 2.0,
        wait_qr_forward_probe_attempts=0,
    )

    dx1, dy1, dist1 = MissionRobotStepMixin._next_wait_qr_forward_probe_offset(helper, ctx)
    dx2, dy2, dist2 = MissionRobotStepMixin._next_wait_qr_forward_probe_offset(helper, ctx)

    assert math.isclose(dx1, 0.0, abs_tol=1e-9)
    assert math.isclose(dy1, 0.015, abs_tol=1e-9)
    assert math.isclose(dist1, 0.015, abs_tol=1e-9)
    assert math.isclose(dx2, 0.0, abs_tol=1e-9)
    assert math.isclose(dy2, 0.030, abs_tol=1e-9)
    assert math.isclose(dist2, 0.030, abs_tol=1e-9)
    assert ctx.wait_qr_forward_probe_attempts == 2
