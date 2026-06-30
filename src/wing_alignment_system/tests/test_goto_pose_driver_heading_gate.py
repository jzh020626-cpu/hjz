from pathlib import Path
import math
from types import SimpleNamespace
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wing_alignment_system import goto_pose_driver


def _make_driver_stub(yaw_deg: float, *, profile_name: str = "default", phase: int = 1):
    profile = {
        "name": profile_name,
        "v_nominal": 0.20,
        "v_max": 0.25,
        "w_max": 0.65,
        "pos_tol": 0.05,
        "yaw_tol_deg": 3.0,
        "slow_radius": 0.30,
        "v_slow_max": 0.08,
        "w_slow_max": 0.30,
        "v_min_far": 0.04,
        "near_rotate_only_deg": 12.0,
        "rotate_only_deg": 20.0,
        "align_yaw_at_goal": True,
    }
    stub = SimpleNamespace(
        dt_nominal=1.0 / 50.0,
        _last_update_sec=None,
        have_pose=True,
        last_pose_rx_sec=1.0,
        pose_timeout_sec=0.25,
        _last_v=0.0,
        _last_w=0.0,
        goal_reached_until=0.0,
        final_stop_until=0.0,
        e_stop_latched=False,
        stop_hold_until=0.0,
        mission_active=True,
        phase=phase,
        goal_x=1.0,
        goal_y=0.0,
        goal_yaw=0.0,
        x=0.0,
        y=0.0,
        yaw=math.radians(-yaw_deg),
        precision_mode_requested=False,
        _near_rotate_latched=False,
        near_rotate_only_exit_deg=6.0,
        k_yaw=1.5,
        stall_fail_stop=False,
        stall_abort_sec=0.0,
        _last_debug_log_sec=0.0,
        robot_name="tracer1",
    )
    stub._get_active_profile = lambda: profile
    stub._reset_near_rotate_latch = lambda: setattr(stub, "_near_rotate_latched", False)
    stub._reset_stall_watch = lambda: None
    stub._goal_status_value = lambda now, reached: reached
    stub._check_stall = lambda now, cmd_v, cmd_w: None
    stub._rate_limit = lambda target_v, target_w, dt, force_zero_v=False: (
        0.0 if force_zero_v else target_v,
        target_w,
    )
    stub._finish_goal = lambda now: ""
    stub._precision_command = lambda prof, dist, yaw_err_final, now: (0.0, 0.0, False)
    stub._phase1_rotate_only = (
        lambda prof, yaw_err_to_goal, near: goto_pose_driver.GotoPoseDriver._phase1_rotate_only(
            stub, prof, yaw_err_to_goal, near
        )
    )
    return stub


def test_phase1_large_heading_error_rotates_before_driving(monkeypatch):
    monkeypatch.setattr(goto_pose_driver, "_now_sec", lambda node: 1.0)
    driver = _make_driver_stub(yaw_deg=30.0)

    action = goto_pose_driver.GotoPoseDriver.update(driver)

    assert action.cmd_v == 0.0
    assert action.cmd_w > 0.0


def test_phase1_small_heading_error_keeps_driving(monkeypatch):
    monkeypatch.setattr(goto_pose_driver, "_now_sec", lambda node: 1.0)
    driver = _make_driver_stub(yaw_deg=2.0)

    action = goto_pose_driver.GotoPoseDriver.update(driver)

    assert action.cmd_v > 0.0
    assert action.cmd_w > 0.0


def test_rate_limit_drops_linear_speed_immediately_for_rotate_only():
    driver = SimpleNamespace(_last_v=0.18, _last_w=0.0, dv_max=0.20, dw_max=0.80)
    prev_v = driver._last_v

    safe_v, safe_w = goto_pose_driver.GotoPoseDriver._rate_limit(
        driver,
        target_v=0.0,
        target_w=0.4,
        dt=0.02,
    )

    assert safe_v < prev_v
    assert safe_w > 0.0


def test_pose_timeout_warning_is_suppressed_while_driver_is_not_executing_a_goal(monkeypatch):
    monkeypatch.setattr(goto_pose_driver, "_now_sec", lambda node: 2.0)
    driver = _make_driver_stub(yaw_deg=0.0)
    driver.last_pose_rx_sec = 1.0
    driver.pose_timeout_sec = 0.25
    driver.mission_active = False

    action = goto_pose_driver.GotoPoseDriver.update(driver)

    assert action.cmd_v == 0.0
    assert action.cmd_w == 0.0
    assert action.log_text == ""


def test_pose_timeout_warning_is_suppressed_while_driver_is_in_stop_hold(monkeypatch):
    monkeypatch.setattr(goto_pose_driver, "_now_sec", lambda node: 2.0)
    driver = _make_driver_stub(yaw_deg=0.0)
    driver.last_pose_rx_sec = 1.0
    driver.pose_timeout_sec = 0.25
    driver.stop_hold_until = 3.0

    action = goto_pose_driver.GotoPoseDriver.update(driver)

    assert action.cmd_v == 0.0
    assert action.cmd_w == 0.0
    assert action.log_text == ""


def test_profile_name_from_code_supports_final_entry_profile():
    driver = SimpleNamespace(
        staging_profile_code=1.0,
        transport_profile_code=2.0,
        final_entry_profile_code=3.0,
    )

    profile_name = goto_pose_driver.GotoPoseDriver._profile_name_from_code(driver, 3.0)

    assert profile_name == "final_entry"


def test_final_entry_large_heading_error_keeps_driving_while_turning(monkeypatch):
    monkeypatch.setattr(goto_pose_driver, "_now_sec", lambda node: 1.0)
    driver = _make_driver_stub(yaw_deg=30.0, profile_name="final_entry", phase=0)

    action = goto_pose_driver.GotoPoseDriver.update(driver)

    assert action.cmd_v > 0.0
    assert action.cmd_w > 0.0


def test_final_entry_still_rotates_in_place_after_reaching_position_tolerance(monkeypatch):
    monkeypatch.setattr(goto_pose_driver, "_now_sec", lambda node: 1.0)
    driver = _make_driver_stub(yaw_deg=0.0, profile_name="final_entry", phase=1)
    driver.goal_x = 0.02
    driver.goal_y = 0.0
    driver.goal_yaw = math.radians(90.0)

    action = goto_pose_driver.GotoPoseDriver.update(driver)

    assert driver.phase == 2
    assert action.cmd_v == 0.0
    assert action.cmd_w > 0.0
