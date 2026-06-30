from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wing_alignment_system.multi_tracer_keyboard_control import (
    DEFAULT_KEY_HOLD_TIMEOUT_SEC,
    DEFAULT_LINEAR_SPEED,
    DEFAULT_ROBOTS,
    DEFAULT_TOPIC_SUFFIX,
    MotionCommand,
    MultiTracerKeyboardControl,
    build_topic_names,
    key_to_motion,
)


def test_default_robots_cover_three_tracers():
    assert DEFAULT_ROBOTS == ("tracer1", "tracer2", "tracer3")
    assert DEFAULT_TOPIC_SUFFIX == "cmd_vel_desired"
    assert DEFAULT_LINEAR_SPEED == 0.08
    assert DEFAULT_KEY_HOLD_TIMEOUT_SEC == 0.25
    assert build_topic_names(DEFAULT_ROBOTS) == (
        "/tracer1/cmd_vel_desired",
        "/tracer2/cmd_vel_desired",
        "/tracer3/cmd_vel_desired",
    )


def test_forward_key_maps_to_positive_linear_velocity():
    assert key_to_motion("i") == MotionCommand(linear=1.0, angular=0.0)


def test_stop_key_maps_to_zero_velocity():
    assert key_to_motion("k") == MotionCommand(linear=0.0, angular=0.0)


def test_unknown_key_produces_no_motion_command():
    assert key_to_motion("?") is None


def test_motion_times_out_without_recent_key_input():
    assert MultiTracerKeyboardControl.is_motion_active(
        last_motion_time_sec=10.0,
        now_sec=10.20,
        key_hold_timeout_sec=0.25,
    )
    assert not MultiTracerKeyboardControl.is_motion_active(
        last_motion_time_sec=10.0,
        now_sec=10.30,
        key_hold_timeout_sec=0.25,
    )


def test_node_initializes_without_attribute_name_conflict(monkeypatch):
    rclpy = pytest.importorskip("rclpy")
    monkeypatch.setenv("ROS_LOG_DIR", "/tmp")

    node = None
    rclpy.init(args=[])
    try:
        node = MultiTracerKeyboardControl()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
