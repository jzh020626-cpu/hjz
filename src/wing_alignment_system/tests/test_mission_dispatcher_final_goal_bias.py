from pathlib import Path
import math
from types import SimpleNamespace
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wing_alignment_system.mission_dispatcher import MissionDispatcherMixin
from wing_alignment_system.mission_types import ToolOffset


def test_predict_target_world_applies_forward_bias_in_goal_heading():
    helper = SimpleNamespace(
        wing_x=1.0,
        wing_y=2.0,
        wing_yaw=0.0,
        tool_offsets={"tracer1": ToolOffset(0.40, 0.20, 0.0)},
        final_goal_forward_bias_m={"tracer1": 0.03},
    )

    xw, yw, yaw_rad = MissionDispatcherMixin.predict_target_world(helper, "tracer1", 0.0, 0.0)

    assert math.isclose(xw, 1.43, abs_tol=1e-9)
    assert math.isclose(yw, 2.20, abs_tol=1e-9)
    assert math.isclose(yaw_rad, 0.0, abs_tol=1e-9)


def test_predict_target_world_rotates_forward_bias_with_wing_yaw():
    helper = SimpleNamespace(
        wing_x=0.0,
        wing_y=0.0,
        wing_yaw=math.pi / 2.0,
        tool_offsets={"tracer3": ToolOffset(0.40, 0.0, 0.0)},
        final_goal_forward_bias_m={"tracer3": 0.02},
    )

    xw, yw, yaw_rad = MissionDispatcherMixin.predict_target_world(helper, "tracer3", 0.0, 0.0)

    assert math.isclose(xw, 0.0, abs_tol=1e-9)
    assert math.isclose(yw, 0.42, abs_tol=1e-9)
    assert math.isclose(yaw_rad, math.pi / 2.0, abs_tol=1e-9)
