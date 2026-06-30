from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wing_alignment_system.cmd_watchdog_policy import WatchdogPolicy
from wing_alignment_system.cmd_watchdog_types import WatchdogConfig


def _policy(*, enabled: bool) -> WatchdogPolicy:
    return WatchdogPolicy(
        WatchdogConfig(
            watchdog_hz=40.0,
            age_safe=0.15,
            age_stop=0.40,
            decay_mode="linear",
            decay_k=3.0,
            enable_execution_mode_output=enabled,
            degraded_linear_scale=0.5,
            degraded_angular_scale=0.25,
        )
    )


def test_default_watchdog_keeps_normal_behavior_when_execution_modes_disabled():
    policy = _policy(enabled=False)
    assert policy.on_cmd(1, 0.2, 0.4, 1.0, execution_mode="degraded") is True

    out = policy.compute(1.05)

    assert out.state == "NORMAL"
    assert out.applied_v == 0.2
    assert out.applied_w == 0.4
    assert out.output_scale == 1.0


def test_degraded_mode_scales_output_only_when_enabled():
    policy = _policy(enabled=True)
    assert policy.on_cmd(1, 0.2, 0.4, 1.0, execution_mode="degraded") is True

    out = policy.compute(1.05)

    assert out.state == "NORMAL"
    assert out.applied_v == 0.1
    assert out.applied_w == 0.1
    assert out.stop_reason == "execution_mode_degraded"


def test_hold_mode_zeroes_output_without_overriding_normal_state_name():
    policy = _policy(enabled=True)
    assert policy.on_cmd(1, 0.2, 0.4, 1.0, execution_mode="hold") is True

    out = policy.compute(1.05)

    assert out.state == "NORMAL"
    assert out.applied_v == 0.0
    assert out.applied_w == 0.0
    assert out.stop_reason == "execution_mode_hold"


def test_safe_stop_mode_zeroes_output_and_is_visible_in_state():
    policy = _policy(enabled=True)
    assert policy.on_cmd(1, 0.2, 0.4, 1.0, execution_mode="safe_stop") is True

    out = policy.compute(1.05)

    assert out.state == "MODE_SAFE_STOP"
    assert out.applied_v == 0.0
    assert out.applied_w == 0.0
    assert out.stop_reason == "execution_mode_safe_stop"


def test_age_stop_keeps_priority_over_requested_execution_mode():
    policy = _policy(enabled=True)
    assert policy.on_cmd(1, 0.2, 0.4, 1.0, execution_mode="normal") is True

    out = policy.compute(1.45)

    assert out.state == "AGE_STOP"
    assert out.applied_v == 0.0
    assert out.applied_w == 0.0
    assert out.stop_reason == "age_stop_exceeded"
