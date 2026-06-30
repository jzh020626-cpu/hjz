from pathlib import Path
from types import SimpleNamespace
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wing_alignment_system import cmd_watchdog_node


def _make_stub():
    return SimpleNamespace(
        _last_ctrl_log_kind=None,
        _last_ctrl_log_state=None,
        _last_ctrl_log_ts=0.0,
    )


def test_logs_first_stop_event():
    node = _make_stub()

    should_log = cmd_watchdog_node.CmdWatchdog._should_log_control_event(
        node,
        now=1.0,
        kind="stop",
        stop_latched=True,
        emergency_latched=False,
    )

    assert should_log is True


def test_suppresses_repeated_identical_stop_event_within_window():
    node = _make_stub()
    cmd_watchdog_node.CmdWatchdog._should_log_control_event(
        node,
        now=1.0,
        kind="stop",
        stop_latched=True,
        emergency_latched=False,
    )

    should_log = cmd_watchdog_node.CmdWatchdog._should_log_control_event(
        node,
        now=1.2,
        kind="stop",
        stop_latched=True,
        emergency_latched=False,
    )

    assert should_log is False


def test_logs_repeated_identical_stop_event_after_window():
    node = _make_stub()
    cmd_watchdog_node.CmdWatchdog._should_log_control_event(
        node,
        now=1.0,
        kind="stop",
        stop_latched=True,
        emergency_latched=False,
    )

    should_log = cmd_watchdog_node.CmdWatchdog._should_log_control_event(
        node,
        now=3.5,
        kind="stop",
        stop_latched=True,
        emergency_latched=False,
    )

    assert should_log is True


def test_logs_when_control_kind_changes():
    node = _make_stub()
    cmd_watchdog_node.CmdWatchdog._should_log_control_event(
        node,
        now=1.0,
        kind="stop",
        stop_latched=True,
        emergency_latched=False,
    )

    should_log = cmd_watchdog_node.CmdWatchdog._should_log_control_event(
        node,
        now=1.2,
        kind="resume",
        stop_latched=False,
        emergency_latched=False,
    )

    assert should_log is True
