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


class _Logger:
    def __init__(self):
        self.messages = []

    def warn(self, msg):
        self.messages.append(("warn", msg))

    def info(self, msg):
        self.messages.append(("info", msg))

    def error(self, msg):
        self.messages.append(("error", msg))


def _make_stub(state="STANDBY"):
    logger = _Logger()
    transitions = []
    actions = []
    robots = ["tracer1", "tracer2", "tracer3"]

    stub = SimpleNamespace(
        managed_phase_mode=True,
        managed_active_phase="",
        managed_completed_phases=set(),
        state=state,
        robots=robots,
        rt={
            rn: SimpleNamespace(
                first_qr_locked=(state != "STANDBY"),
                ready_to_lift=(state in ("ALL_READY_HOLD", "LOAD_STABLE_HOLD")),
            )
            for rn in robots
        },
        transport_enable=True,
        load_level_enable=True,
        _logger=logger,
        _all_first_qr_locked=lambda: all(
            getattr(ctx, "first_qr_locked", False) for ctx in stub.rt.values()
        ),
        _all_ready_to_lift=lambda: all(
            getattr(ctx, "ready_to_lift", False) for ctx in stub.rt.values()
        ),
        _all_load_stable=lambda require_ready_flag=True: state == "LOAD_STABLE_HOLD",
        _start_sync_slide_align=lambda: actions.append("start_sync_slide_align"),
        _start_level_z_all=lambda: actions.append("start_level_z_all"),
        stop_all=lambda: actions.append("stop_all"),
        stop_all_slide_comp=lambda: actions.append("stop_all_slide_comp"),
        get_logger=lambda: logger,
    )

    def _set_global_state(new_state, reason=""):
        transitions.append((stub.state, new_state, reason))
        stub.state = new_state

    stub._set_global_state = _set_global_state
    stub._begin_approach_phase = lambda reason: MissionCoordinator._begin_approach_phase(stub, reason)
    stub._begin_slide_align_phase = lambda reason: MissionCoordinator._begin_slide_align_phase(stub, reason)
    stub._begin_level_recenter_phase = lambda reason: MissionCoordinator._begin_level_recenter_phase(stub, reason)
    stub._begin_transport_phase = lambda reason: MissionCoordinator._begin_transport_phase(stub, reason)
    stub._managed_actions = actions
    stub._managed_transitions = transitions
    return stub


def test_managed_phase_request_starts_approach_from_standby():
    stub = _make_stub(state="STANDBY")

    ok, message = MissionCoordinator._managed_phase_request(stub, "approach")

    assert ok is True
    assert stub.state == "WAIT_WING"
    assert stub.managed_active_phase == "approach"
    assert "WAIT_WING" in message


def test_managed_phase_request_rejects_slide_align_before_approach_boundary():
    stub = _make_stub(state="STANDBY")

    ok, message = MissionCoordinator._managed_phase_request(stub, "slide_align")

    assert ok is False
    assert stub.state == "STANDBY"
    assert "current_state=STANDBY" in message


def test_managed_phase_request_is_idempotent_after_slide_align_boundary():
    stub = _make_stub(state="ALL_READY_HOLD")
    stub.managed_completed_phases.add("slide_align")

    ok, message = MissionCoordinator._managed_phase_request(stub, "slide_align")

    assert ok is True
    assert stub._managed_actions == []
    assert "already completed" in message


def test_managed_hold_boundary_stops_auto_advance_when_phase_completed():
    stub = _make_stub(state="ALL_READY_HOLD")
    stub.managed_completed_phases.add("slide_align")

    assert MissionCoordinator._managed_should_hold_boundary(stub, "ALL_READY_HOLD") is True


def test_managed_status_summary_reports_state_and_phase_progress():
    stub = _make_stub(state="LOAD_STABLE_HOLD")
    stub.managed_active_phase = "level_recenter"
    stub.managed_completed_phases.update({"approach", "slide_align"})

    summary = MissionCoordinator._managed_status_summary(stub)

    assert "current_state=LOAD_STABLE_HOLD" in summary
    assert "active_phase=level_recenter" in summary
    assert "completed=approach,slide_align" in summary
