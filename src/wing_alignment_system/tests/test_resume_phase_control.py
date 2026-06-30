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


def _make_resume_stub(resume_phase="", start_state="", state="WAIT_WING"):
    logger = _Logger()
    transitions = []
    stub = SimpleNamespace(
        managed_phase_mode=False,
        managed_active_phase="",
        managed_completed_phases=set(),
        resume_phase=resume_phase,
        startup_resume_phase="none",
        startup_resume_pending=False,
        start_state=start_state,
        state=state,
        robots=["tracer1", "tracer2", "tracer3"],
        rt={rn: SimpleNamespace(faulted=False) for rn in ["tracer1", "tracer2", "tracer3"]},
        get_logger=lambda: logger,
        _logger=logger,
    )

    def _set_global_state(new_state, reason=""):
        transitions.append((stub.state, new_state, reason))
        stub.state = new_state

    stub._set_global_state = _set_global_state
    stub._transitions = transitions
    return stub


def test_apply_resume_phase_override_prefers_resume_phase_and_enables_managed_mode():
    stub = _make_resume_stub(resume_phase="slide_align", start_state="sync_slide_align")

    MissionCoordinator._apply_resume_phase_override(stub)

    assert stub.start_state == ""
    assert stub.managed_phase_mode is True
    assert stub.startup_resume_phase == "slide_align"
    assert stub.startup_resume_pending is True
    assert any("resume_phase" in msg for level, msg in stub._logger.messages if level == "warn")


def test_startup_resume_request_routes_approach_to_wait_wing():
    stub = _make_resume_stub(resume_phase="approach", state="WAIT_WING")
    stub.startup_resume_phase = "approach"
    stub.startup_resume_pending = True
    stub._begin_approach_phase = lambda reason: stub._set_global_state("WAIT_WING", reason) or (True, "ok")

    ok, message = MissionCoordinator._startup_resume_request(stub)

    assert ok is True
    assert stub.managed_active_phase == "approach"
    assert stub.startup_resume_pending is False
    assert "approach" in message


def test_startup_resume_request_rejects_slide_align_when_resume_precheck_fails():
    stub = _make_resume_stub(resume_phase="slide_align", state="WAIT_WING")
    stub.startup_resume_phase = "slide_align"
    stub.startup_resume_pending = True
    stub._resume_slide_align_precheck_ok = lambda: (False, "coarse final pose missing")

    ok, message = MissionCoordinator._startup_resume_request(stub)

    assert ok is False
    assert stub.startup_resume_pending is True
    assert "coarse final pose missing" in message


def test_managed_status_summary_reports_startup_resume_phase():
    stub = _make_resume_stub(resume_phase="transport", state="LOAD_STABLE_HOLD")
    stub.managed_phase_mode = True
    stub.managed_active_phase = "transport"
    stub.managed_completed_phases.update({"approach", "slide_align", "level_recenter"})
    stub.startup_resume_phase = "transport"

    summary = MissionCoordinator._managed_status_summary(stub)

    assert "startup_resume_phase=transport" in summary
