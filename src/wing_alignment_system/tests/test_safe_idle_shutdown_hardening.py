from pathlib import Path
from types import SimpleNamespace
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SRC_ROOT = REPO_ROOT / "src/wing_alignment_system"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from wing_alignment_system import cmd_scheduler_node, cmd_watchdog_node, mission_coordinator


MODULE_CASES = [
    (cmd_scheduler_node, "CmdScheduler", "events_logger"),
    (cmd_watchdog_node, "CmdWatchdog", None),
    (mission_coordinator, "MissionCoordinator", None),
]


class _FlakyCloser:
    def __init__(self):
        self.calls = 0

    def close(self):
        self.calls += 1
        if self.calls > 1:
            raise ValueError("I/O operation on closed file")


class _DestroyRaisesOnce:
    def __init__(self, message: str):
        self.message = message
        self.calls = 0

    def destroy_node(self):
        self.calls += 1
        raise RuntimeError(self.message)


@pytest.mark.parametrize("module,_,__", MODULE_CASES)
def test_modules_define_shutdown_helpers(module, _, __):
    assert hasattr(module, "_safe_close")
    assert hasattr(module, "_safe_destroy_node")
    assert hasattr(module, "_safe_rclpy_shutdown")


@pytest.mark.parametrize("module,_,__", MODULE_CASES)
def test_safe_close_is_idempotent(module, _, __):
    closer = _FlakyCloser()

    module._safe_close(closer)
    module._safe_close(closer)

    assert closer.calls >= 1


@pytest.mark.parametrize("module,_,__", MODULE_CASES)
def test_safe_destroy_node_tolerates_redundant_destroy(module, _, __):
    node = _DestroyRaisesOnce("cannot use Destroyable because destruction was requested")

    module._safe_destroy_node(node)

    assert node.calls == 1


@pytest.mark.parametrize("module,_,__", MODULE_CASES)
def test_safe_rclpy_shutdown_swallows_duplicate_shutdown(monkeypatch, module, _, __):
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)

    calls = []

    def _shutdown():
        calls.append("shutdown")
        raise RuntimeError("failed to shutdown: rcl_shutdown already called on the given context")

    monkeypatch.setattr(module.rclpy, "shutdown", _shutdown)

    module._safe_rclpy_shutdown()

    assert calls == ["shutdown"]


@pytest.mark.parametrize("module,_,__", MODULE_CASES)
def test_safe_rclpy_shutdown_reraises_unexpected_errors(monkeypatch, module, _, __):
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)

    def _shutdown():
        raise RuntimeError("unexpected shutdown failure")

    monkeypatch.setattr(module.rclpy, "shutdown", _shutdown)

    with pytest.raises(RuntimeError, match="unexpected shutdown failure"):
        module._safe_rclpy_shutdown()


@pytest.mark.parametrize("module,class_name,logger_attr", MODULE_CASES)
def test_main_teardown_tolerates_duplicate_shutdown(monkeypatch, module, class_name, logger_attr):
    logger = _FlakyCloser()
    node = SimpleNamespace(destroy_calls=0)

    def _destroy_node():
        node.destroy_calls += 1

    node.destroy_node = _destroy_node
    if logger_attr is not None:
        setattr(node, logger_attr, logger)

    monkeypatch.setattr(module.rclpy, "init", lambda args=None: None)
    monkeypatch.setattr(module.rclpy, "spin", lambda current_node: None)
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(
        module.rclpy,
        "shutdown",
        lambda: (_ for _ in ()).throw(RuntimeError("failed to shutdown: rcl_shutdown already called on the given context")),
    )
    monkeypatch.setattr(module, class_name, lambda: node)

    module.main()

    assert node.destroy_calls == 1
    if logger_attr is not None:
        assert logger.calls >= 1


@pytest.mark.parametrize("module,class_name,logger_attr", MODULE_CASES)
def test_main_teardown_tolerates_external_shutdown_exception(monkeypatch, module, class_name, logger_attr):
    logger = _FlakyCloser()
    node = SimpleNamespace(destroy_calls=0)

    def _destroy_node():
        node.destroy_calls += 1

    node.destroy_node = _destroy_node
    if logger_attr is not None:
        setattr(node, logger_attr, logger)

    monkeypatch.setattr(module.rclpy, "init", lambda args=None: None)
    monkeypatch.setattr(module.rclpy, "ok", lambda: False)
    monkeypatch.setattr(module.rclpy, "shutdown", lambda: None)
    monkeypatch.setattr(module.rclpy, "spin", lambda current_node: (_ for _ in ()).throw(module.ExternalShutdownException()))
    monkeypatch.setattr(module, class_name, lambda: node)

    module.main()

    assert node.destroy_calls == 1
    if logger_attr is not None:
        assert logger.calls >= 1


@pytest.mark.parametrize("module,class_name,logger_attr", MODULE_CASES)
def test_main_teardown_tolerates_wait_set_context_invalidation_rcl_error(monkeypatch, module, class_name, logger_attr):
    logger = _FlakyCloser()
    node = SimpleNamespace(destroy_calls=0)

    class FakeRCLError(RuntimeError):
        pass

    def _destroy_node():
        node.destroy_calls += 1

    node.destroy_node = _destroy_node
    if logger_attr is not None:
        setattr(node, logger_attr, logger)

    monkeypatch.setattr(module, "RCLError", FakeRCLError)
    monkeypatch.setattr(module.rclpy, "init", lambda args=None: None)
    monkeypatch.setattr(module.rclpy, "ok", lambda: False)
    monkeypatch.setattr(module.rclpy, "shutdown", lambda: None)
    monkeypatch.setattr(
        module.rclpy,
        "spin",
        lambda current_node: (_ for _ in ()).throw(
            FakeRCLError("failed to initialize wait set: the given context is not valid, either rcl_init() was not called or rcl_shutdown() was called.")
        ),
    )
    monkeypatch.setattr(module, class_name, lambda: node)

    module.main()

    assert node.destroy_calls == 1


@pytest.mark.parametrize("module,class_name,logger_attr", MODULE_CASES)
def test_main_reraises_unexpected_rcl_error(monkeypatch, module, class_name, logger_attr):
    logger = _FlakyCloser()
    node = SimpleNamespace(destroy_calls=0)

    class FakeRCLError(RuntimeError):
        pass

    def _destroy_node():
        node.destroy_calls += 1

    node.destroy_node = _destroy_node
    if logger_attr is not None:
        setattr(node, logger_attr, logger)

    monkeypatch.setattr(module, "RCLError", FakeRCLError)
    monkeypatch.setattr(module.rclpy, "init", lambda args=None: None)
    monkeypatch.setattr(module.rclpy, "ok", lambda: False)
    monkeypatch.setattr(module.rclpy, "shutdown", lambda: None)
    monkeypatch.setattr(
        module.rclpy,
        "spin",
        lambda current_node: (_ for _ in ()).throw(FakeRCLError("unexpected wait set failure")),
    )
    monkeypatch.setattr(module, class_name, lambda: node)

    with pytest.raises(FakeRCLError, match="unexpected wait set failure"):
        module.main()

    assert node.destroy_calls == 1


@pytest.mark.parametrize("module,class_name,logger_attr", MODULE_CASES)
def test_main_propagates_unexpected_spin_errors(monkeypatch, module, class_name, logger_attr):
    logger = _FlakyCloser()
    node = SimpleNamespace(destroy_calls=0)

    def _destroy_node():
        node.destroy_calls += 1

    node.destroy_node = _destroy_node
    if logger_attr is not None:
        setattr(node, logger_attr, logger)

    monkeypatch.setattr(module.rclpy, "init", lambda args=None: None)
    monkeypatch.setattr(module.rclpy, "ok", lambda: False)
    monkeypatch.setattr(module.rclpy, "shutdown", lambda: None)
    monkeypatch.setattr(module.rclpy, "spin", lambda current_node: (_ for _ in ()).throw(RuntimeError("unexpected spin failure")))
    monkeypatch.setattr(module, class_name, lambda: node)

    with pytest.raises(RuntimeError, match="unexpected spin failure"):
        module.main()

    assert node.destroy_calls == 1


def test_shutdown_exceptions_are_handled_explicitly_in_all_three_nodes():
    files = [cmd_scheduler_node.__file__, cmd_watchdog_node.__file__, mission_coordinator.__file__]
    for file_path in files:
        text = Path(file_path).read_text(encoding="utf-8")
        spin_block = text.split("rclpy.spin", 1)[1].split("finally", 1)[0]
        assert "ExternalShutdownException" in text
        assert "except RCLError as exc" in spin_block
        assert "except Exception" not in spin_block
