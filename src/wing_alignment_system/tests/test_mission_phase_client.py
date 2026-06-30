from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
MODULE_PATH = REPO_ROOT / "src/wing_alignment_system/wing_alignment_system/mission_phase_client.py"


def _load_module():
    if not MODULE_PATH.exists():
        return None

    spec = spec_from_file_location("mission_phase_client", MODULE_PATH)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_phase_client_module_exists_and_parses_phase_argument():
    module = _load_module()

    assert module is not None
    args = module._parse_args(["--phase", "transport"])
    assert args.phase == "transport"


def test_phase_client_maps_phase_to_service_name():
    module = _load_module()

    assert module is not None
    assert module._service_name_for_phase("approach") == "/mission/start_approach"
    assert module._service_name_for_phase("slide_align") == "/mission/start_slide_align"
    assert module._service_name_for_phase("level_recenter") == "/mission/start_level_recenter"
    assert module._service_name_for_phase("transport") == "/mission/start_transport"
