from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
MISSION_COORDINATOR = REPO_ROOT / "src/wing_alignment_system/wing_alignment_system/mission_coordinator.py"
MISSION_PARAMS = REPO_ROOT / "src/wing_alignment_system/config/mission_params.yaml"


def test_mission_coordinator_uses_unified_slide_recenter_targets():
    text = MISSION_COORDINATOR.read_text(encoding="utf-8")
    assert "slide_transport_recenter_targets_flat', [125.0, 135.0, 126.0, 141.0, 126.0, 134.0]" in text
    assert "'tracer1': (125.0, 135.0)" in text
    assert "'tracer2': (126.0, 141.0)" in text
    assert "'tracer3': (126.0, 134.0)" in text
    assert "'slide_physical_recenter_targets_flat'" in text
    assert "[125.0, 135.0, 0.0, 126.0, 141.0, 0.0, 126.0, 134.0, 0.0]" in text
    assert "'tracer1': (125.0, 135.0, 0.0)" in text
    assert "'tracer2': (126.0, 141.0, 0.0)" in text
    assert "'tracer3': (126.0, 134.0, 0.0)" in text
    assert "slide_center_xyz_mm', [125.0, 135.0, 0.0]" in text


def test_mission_params_match_slide_recenter_defaults():
    text = MISSION_PARAMS.read_text(encoding="utf-8")
    assert "slide_transport_recenter_targets_flat: [125.0, 135.0, 126.0, 141.0, 126.0, 134.0]" in text
    assert "slide_physical_recenter_targets_flat: [125.0, 135.0, 0.0, 126.0, 141.0, 0.0, 126.0, 134.0, 0.0]" in text
    assert "slide_center_xyz_mm: [125.0, 135.0, 0.0]" in text
