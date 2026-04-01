from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_rtk_start_gate_contains_embedded_rtk_semantics() -> None:
    gate_contents = (
        PACKAGE_ROOT / "navegacion_gps" / "rtk_start_gate.py"
    ).read_text(encoding="utf-8")

    assert "NAVSAT_STATUS_GBAS_FIX = 2" in gate_contents
    assert "def status_text_is_rtk(status_text: str) -> bool:" in gate_contents
    assert '("rtk_fixed" in text) or ("rtk_float" in text) or ("rtk_fix" in text)' in gate_contents
    assert "def fix_type_is_rtk(fix_type: Optional[int]) -> bool:" in gate_contents
    assert "return numeric >= 5" in gate_contents
    assert "def navsat_status_is_rtk(status: Optional[int]) -> bool:" in gate_contents
    assert "return numeric >= NAVSAT_STATUS_GBAS_FIX" in gate_contents
    assert "def any_rtk_signal(" in gate_contents
