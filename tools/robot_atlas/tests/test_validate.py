from robot_atlas.validate import build_validation_plan


def test_validate_b03_contains_evidence_chain() -> None:
    text = build_validation_plan("B03")

    assert "验证计划" in text
    assert "tracking_error" in text
    assert "cost_history" in text
    assert "禁止" in text
