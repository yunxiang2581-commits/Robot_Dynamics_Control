from pathlib import Path

from robot_atlas.demo_assets import check_demo_assets


def test_demo_assets_reports_missing_without_crashing(tmp_path: Path) -> None:
    report = check_demo_assets("B03", tmp_path)

    assert report.stage == "B03"
    assert report.missing
    assert "缺失" in report.conclusion
