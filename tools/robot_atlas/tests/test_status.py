from pathlib import Path

from robot_atlas.scanner import scan_repo
from robot_atlas.status import build_status


def test_status_b_includes_builtin_stage_and_scan_summary(tmp_path: Path) -> None:
    project = tmp_path / "projects" / "B_mujoco_mpc_study"
    project.mkdir(parents=True)
    (project / "demo.py").write_text("# TODO\n", encoding="utf-8")

    text = build_status("B", scan_repo(tmp_path))

    assert "B03-R4C" in text
    assert "B03-R4C-1B" in text
    assert "扫描补充" in text
    assert "Python 文件数量" in text
