from pathlib import Path

from robot_atlas.external_repo import (
    analyze_external_repo,
    build_entrypoints_report,
    build_port_to_b_report,
    build_read_repo_report,
    build_reproduce_plan,
    generate_codex_reproduce_prompt,
)


def test_external_repo_static_analysis_finds_readme_entrypoints_and_risks(tmp_path: Path) -> None:
    (tmp_path / "README.md").write_text("demo", encoding="utf-8")
    (tmp_path / "demo.py").write_text("print('demo')\n", encoding="utf-8")
    (tmp_path / "requirements.txt").write_text("mujoco\n", encoding="utf-8")
    (tmp_path / "controllers").mkdir()

    report = analyze_external_repo(tmp_path)

    assert report.main_language_guess == "Python"
    assert report.readme_files
    assert report.entrypoints
    assert report.dependency_files
    assert report.robot_related_dirs
    assert "依赖安装" in "\n".join(report.reproduce_risks)


def test_external_repo_reports_and_prompt_include_readonly_boundaries(tmp_path: Path) -> None:
    (tmp_path / "README.md").write_text("demo", encoding="utf-8")
    (tmp_path / "demo.py").write_text("print('demo')\n", encoding="utf-8")

    assert "外部 Repo 审读报告" in build_read_repo_report(tmp_path)
    assert "入口判断结果" in build_entrypoints_report(tmp_path)
    assert "复现路线" in build_reproduce_plan(tmp_path)
    assert "B_mujoco_mpc_study" in build_port_to_b_report(tmp_path)
    prompt = generate_codex_reproduce_prompt(tmp_path)
    assert "只读" in prompt
    assert "不要修改外部 repo 源码" in prompt
