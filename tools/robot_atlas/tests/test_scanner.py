from pathlib import Path

from robot_atlas.scanner import scan_repo


def test_scan_repo_counts_project_lines_and_markers(tmp_path: Path) -> None:
    docs = tmp_path / "docs" / "00_project_management"
    project = tmp_path / "projects" / "B_mujoco_mpc_study"
    docs.mkdir(parents=True)
    project.mkdir(parents=True)
    (docs / "stepB03.md").write_text("stage note", encoding="utf-8")
    (project / "run_demo.py").write_text("# TODO\nraise NotImplementedError\n", encoding="utf-8")
    (project / "test_demo.py").write_text("def test_demo(): pass\n", encoding="utf-8")
    (tmp_path / "external" / "open_source_repos").mkdir(parents=True)
    ignored = tmp_path / "external" / "open_source_repos" / "todo.py"
    ignored.write_text("# TODO\n", encoding="utf-8")

    result = scan_repo(tmp_path)

    assert set(result.project_lines) == {"A", "B", "C", "D"}
    assert result.project_lines["B"].exists is True
    assert len(result.project_lines["B"].python_files) == 2
    assert len(result.project_lines["B"].test_files) == 1
    assert result.project_lines["B"].todo_count == 1
    assert result.project_lines["B"].not_implemented_count == 1
    assert result.docs_count == 1
    assert result.total_todos == 1
