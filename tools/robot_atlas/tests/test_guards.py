from robot_atlas.guards import build_guards_report


def test_guards_report_lists_forbidden_actions_and_paths() -> None:
    text = build_guards_report()

    assert "git add" in text
    assert "git commit" in text
    assert "git push" in text
    assert "external/open_source_repos" in text
