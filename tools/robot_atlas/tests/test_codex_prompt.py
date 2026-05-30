from robot_atlas.codex_prompt import generate_codex_prompt


def test_generate_codex_prompt_contains_safety_boundaries() -> None:
    text = generate_codex_prompt("B03-R4C-1B")

    assert "B03-R4C-1B" in text
    assert "不要执行 git add" in text
    assert "不要执行 git commit" in text
    assert "不要执行 git push" in text
    assert "state tracking smoke run" in text
