from robot_atlas.explain import explain_topic


def test_explain_ilqr_uses_learning_note_format() -> None:
    text = explain_topic("iLQR")

    assert "iLQR" in text
    assert "学习目标" in text
    assert "直觉" in text
    assert "输入" in text
    assert "输出" in text
