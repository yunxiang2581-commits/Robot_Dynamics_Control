from robot_atlas.next_step import build_next_step


def test_next_step_b03_contains_required_sections() -> None:
    text = build_next_step("B03")

    for heading in [
        "当前阶段定位",
        "上一阶段结果",
        "下一步任务",
        "为什么做这一步",
        "输入输出",
        "需要修改文件",
        "需要新增文件",
        "验证标准",
        "禁止事项",
    ]:
        assert heading in text
    assert "B03-R4C-1B" in text
