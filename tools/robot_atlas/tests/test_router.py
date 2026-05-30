from robot_atlas.router import route_request


def test_route_b03_next_step() -> None:
    result = route_request("B03 下一步做什么")

    assert "下一步" in result.request_type
    assert "robot-atlas next B03" in result.commands
    assert "B03" in result.reason


def test_route_codex_prompt_with_task_id() -> None:
    result = route_request("生成 B03-R4C-1B 的 Codex 提示词")

    assert "Codex" in result.request_type
    assert "robot-atlas generate-codex B03-R4C-1B" in result.commands


def test_route_algorithm_explain() -> None:
    result = route_request("iLQR 原理是什么")

    assert "算法解释" in result.request_type
    assert "robot-atlas explain iLQR" in result.commands


def test_route_external_repo_reproduction_without_path() -> None:
    result = route_request("这个 repo 怎么复现")

    assert "外部 repo" in result.request_type
    assert "robot-atlas read-repo <repo_path>" in result.commands
    assert "需要提供本地 repo 路径" in result.notes


def test_route_understand_graph_without_path() -> None:
    result = route_request("用 Understand Anything 图谱分析这个项目")

    assert "Understand Anything" in result.request_type
    assert "robot-atlas import-understand <repo_path>" in result.commands
    assert "robot-atlas understand-summary <repo_path>" in result.commands
    assert "需要提供本地 repo 路径" in result.notes


def test_route_demo_assets() -> None:
    result = route_request("检查 B03 demo 产物")

    assert "demo" in result.request_type
    assert "robot-atlas demo-assets B03" in result.commands


def test_route_guards_have_highest_priority() -> None:
    result = route_request("检查禁止事项，并生成 B03-R4C-1B 的 Codex 提示词")

    assert "安全边界" in result.request_type
    assert result.commands == ["robot-atlas guards"]


def test_route_text_output_format() -> None:
    text = route_request("B 项目现在什么状态").as_text()

    assert "识别到的需求类型" in text
    assert "推荐命令" in text
    assert "理由" in text
    assert "注意事项" in text
    assert "robot-atlas status B" in text
