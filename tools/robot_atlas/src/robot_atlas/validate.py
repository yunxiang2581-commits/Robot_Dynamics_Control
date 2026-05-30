from __future__ import annotations


def build_validation_plan(stage: str) -> str:
    if stage.upper() != "B03":
        return f"验证计划：{stage}\n- 暂无内置计划，请先补充阶段文档。"
    return """验证计划：B03

目标：
- 验证 two-link state tracking smoke run 是否形成最小证据链。

静态检查：
- 确认 smoke script 存在。
- 确认 adapter contract 文档存在。
- 确认测试文件覆盖 skeleton 或最小 smoke 行为。

产物检查：
- tracking_error.png
- cost_history.png
- torque_profile.png
- smoke_summary.json
- state_tracking_smoke.mp4 或明确说明尚未生成

运行建议：
- 只运行短时间 smoke test。
- 优先 pytest 精确测试文件。

禁止：
- 禁止长时间 MuJoCo 仿真。
- 禁止运行外部 repo 训练。
- 禁止 git add / git commit / git push。"""
