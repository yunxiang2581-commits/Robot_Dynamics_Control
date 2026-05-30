from __future__ import annotations

from .models import ScanResult


def build_status(project_id: str, scan_result: ScanResult) -> str:
    pid = project_id.upper()
    line = scan_result.project_lines.get(pid)
    if line is None:
        return f"未知项目线：{project_id}"
    if pid == "B":
        required = [
            "projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqr_lite_two_link_smoke.py",
            "projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_smoke_skeleton.py",
            "docs/00_project_management/stepB03R4C1A_preserve_skeleton_adapter_contract.md",
        ]
        missing = [path for path in required if not (scan_result.repo_root / path).exists()]
        confirm = "\n".join(f"- 未扫描到 {item}" for item in missing) if missing else "- 暂无"
        return f"""项目目标：
MuJoCo MPC / iLQR / iLQG 学习与求职 demo，要求有真实可运行仿真、视频、图表和复现实验命令。

当前阶段：
B03-R4C

已完成内容：
- B03-R4C-1A 已完成
- 已保留 TODO 教学骨架
- 已新增 two-link dynamics adapter contract
- 已有 smoke script 和 skeleton test

未完成内容：
- iLQR-lite 尚未真正接入 B02 two-link dynamics
- state tracking smoke run 尚未完成
- task-space tracking 尚未完成
- CEM/MPPI warm-start 尚未完成
- MP4/GIF demo 产物不足

核心文件：
- projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqr_lite_two_link_smoke.py
- projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_smoke_skeleton.py
- docs/00_project_management/stepB03R4C1A_preserve_skeleton_adapter_contract.md

当前风险：
- 当前仍偏 TODO 骨架，不是完整 demo
- 还不能作为求职展示项目最终版本
- 如果没有 tracking error / cost / torque / video，则 B03 证据链不足

下一步建议：
进入 B03-R4C-1B：实现最小 state tracking smoke run。

扫描补充：
- Python 文件数量：{len(line.python_files)}
- 测试文件数量：{len(line.test_files)}
- Markdown 文档数量：{len(line.markdown_files)}
- TODO 数量：{line.todo_count}
- NotImplementedError 数量：{line.not_implemented_count}

需要人工确认：
{confirm}"""
    entrypoints = "\n".join(f"- {path}" for path in line.entrypoint_files[:10]) or "- 未发现"
    tests = "\n".join(f"- {path}" for path in line.test_files[:10]) or "- 未发现"
    return f"""项目目标：
{line.goal}

扫描摘要：
{line.summary()}

可能入口文件：
{entrypoints}

测试文件：
{tests}

当前风险：
- 当前阶段需要根据 docs/00_project_management 进一步确认。

下一步建议：
- 先补充阶段文档，再生成更具体的任务拆解。"""
