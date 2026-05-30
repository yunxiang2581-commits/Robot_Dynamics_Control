from __future__ import annotations


def generate_codex_prompt(task_id: str) -> str:
    return f"""# Codex 任务提示词：{task_id}

任务背景：
你在 Robot_Dynamics_Control 仓库中推进机器人运动控制学习项目 B_mujoco_mpc_study。

目标：
完成 {task_id}，重点是 B03-R4C-1B 的最小 state tracking smoke run。

工作范围：
- 只围绕 B03 two-link iLQR-lite smoke run 做小步实现。
- 保留教学型 TODO 和清晰中文注释。
- 输出可验证的 tracking error / cost / torque / smoke summary。

验证要求：
- 优先运行相关短测试或 smoke 脚本。
- 不运行长时间仿真。
- 汇报输入、输出、数学逻辑是否变化和风险。

禁止事项：
- 不要执行 git add。
- 不要执行 git commit。
- 不要执行 git push。
- 不要修改 external/open_source_repos。
- 不要修改 shared/robot_assets。
- 不要把外部 repo 代码直接复制进 B 项目。

最终汇报：
1. 修改文件
2. 实现内容
3. 验证命令和结果
4. 当前限制
5. 是否执行 git add/commit/push，必须说明没有"""
