from __future__ import annotations


def build_next_step(stage_id: str) -> str:
    if stage_id.upper() != "B03":
        return f"阶段 {stage_id} 暂无内置下一步知识。建议先查看 docs/00_project_management。"
    return """当前阶段定位
B03 当前处于 B03-R4C：把 iLQR-lite 从教学骨架推进到可验证 two-link state tracking smoke run。

上一阶段结果
- B03-R4C-1A 已保留教学 TODO 骨架。
- 已明确 two-link dynamics adapter contract。
- 已有 smoke script 和 skeleton test，可继续沿着最小闭环验证推进。

下一步任务
B03-R4C-1B：实现最小 state tracking smoke run。

为什么做这一步
它把“控制算法骨架”变成“可运行证据链”：能看到 tracking error、cost history、torque profile 和 smoke_summary。

输入输出
- 输入：two-link 初始状态、参考轨迹、短时域 horizon、控制限制。
- 输出：状态轨迹、控制序列、误差曲线、代价曲线、运行摘要。

需要修改文件
- projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqr_lite_two_link_smoke.py
- projects/B_mujoco_mpc_study/simulator/adapters/b02_to_b03_adapter.py

需要新增文件
- docs/00_project_management/stepB03R4C1B_state_tracking_smoke_run.md
- outputs/B03/state_tracking_smoke/ 下的图、日志和视频占位或产物。

验证标准
- smoke script 能在短时间内完成。
- tracking_error.png、cost_history.png、torque_profile.png 存在。
- smoke_summary.json 包含关键参数和最终误差。
- 不要求长时间仿真，不要求完整求职 demo。

禁止事项
- 不要执行 git add / git commit / git push。
- 不要修改 external/open_source_repos。
- 不要把外部 repo 代码直接复制进 B 项目。
- 不要运行长时间 MuJoCo 仿真。"""
