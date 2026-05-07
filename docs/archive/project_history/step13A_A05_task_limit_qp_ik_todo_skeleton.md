# Step 13A: A05 Task + Limit + QP-IK 历史规划与当前状态

当前状态更新：A05 已完成最小 box-constrained QP-IK。本文档记录的是早期规划步骤，当前 A05 已推进到最小可运行实现。

## 1. 为什么 A05 当时先做规划骨架

A05 要从 A04 的无约束 DLS IK 进入 task + limit + QP-IK。这个跨度包括任务抽象、姿态任务、速度限制、关节位置限制、QP 目标函数和 solver 选择。如果直接写成最小可运行实现，容易把数学对象、单位、日志字段和后续 A06/A07/A08 边界混在一起。

因此 Step 13A 当时先做学习型规划骨架：先明确每个模块要实现什么、为什么存在、对标 mink 的哪个概念、推荐 API、输入输出和验证方式。当前 A05 已完成最小 box-constrained QP-IK。

## 2. A05 与 A04 的关系

A04 已完成 position-mode DLS differential IK，证明 `attachment_site` 可以通过 Jacobian 驱动位置误差收敛。A05 已继续复用 A01-A04 的模型、site、Jacobian 验证和 A04 q trajectory，把无约束 DLS 扩展成最小 box-constrained QP-IK。

A04 解决“能不能用 Jacobian 收敛”；A05 已完成“如何在 task weights 和 limits 下安全地求 `dq`”的最小实现。

## 3. 本次 Markdown 文档补充的算法细节

`projects/A_self_baseline/docs/05_task_limit_qp_ik.md` 已规划以下算法内容：

- QP-IK 核心形式 `J_task dq ≈ v_task`。
- least-squares 到标准 QP 的推导。
- FrameTask 的 position/orientation error 与 Jacobian。
- PostureTask 的参考姿态误差和 identity Jacobian。
- VelocityLimit 与 JointPositionLimit 的区别和合并方式。
- box-constrained QP 在 OSQP / scipy bounds 中的表达。
- 符号表、物理意义、伪代码、验证标准和常见错误。

## 4. A05 与 A06/A07/A08 的边界

- A05 已完成最小受约束 IK 轨迹生成。
- A06 才定义 target tracking。
- A07 才执行 actuator tracking / MuJoCo 控制。
- A08 才在 QP 框架上加入 collision avoidance。
- Step 13A 不生成 video，不调用 mink，不修改外部 mink 源码。

## 5. 修改文件清单

- `projects/A_self_baseline/scripts/05_task_limit_qp_ik.py`
- `projects/A_self_baseline/docs/05_task_limit_qp_ik.md`
- `docs/00_project_management/step13A_A05_task_limit_qp_ik_todo_skeleton.md`
- `projects/A_self_baseline/README.md`
- `projects/A_self_baseline/docs/A_pipeline_contract.md`
- `projects/A_self_baseline/docs/A_mink_alignment_plan.md`
- `projects/A_self_baseline/docs/A_simulation_only_full_motion_control_plan.md`

## 6. TODO 任务清单

1. 读取 A01/A02/A03/A04 前置产物。
2. 读取 `robot.yaml` / `qp_ik.yaml`。
3. 加载 MuJoCo model / data。
4. 定义 FrameTask。
5. 定义 PostureTask。
6. 组合任务目标。
7. 构造 QP 目标函数。
8. 定义 VelocityLimit。
9. 定义 JointPositionLimit。
10. 构造 QP 约束。
11. 选择 QP solver。
12. 迭代更新 `q`。
13. 记录轨迹、误差和约束日志。
14. 规划误差图输出。
15. 规划 Markdown report 输出。
16. 说明 A05 不进入 A06/A07/A08。

## 7. 历史未实现算法说明

Step 13A 当时不在 Python 中实现真实 QP-IK。当前状态已经更新：A05 已完成最小 box-constrained QP-IK，并可以生成受约束轨迹。

未实现内容包括：

- actuator tracking。
- MuJoCo 控制 rollout。
- collision avoidance。
- video recording。
- mink 替代实现。

## 8. 未修改 external / legacy

本步骤未修改：

- `external/mink_upstream`
- `legacy_imported`

## 9. 下一步

Step 13B：A05 minimal box-constrained QP-IK 已完成。

当前第一版已实现 position-only FrameTask + VelocityLimit + JointPositionLimit，并验证：

- `H/c/lower/upper` 维度正确。
- `dq` 满足 box constraints。
- final position error 小于 initial position error。
- 不进入 actuator tracking。

## 10. 验收清单

- [x] 只补充 TODO skeleton。
- [x] Markdown 文档包含 QP-IK 算法细节。
- [x] Markdown 文档包含 QP 推导、task、limit、符号表、物理意义、伪代码、验证标准和常见错误。
- [x] 当前 A05 已完成最小 box-constrained QP-IK。
- [x] 未实现 actuator tracking。
- [x] 未实现 collision avoidance。
- [x] 未调用 mink。
- [x] 未修改 external/mink_upstream。
- [x] 未修改 legacy_imported。
- [x] py_compile 通过。
