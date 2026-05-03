# Step 12A - A04 DLS Differential IK TODO Skeleton

## 1. 为什么 A04 先做 TODO skeleton

A04 是从已验证 Jacobian 进入 IK 的第一步。DLS differential IK 涉及误差定义、Jacobian、阻尼、gain、积分更新、收敛判断和日志输出。先做 TODO skeleton 可以把算法边界讲清楚，避免在还没有统一符号和验证标准前直接写完整 IK。

## 2. A04 与 A03 的关系

A03 已验证 `site velocity = J(q) dq`，并输出：

- `outputs/cache/A03_jacobian_check.json`
- `outputs/reports/A03_jacobian_check_report.md`
- `outputs/figures/A03_jacobian_fd_error.png`

A04 会依赖 A03 已验证的 target site、初始 q 和 `J_pos` 维度约定。A04 不重新定义目标 site，也不跳过 Jacobian 检查。

## 3. Markdown 文档补充的算法细节

`projects/A_self_baseline/docs/04_dls_differential_ik.md` 已补充：

- differential IK 的定义。
- `x = f(q)`、`dot{x} = J(q) dot{q}`、`Delta x ≈ J(q) Delta q`。
- position error `e = x_target - x_current`。
- DLS 目标函数和阻尼项。
- DLS 解 `dq = J.T @ solve(J @ J.T + lambda * I, gain * e)`。
- 符号含义表。
- 物理意义。
- 伪代码。
- 推荐 API。
- 输入输出。
- 验证标准。
- 常见错误。

## 4. A04 与 A05/A07 的边界

A04 只做无约束 DLS differential IK 的学习骨架。A05 才加入 task、limit 和 QP-IK。A07 才做 MuJoCo actuator tracking。

本步骤不实现：

- QP-IK。
- joint limit。
- velocity limit。
- actuator tracking。
- MuJoCo 控制。
- collision avoidance。
- video recording。

## 5. 修改文件清单

- `projects/A_self_baseline/scripts/04_dls_differential_ik.py`
- `projects/A_self_baseline/docs/04_dls_differential_ik.md`
- `docs/00_project_management/step12A_A04_dls_differential_ik_todo_skeleton.md`
- `projects/A_self_baseline/README.md`
- `projects/A_self_baseline/docs/A_pipeline_contract.md`
- `projects/A_self_baseline/docs/A_mink_alignment_plan.md`
- `projects/A_self_baseline/docs/A_simulation_only_full_motion_control_plan.md`

## 6. TODO 任务清单

| TODO | 主题 |
| --- | --- |
| 1 | 读取 A01 / A02 / A03 前置产物 |
| 2 | 读取 robot.yaml / ik.yaml 并解析 scene.xml |
| 3 | 加载 MuJoCo model 和 data |
| 4 | 恢复初始 q |
| 5 | 定义 target pose |
| 6 | 计算当前 site pose 和误差 e |
| 7 | 计算 site position Jacobian J_pos |
| 8 | DLS 求解 dq |
| 9 | 积分更新 q |
| 10 | 迭代终止条件 |
| 11 | 记录 q trajectory 和 error log |
| 12 | 规划误差图输出 |
| 13 | 规划 Markdown report 输出 |
| 14 | 说明 A04 不进入 A05/A07 |

## 7. 未实现算法说明

本步骤只补充 TODO skeleton 和算法学习文档，未在 Python 中实现 DLS IK。核心逻辑仍保留 `NotImplementedError`。

未实现：

- DLS IK。
- QP-IK。
- actuator tracking。
- MuJoCo 控制。
- collision avoidance。
- video recording。

## 8. 未修改外部目录

本步骤未修改：

- `external/mink_upstream/`
- `legacy_imported/`

## 9. 下一步 Step 12B

下一步 Step 12B：A04 minimal DLS differential IK。届时只实现 position-only DLS IK，并输出 q trajectory、error log、error figure 和 report。

## 10. 验收清单

- [x] 只补充 TODO skeleton。
- [x] Markdown 文档包含 DLS IK 算法细节。
- [x] Markdown 文档包含公式、符号、物理意义、伪代码、验证标准和常见错误。
- [x] 未在 Python 中实现 DLS IK。
- [x] 未实现 QP-IK。
- [x] 未实现 actuator tracking。
- [x] 未调用 mink。
- [x] 未修改 external/mink_upstream。
- [x] 未修改 legacy_imported。
- [x] py_compile 通过。
