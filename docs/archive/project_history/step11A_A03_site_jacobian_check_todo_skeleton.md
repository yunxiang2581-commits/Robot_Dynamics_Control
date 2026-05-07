# Step 11A - A03 Site Jacobian Check TODO Skeleton

当前状态更新：A03 已完成最小可运行 site Jacobian finite difference check。本文档记录的是早期规划步骤。

## 1. 为什么 A03 先做 TODO skeleton

A03 是从 pose 查询进入 differential kinematics 的第一步。它要学习的不是直接写出 IK，而是先弄清楚 `site velocity = J(q) dq` 的输入、输出、API、验证方式和边界。

先做 TODO skeleton 的目的，是把 Jacobian、finite difference、JSON/report/figure 输出和 A04 的连接关系拆清楚，避免在还没有验证 velocity mapping 之前就进入 DLS IK 或 QP。

## 2. A03 与 A02 的关系

A02 已完成最小可运行 site/body pose 查询，并输出：

| 文件 | 作用 |
| --- | --- |
| `projects/A_self_baseline/outputs/cache/A02_site_pose.json` | 记录 q source、q、target site/body 和 pose |
| `projects/A_self_baseline/outputs/reports/A02_site_pose_report.md` | 解释 `q -> data -> site/body pose` 数据流 |

A03 依赖 A02 的 q、target site 和 site pose。A03 的 Jacobian 必须在 A02 确认的同一个 configuration 上进行检查。

## 3. A03 与 A04/A05 的边界

A03 只验证 velocity mapping：

```text
site velocity = J(q) dq
```

A04 才会使用 `J(q)` 和 task-space error 求解 DLS differential IK。A05 才会把 task、limit 和 QP 约束组织起来。因此 Step 11A 不实现 IK、不实现 QP，也不做 target tracking 或 actuator tracking。

## 4. 修改文件清单

| 文件 | 修改类型 |
| --- | --- |
| `projects/A_self_baseline/scripts/03_site_jacobian_check.py` | 重构为详细 A03 TODO learning skeleton |
| `projects/A_self_baseline/docs/03_site_jacobian_check.md` | 新增 A03 学习说明文档 |
| `docs/00_project_management/step11A_A03_site_jacobian_check_todo_skeleton.md` | 新增 Step 11A 管理文档 |
| `projects/A_self_baseline/README.md` | 追加 A03 TODO 状态说明 |
| `projects/A_self_baseline/docs/A_pipeline_contract.md` | 追加 A03 TODO 状态说明 |
| `projects/A_self_baseline/docs/A_mink_alignment_plan.md` | 追加 A03 TODO 状态说明 |

## 5. TODO 任务清单

| TODO | 主题 |
| --- | --- |
| 1 | 读取 A01/A02 前置产物 |
| 2 | 读取 `robot.yaml` 和解析 `scene.xml` |
| 3 | 加载 MuJoCo model 和 data |
| 4 | 恢复 A02 使用的 q |
| 5 | 选择 dq 测试向量 |
| 6 | 计算 site Jacobian |
| 7 | 计算 Jacobian 预测速度 |
| 8 | 有限差分验证 position velocity |
| 9 | 规划 angular velocity 验证 |
| 10 | 组织 A03 summary |
| 11 | 规划 JSON 输出 |
| 12 | 规划 Markdown report 输出 |
| 13 | 规划误差图输出 |
| 14 | 说明 A03 不进入 A04/A05 |

每个 TODO 都按 pinocchio-learning 约束写清楚：要做什么、为什么存在、对标 mink 的哪个概念、推荐 API、输入、输出和如何验证。

## 6. 未实现算法说明

本步骤只补充 TODO learning skeleton：

- 未实现真实 Jacobian 计算。
- 未实现 finite difference 验证。
- 未实现 IK。
- 未实现 QP。
- 未实现 WBC。
- 未实现 MuJoCo 控制。
- 未实现 collision avoidance。
- 未实现 video recording。
- 未调用 mink 替代自己的实现。

## 7. 未修改外部目录

本步骤未修改：

- `external/mink_upstream/`
- `legacy_imported/`

## 8. 后续 Step 11B 状态

Step 11B 已完成 A03 minimal Jacobian finite difference check。当前已实现最小可运行的 `attachment_site` linear Jacobian 和 finite difference 验证，并生成 A03 cache/report/figure 输出。

## 9. 验收清单

- [x] 当时只补充 TODO skeleton；当前 A03 已完成最小实现。
- [x] 未实现 Jacobian。
- [x] 未实现 finite difference。
- [x] 未实现 IK / QP。
- [x] 未调用 mink。
- [x] 未修改 external/mink_upstream。
- [x] 未修改 legacy_imported。
- [x] py_compile 通过。
