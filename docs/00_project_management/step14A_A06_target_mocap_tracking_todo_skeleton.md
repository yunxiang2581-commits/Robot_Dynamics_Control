# Step 14A - A06 Target / Mocap-Style Tracking TODO Skeleton

## 1. 为什么 A06 先做 TODO skeleton

A06 连接 IK 和 actuator tracking，但它本身不是控制器。先做 TODO skeleton 可以把 target pose、target sequence、mocap-style target 和 `trajectory_source` 的数据边界讲清楚，避免过早把 A06 写成 A07 控制闭环。

当前只规划输入、输出、数据结构和验证方式，不实现真实 target tracking。

## 2. A06 与 A05 的关系

A05 已完成最小 box-constrained QP-IK，并输出 `A05_qp_ik_q_traj.npy`。A06 第一版优先使用 A05 轨迹作为 `trajectory_source`，同时保留 A04 DLS 轨迹作为无约束对照。

A06 不重新实现 A05 的 QP-IK。

## 3. A06 与 A07 的边界

A06 负责 target generation / target management。A07 才负责 actuator tracking。

A06 不调用 `data.ctrl`，不运行 MuJoCo control loop，不录视频，不输出 actuator torque / ctrl。

## 4. 本次 Markdown 文档补充的算法细节

`projects/A_self_baseline/docs/06_target_mocap_tracking.md` 已补充：

- target pose 表达。
- fixed target。
- pose sequence。
- mocap-style target。
- target error 公式。
- 符号表。
- 物理意义。
- 未来实现伪代码。
- 验证标准。
- 常见错误。
- 与 A04/A05/A07 和 mink 的关系。

## 5. 修改文件清单

- `projects/A_self_baseline/scripts/06_target_mocap_tracking.py`
- `projects/A_self_baseline/docs/06_target_mocap_tracking.md`
- `projects/A_self_baseline/configs/target_tracking.yaml`
- `docs/00_project_management/step14A_A06_target_mocap_tracking_todo_skeleton.md`
- `projects/A_self_baseline/README.md`
- `projects/A_self_baseline/docs/A_pipeline_contract.md`
- `projects/A_self_baseline/docs/A_mink_alignment_plan.md`
- `projects/A_self_baseline/docs/A_simulation_only_full_motion_control_plan.md`

## 6. TODO 任务清单

1. 读取 A02 / A04 / A05 前置产物。
2. 读取 `robot.yaml` / `target_tracking.yaml`。
3. 定义 target pose 数据结构。
4. fixed target 模式。
5. pose sequence 模式。
6. mocap-style target placeholder。
7. target error 定义。
8. IK backend 选择。
9. target tracking log 规划。
10. target definition JSON 规划。
11. target path preview figure 规划。
12. Markdown report 规划。
13. 说明 A06 不进入 A07。
14. 说明后续 A06 最小实现路线。

每个 TODO 都在 Python skeleton 中写明要做什么、为什么存在、对标 mink 的哪个概念、推荐 API、输入、输出和验证方式。

## 7. target_tracking.yaml 配置说明

`target_tracking.yaml` 是 A06 的配置模板，不要求当前运行。主要字段：

- `target.site_name`：默认 `attachment_site`。
- `target.body_name`：默认 `wrist_3_link`。
- `target.mode`：默认 `fixed_pose`。
- `target.position_offset`：默认 `[0.03, 0.0, 0.0]`。
- `target.orientation_mode`：默认 `keep_current`。
- `tracking.ik_backend`：默认 `a05_qp`。
- `tracking.num_waypoints` / `tracking.duration`：供 pose sequence 使用。
- `mocap.enabled`：当前为 `false`，只作 placeholder。
- `outputs`：规划未来 A06 JSON、CSV、figure、report 路径。

## 8. 未实现算法说明

本步未实现：

- fixed target 最小可运行逻辑。
- pose sequence 生成。
- mocap body 写入。
- target tracking log 输出。
- report 输出。
- actuator tracking。
- MuJoCo 控制闭环。
- video recording。
- collision avoidance。

核心逻辑继续保留 `NotImplementedError`。

## 9. 未修改外部和 legacy

本步未修改：

- `external/mink_upstream`
- `legacy_imported`

本步不下载外部仓库，不调用 mink 替代自己的实现。

## 10. 下一步

Step 14B：A06 fixed target + trajectory metadata minimal implementation。

目标是读取 A02/A04/A05 前置产物，选择 `trajectory_source`，生成 `A06_target_definition.json` 和最小 report。仍不进入 A07 actuator tracking。

## 11. 验收清单

- [x] 只补充 A06 TODO skeleton。
- [x] Markdown 文档包含 target pose / mocap-style tracking 算法细节。
- [x] Markdown 文档包含公式、符号表、物理意义、伪代码、验证标准和常见错误。
- [x] 未实现 actuator tracking。
- [x] 未实现 MuJoCo 控制闭环。
- [x] 未实现 video。
- [x] 未调用 mink。
- [x] 未修改 external/mink_upstream。
- [x] 未修改 legacy_imported。
- [x] py_compile 通过。
