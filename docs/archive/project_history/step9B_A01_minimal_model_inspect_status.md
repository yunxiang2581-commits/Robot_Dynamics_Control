# Step 9B - A01 Minimal Model Inspect Status

## 1. 当前状态

A01 已从 TODO learning skeleton 推进到最小可运行 model inspect。

当前实现只做模型检查和报告输出，不实现 FK、Jacobian、IK、QP、WBC、MuJoCo 控制或 video recording。

## 2. 输入

- 配置文件：`projects/A_self_baseline/configs/robot.yaml`
- MuJoCo MJCF：`shared/robot_assets/models/mink_universal_robots_ur5e/scene.xml`
- 末端候选：`attachment_site`、`tool0`、`ee_link`、`wrist_3_link`

## 3. 输出

- `projects/A_self_baseline/outputs/cache/A01_model_summary.json`
- `projects/A_self_baseline/outputs/reports/A01_model_inspect_report.md`

## 4. 当前检查结果

| 项目 | 结果 |
|---|---|
| `nq` | 6 |
| `nv` | 6 |
| `nu` | 6 |
| joint count | 6 |
| body count | 10 |
| site count | 1 |
| actuator count | 6 |
| keyframe count | 1 |

关键名称：

- joints: `shoulder_pan`, `shoulder_lift`, `elbow`, `wrist_1`, `wrist_2`, `wrist_3`
- site: `attachment_site`
- keyframe: `home`
- actuators: `shoulder_pan`, `shoulder_lift`, `elbow`, `wrist_1`, `wrist_2`, `wrist_3`

末端候选检查：

- `attachment_site`: found
- `tool0`: missing
- `ee_link`: missing
- `wrist_3_link`: found

## 5. 学习意义

A01 的作用是先确认模型维度和对象名称。A02-A07 不应该靠猜测直接写 site、body、actuator 名称，而应使用 A01 的报告和 JSON cache 作为输入依据。

这一步对标 mink UR5e 示例中的 MuJoCo model loading 和 `Configuration` 前置模型检查。

## 6. 未实现内容

本步骤未实现：

- FK / site pose
- Jacobian
- IK
- QP-IK
- collision avoidance
- actuator tracking
- video recording
- mink 替代实现

## 7. 下一步

下一步进入 A02 configuration / site pose：

- 输入 A01 确认的 MJCF 模型和 `attachment_site`。
- 创建 `mujoco.MjData`。
- 写入或选择一个 `q`。
- 调用 `mujoco.mj_forward`。
- 读取 `data.site_xpos` 和 `data.site_xmat`。
- 输出 A02 site pose report/cache。

不要在 A02 扩展到 Jacobian、IK 或 QP。
