# A02 Configuration / Site Pose Status

## 1. A02 当前定位

A02 是 A00-A10 pipeline 的第二个学习入口：configuration / site pose。

当前状态：**A02 最小可运行 site/body pose 已完成**。

本步骤已经实现 `q -> MuJoCo data -> site/body pose` 的最小数据流，并写出 JSON cache 与 Markdown report。当前仍不实现 Jacobian、finite difference、IK 或 QP。

## 2. 为什么 A02 在 A01 之后

A01 已经确认 MuJoCo 模型维度和对象名称。A02 必须依赖这些结果，避免靠猜测写 site、body 或 keyframe 名称。

A02 的任务是：在 A01 确认的模型上，选择一个 configuration `q`，刷新 MuJoCo `data`，然后读取目标 site/body 的世界系 pose。

## 3. A01 已确认的前置数据

- `nq = 6`
- `nv = 6`
- `nu = 6`
- joint names: `shoulder_pan`, `shoulder_lift`, `elbow`, `wrist_1`, `wrist_2`, `wrist_3`
- site names: `attachment_site`
- actuator names: `shoulder_pan`, `shoulder_lift`, `elbow`, `wrist_1`, `wrist_2`, `wrist_3`
- keyframe names: `home`
- end-effector candidates:
  - `attachment_site=True`
  - `wrist_3_link=True`
  - `tool0=False`
  - `ee_link=False`

## 4. 对标 mink 的 Configuration

mink 的 `Configuration` 概念把当前机器人状态 `q`、MuJoCo model/data 和 frame/site pose 查询组织在一起。

A 项目不直接调用 mink，而是先手动理解同一条数据流：

```text
q
  -> MuJoCo MjData
  -> mujoco.mj_forward
  -> data.site_xpos / data.site_xmat
  -> data.xpos / data.xmat
  -> site/body pose summary
```

## 5. q -> data -> site/body pose 数据流

1. 从 A01 summary 和 robot.yaml 确认模型、site、body 和 keyframe。
2. 加载同一个 scene.xml。
3. 创建 `mujoco.MjData(model)`。
4. 选择 `q`，第一版优先规划 keyframe `home`。
5. 把 `q` 写入 `data.qpos`。
6. 调用 `mujoco.mj_forward(model, data)`。
7. 从 `data.site_xpos` / `data.site_xmat` 读取 site pose。
8. 从 `data.xpos` / `data.xmat` 读取 body pose。

## 6. site 与 body 的区别

body 是 MuJoCo 模型中的刚体节点，例如 `wrist_3_link`。它表示机械臂末端 link 的刚体坐标。

site 是 MJCF 中附着在模型上的任务点或测量点，例如 `attachment_site`。IK 和 Jacobian 往往更关心 site，因为它可以表示工具中心点或任务目标点。

A02 同时规划 site pose 和 body pose，是为了让后续 A03/A04 能明确“任务点”和“末端刚体”的差异。

## 7. A02 输入

- `projects/A_self_baseline/configs/robot.yaml`
- `projects/A_self_baseline/outputs/cache/A01_model_summary.json`
- `shared/robot_assets/models/mink_universal_robots_ur5e/scene.xml`
- target site: `attachment_site`
- target body: `wrist_3_link`
- q source: `keyframe:home`、zero/default q、文件轨迹或 CLI joint values

## 8. A02 当前输出

- `outputs/cache/A02_site_pose.json`
- `outputs/reports/A02_site_pose_report.md`

当前检查结果摘要：

- q source: `keyframe:home`
- target site: `attachment_site`
- site position: `[0.49199929841248197, 0.1339978254660598, 0.48800036731899227]`
- target body: `wrist_3_link`
- body position: `[0.4919989310933209, 0.13399819278791936, 0.5880003673176429]`
- model dimensions: `nq=6`, `nv=6`, `nu=6`

JSON 字段至少包括：

- `q_source`
- `q`
- `site_name`
- `site_position`
- `site_rotation_matrix`
- `body_name`
- `body_position`
- `body_rotation_matrix`
- `model_nq`
- `model_nv`
- `model_nu`
- `source_mjcf`

## 9. TODO 1-11 状态表

| TODO | 任务 | 验证重点 |
|---|---|---|
| TODO 1 | 读取 A01 model summary | 已完成：`nq=6, nv=6, nu=6` 且 `attachment_site` 存在 |
| TODO 2 | 读取 robot.yaml 和解析 scene.xml | 已完成：scene.xml exists=True |
| TODO 3 | 加载 MuJoCo model 并创建 data | 已完成：`data.qpos` 对齐 `nq`，`data.qvel` 对齐 `nv` |
| TODO 4 | 选择 q source | 已完成第一版：`keyframe:home`，`q.shape == (model.nq,)` |
| TODO 5 | 执行 forward kinematics 数据刷新 | 已完成：调用 `mujoco.mj_forward` |
| TODO 6 | 查询 site pose | 已完成：position=(3,), rotation=(3,3) |
| TODO 7 | 查询 body pose | 已完成：position=(3,), rotation=(3,3) |
| TODO 8 | 组织 pose summary | 已完成：summary 可 `json.dumps` |
| TODO 9 | 规划 JSON 输出 | 已完成：JSON 可读取，字段完整 |
| TODO 10 | 规划 Markdown report 输出 | 已完成：报告可读，明确 target site/body |
| TODO 11 | 说明 A02 不进入 A03/A04 | 已完成：日志和文档明确不做 Jacobian/IK/QP |

## 10. 当前不做什么

当前 A02 不做：

- Jacobian
- finite difference
- IK
- QP
- WBC
- MuJoCo 控制
- collision avoidance
- video recording
- mink 替代实现

## 11. 下一步 A03

A02 最小实现完成后，下一步进入 A03 site Jacobian check。

A03 会在 A02 确认的同一个 q、同一个 `attachment_site` 上学习：

```text
site velocity = J(q) dq
```

并用 finite difference 验证 Jacobian。A02 当前只负责 pose 数据流，不负责速度映射。
