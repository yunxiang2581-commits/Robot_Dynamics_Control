# Step 10B - A02 Configuration Site Pose TODO Skeleton

## 1. 为什么 A02 先做 TODO skeleton

A02 是从 A01 model inspect 进入 configuration / site pose 的第一步。它看起来像 FK，但当前重点不是完整实现算法，而是先把 `q -> MuJoCo data -> site/body pose` 的数据流、输入输出和边界写清楚。

先做 TODO skeleton 可以避免直接跳到 Jacobian、IK 或 QP，也能让后续 Step 10C 只补 A02 最小实现。

## 2. A02 与 A01 的关系

A01 已完成最小可运行 model inspect，并确认：

- `nq=6`
- `nv=6`
- `nu=6`
- site names: `attachment_site`
- body names 包含 `wrist_3_link`
- keyframe names: `home`

A02 未来应读取 `outputs/cache/A01_model_summary.json`，确认目标 site/body 存在，然后使用同一个 scene.xml 查询 pose。

## 3. A02 与 A03/A04 的边界

A02 只负责 pose 数据流：

```text
q -> data -> site/body pose
```

A03 才负责 Jacobian 和 finite difference：

```text
site velocity = J(q) dq
```

A04 才负责 DLS differential IK。

因此 Step 10B 不实现 Jacobian、finite difference、IK、QP、WBC、MuJoCo 控制或 video recording。

## 4. 修改文件清单

- `projects/A_self_baseline/scripts/02_configuration_site_pose.py`
- `projects/A_self_baseline/docs/02_configuration_site_pose.md`
- `docs/00_project_management/step10B_A02_configuration_site_pose_todo_skeleton.md`
- `projects/A_self_baseline/README.md`
- `projects/A_self_baseline/docs/A_pipeline_contract.md`
- `projects/A_self_baseline/docs/A_mink_alignment_plan.md`

## 5. TODO 任务清单

1. 读取 A01 model summary。
2. 读取 robot.yaml 和解析 scene.xml。
3. 加载 MuJoCo model 并创建 data。
4. 选择 q source。
5. 执行 forward kinematics 数据刷新。
6. 查询 site pose。
7. 查询 body pose。
8. 组织 pose summary。
9. 规划 JSON 输出。
10. 规划 Markdown report 输出。
11. 说明 A02 不进入 A03/A04。

每个 TODO 都在脚本中写明：要做什么、为什么这一步存在、对标 mink 的哪个概念、推荐 API、输入、输出和如何验证。

## 6. 未实现算法说明

本步骤未实现：

- site pose 查询
- body pose 查询
- Jacobian
- finite difference
- IK
- QP
- WBC
- MuJoCo 控制
- collision avoidance
- video recording
- mink 替代实现

核心逻辑继续保留 `NotImplementedError`。

## 7. 未修改外部与 legacy

本步骤未修改：

- `external/mink_upstream`
- `projects/A_self_baseline/external/mink`
- `legacy_imported`

本步骤未下载外部仓库。

## 8. 原计划 Step 10C

原计划 Step 10C：A02 最小可运行 site pose。

这些内容已在后续状态更新中完成：

- 读取 A01 summary。
- 读取 robot.yaml。
- 加载 scene.xml。
- 创建 `mujoco.MjData(model)`。
- 选择 keyframe `home` 作为第一版 q source。
- 调用 `mujoco.mj_forward(model, data)`。
- 读取 `attachment_site` 和 `wrist_3_link` 的 pose。
- 写出 `A02_site_pose.json` 和 `A02_site_pose_report.md`。

## 9. 验收清单

- [x] 只补充 TODO skeleton；
- [x] 未实现 site pose 查询；
- [x] 未实现 Jacobian / IK / QP；
- [x] 未调用 mink；
- [x] 未修改 external/mink_upstream；
- [x] 未修改 legacy_imported；
- [x] py_compile 通过。

## 10. 后续状态更新

A02 已从 TODO skeleton 推进到最小可运行 configuration / site pose：

- 已读取 `outputs/cache/A01_model_summary.json`。
- 已读取 `configs/robot.yaml` 并解析同一个 `scene.xml`。
- 已加载 MuJoCo model 并创建 `mujoco.MjData(model)`。
- 已使用 `keyframe:home` 作为第一版 q source。
- 已调用 `mujoco.mj_forward(model, data)`。
- 已读取 `attachment_site` 的 position 和 rotation matrix。
- 已读取 `wrist_3_link` 的 position 和 rotation matrix。
- 已写出 `outputs/cache/A02_site_pose.json`。
- 已写出 `outputs/reports/A02_site_pose_report.md`。

当前 A02 检查结果：

- q source: `keyframe:home`
- target site: `attachment_site`
- site position: `[0.49199929841248197, 0.1339978254660598, 0.48800036731899227]`
- target body: `wrist_3_link`
- body position: `[0.4919989310933209, 0.13399819278791936, 0.5880003673176429]`

当前仍不做：

- Jacobian
- finite difference
- IK
- QP
- WBC
- MuJoCo 控制
- video recording

下一步进入 A03 site Jacobian check。
