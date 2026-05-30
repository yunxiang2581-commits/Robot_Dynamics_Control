# Step 10C - A02 Minimal Site Pose Status

## 1. 当前状态

A02 已从 TODO learning skeleton 推进到最小可运行 configuration / site pose。

当前实现只做 `q -> MuJoCo data -> site/body pose`，不实现 Jacobian、finite difference、IK、QP、WBC、MuJoCo 控制或 video recording。

## 2. 输入

- A01 summary：`projects/A_self_baseline/outputs/cache/A01_model_summary.json`
- 配置文件：`projects/A_self_baseline/configs/robot.yaml`
- MuJoCo MJCF：`shared/robot_assets/models/mink_universal_robots_ur5e/scene.xml`
- target site：`attachment_site`
- target body：`wrist_3_link`
- q source：`keyframe:home`

## 3. 输出

- `projects/A_self_baseline/outputs/cache/A02_site_pose.json`
- `projects/A_self_baseline/outputs/reports/A02_site_pose_report.md`

## 4. 当前检查结果

| 项目 | 结果 |
|---|---|
| q source | `keyframe:home` |
| model `nq` | 6 |
| model `nv` | 6 |
| model `nu` | 6 |
| target site | `attachment_site` |
| target body | `wrist_3_link` |

site position:

```text
[0.49199929841248197, 0.1339978254660598, 0.48800036731899227]
```

body position:

```text
[0.4919989310933209, 0.13399819278791936, 0.5880003673176429]
```

## 5. 学习意义

A02 的作用是把 A01 确认的模型对象推进到具体 configuration 下的位姿查询。后续 A03 不应该重新猜 q 或 target site，而应复用 A02 的 `A02_site_pose.json`。

这一步对标 mink 的 `Configuration`：从 `q` 更新 MuJoCo `data`，然后读取 site/body pose。

## 6. 未实现内容

本步骤未实现：

- Jacobian
- finite difference
- IK
- QP-IK
- collision avoidance
- actuator tracking
- video recording
- mink 替代实现

## 7. 下一步

下一步进入 A03 site Jacobian check：

- 输入 A02 确认的 `q` 和 `attachment_site`。
- 调用 MuJoCo Jacobian API。
- 学习 `site velocity = J(q) dq`。
- 用 finite difference 验证 Jacobian。

不要在 A03 扩展到 IK 或 QP。
