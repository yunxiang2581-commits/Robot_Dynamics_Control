# A03 Site Jacobian Check

## 1. A03 当前定位

A03 是 `site Jacobian check` 的 TODO learning skeleton。当前阶段只整理学习任务、输入输出边界、推荐 API 和验证方式，不实现真实 Jacobian 计算，也不实现 finite difference 验证。

A03 要学习的核心关系是：

```text
site velocity = J(q) dq
```

其中 `q` 是 MuJoCo configuration，`dq` 是关节速度扰动，`J(q)` 是在当前 configuration 下 target site 的 Jacobian。

## 2. 为什么 A03 在 A02 之后

A02 已经确认了 `q -> MuJoCo data -> site/body pose` 的数据流，并输出了 `outputs/cache/A02_site_pose.json`。A03 必须建立在这个结果之上，因为 Jacobian 是在某一个具体 `q` 下定义的。

如果还没有确认 target site 的 pose，就直接进入 Jacobian，会导致后续无法判断 finite difference 前后的 site position 是否对应同一个目标对象。

## 3. A01/A02 已确认的前置数据

来自 A01 的模型检查结果：

| 项目 | 当前值 |
| --- | --- |
| `nq` | 6 |
| `nv` | 6 |
| `nu` | 6 |
| target site | `attachment_site` |
| target body | `wrist_3_link` |
| keyframe | `home` |

来自 A02 的 site/body pose 结果：

| 项目 | 当前值 |
| --- | --- |
| q source | `keyframe:home` |
| target site | `attachment_site` |
| target body | `wrist_3_link` |
| cache | `outputs/cache/A02_site_pose.json` |
| report | `outputs/reports/A02_site_pose_report.md` |

## 4. 对标 mink 的 differential IK velocity mapping

mink 的 `solve_ik` 会在任务空间误差和关节速度之间建立映射。A03 不调用 mink，也不实现 IK，只学习这个映射背后的基础对象：site Jacobian。

在 A 项目中，A03 只规划如何得到 `J(q)`，并用有限差分检查 `J(q) dq` 是否能预测 site position 的瞬时速度。

## 5. `site velocity = J(q) dq`

Jacobian 可以理解成 configuration 变化到 site 速度的局部线性映射：

```text
J(q): dq -> site velocity
```

未来 A03 会分别关注：

| 对象 | 含义 |
| --- | --- |
| `J_pos` | site 平移速度对关节速度的映射 |
| `J_rot` | site 角速度对关节速度的映射 |
| `J_6d` | 平移和旋转 Jacobian 的组合形式 |

## 6. linear Jacobian 与 angular Jacobian

linear Jacobian 对应 site position 的速度预测，通常用于检查：

```text
v_pos = J_pos @ dq
```

angular Jacobian 对应 site orientation 的角速度预测。rotation 的有限差分比 position 更复杂，因此第一版最小实现应优先验证 position velocity，angular check 先保留 TODO 和设计说明。

## 7. finite difference 验证思想

有限差分验证的基本思路是：

```text
q_next = q + dq * dt
v_fd = (site_position(q_next) - site_position(q)) / dt
```

如果 `dt` 足够小，并且 q 更新方式符合 MuJoCo 的 position space 规则，那么 `v_fd` 应该接近 `J_pos @ dq`。A03 的报告需要记录误差范数，并说明不同 `dt` 下误差是否符合预期。

## 8. A03 输入

| 输入 | 说明 |
| --- | --- |
| `configs/robot.yaml` | A 项目模型配置入口 |
| `outputs/cache/A01_model_summary.json` | A01 模型维度和对象名称摘要 |
| `outputs/cache/A02_site_pose.json` | A02 使用的 q、site、body 和 pose 摘要 |
| `scene.xml` | MuJoCo MJCF 模型 |
| target site | 默认 `attachment_site` |
| q | 默认来自 A02 的 `keyframe:home` |
| dq | 用于测试的关节速度扰动 |
| dt | finite difference 时间步长 |

## 9. A03 未来输出

| 输出 | 说明 |
| --- | --- |
| `outputs/cache/A03_jacobian_check.json` | JSON-serializable Jacobian check 摘要 |
| `outputs/reports/A03_jacobian_check_report.md` | 可复盘的 Markdown 报告 |
| `outputs/figures/A03_jacobian_fd_error.png` | 不同 `dt` 下的有限差分误差图 |

## 10. TODO 1-14 任务表

| TODO | 要做什么 | 为什么 | 对标 mink | 推荐 API | 输入 | 输出 | 验证 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 读取 A01/A02 前置产物 | 保证 Jacobian 使用同一模型和同一 q | configuration/task 前置状态 | `json.loads`, `Path.read_text` | A01 summary, A02 pose cache | model summary, pose summary | `nq=6`, `nv=6`, target site 存在 |
| 2 | 读取 `robot.yaml` 并解析 `scene.xml` | 保持 A01/A02/A03 使用同一 MJCF | model loading | `model_loader.load_yaml_config`, `model_loader.resolve_path` | `robot.yaml` | `mjcf_path` | `scene.xml exists=True` |
| 3 | 加载 MuJoCo model 和 data | Jacobian 需要 `MjModel` 和 `MjData` | MuJoCo backend | `model_loader.load_mujoco_model`, `mujoco.MjData` | `mjcf_path` | model, data | `model.nq` 和 A01 对齐 |
| 4 | 恢复 A02 使用的 q | Jacobian 必须在同一 configuration 下检查 | `Configuration.q` | `model.key_qpos`, `numpy.asarray` | q source | q | `q.shape == (model.nq,)` |
| 5 | 选择 dq 测试向量 | 需要速度扰动验证 `J(q)dq` | task velocity input | `numpy.zeros`, joint lookup | `--joint-name` 或 `--dq-source` | dq | `dq.shape == (model.nv,)` 且 norm > 0 |
| 6 | 计算 site Jacobian | 得到速度映射矩阵 | task Jacobian | `mujoco.mj_forward`, `mujoco.mj_jacSite` | model, data, site id | `J_pos`, `J_rot`, `J_6d` | shape 为 `(3, nv)` |
| 7 | 计算 Jacobian 预测速度 | 验证 differential IK 的核心映射 | `solve_ik` 内部速度映射 | `numpy.matmul` | `J_pos`, `J_rot`, dq | predicted site velocity | 维度正确，无 NaN |
| 8 | 有限差分验证 position velocity | 检查 `J_pos @ dq` 是否可信 | Jacobian numerical check | `mujoco.mj_integratePos`, `mujoco.mj_forward`, `data.site_xpos` | q, dq, dt | finite difference velocity | 与 `J_pos @ dq` 误差较小 |
| 9 | 规划 angular velocity 验证 | rotation 差分更复杂，需要单独说明 | rotational task Jacobian | rotation matrix/quaternion helper | site rotation before/after | angular velocity check | 文档说明暂缓原因 |
| 10 | 组织 A03 summary | 给 A04 提供可读、可缓存的检查结果 | task diagnostic summary | `dict`, `float` conversion | Jacobian check 结果 | JSON-serializable summary | `json.dumps` 通过 |
| 11 | 规划 JSON 输出 | 固化后续步骤的机器可读输入 | cached task state | `json.dump`, `Path.write_text` | summary | `A03_jacobian_check.json` | 字段完整，可读取 |
| 12 | 规划 Markdown report 输出 | 固化学习解释和误差结论 | example report | Markdown string, `Path.write_text` | summary | `A03_jacobian_check_report.md` | 能说明 `site velocity = J(q)dq` |
| 13 | 规划误差图输出 | 观察 finite difference 随 `dt` 的收敛趋势 | numerical validation plot | `matplotlib` | dt list, error norm list | `A03_jacobian_fd_error.png` | 图能显示误差趋势 |
| 14 | 说明 A03 不进入 A04/A05 | 保持学习边界清晰 | pipeline boundary | logging, docs | A03 边界说明 | 日志和文档说明 | 不出现 IK/QP/tracking 实现 |

## 11. 当前不做什么

A03 当前 TODO skeleton 不做以下内容：

- 不实现真实 Jacobian 计算。
- 不实现 finite difference 验证。
- 不实现 IK。
- 不实现 QP。
- 不实现 target tracking。
- 不实现 actuator tracking。
- 不实现 collision avoidance。
- 不实现 video recording。
- 不调用 mink 替代自己的实现。

## 12. 下一步 A03 最小实现

下一步 Step 11B 将进入 A03 minimal Jacobian finite difference check。届时只实现最小可运行的 `attachment_site` position Jacobian 和有限差分验证，并输出 JSON、Markdown report 和可选误差图。

## 13. 再下一步 A04

A04 是 DLS differential IK。A04 会使用 A03 验证过的 `J(q)`，根据 task-space error 求解关节速度 `dq`。因此 A03 的重点是让 Jacobian 和 finite difference check 足够可信，而不是提前进入 IK。
