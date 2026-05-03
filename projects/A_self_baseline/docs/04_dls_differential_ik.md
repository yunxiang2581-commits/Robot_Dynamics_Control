# A04 DLS Differential IK

## 1. A04 当前定位

A04 是 DLS differential IK，位于 A03 site Jacobian check 之后，A05 task + limit + QP-IK 之前。当前仍是 TODO skeleton 阶段：Python 脚本只规划学习骨架，不实现完整 IK。

本步骤不实现 QP、actuator tracking、MuJoCo 控制、collision avoidance 或 video recording，也不调用 mink 替代自己的实现。

## 2. 为什么 A04 在 A03 之后

A03 已验证：

```text
site velocity = J(q) dq
```

DLS IK 依赖可信的 Jacobian。如果 Jacobian 的 site id、坐标方向、维度或 dq 索引错误，IK 会朝错误方向更新，误差可能不降反升。

A03 的意义是先确认 `J(q) dq` 与 finite difference velocity 对齐；A04 才能把这个 Jacobian 用来从任务空间误差求关节速度。

## 3. A01 / A02 / A03 已确认的前置数据

| 项目 | 当前值 |
| --- | --- |
| `nq` | 6 |
| `nv` | 6 |
| `nu` | 6 |
| target site | `attachment_site` |
| target body | `wrist_3_link` |
| q source | `keyframe:home` |
| A03 small-step linear velocity error norm | 约 `2.55e-7` |
| A03 small-step angular velocity error norm | 约 `3.01e-10` |

说明：A03 当前脚本还包含 multi-step sweep，用于观察大位移下局部线性化误差增长；A04 判断 Jacobian API 是否可信时，优先看小步 finite difference 误差。

## 4. Differential IK 是什么

普通 IK 试图直接求一个关节配置：

```text
q such that x_target = f(q)
```

differential IK 不直接一次求完整 `q`，而是在当前 `q` 附近求一个小的 `dq`，通过多次迭代让末端逐步靠近目标。它是局部线性化方法。

运动学函数写成：

```text
x = f(q)
```

速度层关系是：

```text
dot{x} = J(q) dot{q}
```

对于小步长：

```text
Delta x ≈ J(q) Delta q
```

因此，如果当前末端离目标还有一个小误差，就可以用 Jacobian 反推一个关节更新方向。

## 5. 任务空间误差定义

第一版 A04 只做 position IK：

```text
e = x_target - x_current
```

其中：

- `x_current` 是当前 `attachment_site` position。
- `x_target` 是目标 position。
- `e` 是三维位置误差。
- 第一版只减少三维位置误差，不处理 orientation error。

## 6. 为什么第一版只做 position IK

position IK 只需要 `J_pos`：

```text
J_pos shape = (3, nv)
```

UR5e 当前：

```text
nv = 6
```

所以第一版问题是用 6 个关节速度自由度减少 3 维末端位置误差。这个问题足够学习 Jacobian、DLS、damping、gain、dt 和迭代收敛。

orientation IK 涉及 SO(3)、旋转矩阵误差、四元数误差、角速度坐标系和姿态插值，容易把 A04 的核心学习目标变复杂，因此后续再扩展。

## 7. DLS 公式与推导

最小二乘目标是让 Jacobian 预测速度接近期望的任务空间速度：

```text
minimize ||J dq - gain * e||^2
```

普通最小二乘或伪逆会遇到几个问题：

- `J` 不一定是方阵。
- `J` 可能接近奇异。
- `dq` 可能过大。
- 目标可能不可达。

为了让求解更稳定，引入阻尼项：

```text
minimize ||J dq - gain * e||^2 + lambda ||dq||^2
```

这里 `lambda` 是 damping 对应的阻尼权重。它的物理意义是惩罚过大的关节速度，让解在奇异附近更温和。

DLS 的常用解法写成：

```text
dq = J.T @ solve(J @ J.T + lambda * I, gain * e)
```

注意：有些实现把阻尼写成 `damping`，有些写成 `damping^2`。本项目后续实现时必须在变量命名和报告中保持一致，例如明确：

```text
lambda = damping
```

或：

```text
lambda = damping^2
```

A04 TODO skeleton 保留数学结构：

```text
e = target - current
dq = J.T @ solve(J @ J.T + lambda * I, gain * e)
q_next = integrate(q, dq, dt)
```

## 8. 符号含义表

| 符号 | 含义 | 维度 | 在本项目中的来源 |
|---|---|---|---|
| `q` | 当前 MuJoCo configuration | `(nq,) = (6,)` | A02 `keyframe:home` 或迭代后的 q |
| `dq` | 关节速度 / 小步更新方向 | `(nv,) = (6,)` | DLS 公式求解 |
| `x_current` | 当前末端 site position | `(3,)` | `data.site_xpos[site_id]` |
| `x_target` | 目标末端 position | `(3,)` | current position + target offset |
| `e` | 任务空间位置误差 | `(3,)` | `x_target - x_current` |
| `J_pos` | site position Jacobian | `(3, nv) = (3, 6)` | `mujoco.mj_jacSite` |
| `lambda / damping` | DLS 阻尼权重 | 标量 | CLI / `ik.yaml` |
| `gain` | 误差缩放系数 | 标量 | CLI / `ik.yaml` |
| `dt` | q 积分步长 | 标量 | CLI / `ik.yaml` |
| `q_next` | 下一步 configuration | `(nq,) = (6,)` | `mujoco.mj_integratePos` |
| `tolerance` | 收敛阈值 | 标量 | CLI / `ik.yaml` |
| `max_iter` | 最大迭代次数 | 标量 | CLI / `ik.yaml` |

## 9. 物理意义

`J` 告诉我们关节速度如何影响末端速度。`e` 告诉我们末端离目标还差多少。DLS 求一个温和稳定的 `dq`，使末端朝目标方向移动。

`damping` 防止速度爆炸，尤其在 Jacobian 接近奇异或目标方向难以实现时。`gain` 控制每步向目标靠近的强度，过小会慢，过大会震荡。`dt` 控制每次积分更新的步长，过大可能破坏局部线性化假设。

## 10. A04 迭代流程

伪代码如下。注意：这是算法说明，不是当前 Python 脚本的完整实现。

```text
1. load model and data
2. q = keyframe home
3. current = site position
4. target = current + small offset
5. for iter in max_iter:
   - forward q
   - current = site position
   - e = target - current
   - if norm(e) < tolerance: break
   - compute J_pos
   - dq = DLS(J_pos, e, damping, gain)
   - q = integrate(q, dq, dt)
   - log error
6. save q_traj
7. save error log
8. save report
```

## 11. 推荐 API

未来最小实现推荐使用：

- `model_loader.load_yaml_config`
- `model_loader.resolve_path`
- `model_loader.load_mujoco_model`
- `mujoco.MjData`
- `mujoco.mj_forward`
- `mujoco.mj_jacSite`
- `mujoco.mj_integratePos` 或固定基下的 `q + dq * dt`
- `numpy.linalg.solve`
- `numpy.linalg.norm`
- `numpy.save`
- `csv.DictWriter`
- `matplotlib`

## 12. 输入与输出

输入：

- `A01_model_summary.json`
- `A02_site_pose.json`
- `A03_jacobian_check.json`
- `scene.xml`
- target offset
- damping
- gain
- dt
- max_iter
- tolerance

输出：

- `outputs/trajectories/A04_dls_ik_q_traj.npy`
- `outputs/logs/A04_dls_ik_error.csv`
- `outputs/figures/A04_dls_ik_error.png`
- `outputs/reports/A04_dls_ik_report.md`

## 13. 验证标准

A04 后续最小实现成功的标准：

- 能加载同一个 `scene.xml`。
- `q0` 来自 `keyframe:home`。
- target site 是 `attachment_site`。
- target position 是 current position + small offset。
- `J_pos shape = (3, 6)`。
- `dq shape = (6,)`。
- `dq` 无 NaN。
- error norm 下降。
- final error 小于 initial error。
- `q_traj` 长度与 error log 一致。
- 不进入 QP 或 actuator tracking。

## 14. 常见错误

- `J_pos` 和 `J_rot` 混用。
- `dq` 维度使用 `nq` 而不是 `nv`。
- damping 太小导致 `dq` 过大。
- gain 太大导致震荡。
- dt 太大导致更新不稳定。
- target offset 太大导致一步线性化失效。
- 忘记每轮 `mj_forward`。
- 误把 body pose 当 site pose。
- 未检查 NaN。
- 在 A04 过早加入 joint limit / QP。

## 15. 与 A05 的关系

A04 是无约束 DLS IK。A05 才加入 `FrameTask`、`PostureTask`、`ConfigurationLimit`、`VelocityLimit`，并把问题写成 QP。

A04 的 `q_traj` 可以作为 A05/A06/A07 的参考：A05 对同样目标加入限制，A06 把 target 管理起来，A07 把轨迹送入 MuJoCo actuator tracking。

## 16. 当前不做什么

- 不做 QP。
- 不做 joint limit。
- 不做 velocity limit。
- 不做 collision avoidance。
- 不做 actuator tracking。
- 不做 video。
- 不调用 mink 替代自己的实现。

## 17. 下一步

Step 12B：A04 minimal DLS differential IK。

再下一步 A05：task + limit + QP-IK。
