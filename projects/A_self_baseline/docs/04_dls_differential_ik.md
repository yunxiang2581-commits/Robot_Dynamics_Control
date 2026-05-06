# A04 DLS Differential IK

## 1. A04 当前定位

A04 是 DLS differential IK，位于 A03 site Jacobian check 之后，A05 task + limit + QP-IK 之前。当前 A04 已完成最小 position-mode DLS differential IK。

本步骤不实现 QP、actuator tracking、MuJoCo 控制、collision avoidance 或 video recording，也不调用 mink 替代自己的实现。A04 已输出 q trajectory、error log、error figure 和 report。

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

A04 最小实现使用的数学结构：

```text
e = target - current
dq = J.T @ solve(J @ J.T + lambda * I, gain * e)
q_next = integrate(q, dq, dt)
```

## 7A. Pose-aware DLS IK 扩展规划

### 为什么 A04 要考虑旋转

机械臂末端任务通常不只是到达某个点，还包括以正确姿态到达。抓取、插入、对接和工具操作都需要 orientation；如果只跟踪 position，末端可能已经到点，但工具方向仍然错误。

A05 对标 mink 的 `FrameTask`，而 `FrameTask` 天然包含 position + orientation。因此 A04 先把 pose-aware DLS 的接口和数学边界写清楚：Step 12B 继续保留并验证 position mode，Step 12C 再扩展 pose_6d mode。

### position-only IK

第一版 position-only IK 使用三维位置误差：

```text
e_pos = p_target - p_current
```

其中：

```text
J_pos in R^{3 x nv}
```

DLS 更新为：

```text
dq = J_pos.T @ solve(J_pos J_pos.T + lambda I, gain e_pos)
```

### 为什么不能用欧拉角直接相减

orientation error 不建议直接用欧拉角相减，原因包括：

- 欧拉角存在奇异性。
- 欧拉角顺序相关，例如 xyz 和 zyx 表达的含义不同。
- 角度存在 wrap-around，例如 `179 deg` 和 `-179 deg` 的直接差值会误导误差大小。
- 小角度下欧拉角差可以近似使用，但不适合作为统一实现。

### SO(3) rotation error

pose-aware IK 应使用旋转矩阵误差：

```text
R_err = R_target R_current^T
```

再用 SO(3) log map 得到三维旋转向量：

```text
e_rot = log(R_err)
```

其中：

- `R_current` 是当前 `attachment_site` 的 rotation matrix。
- `R_target` 是目标 rotation matrix。
- `e_rot` 是 rotation vector。
- `e_rot` 的方向是旋转轴。
- `e_rot` 的模长是旋转角。
- `e_rot` 的单位是 rad。

注意 `R_err` 的顺序必须和后续 `J_rot` 使用的角速度坐标系保持一致。顺序写反会让姿态误差方向反过来。

### 6D pose error

position 和 rotation 的单位不同，不能不加权直接拼接。A04 规划使用：

```text
e_task = [w_pos e_pos;
          w_rot e_rot]
```

其中：

- `w_pos` 是 position weight。
- `w_rot` 是 orientation weight。
- 位置单位是 m。
- 旋转单位是 rad。
- 权重用于平衡数值尺度和任务优先级。

position mode 是这个公式的三维特例：

```text
e_task = w_pos e_pos
```

### 6D task Jacobian

MuJoCo `mj_jacSite` 可以提供 site 的 linear Jacobian 和 angular Jacobian：

```text
J_task = [w_pos J_pos;
          w_rot J_rot]
```

其中：

- `J_pos` 映射关节速度到 site linear velocity。
- `J_rot` 映射关节速度到 site angular velocity。
- position mode 中 `J_task shape = (3, nv)`。
- pose_6d mode 中 `J_task shape = (6, nv)`。

position mode 的三维形式是：

```text
J_task = w_pos J_pos
```

### Pose-aware DLS

统一的 DLS 公式写成：

```text
dq = J_task.T @ solve(J_task J_task.T + lambda I, gain e_task)
```

实现时必须检查：

- `e_task` 维度是 3 或 6。
- `I` 的维度必须匹配 task dimension。
- `dq` 的维度必须是 `nv`。
- `lambda / damping` 控制奇异附近和大误差时的稳定性。

### pose-aware 符号含义表

| 符号 | 含义 | 维度 | 单位 | 来源 |
|---|---|---|---|---|
| `q` | 当前 MuJoCo configuration | `(nq,) = (6,)` | rad | A02 `keyframe:home` 或迭代后的 q |
| `dq` | DLS 求出的关节速度 / 更新方向 | `(nv,) = (6,)` | rad/s 或 rad/update | DLS 公式 |
| `p_current` | 当前 site position | `(3,)` | m | `data.site_xpos[site_id]` |
| `p_target` | 目标 site position | `(3,)` | m | current position + target position offset |
| `R_current` | 当前 site rotation matrix | `(3, 3)` | 无量纲 | `data.site_xmat[site_id]` reshape |
| `R_target` | 目标 site rotation matrix | `(3, 3)` | 无量纲 | keep_current / fixed_rpy / fixed_quat |
| `e_pos` | position error | `(3,)` | m | `p_target - p_current` |
| `e_rot` | SO(3) rotation error vector | `(3,)` | rad | `log(R_target R_current^T)` |
| `e_task` | 加权任务误差 | `(3,)` 或 `(6,)` | 加权后混合单位 | position / pose_6d 分支 |
| `J_pos` | site linear Jacobian | `(3, nv)` | m/rad | `mujoco.mj_jacSite` |
| `J_rot` | site angular Jacobian | `(3, nv)` | rad/rad | `mujoco.mj_jacSite` |
| `J_task` | 加权任务 Jacobian | `(3, nv)` 或 `(6, nv)` | 加权后混合单位 | `J_pos` / `J_rot` 组合 |
| `w_pos` | position weight | 标量 | 1/m 或调参权重 | CLI / `ik.yaml` |
| `w_rot` | orientation weight | 标量 | 1/rad 或调参权重 | CLI / `ik.yaml` |
| `lambda / damping` | DLS 阻尼权重 | 标量 | 调参权重 | CLI / `ik.yaml` |
| `gain` | task error 缩放系数 | 标量 | 调参权重 | CLI / `ik.yaml` |
| `dt` | q 积分步长 | 标量 | s 或 update scale | CLI / `ik.yaml` |

### 未来实现伪代码

```text
load model and data
q = keyframe home
forward q
p_current, R_current = current site pose
p_target = p_current + target_position_offset

if target_orientation_mode == keep_current:
    R_target = R_current
elif target_orientation_mode == fixed_rpy:
    R_target = rotation_from_rpy(...)
elif target_orientation_mode == fixed_quat:
    R_target = rotation_from_quat(...)

for iter in max_iter:
    forward q
    p_current, R_current = current site pose
    e_pos = p_target - p_current
    J_pos, J_rot = site Jacobian

    if task_mode == position:
        e_task = w_pos * e_pos
        J_task = w_pos * J_pos
        orientation_error_norm = 0
    elif task_mode == pose_6d:
        R_err = R_target R_current^T
        e_rot = log(R_err)
        e_task = concat(w_pos * e_pos, w_rot * e_rot)
        J_task = stack(w_pos * J_pos, w_rot * J_rot)

    if norm(e_task) < tolerance:
        break

    dq = J_task.T @ solve(J_task J_task.T + lambda I, gain e_task)
    q = integrate(q, dq, dt)
    log position_error_norm, orientation_error_norm, task_error_norm, dq_norm
```

### pose-aware 验证标准

- position mode 下 position error 下降。
- pose_6d mode 下 position error 和 orientation error 都应下降。
- keep_current 时 orientation error 初始应接近 0。
- `J_task` shape 正确：position mode 为 `(3, nv)`，pose_6d mode 为 `(6, nv)`。
- `dq shape = (nv,)`。
- `dq`、`e_task`、`J_task` 没有 NaN。
- 不进入 QP 或 actuator tracking。

### pose-aware 常见错误

- 欧拉角直接相减。
- `R_err` 顺序写反。
- `J_rot` 和 `e_rot` 坐标系不一致。
- position 和 rotation 不加权直接拼接。
- damping 太小导致 `dq` 爆炸。
- gain 太大导致震荡。
- dt 太大导致不稳定。
- `J_task` 行数和 `I` 维度不一致。
- `dq` 用 `nq` 维度而不是 `nv`。
- 忘记每轮 `mj_forward`。
- 误把 body rotation 当 site rotation。

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

A04 minimal DLS differential IK 已完成。A05 task + limit + QP-IK 也已完成最小 box-constrained QP-IK；下一步进入 A06 target / mocap-style tracking。
