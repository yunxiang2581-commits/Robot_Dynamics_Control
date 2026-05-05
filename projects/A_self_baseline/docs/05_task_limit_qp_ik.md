# A05 Task + Limit + QP-IK

## 1. A05 当前定位

A05 是从 A04 DLS IK 到 mink-style QP-IK 的过渡步骤。A04 已经完成 position-only 的无约束 DLS differential IK，证明 `attachment_site` 的位置误差可以被 Jacobian 驱动收敛；A05 则开始引入 task、weight、velocity limit、joint position limit 和 posture preference。

当前 A05 仍是 TODO learning skeleton，不实现真实 QP 求解器，不生成轨迹输出，也不调用 mink 替代自己的实现。

## 2. 为什么 A05 在 A04 之后

A04 已证明 `J dq` 可以解释末端局部运动，并能把末端位置误差从初始值逐步降下来。但是 A04 的 DLS 解没有显式考虑关节位置边界、速度边界和多个任务之间的权重关系。

A05 用 QP 统一处理 task 和 limit，更接近 `mink.solve_ik` 背后的思想：先把任务写成二次代价，再把机器人限制写成约束，最后由 QP solver 在目标和限制之间求一个可执行的 `dq`。

## 3. A04 的局限

- 不考虑 joint position limit。
- 不考虑 velocity limit。
- 不考虑 posture preference。
- 不考虑 task priority / weight。
- 不方便加入 collision avoidance。
- 不方便扩展多任务。

## 4. QP-IK 的核心思想

QP-IK 仍然是 differential IK。每一步不直接求完整目标姿态对应的最终 `q`，而是求当前状态下的一小步关节速度 `dq`：

```text
J_task dq ≈ v_task
v_task = gain * e_task
```

最小二乘目标写成：

```text
minimize ||J_task dq - v_task||² + damping ||dq||²
```

第一项让机器人追踪任务速度，第二项限制 `dq` 过大。这个 damping 和 A04 DLS 的阻尼思想一致，都用于改善奇异点附近或病态 Jacobian 下的数值稳定性。不同之处在于，QP 可以继续加入显式约束，例如速度上下限和关节位置上下限。

## 5. 标准 QP 形式

A05 未来采用 box-constrained QP：

```text
minimize 1/2 dq^T H dq + c^T dq
subject to lower <= dq <= upper
```

从最小二乘项展开：

```text
||J dq - v||²
= (J dq - v)^T (J dq - v)
= dq^T J^T J dq - 2 v^T J dq + v^T v
```

如果把 damping 一起放入二次项，可以取：

```text
H = 2(J^T J + damping I)
c = -2 J^T v
```

如果代码使用标准 `1/2 dq^T H dq` 约定，也可以统一写成：

```text
H = J^T J + damping I
c = -J^T v
```

本项目后续代码建议统一使用第二种 1/2 约定：`H = J_task.T @ J_task + damping * I`，`c = -J_task.T @ v_task`。`H` 应该是对称半正定矩阵，加上正的 damping 后会更稳定。

## 6. FrameTask

FrameTask 控制末端 `attachment_site`。位置误差为：

```text
e_pos = p_target - p_current
```

姿态误差可写成：

```text
R_err = R_target R_current^T
e_rot = log(R_err)
```

加权后的 task error：

```text
e_frame = [w_pos e_pos; w_rot e_rot]
```

对应 Jacobian：

```text
J_frame = [w_pos J_pos; w_rot J_rot]
```

position-only 模式可以只取前三行，此时 `J_frame.shape == (3, nv)`；pose_6d 模式使用位置和姿态，此时 `J_frame.shape == (6, nv)`。

## 7. PostureTask

PostureTask 让当前关节位置接近参考姿态：

```text
e_posture = q_ref - q
J_posture = I
```

参考姿态可以是 keyframe `home`，也可以是 A04 初始 `q`。它的作用是抑制不必要关节运动，让解更接近自然姿态，并提升轨迹稳定性。UR5e 虽然是 6DOF，不是强冗余机械臂，但 PostureTask 仍可作为稳定项，避免任务弱约束方向出现不必要摆动。

## 8. 任务组合

把 FrameTask 和 PostureTask 组合成统一 least-squares task：

```text
J_task = stack([
  J_frame,
  w_posture * I
])

v_task = concat([
  gain_frame * e_frame,
  gain_posture * e_posture
])
```

不同 task 通过权重平衡。权重过大会压制其它任务；权重过小则该任务几乎不起作用。A05 的学习重点之一就是观察这些权重如何影响收敛速度、姿态偏移和约束余量。

## 9. VelocityLimit

速度限制写成：

```text
dq_min <= dq <= dq_max
```

`dq` 如果表示 joint velocity，单位是 rad/s；如果表示 joint increment，单位是 rad。本项目应统一为 joint velocity，然后用 `dt` 积分：

```text
q_next = q + dq dt
```

这样 velocity limit 的单位、joint position limit 的推导和日志含义都更清楚。

## 10. JointPositionLimit

关节位置限制是积分后不能越界：

```text
q_min <= q + dq dt <= q_max
```

推导为速度空间约束：

```text
(q_min - q) / dt <= dq <= (q_max - q) / dt
```

与 VelocityLimit 合并：

```text
lower = max(dq_velocity_min, (q_min - q) / dt)
upper = min(dq_velocity_max, (q_max - q) / dt)
```

每次求解前都必须检查 `lower <= upper`。如果某个关节已经非常接近边界，可行速度范围会变窄，QP 可能牺牲一部分 task tracking 来保持可行。

## 11. Box-constrained QP

第一版 A05 可以只做 box constraints：

```text
lower <= dq <= upper
```

如果使用 OSQP，可写成：

```text
A = I
l = lower
u = upper
```

即：

```text
l <= A dq <= u
```

如果使用 `scipy.optimize`，可以通过 bounds 表达同样的上下限。

## 12. 符号含义表

| 符号 | 含义 | 维度 | 单位 | 来源 |
|---|---|---|---|---|
| q | 当前关节位置 | `(nq,)` | rad | MuJoCo `data.qpos` / A04 trajectory |
| dq | 当前步关节速度 | `(nv,)` | rad/s | QP solver |
| dt | 积分步长 | scalar | s | `qp_ik.yaml` |
| J_frame | 末端任务 Jacobian | `(3,nv)` 或 `(6,nv)` | m/rad, rad/rad | `mujoco.mj_jacSite` |
| J_posture | 姿态保持 Jacobian | `(nv,nv)` | 1 | identity |
| J_task | 组合任务 Jacobian | `(m,nv)` | mixed | stack tasks |
| e_frame | 末端位置/姿态误差 | `(3,)` 或 `(6,)` | m, rad | target pose 与 current pose |
| e_posture | 姿态参考误差 | `(nv,)` | rad | `q_ref - q` |
| v_task | 期望任务速度 | `(m,)` | mixed/s | `gain * error` |
| H | QP 二次项矩阵 | `(nv,nv)` | cost | `J_task.T @ J_task + damping I` |
| c | QP 线性项 | `(nv,)` | cost | `-J_task.T @ v_task` |
| damping | 阻尼系数 | scalar | cost | `qp_ik.yaml` |
| lower | 合并后的速度下界 | `(nv,)` | rad/s | velocity 与 position limit |
| upper | 合并后的速度上界 | `(nv,)` | rad/s | velocity 与 position limit |
| q_min | 关节位置下界 | `(nv,)` | rad | MuJoCo model / config |
| q_max | 关节位置上界 | `(nv,)` | rad | MuJoCo model / config |
| dq_min | 关节速度下界 | `(nv,)` | rad/s | `qp_ik.yaml` |
| dq_max | 关节速度上界 | `(nv,)` | rad/s | `qp_ik.yaml` |
| w_pos | 位置任务权重 | scalar 或 `(3,)` | cost/m | `qp_ik.yaml` |
| w_rot | 姿态任务权重 | scalar 或 `(3,)` | cost/rad | `qp_ik.yaml` |
| w_posture | PostureTask 权重 | scalar | cost/rad | `qp_ik.yaml` |

## 13. 物理意义

FrameTask 让末端去目标；PostureTask 让机器人姿态更自然；VelocityLimit 防止动作太快；JointPositionLimit 防止积分后越过机械边界；damping 防止解在奇异点或病态 Jacobian 附近发散。QP solver 的作用是在这些目标和限制之间求一个折中：尽量完成任务，但不能违反明确约束。

## 14. 未来实现伪代码

```text
1. load model/data/config
2. q = keyframe home or A04 final q
3. define target pose
4. for iter:
   - forward q
   - compute frame error
   - compute frame Jacobian
   - compute posture error
   - build J_task and v_task
   - build H and c
   - build velocity limit
   - build joint position limit
   - merge lower/upper
   - solve QP
   - integrate q
   - log error and constraint violation
5. save q_traj
6. save error log
7. save constraint log
8. save report
```

## 15. 验证标准

- QP 维度正确。
- `H shape = (nv, nv)`。
- `c shape = (nv,)`。
- `lower / upper shape = (nv,)`。
- `lower <= upper`。
- `dq shape = (nv,)`。
- `dq` 无 NaN。
- 约束 violation 接近 0。
- final error 小于 initial error。
- 与 A04 相比不一定更快，但更受约束。
- 不进入 actuator tracking。

## 16. 常见错误

- `dq` 含义混乱：velocity 和 increment 混用。
- `dt` 忘记用于 joint position limit。
- `lower > upper` 没有检查。
- `H` 不对称。
- damping 太小导致数值不稳定。
- task weight 过大导致 posture task 失效。
- posture weight 过大导致末端不收敛。
- `J_frame` 和 `e_frame` 行数不一致。
- position 和 rotation 不加权直接拼接。
- joint limit 单位不一致。
- 在 A05 过早加入 collision avoidance。
- 在 A05 直接做 actuator tracking。

## 17. 与 mink 的关系

mink 的完整实现是 QP-based differential IK。A05 是本项目更贴近 `mink.solve_ik` 的模块，但本项目不调用 mink 替代自己的实现。A05 的目标是通过自己构造 task、limit 和 QP，学习 mink 背后的思想，而不是把学习过程封装掉。

## 18. 与 A06/A07/A08 的关系

A06 使用 A05 的 `q_traj` 做 target tracking；A07 用 actuator tracking 执行 A05 轨迹；A08 在 A05 的 QP 框架上加入 collision avoidance constraint。A05 的输出是后续控制和 demo 的关键输入，但 A05 本身只负责受约束 IK 轨迹生成。

## 19. 当前不做什么

- 不实现真实 QP。
- 不做 actuator tracking。
- 不做 collision avoidance。
- 不做 video。
- 不调用 mink 替代实现。

## 20. 下一步

Step 13B 将进入 A05 minimal box-constrained QP-IK。第一版可先做 position-only FrameTask + VelocityLimit，再扩展 PostureTask、JointPositionLimit 和 pose_6d task。
