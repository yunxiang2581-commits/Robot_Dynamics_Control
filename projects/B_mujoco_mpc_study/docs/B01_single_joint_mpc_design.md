# B01 单关节 MPC 闭环设计文档

## 1. 本步骤在学习链条中的位置

B01 的目标不是直接写出完整控制器，而是先用最小系统理解 MPC 闭环：

```text
当前状态 -> 预测未来 -> 评价 cost -> 选 torque 序列 -> 只执行第一个 torque -> 进入下一轮
```

它对应 Project B 的第一步：

```text
B01 单关节 MPC -> B02 二连杆 MPC -> B03 predictive sampling -> B04+ 人形模型 MPC
```

单关节系统足够小，便于先学清楚这些概念：

- 状态 state 是什么。
- 控制 control 是什么。
- rollout 为什么需要“当前状态 + 未来 torque 序列”。
- cost 为什么要同时惩罚角度误差、速度误差和控制量。
- MPC 为什么每次只执行最优序列的第一个 torque。

本阶段不追求复杂优化器，也不进入人形机器人接触、浮动基、WBC 或动力学全链路。

## 2. 最小物理对象

B01 只考虑一个可转动关节，例如一个绕固定轴转动的 pendulum / hinge joint。

系统可以先抽象成：

```text
q   : 当前关节角度
dq  : 当前关节角速度
tau : 当前关节 torque
```

如果后续用 MuJoCo 实现，`q` 对应 `qpos` 中该关节的位置，`dq` 对应 `qvel` 中该关节的速度，`tau` 对应 actuator control 或外加力矩。

## 3. 状态是什么

B01 的状态定义为：

```text
x = [q, dq]
```

含义：

- `q`：关节角度，表示当前关节转到了哪里。
- `dq`：关节角速度，表示当前关节正在以多快的速度运动。

为什么只用这两个量：

- 对单自由度机械系统来说，位置和速度通常足够描述当前运动状态。
- MPC 需要从当前状态出发预测未来，所以状态必须包含能推进动力学仿真的信息。
- 后续扩展到多关节时，状态会自然变成 `qpos + qvel`。

## 4. 控制是什么

B01 的控制量定义为：

```text
u = tau
```

含义：

- `tau` 是施加在单关节上的力矩。
- 在每个控制步，控制器最终只会真正执行一个 torque。
- 在优化时，MPC 会临时考虑一串未来 torque：

```text
U = [tau_0, tau_1, tau_2, ..., tau_{H-1}]
```

这里 `H` 是 horizon 长度，也就是一次 MPC 向未来预测多少步。

## 5. 目标是什么

B01 的目标是让关节角度跟踪一个目标角：

```text
q_target = target angle
```

第一版建议使用常数目标角，例如：

```text
q_target = 0.8 rad
```

暂时不做复杂目标轨迹，因为当前学习重点是 MPC 闭环结构，而不是轨迹生成。

目标可以拆成两个期望：

- 角度接近目标：`q` 接近 `q_target`。
- 到达目标后尽量停住：`dq` 接近 `0`。

## 6. Rollout 输入是什么

rollout 的作用是：

```text
给定当前状态和一串未来 torque，预测未来会发生什么。
```

输入：

```text
current_state = [q_now, dq_now]
torque_sequence = [tau_0, tau_1, ..., tau_{H-1}]
```

输出：

```text
state_sequence = [
    [q_1, dq_1],
    [q_2, dq_2],
    ...,
    [q_H, dq_H],
]
```

如果用 MuJoCo，rollout 的核心就是：

1. 从当前 `qpos, qvel` 复制一份临时仿真状态。
2. 依次设置 `ctrl = tau_k`。
3. 每设置一次 torque，就调用一次或多次 `mj_step`。
4. 记录每一步得到的 `q, dq`。

注意：rollout 是“假设这个 torque 序列被执行以后会怎样”，不代表这些 torque 都会真的被执行。

## 7. Cost 怎么算

B01 的 cost 用三个部分组成：

```text
cost = angle_cost + velocity_cost + torque_cost
```

### 7.1 角度误差 cost

```text
angle_error = q_k - q_target
angle_cost = w_q * angle_error^2
```

作用：

- 惩罚当前预测角度偏离目标角。
- `w_q` 越大，控制器越重视快速靠近目标角。

### 7.2 速度误差 cost

```text
velocity_error = dq_k - 0
velocity_cost = w_dq * velocity_error^2
```

作用：

- 惩罚过大的角速度。
- 避免关节到达目标附近后继续高速冲过目标。

### 7.3 torque penalty

```text
torque_cost = w_tau * tau_k^2
```

作用：

- 惩罚过大的控制力矩。
- 避免控制器为了降低角度误差而输出非常激进的 torque。
- 让控制序列更平滑，更接近真实控制器可接受的行为。

### 7.4 Horizon 总 cost

对整个 horizon 累加：

```text
total_cost = sum(
    w_q * (q_k - q_target)^2
    + w_dq * dq_k^2
    + w_tau * tau_k^2
)
```

其中 `k = 0 ... H-1`。

第一版可以先使用简单权重：

```text
w_q = 10.0
w_dq = 1.0
w_tau = 0.01
```

这些权重只是教学起点，不是最终调参结论。

## 8. 每次执行什么

MPC 的关键点是：

```text
虽然优化得到一整串 torque，但每次只执行第一个 torque。
```

假设本轮找到的最优序列是：

```text
U_best = [tau_0*, tau_1*, tau_2*, ..., tau_{H-1}*]
```

真实系统只执行：

```text
tau_apply = tau_0*
```

然后系统前进一步，得到新的状态：

```text
x_next = [q_next, dq_next]
```

下一轮 MPC 会重新从 `x_next` 出发，再生成新的候选 torque 序列、重新 rollout、重新算 cost、重新选择最优序列。

这就是 receding horizon control：

```text
看未来很多步，但只执行当前一步。
```

为什么这样做：

- 真实系统每一步都会受到模型误差、仿真误差、扰动和数值误差影响。
- 如果一次性执行完整序列，中途偏了也无法修正。
- 每步重新观察状态、重新规划，可以形成闭环反馈。

## 9. 最小闭环流程

B01 的控制循环可以设计为：

```text
for control_step in range(num_control_steps):
    1. 读取当前状态 x_now = [q_now, dq_now]
    2. 生成若干条候选 torque 序列
    3. 对每条候选序列做 rollout
    4. 对每条 rollout 轨迹计算 total_cost
    5. 选择 cost 最低的 torque 序列 U_best
    6. 只执行 U_best 的第一个 torque
    7. 真实仿真前进一步
    8. 记录 q, dq, tau, cost, error
```

第一版候选 torque 序列可以先用简单方式生成，例如：

- 固定几个 torque 值组合。
- 在上一轮最优序列附近加入随机扰动。
- 从均匀分布中采样 torque 序列。

当前不要求实现高级优化器。

## 10. 输入、输出和数学逻辑变化

### 输入

- 初始角度 `q0`。
- 初始角速度 `dq0`。
- 目标角 `q_target`。
- horizon 长度 `H`。
- 每轮候选 torque 序列数量。
- torque 上限。
- cost 权重 `w_q, w_dq, w_tau`。

### 输出

- 每个真实控制步执行的 torque。
- 每个真实控制步的 `q, dq`。
- 每个真实控制步的角度误差。
- 每轮最优 rollout 的 cost。
- 最终误差、平均误差、最大 torque、平均计算耗时。

### 数学逻辑是否变化

B01 不改变已有 A 项目的 FK / Jacobian / IK 数学逻辑。

它新增的是 MPC 的时间维度：

```text
A 项目：当前状态下解一个瞬时目标
B01：从当前状态出发，预测未来多步，再选择当前要执行的 torque
```

## 11. 推荐产出

本步骤建议只产出文档，不写完整代码：

```text
projects/B_mujoco_mpc_study/docs/B01_single_joint_mpc_design.md
```

下一步再产出 TODO skeleton：

```text
projects/B_mujoco_mpc_study/simulator/B01_single_joint_mpc_demo.py
```

后续真正运行 demo 时，再按 Project B 约定输出：

```text
projects/B_mujoco_mpc_study/outputs/runs/B01_single_joint_mpc_demo/<run_id>/logs/B01_run_log.txt
projects/B_mujoco_mpc_study/outputs/runs/B01_single_joint_mpc_demo/<run_id>/metrics/B01_metrics.csv
projects/B_mujoco_mpc_study/outputs/runs/B01_single_joint_mpc_demo/<run_id>/figures/B01_angle_error.png
projects/B_mujoco_mpc_study/outputs/runs/B01_single_joint_mpc_demo/<run_id>/videos/B01_single_joint_mpc_demo.mp4
```

## 12. 后续 TODO skeleton 应保留的关键 TODO

下一步写代码骨架时，建议保留这些 TODO，让用户手动补关键逻辑：

1. TODO：读取或创建单关节 MuJoCo 模型。
2. TODO：从仿真状态中提取 `q, dq`。
3. TODO：根据当前状态和 torque 序列执行 rollout。
4. TODO：根据 rollout 轨迹计算 horizon cost。
5. TODO：从候选序列中选出 cost 最小的一条。
6. TODO：只执行最优序列的第一个 torque。
7. TODO：记录误差、torque 和 cost。

这些 TODO 对应 MPC 的核心结构，不建议第一版直接全部自动隐藏到复杂类里。

## 13. 主要风险

- `w_q` 太大、`w_tau` 太小：torque 可能过大，控制动作激进。
- `w_dq` 太小：容易冲过目标角并振荡。
- horizon 太短：控制器看不到足够远的后果。
- horizon 太长或候选序列太多：计算会变慢。
- rollout 状态没有从真实当前状态复制：预测会从错误状态开始。
- 忘记“只执行第一个 torque”：会把 MPC 误写成开环控制。

## 14. 本阶段验收标准

完成 B01 设计文档后，应能清楚回答：

- 状态是什么：`q, dq`。
- 控制是什么：`torque`。
- 目标是什么：`target angle`。
- rollout 输入是什么：当前状态 + torque 序列。
- cost 怎么算：角度误差 + 速度误差 + torque penalty。
- 每次执行什么：只执行最优序列第一个 torque。

如果这些问题能不用看代码就讲清楚，B01 设计文档就达到了目的。
