# 机器人控制常用算法总结笔记

> 面向机器人运动控制学习、MuJoCo / Pinocchio / QP / MPC 项目实践与求职面试复习。
> 本笔记按工程控制层级组织：从低层关节控制，到运动学控制、动力学控制、QP/WBC/MPC、腿足机器人控制与学习型控制。

---

## 0. 机器人控制算法总地图

机器人控制算法不是平铺的，而是有清晰层级：

```text
任务目标
  ↓
轨迹规划 / 参考轨迹生成
  ↓
运动学控制
  ↓
动力学控制
  ↓
约束优化控制
  ↓
预测控制 / 最优控制
  ↓
学习型控制
```

总览：

```text
低层控制：
  PID / PD / 前馈 / 重力补偿

运动学控制：
  IK / 微分 IK / Jacobian 转置 / Jacobian 伪逆 / Null-space 控制

动力学控制：
  逆动力学控制 / 计算力矩控制 / 操作空间控制

力交互控制：
  阻抗控制 / 导纳控制 / 力控制 / 混合位置-力控制

优化控制：
  LQR / QP / WBC / MPC / iLQR / DDP / NMPC

腿足机器人常用：
  ZMP / Capture Point / Centroidal MPC / Whole-body QP / Contact Force Optimization

学习控制：
  强化学习 / 模仿学习 / Residual Learning / Learning-based MPC
```

### 0.1 通用控制循环伪代码

本节给出偏工程实现的伪代码。伪代码不是某一种固定 API，而是为了帮助你把公式转换成代码结构。实际项目中可以用：

- MuJoCo：负责仿真、积分、接触和可视化。
- Pinocchio：负责正运动学、雅可比、动力学项、逆动力学。
- OSQP / qpOASES / ProxQP：负责 QP 求解。
- CasADi / IPOPT / acados：负责 NLP / NMPC 建模和求解。

统一控制循环可以写成：

```text
initialize robot model, simulator, controller parameters
while simulation_is_running:
    q, qdot = read_robot_state()
    reference = read_reference(t)
    tau = controller(q, qdot, reference)
    tau = clamp(tau, tau_min, tau_max)
    send_torque_to_simulator(tau)
    simulator.step()
    log(q, qdot, tau, reference)
    t = t + dt
```


---

## 1. 低层关节控制

### 1.1 P 控制

最简单的关节位置控制：

$$
\tau = K_p(q_d-q)
$$

其中：

- $q_d$：目标关节角
- $q$：当前关节角
- $K_p$：比例增益
- $\tau$：关节力矩

物理意义：

> 位置误差越大，输出力矩越大。

优点：

- 简单
- 容易实现
- 适合入门仿真

缺点：

- 容易振荡
- 没有速度阻尼
- 无法精确处理动力学耦合

---

### 1.2 PD 控制

机器人关节控制最常见基础形式：

$$
\tau = K_p(q_d-q)+K_d(\dot q_d-\dot q)
$$

如果目标速度为 0：

$$
\tau = K_p(q_d-q)-K_d\dot q
$$

物理意义：

- P 项负责拉向目标位置
- D 项负责抑制振荡

PD 控制常用于：

- 机械臂关节位置控制
- MuJoCo actuator tracking
- 四足机器人关节伺服
- 人形机器人基础姿态保持

工程理解：

- $K_p$ 太大：响应快，但可能振荡
- $K_d$ 太小：阻尼不足
- $K_d$ 太大：动作迟钝，甚至数值不稳定

---

### 1.3 PID 控制

PID 多了积分项：

$$
\tau =
K_p e
+
K_i \int e\,dt
+
K_d \dot e
$$

其中：

$$
e=q_d-q
$$

作用：

- P：当前误差
- I：长期累计误差
- D：误差变化趋势

机器人里 PID 不如工业过程控制那么核心，因为机器人动力学强耦合、高维、非线性，但它仍常用于：

- 电机低层控制
- 关节位置伺服
- 速度闭环
- 硬件驱动器内部控制

常见问题：

- 积分饱和 windup
- 对噪声敏感
- 多关节耦合处理能力弱

#### 实现伪代码：PD / PID 关节控制

适用场景：单关节或多关节轨迹跟踪、MuJoCo actuator tracking、低层伺服控制。

输入：

- 当前关节角：$q$
- 当前关节速度：$\dot q$
- 期望关节角：$q_d$
- 期望关节速度：$\dot q_d$
- 增益：$K_p, K_i, K_d$
- 时间步长：$dt$

输出：

- 关节力矩：$\tau$

```text
initialize integral_error = 0

function PID_CONTROLLER(q, qdot, q_des, qdot_des, dt):
    error = q_des - q
    error_dot = qdot_des - qdot

    integral_error = integral_error + error * dt
    integral_error = clamp(integral_error, integral_min, integral_max)

    tau = Kp * error + Ki * integral_error + Kd * error_dot
    tau = clamp(tau, tau_min, tau_max)

    return tau
```

如果只用 PD，则去掉积分项：

```text
function PD_CONTROLLER(q, qdot, q_des, qdot_des):
    error = q_des - q
    error_dot = qdot_des - qdot
    tau = Kp * error + Kd * error_dot
    return clamp(tau, tau_min, tau_max)
```

实现要点：

- 多关节时，$K_p, K_d$ 通常是对角矩阵或逐关节数组。
- 力矩输出必须做限幅。
- PID 的积分项需要 anti-windup，避免积分饱和。
- 在仿真中优先从 PD 开始，不要一开始加入积分项。


---

### 1.4 前馈 + PD 控制

比纯 PD 更常用的是：

$$
\tau =
\tau_{ff}
+
K_p(q_d-q)
+
K_d(\dot q_d-\dot q)
$$

其中：

- $\tau_{ff}$：前馈力矩

如果有期望加速度，可以用动力学计算前馈：

$$
\tau_{ff}=M(q)\ddot q_d+C(q,\dot q)\dot q_d+g(q)
$$

核心理解：

> 前馈负责主要运动趋势，PD 负责误差修正。

在 MuJoCo 轨迹跟踪中经常使用：

```text
PD tracking + torque feedforward
```

#### 实现伪代码：前馈 + PD 控制

适用场景：有参考轨迹 $q_d, \dot q_d, \ddot q_d$ 的关节轨迹跟踪。

核心结构：

$$
\tau = \tau_{ff}+K_p(q_d-q)+K_d(\dot q_d-\dot q)
$$

```text
function FEEDFORWARD_PD(q, qdot, q_des, qdot_des, qddot_des):
    M = compute_mass_matrix(q)
    h = compute_nonlinear_terms(q, qdot)      # h = C(q,qdot)qdot + g(q)

    tau_ff = M * qddot_des + h

    error = q_des - q
    error_dot = qdot_des - qdot

    tau_fb = Kp * error + Kd * error_dot
    tau = tau_ff + tau_fb

    return clamp(tau, tau_min, tau_max)
```

实现要点：

- 如果参考加速度不可靠，可以只用重力补偿作为前馈。
- 如果动力学模型不准，前馈项可能引入误差，因此仍然需要 PD 反馈。
- 对高频控制，动力学计算需要足够快。


---

## 2. 运动学控制算法

运动学控制只关心位置、姿态、速度关系，暂时不显式考虑质量、惯量、力矩。

---

### 2.1 正运动学 FK

正运动学回答：

> 已知关节角 $q$，末端在哪里？

$$
x=f(q)
$$

其中：

- $q$：关节角
- $x$：末端位姿

用于：

- 机械臂末端位置计算
- 人形机器人足端位置计算
- 轨迹跟踪误差计算
- 可视化验证

---

### 2.2 逆运动学 IK

逆运动学回答：

> 想让末端到 $x_d$，关节角 $q$ 应该是多少？

$$
q = f^{-1}(x_d)
$$

一般没有简单解析解，尤其是冗余机器人、人形机器人、浮动基机器人。

常用数值形式：

$$
\min_q \|f(q)-x_d\|^2
$$

IK 本质上就是一个优化问题。

#### 实现伪代码：数值 IK

适用场景：给定末端目标位姿，求关节角。

```text
function ITERATIVE_IK(q_init, x_des):
    q = q_init

    for iter in range(max_iter):
        x = forward_kinematics(q)
        error = pose_error(x_des, x)

        if norm(error) < tolerance:
            break

        J = compute_frame_jacobian(q)
        delta_q = solve_least_squares(J, error)

        delta_q = clamp(delta_q, -dq_max, dq_max)
        q = q + alpha * delta_q
        q = clamp(q, q_min, q_max)

    return q
```

实现要点：

- 位姿误差要区分位置误差和姿态误差。
- 姿态误差不要直接用欧拉角相减，通常用旋转矩阵误差或李代数误差。
- 迭代步长 $\alpha$ 太大可能震荡，太小收敛慢。


---

### 2.3 雅可比矩阵控制

末端速度和关节速度的关系：

$$
\dot x = J(q)\dot q
$$

其中：

- $J(q)$：雅可比矩阵
- $\dot q$：关节速度
- $\dot x$：末端速度

这是机器人控制最核心公式之一。

---

### 2.4 Jacobian 伪逆控制

如果想让末端具有期望速度 $\dot x_d$，可以求：

$$
\dot q = J^\dagger \dot x_d
$$

其中：

$$
J^\dagger = J^T(JJ^T)^{-1}
$$

适合：

- 机械臂末端速度控制
- 差分 IK
- 轨迹跟踪
- 冗余机器人控制

缺点：

- 接近奇异位形时数值不稳定
- 可能产生很大的关节速度
- 不直接处理关节限位

---

### 2.5 阻尼最小二乘 Damped Least Squares

为了解决奇异性问题，可以用：

$$
\dot q =
J^T(JJ^T+\lambda^2I)^{-1}\dot x_d
$$

其中：

- $\lambda$：阻尼系数

物理意义：

> 牺牲一点精度，换取奇异点附近的稳定性。

适合：

- 机械臂 IK
- 人形机器人足端控制
- 冗余机械臂实时控制

#### 实现伪代码：Jacobian 伪逆 / 阻尼最小二乘

适用场景：微分 IK、末端速度控制、冗余机械臂实时控制。

```text
function DIFFERENTIAL_IK_DLS(q, xdot_des):
    J = compute_frame_jacobian(q)

    # Damped Least Squares:
    # qdot = J^T (J J^T + lambda^2 I)^(-1) xdot_des
    A = J * transpose(J) + lambda^2 * Identity(task_dim)
    qdot = transpose(J) * solve_linear_system(A, xdot_des)

    qdot = clamp(qdot, qdot_min, qdot_max)
    return qdot
```

如果有末端位置误差，可以先构造期望末端速度：

```text
xdot_des = xdot_ref + Kx * pose_error(x_ref, x_current)
qdot = DIFFERENTIAL_IK_DLS(q, xdot_des)
```

实现要点：

- $\lambda$ 越大越稳定，但跟踪精度下降。
- 接近奇异位形时，DLS 比普通伪逆更稳定。
- 输出 $\dot q$ 后，还需要积分得到下一步 $q$ 或送入速度控制器。


---

### 2.6 Jacobian 转置控制

另一种简单控制律：

$$
\tau = J^T F
$$

如果末端希望产生一个虚拟力：

$$
F = K_p(x_d-x)-K_d\dot x
$$

则：

$$
\tau = J^T[K_p(x_d-x)-K_d\dot x]
$$

物理意义：

> 把任务空间的力映射成关节力矩。

这是静力学关系：

$$
\tau = J^T F
$$

非常重要，后面操作空间控制、阻抗控制、WBC 都会用到。

#### 实现伪代码：Jacobian 转置任务空间力控制

适用场景：简单末端吸引控制、虚拟弹簧力控制、任务空间 PD。

```text
function JACOBIAN_TRANSPOSE_CONTROL(q, qdot, x_des, xdot_des):
    x = forward_kinematics(q)
    J = compute_frame_jacobian(q)
    xdot = J * qdot

    error_x = x_des - x
    error_xdot = xdot_des - xdot

    F_task = Kx * error_x + Dx * error_xdot
    tau = transpose(J) * F_task

    return clamp(tau, tau_min, tau_max)
```

实现要点：

- 该方法简单，但没有显式考虑操作空间惯性。
- 适合入门或低速任务，不如操作空间控制精确。
- 如果有重力影响，通常需要额外加 $g(q)$。


---

### 2.7 Null-space 控制

对于冗余机器人，关节自由度多于任务维度。

主任务：

$$
\dot q_1 = J^\dagger \dot x_d
$$

零空间附加任务：

$$
\dot q =
J^\dagger \dot x_d
+
(I-J^\dagger J)\dot q_0
$$

其中：

$$
N=I-J^\dagger J
$$

是零空间投影矩阵。

用途：

- 主任务：末端跟踪
- 副任务：避关节限位
- 副任务：保持姿态
- 副任务：避障
- 副任务：优化可操作度

核心思想：

> 不影响末端任务的情况下，调整冗余自由度。

#### 实现伪代码：Null-space 冗余控制

适用场景：机械臂主任务末端跟踪，同时做避关节限位、姿态保持或可操作度优化。

```text
function NULL_SPACE_CONTROL(q, xdot_des):
    J = compute_frame_jacobian(q)
    J_pinv = damped_pseudoinverse(J, lambda)

    qdot_task = J_pinv * xdot_des

    # 副任务：让关节远离限位，或者靠近舒适姿态 q_nominal
    qdot_secondary = K_null * (q_nominal - q)

    N = Identity(nv) - J_pinv * J
    qdot = qdot_task + N * qdot_secondary

    qdot = clamp(qdot, qdot_min, qdot_max)
    return qdot
```

实现要点：

- 主任务通过 $J^\dagger \dot x_d$ 实现。
- 副任务通过 $N\dot q_0$ 加入，理论上不影响主任务。
- 实际数值误差会导致轻微耦合，需要调小副任务权重。


---

## 3. 动力学控制算法

动力学控制考虑：

- 质量
- 惯量
- 重力
- 科氏力
- 离心力
- 关节力矩
- 接触力

机器人动力学标准形式：

$$
M(q)\ddot q+C(q,\dot q)\dot q+g(q)=\tau
$$

更完整地，如果有接触力：

$$
M(q)\ddot q+h(q,\dot q)=S^T\tau+J_c^T f_c
$$

其中：

- $M(q)$：惯性矩阵
- $h(q,\dot q)=C(q,\dot q)\dot q+g(q)$
- $S$：选择矩阵
- $J_c$：接触雅可比
- $f_c$：接触力

---

### 3.1 重力补偿控制

只补偿重力：

$$
\tau = g(q)
$$

如果加上 PD：

$$
\tau =
g(q)
+
K_p(q_d-q)
+
K_d(\dot q_d-\dot q)
$$

作用：

- 抵消机器人自身重量
- 让 PD 不必额外承担重力
- 提高静态定位精度

常用于：

- 机械臂悬停
- 低速轨迹跟踪
- 仿真入门控制器

#### 实现伪代码：重力补偿控制

适用场景：机械臂低速运动、悬停、减少静态误差。

```text
function GRAVITY_COMPENSATION_PD(q, qdot, q_des, qdot_des):
    g = compute_gravity_torque(q)

    error = q_des - q
    error_dot = qdot_des - qdot

    tau = g + Kp * error + Kd * error_dot
    return clamp(tau, tau_min, tau_max)
```

实现要点：

- 重力项只和 $q$ 有关。
- 适合低速控制，不适合大加速度快速运动的完整补偿。
- 在 Pinocchio 中常用 `computeGeneralizedGravity` 或 RNEA 特殊形式计算。


---

### 3.2 逆动力学控制

如果希望机器人产生指定加速度 $\ddot q_d$，可以用：

$$
\tau =
M(q)\ddot q_d+C(q,\dot q)\dot q+g(q)
$$

输入：

- 当前 $q, \dot q$
- 期望 $\ddot q_d$
- 动力学模型 $M, C, g$

输出：

- 关节力矩 $\tau$

适合：

- 轨迹跟踪
- 力矩前馈
- QP 控制后的力矩计算

---

### 3.3 计算力矩控制 Computed Torque Control

计算力矩控制是逆动力学 + PD 误差反馈。

先设计期望加速度：

$$
\ddot q_{cmd}
=
\ddot q_d
+
K_d(\dot q_d-\dot q)
+
K_p(q_d-q)
$$

然后用逆动力学：

$$
\tau =
M(q)\ddot q_{cmd}
+
C(q,\dot q)\dot q
+
g(q)
$$

合并：

$$
\tau =
M(q)
[
\ddot q_d
+
K_d(\dot q_d-\dot q)
+
K_p(q_d-q)
]
+
C(q,\dot q)\dot q
+
g(q)
$$

核心意义：

> 用动力学模型抵消非线性项，让闭环误差近似变成线性二阶系统。

误差动力学近似为：

$$
\ddot e+K_d\dot e+K_pe=0
$$

其中：

$$
e=q_d-q
$$

优点：

- 轨迹跟踪效果好
- 理论清晰
- 是理解逆动力学控制的核心算法

缺点：

- 依赖动力学模型准确性
- 不直接处理力矩限幅
- 不直接处理接触约束

#### 实现伪代码：计算力矩控制 Computed Torque

适用场景：模型较准确的机械臂关节空间轨迹跟踪。

```text
function COMPUTED_TORQUE(q, qdot, q_des, qdot_des, qddot_des):
    error = q_des - q
    error_dot = qdot_des - qdot

    qddot_cmd = qddot_des + Kd * error_dot + Kp * error

    M = compute_mass_matrix(q)
    h = compute_nonlinear_terms(q, qdot)      # C(q,qdot)qdot + g(q)

    tau = M * qddot_cmd + h
    return clamp(tau, tau_min, tau_max)
```

实现要点：

- 本质是“逆动力学前馈 + PD 误差闭环”。
- 如果动力学模型准确，误差动力学近似为 $\ddot e+K_d\dot e+K_pe=0$。
- 不直接处理力矩约束，力矩限幅后会破坏理想误差动力学。


---

## 4. 操作空间控制 Operational Space Control

关节空间控制关注 $q$，操作空间控制关注末端 $x$。

末端加速度关系：

$$
\ddot x =
J(q)\ddot q+\dot J(q,\dot q)\dot q
$$

操作空间动力学可以写成：

$$
\Lambda(q)\ddot x+\mu(q,\dot q)+p(q)=F
$$

其中：

- $\Lambda(q)$：操作空间惯性矩阵
- $\mu$：操作空间速度相关项
- $p$：操作空间重力项
- $F$：末端广义力

操作空间惯性矩阵：

$$
\Lambda = (JM^{-1}J^T)^{-1}
$$

末端控制力可以设计为：

$$
F =
\Lambda
[
\ddot x_d
+
K_d(\dot x_d-\dot x)
+
K_p(x_d-x)
]
+
\mu+p
$$

再映射到关节力矩：

$$
\tau = J^T F
$$

用途：

- 机械臂末端轨迹跟踪
- 末端力控制
- 人形机器人足端控制
- 操作任务控制

核心理解：

> 关节空间控制直接控制 $q$，操作空间控制直接控制末端 $x$。

### 实现伪代码：操作空间控制 OSC

适用场景：末端轨迹跟踪、机械臂任务空间控制、人形机器人足端控制。

```text
function OPERATIONAL_SPACE_CONTROL(q, qdot, x_des, xdot_des, xddot_des):
    M = compute_mass_matrix(q)
    h = compute_nonlinear_terms(q, qdot)

    x = forward_kinematics(q)
    J = compute_frame_jacobian(q)
    Jdot_qdot = compute_Jdot_times_qdot(q, qdot)
    xdot = J * qdot

    error_x = x_des - x
    error_xdot = xdot_des - xdot

    xddot_cmd = xddot_des + Dx * error_xdot + Kx * error_x

    Lambda = inverse(J * inverse(M) * transpose(J))

    # 简化写法：先求期望关节加速度，再逆动力学
    qddot_cmd = inverse(M) * transpose(J) * Lambda * (xddot_cmd - Jdot_qdot)
    tau = M * qddot_cmd + h

    return clamp(tau, tau_min, tau_max)
```

实现要点：

- 完整 OSC 需要处理动态一致逆、零空间项和奇异性。
- $JM^{-1}J^T$ 接近奇异时要加阻尼。
- 对冗余机械臂，可以加入 null-space 姿态控制。


---

## 5. 力控制与交互控制

机器人接触环境时，不能只控制位置。

典型场景：

- 机械臂打磨
- 人形机器人脚底接触地面
- 四足机器人足端接触
- 机器人推门
- 协作机器人和人交互

---

### 5.1 纯力控制

目标是让接触力达到期望值：

$$
F \rightarrow F_d
$$

简单形式：

$$
u = K_f(F_d-F)
$$

问题：

- 环境刚度未知时容易不稳定
- 不能单独用于所有方向

---

### 5.2 阻抗控制 Impedance Control

阻抗控制不直接强制位置或力，而是规定力和运动之间的动态关系：

$$
F =
M_d(\ddot x_d-\ddot x)
+
D_d(\dot x_d-\dot x)
+
K_d(x_d-x)
$$

也可以写成：

$$
M_d\ddot e+D_d\dot e+K_de=F_{ext}
$$

其中：

- $M_d$：期望惯性
- $D_d$：期望阻尼
- $K_d$：期望刚度
- $F_{ext}$：外力

物理意义：

> 让机器人表现得像一个“弹簧-阻尼-质量系统”。

适合：

- 机械臂柔顺控制
- 人机交互
- 接触任务
- 足端缓冲

核心理解：

> 阻抗控制：输入运动误差，输出力/力矩。

#### 实现伪代码：阻抗控制

适用场景：接触任务、人机交互、柔顺末端控制、足端缓冲。

```text
function IMPEDANCE_CONTROL(q, qdot, x_des, xdot_des, xddot_des):
    x = forward_kinematics(q)
    J = compute_frame_jacobian(q)
    xdot = J * qdot

    error_x = x_des - x
    error_xdot = xdot_des - xdot

    # 期望任务空间力：虚拟弹簧-阻尼-质量
    F_imp = Md * xddot_des + Dd * error_xdot + Kd * error_x

    tau = transpose(J) * F_imp + compute_gravity_torque(q)
    return clamp(tau, tau_min, tau_max)
```

实现要点：

- 刚度 $K_d$ 越大，越像位置控制；越小，越柔顺。
- 阻尼 $D_d$ 决定接触时是否振荡。
- 力矩控制型机器人更适合直接做阻抗控制。


---

### 5.3 导纳控制 Admittance Control

导纳控制和阻抗控制方向相反。

给定外力，生成运动：

$$
M_d\ddot x + D_d\dot x + K_d x = F_{ext}
$$

核心理解：

> 导纳控制：输入外力，输出期望运动。

适合：

- 位置控制型机器人
- 协作机器人
- 外力传感器驱动的柔顺运动

阻抗 vs 导纳：

| 对比 | 阻抗控制 | 导纳控制 |
|---|---|---|
| 输入 | 运动误差 | 外力 |
| 输出 | 力/力矩 | 位置/速度指令 |
| 适合 | 力矩控制机器人 | 位置控制机器人 |
| 常见场景 | 高性能机械臂、腿足机器人 | 协作机器人、工业机械臂 |

#### 实现伪代码：导纳控制

适用场景：位置控制型机器人、协作机器人、外力驱动运动。

```text
initialize x_ref, xdot_ref

function ADMITTANCE_CONTROLLER(F_ext, x_cmd, dt):
    # M_d xddot + D_d xdot + K_d (x - x_cmd) = F_ext
    xddot_ref = inverse(Md) * (F_ext - Dd * xdot_ref - Kd * (x_ref - x_cmd))

    xdot_ref = xdot_ref + xddot_ref * dt
    x_ref = x_ref + xdot_ref * dt

    return x_ref, xdot_ref, xddot_ref
```

实现要点：

- 导纳控制输出的是位置/速度参考，不是直接输出力矩。
- 常作为外层控制器，内层仍由位置控制器或轨迹跟踪器执行。
- 外力信号要滤波，否则会把噪声转化成抖动运动。


---

### 5.4 混合位置-力控制

有些方向控制位置，有些方向控制力。

例如机械臂擦桌子：

```text
水平方向：控制位置轨迹
竖直方向：控制接触力
```

可以用选择矩阵 $S$：

$$
u =
S u_{pos}
+
(I-S)u_{force}
$$

适合：

- 打磨
- 装配
- 插孔
- 擦拭
- 足端接触

#### 实现伪代码：混合位置-力控制

适用场景：擦拭、打磨、插孔、足端接触等部分方向控位置、部分方向控力的任务。

```text
function HYBRID_POSITION_FORCE_CONTROL(q, qdot, x_des, F_des, F_meas):
    x = forward_kinematics(q)
    J = compute_frame_jacobian(q)
    xdot = J * qdot

    F_pos = Kx * (x_des - x) - Dx * xdot
    F_force = Kf * (F_des - F_meas)

    # S 选择位置控制方向，I-S 选择力控制方向
    F_cmd = S * F_pos + (Identity(task_dim) - S) * F_force

    tau = transpose(J) * F_cmd + compute_gravity_torque(q)
    return clamp(tau, tau_min, tau_max)
```

实现要点：

- 选择矩阵 $S$ 的定义必须和任务坐标系一致。
- 接触方向通常控力，切向方向通常控位置或速度。
- 力传感器信号需要滤波和坐标变换。


---

## 6. LQR / LQG / 最优控制基础

### 6.1 LQR

LQR 解决：

```text
线性系统 + 二次代价 + 无显式不等式约束
```

系统：

$$
x_{k+1}=Ax_k+Bu_k
$$

代价：

$$
J=
\sum_{k=0}^{N-1}
(x_k^TQx_k+u_k^TRu_k)
+
x_N^TQ_fx_N
$$

最优控制律：

$$
u_k=-K_kx_k
$$

无限时域时：

$$
u=-Kx
$$

LQR 意义：

- 最优控制入门算法
- MPC 的基础
- iLQR / DDP 的基础
- 机器人稳定控制常用基准方法

适合：

- 倒立摆
- 线性化机械臂
- 小扰动稳定控制
- 轨迹附近局部稳定

缺点：

- 不能直接处理输入限制
- 不能直接处理状态约束
- 依赖线性化模型

#### 实现伪代码：LQR

适用场景：线性系统稳定控制、倒立摆、轨迹附近局部控制。

有限时域离散 LQR：

```text
function FINITE_HORIZON_LQR(A, B, Q, R, Qf, N):
    P[N] = Qf

    for k = N-1 down to 0:
        S = R + transpose(B) * P[k+1] * B
        K[k] = inverse(S) * transpose(B) * P[k+1] * A
        P[k] = Q + transpose(A) * P[k+1] * A \
               - transpose(A) * P[k+1] * B * K[k]

    return K[0:N]
```

在线控制：

```text
function LQR_CONTROLLER(x, x_ref, K):
    error = x - x_ref
    u = -K * error
    return clamp(u, u_min, u_max)
```

实现要点：

- LQR 本身不处理硬约束，限幅是工程补丁，不是优化约束。
- 对非线性机器人，需要先在平衡点或参考轨迹附近线性化。
- 有限时域 LQR 输出时变增益 $K_k$，无限时域 LQR 输出固定增益 $K$。


---

### 6.2 LQG

LQG = LQR + Kalman Filter。

当状态不能完全测量时，用卡尔曼滤波估计状态：

$$
\hat x
$$

然后控制：

$$
u=-K\hat x
$$

适合：

- 带传感器噪声的线性系统
- 状态估计 + 最优控制

机器人里它的直接使用不如 LQR/MPC 常见，但思想很重要：

> 控制器需要状态估计，估计和控制可以分层设计。

#### 实现伪代码：Kalman Filter / LQG

适用场景：状态不可完全测量、传感器有噪声的线性系统。

```text
function KALMAN_FILTER_STEP(x_hat, P, u, y):
    # Prediction
    x_pred = A * x_hat + B * u
    P_pred = A * P * transpose(A) + W

    # Correction
    innovation = y - C * x_pred
    S = C * P_pred * transpose(C) + V
    L = P_pred * transpose(C) * inverse(S)

    x_hat_new = x_pred + L * innovation
    P_new = (Identity(nx) - L * C) * P_pred

    return x_hat_new, P_new

function LQG_CONTROLLER(x_hat, x_ref, K_lqr):
    return -K_lqr * (x_hat - x_ref)
```

实现要点：

- LQG = Kalman Filter 状态估计 + LQR 控制。
- $W$ 是过程噪声协方差，$V$ 是测量噪声协方差。
- 机器人中常和 IMU、编码器、视觉估计结合。


---

## 7. QP 控制

QP 是机器人控制中非常核心的工程工具。

标准形式：

$$
\min_x
\frac{1}{2}x^THx+g^Tx
$$

约束：

$$
l \le Ax \le u
$$

在机器人中，QP 常用于：

- 逆运动学 QP
- 逆速度 QP
- 逆加速度 QP
- 力矩约束控制
- 接触力优化
- WBC 全身控制
- 线性 MPC

---

### 7.1 逆运动学 QP

优化变量：

$$
\Delta q
$$

目标：

$$
\min_{\Delta q}
\|J(q)\Delta q-\Delta x_d\|^2
+
\lambda \|\Delta q\|^2
$$

约束：

$$
q_{min} \le q+\Delta q \le q_{max}
$$

优点：

- 可以处理关节限位
- 可以处理速度限制
- 比普通伪逆更工程化

#### 实现伪代码：QP-IK

适用场景：带关节限位、速度限制的逆运动学。

优化变量：$\Delta q$ 或 $\dot q$。

```text
function QP_IK(q, x_des):
    x = forward_kinematics(q)
    error = pose_error(x_des, x)
    J = compute_frame_jacobian(q)

    # min 0.5 ||J delta_q - error||_Q^2 + 0.5 ||delta_q||_R^2
    H = transpose(J) * Q * J + R
    g = -transpose(J) * Q * error

    # joint limit constraints: q_min <= q + delta_q <= q_max
    A = Identity(nq)
    lower = q_min - q
    upper = q_max - q

    delta_q = solve_qp(H, g, A, lower, upper)
    delta_q = clamp(delta_q, -dq_max, dq_max)

    return q + delta_q
```

实现要点：

- QP-IK 比伪逆 IK 更适合工程，因为能显式加入约束。
- 如果误差很大，可以循环调用多次，每次只走小步。
- 可以加入松弛变量，避免目标不可达时 QP infeasible。


---

### 7.2 逆加速度 QP

优化变量：

$$
\ddot q
$$

目标：

$$
\min_{\ddot q}
\|J\ddot q+\dot J\dot q-a_d\|^2
+
\lambda \|\ddot q\|^2
$$

可以加力矩约束：

$$
\tau_{min}
\le
M(q)\ddot q+h(q,\dot q)
\le
\tau_{max}
$$

这是机器人运动控制项目中很适合实践的核心任务。

#### 实现伪代码：逆加速度 QP 控制

适用场景：二连杆或机械臂末端加速度跟踪，同时满足力矩约束。

优化变量：$\ddot q$。

```text
function INVERSE_ACCELERATION_QP(q, qdot, xddot_des):
    J = compute_frame_jacobian(q)
    Jdot_qdot = compute_Jdot_times_qdot(q, qdot)

    M = compute_mass_matrix(q)
    h = compute_nonlinear_terms(q, qdot)

    # task: J qddot + Jdot_qdot = xddot_des
    b = xddot_des - Jdot_qdot

    # min 0.5 ||J qddot - b||_Q^2 + 0.5 ||qddot||_R^2
    H = transpose(J) * Q * J + R
    g = -transpose(J) * Q * b

    # torque constraints:
    # tau_min <= M qddot + h <= tau_max
    A = M
    lower = tau_min - h
    upper = tau_max - h

    qddot_star = solve_qp(H, g, A, lower, upper)
    tau = M * qddot_star + h

    return clamp(tau, tau_min, tau_max), qddot_star
```

实现要点：

- 这是把机器人动力学和 QP 连接起来的关键算法。
- 任务目标在代价函数里，力矩限制在约束里。
- 若还要加关节加速度限制，可以追加 $qddot_{min}\le qddot\le qddot_{max}$。


---

### 7.3 QP 的机器人意义

QP 的价值在于：

> 可以把“任务目标”和“物理约束”统一写进一个优化问题。

例如：

- 目标：末端跟踪
- 约束：关节限位
- 约束：速度限幅
- 约束：力矩限幅
- 约束：接触力非负
- 约束：摩擦锥

这就是为什么 QP 是 WBC 和 MPC 的基础。

---

## 8. WBC 全身控制

WBC，全称 Whole-Body Control，全身控制。

主要用于：

- 人形机器人
- 四足机器人
- 浮动基机械系统
- 多接触机器人

浮动基机器人动力学：

$$
M(q)\ddot q+h(q,\dot q)=S^T\tau+J_c^Tf_c
$$

其中：

- 浮动基不能直接施加力矩
- 关节可以施加力矩
- 接触力 $f_c$ 由地面产生

WBC 常见优化变量：

$$
z =
\begin{bmatrix}
\ddot q \\
\tau \\
f_c
\end{bmatrix}
$$

目标可能包括：

- 质心加速度跟踪
- 躯干姿态跟踪
- 摆动腿足端轨迹跟踪
- 接触力正则化
- 关节加速度正则化
- 力矩正则化

约束包括：

- 机器人动力学方程
- 接触点不滑动
- 摩擦锥
- 法向力非负
- 关节力矩限制
- 关节加速度限制

典型 QP：

$$
\min_z
\sum_i
\|A_i z-b_i\|_{W_i}^2
$$

约束：

$$
M\ddot q+h=S^T\tau+J_c^Tf_c
$$

$$
J_c\ddot q+\dot J_c\dot q=0
$$

$$
f_z \ge 0
$$

$$
|f_x| \le \mu f_z
$$

$$
|f_y| \le \mu f_z
$$

WBC 核心理解：

> WBC 不是单一算法，而是一类把全身任务、动力学和接触约束统一进 QP 的控制框架。

### 实现伪代码：WBC-QP 全身控制

适用场景：浮动基机器人、人形机器人、四足机器人、多任务多接触控制。

优化变量：

$$
z = [\ddot q,\ \tau,\ f_c]
$$

```text
function WHOLE_BODY_CONTROL_QP(q, qdot, tasks, contacts):
    M = compute_mass_matrix(q)
    h = compute_nonlinear_terms(q, qdot)
    S = build_actuation_selection_matrix()
    Jc = compute_contact_jacobian(q, contacts)
    Jc_dot_qdot = compute_contact_Jdot_qdot(q, qdot, contacts)

    H = zero_matrix()
    g = zero_vector()

    for task in tasks:
        J_task = compute_task_jacobian(q, task)
        Jdot_qdot_task = compute_task_Jdot_qdot(q, qdot, task)
        a_des = task.desired_acceleration(q, qdot)

        # task equation: J_task qddot = a_des - Jdot*qdot
        A_task = build_matrix_selecting_qddot(J_task)
        b_task = a_des - Jdot_qdot_task

        H += transpose(A_task) * task.weight * A_task
        g += -transpose(A_task) * task.weight * b_task

    # Equality constraints: floating-base dynamics
    # M qddot + h = S^T tau + Jc^T fc
    A_dyn, b_dyn = build_dynamics_equality(M, h, S, Jc)

    # Contact no-motion constraint:
    # Jc qddot + Jc_dot qdot = 0
    A_contact, b_contact = build_contact_acceleration_constraint(Jc, Jc_dot_qdot)

    # Inequality constraints: torque limits, friction cone, normal force
    A_ineq, lower, upper = build_wbc_inequality_constraints()

    z_star = solve_qp(H, g,
                      equality=[A_dyn, b_dyn, A_contact, b_contact],
                      inequality=[A_ineq, lower, upper])

    qddot_star, tau_star, fc_star = unpack(z_star)
    return tau_star, qddot_star, fc_star
```

实现要点：

- WBC 的重点不是单一公式，而是如何把任务、动力学、接触和约束装进一个 QP。
- 多任务可以用加权和，也可以用层级 QP。
- 浮动基前 6 个自由度没有电机力矩，必须靠接触力实现运动和平衡。


---

## 9. MPC 模型预测控制

MPC 的核心思想：

> 用模型预测未来；在未来窗口内优化控制序列；只执行第一个控制量；下一时刻重新优化。

优化变量：

$$
U=\{u_0,u_1,\dots,u_{N-1}\}
$$

或者：

$$
X,U=
\{x_0,\dots,x_N,u_0,\dots,u_{N-1}\}
$$

目标：

$$
\min_{X,U}
\sum_{k=0}^{N-1}
[
(x_k-x_k^{ref})^TQ(x_k-x_k^{ref})
+
u_k^TRu_k
]
$$

动力学约束：

$$
x_{k+1}=f(x_k,u_k)
$$

输入约束：

$$
u_{min}\le u_k\le u_{max}
$$

状态约束：

$$
x_{min}\le x_k\le x_{max}
$$

MPC 特点：

- 可以处理约束
- 可以预测未来
- 适合多变量系统
- 适合轨迹跟踪
- 适合机器人运动控制

缺点：

- 计算量大
- 需要模型
- 调参复杂
- 实时性要求高

---

### 9.1 线性 MPC

如果：

$$
x_{k+1}=Ax_k+Bu_k
$$

代价是二次型，约束是线性的，那么 MPC 可以转化为 QP。

```text
线性 MPC → QP → OSQP / qpOASES / HPIPM
```

适合：

- 倒立摆
- 线性化机器人系统
- 质心模型 MPC
- 轨迹附近控制

#### 实现伪代码：线性 MPC

适用场景：线性系统、线性化系统、倒立摆、质心模型、轨迹附近控制。

```text
function LINEAR_MPC_STEP(x0, x_ref_seq):
    # Decision variables: X[0:N], U[0:N-1]
    # Build cost:
    # sum ||x_k - x_ref_k||_Q^2 + ||u_k||_R^2
    H, g = build_mpc_quadratic_cost(Q, R, Qf, x_ref_seq)

    # Build dynamics constraints:
    # x_{k+1} = A x_k + B u_k
    Aeq, beq = build_linear_dynamics_constraints(A, B, x0, N)

    # Build bounds:
    # x_min <= x_k <= x_max
    # u_min <= u_k <= u_max
    Aineq, lower, upper = build_state_input_bounds(x_min, x_max, u_min, u_max)

    solution = solve_qp(H, g,
                        equality=[Aeq, beq],
                        inequality=[Aineq, lower, upper])

    X_star, U_star = unpack_mpc_solution(solution)

    # Receding horizon: only apply the first control
    u_apply = U_star[0]
    return u_apply, X_star, U_star
```

在线循环：

```text
while running:
    x0 = read_state()
    x_ref_seq = get_reference_over_horizon(t, N)
    u = LINEAR_MPC_STEP(x0, x_ref_seq)
    apply_control(u)
    step_simulation()
```

实现要点：

- MPC 每个控制周期都重新求解。
- 只执行第一个控制量，剩下控制序列作为 warm start。
- 线性 MPC 最终通常变成 QP。


---

### 9.2 非线性 MPC

如果动力学是非线性的：

$$
x_{k+1}=f(x_k,u_k)
$$

则得到 NLP：

```text
NMPC → NLP → IPOPT / SQP / acados
```

适合：

- 机械臂大范围运动
- 腿足机器人
- 无人车
- 非线性轨迹优化

缺点：

- 计算更重
- 需要自动微分
- 需要 warm start
- 实时实现难度更高

#### 实现伪代码：NMPC

适用场景：非线性动力学、大范围运动、非线性约束、机器人轨迹优化。

```text
function NMPC_STEP(x0, x_ref_seq, previous_solution):
    create_decision_variables X[0:N], U[0:N-1]

    cost = 0
    constraints = []

    constraints.append(X[0] == x0)

    for k in 0 to N-1:
        cost += norm_Q(X[k] - x_ref_seq[k])^2
        cost += norm_R(U[k])^2

        # nonlinear dynamics
        x_next_pred = integrate_dynamics(X[k], U[k], dt)
        constraints.append(X[k+1] == x_next_pred)

        constraints.append(u_min <= U[k] <= u_max)
        constraints.append(x_min <= X[k] <= x_max)

    cost += terminal_cost(X[N], x_ref_seq[N])

    solution = solve_nlp(cost, constraints, warm_start=previous_solution)
    U_star = solution.U

    return U_star[0], solution
```

实现要点：

- NMPC 通常需要自动微分。
- CasADi 常用于建模，IPOPT/acados 常用于求解。
- 实时 NMPC 需要 warm start、短 horizon、合适离散化和求解器调参。


---

## 10. iLQR / DDP / iLQG

这些属于轨迹优化和非线性最优控制算法。

---

### 10.1 iLQR

iLQR = iterative LQR。

解决非线性系统：

$$
x_{k+1}=f(x_k,u_k)
$$

非线性代价：

$$
J=\sum l(x_k,u_k)+l_f(x_N)
$$

基本流程：

```text
1. 给定初始控制序列
2. 正向 rollout 得到状态轨迹
3. 沿轨迹线性化动力学
4. 二次近似代价
5. 反向 Riccati 递推求局部反馈
6. 前向 rollout 更新控制
7. 重复迭代
```

适合：

- 非线性轨迹优化
- 倒立摆 swing-up
- 机械臂轨迹优化
- MuJoCo shooting-based planner

优点：

- 比通用 NLP 更利用最优控制结构
- 适合中小规模非线性系统
- 控制效果好

缺点：

- 处理不等式约束不如 MPC/QP 直接
- 对初值敏感
- 需要动力学导数

#### 实现伪代码：iLQR

适用场景：非线性轨迹优化、shooting-based planning、倒立摆 swing-up、MuJoCo planner。

```text
function ILQR(x0, U_init):
    U = U_init

    for iteration in range(max_iter):
        # 1. Forward rollout
        X = rollout_dynamics(x0, U)
        cost_old = compute_total_cost(X, U)

        # 2. Linearize dynamics and quadratize cost along trajectory
        for k in 0 to N-1:
            A[k], B[k] = linearize_dynamics(X[k], U[k])
            lx[k], lu[k], lxx[k], luu[k], lux[k] = quadratize_stage_cost(X[k], U[k])
        lf_x, lf_xx = quadratize_terminal_cost(X[N])

        # 3. Backward pass
        Vx = lf_x
        Vxx = lf_xx

        for k = N-1 down to 0:
            Qx = lx[k] + transpose(A[k]) * Vx
            Qu = lu[k] + transpose(B[k]) * Vx
            Qxx = lxx[k] + transpose(A[k]) * Vxx * A[k]
            Quu = luu[k] + transpose(B[k]) * Vxx * B[k]
            Qux = lux[k] + transpose(B[k]) * Vxx * A[k]

            Quu_reg = regularize(Quu)
            k_ff[k] = -inverse(Quu_reg) * Qu
            K_fb[k] = -inverse(Quu_reg) * Qux

            Vx = Qx + transpose(K_fb[k]) * Quu * k_ff[k] \
                    + transpose(K_fb[k]) * Qu + transpose(Qux) * k_ff[k]
            Vxx = Qxx + transpose(K_fb[k]) * Quu * K_fb[k] \
                    + transpose(K_fb[k]) * Qux + transpose(Qux) * K_fb[k]

        # 4. Forward line search
        accepted = false
        for alpha in [1.0, 0.5, 0.25, 0.1, 0.05]:
            X_new, U_new = rollout_with_feedback(x0, X, U, k_ff, K_fb, alpha)
            cost_new = compute_total_cost(X_new, U_new)
            if cost_new < cost_old:
                U = U_new
                accepted = true
                break

        if not accepted or abs(cost_old - cost_new) < tolerance:
            break

    return X, U, K_fb
```

实现要点：

- iLQR 需要动力学一阶导数和代价二阶近似。
- 反向传播阶段的 $Q_{uu}$ 必须正定或正则化。
- 前向 line search 用于保证 cost 下降。
- 对控制输入硬约束，需要额外方法，例如 box-constrained iLQR 或投影。


---

### 10.2 DDP

DDP = Differential Dynamic Programming。

它和 iLQR 很接近，但 DDP 保留更多二阶动力学信息。

简单理解：

```text
iLQR：动力学一阶线性化 + 代价二阶近似
DDP：动力学也考虑二阶项
```

优点：

- 理论更完整
- 局部收敛性质更好

缺点：

- 实现复杂
- 二阶导数计算更重

#### 实现伪代码：DDP

适用场景：和 iLQR 类似，但理论上考虑更多二阶动力学信息。

```text
function DDP(x0, U_init):
    U = U_init

    for iteration in range(max_iter):
        X = rollout_dynamics(x0, U)

        for k in 0 to N-1:
            A[k], B[k] = first_order_dynamics_derivatives(X[k], U[k])
            fxx[k], fxu[k], fuu[k] = second_order_dynamics_derivatives(X[k], U[k])
            cost_derivatives[k] = compute_cost_derivatives(X[k], U[k])

        # Backward pass is like iLQR, but Qxx, Quu, Qux include
        # additional terms from second-order dynamics derivatives.
        k_ff, K_fb = backward_pass_with_second_order_dynamics()

        X_new, U_new = forward_rollout_with_line_search(x0, X, U, k_ff, K_fb)
        U = U_new

        if convergence_reached:
            break

    return X_new, U_new, K_fb
```

实现要点：

- DDP 比 iLQR 更复杂，因为需要动力学二阶导数。
- 工程入门通常先实现 iLQR，再学习 DDP。
- MuJoCo 中如果没有解析导数，DDP 的二阶导数实现成本较高。


---

### 10.3 iLQG

iLQG 可以理解为 iLQR 在带噪声或高斯假设下的扩展。

在机器人项目中，可以先重点掌握：

```text
LQR → iLQR → MPC
```

不必一开始深入 iLQG 的随机控制细节。

---

## 11. 轨迹规划与时间参数化

严格说，轨迹规划不完全等于控制，但它是控制器的上游。

---

### 11.1 多项式轨迹

常用五次多项式：

$$
q(t)=a_0+a_1t+a_2t^2+a_3t^3+a_4t^4+a_5t^5
$$

可以保证：

- 起点位置
- 起点速度
- 起点加速度
- 终点位置
- 终点速度
- 终点加速度

适合：

- 机械臂平滑轨迹
- 关节空间轨迹
- 教学实验

#### 实现伪代码：多项式轨迹生成

适用场景：机械臂点到点轨迹、关节空间参考轨迹生成。

```text
function QUINTIC_POLYNOMIAL_TRAJECTORY(q0, qf, v0, vf, a0, af, T):
    # Solve coefficients a0...a5 from boundary conditions:
    # q(0), qdot(0), qddot(0), q(T), qdot(T), qddot(T)
    coeff = solve_6x6_linear_system(q0, v0, a0, qf, vf, af, T)

    function evaluate(t):
        q = coeff[0] + coeff[1]*t + coeff[2]*t^2 + coeff[3]*t^3 + coeff[4]*t^4 + coeff[5]*t^5
        qdot = coeff[1] + 2*coeff[2]*t + 3*coeff[3]*t^2 + 4*coeff[4]*t^3 + 5*coeff[5]*t^4
        qddot = 2*coeff[2] + 6*coeff[3]*t + 12*coeff[4]*t^2 + 20*coeff[5]*t^3
        return q, qdot, qddot

    return evaluate
```

实现要点：

- 五次多项式可以同时满足起终点位置、速度、加速度。
- 多关节轨迹可以对每个关节分别生成，但要统一总时间 $T$。
- 输出 $q_d,\dot q_d,\ddot q_d$ 可以直接接 computed torque 或 PD+feedforward。


---

### 11.2 梯形速度规划

典型过程：

```text
加速
匀速
减速
```

适合：

- 工业机械臂
- 关节轨迹执行
- 简单点到点运动

#### 实现伪代码：梯形速度规划

适用场景：工业机械臂点到点运动、简单关节轨迹。

```text
function TRAPEZOIDAL_PROFILE(q0, qf, vmax, amax):
    distance = abs(qf - q0)
    direction = sign(qf - q0)

    t_acc = vmax / amax
    d_acc = 0.5 * amax * t_acc^2

    if 2 * d_acc >= distance:
        # triangular profile: cannot reach vmax
        t_acc = sqrt(distance / amax)
        t_flat = 0
        vmax_actual = amax * t_acc
    else:
        t_flat = (distance - 2 * d_acc) / vmax
        vmax_actual = vmax

    T = 2 * t_acc + t_flat

    function evaluate(t):
        if t < t_acc:
            q = q0 + direction * 0.5 * amax * t^2
            qdot = direction * amax * t
            qddot = direction * amax
        else if t < t_acc + t_flat:
            tau = t - t_acc
            q = q0 + direction * (d_acc + vmax_actual * tau)
            qdot = direction * vmax_actual
            qddot = 0
        else:
            tau = T - t
            q = qf - direction * 0.5 * amax * tau^2
            qdot = direction * amax * tau
            qddot = -direction * amax
        return q, qdot, qddot

    return evaluate, T
```

实现要点：

- 梯形速度的加速度不连续，实际系统中会产生 jerk 冲击。
- 若路径太短，会退化成三角速度曲线。
- 高性能机械臂更常用 S 曲线或 TOPP。


---

### 11.3 S 曲线规划

相比梯形速度，S 曲线限制 jerk：

$$
j = \dddot q
$$

优点：

- 运动更平滑
- 对电机和机械结构更友好

---

### 11.4 TOPP 时间最优路径参数化

已知几何路径：

$$
q=s(\theta)
$$

再求最优时间参数：

$$
\theta(t)
$$

目标：

> 在速度、加速度、力矩约束下，尽可能快地走完路径。

适合：

- 机械臂轨迹后处理
- 工业机器人运动规划
- 高性能轨迹执行

---

## 12. 运动规划算法

运动规划解决：

- 从起点到终点怎么走
- 避障
- 满足运动学约束

常见算法：

```text
A*
Dijkstra
RRT
RRT*
PRM
CHOMP
STOMP
TrajOpt
```

和控制的关系：

```text
规划器生成路径
轨迹生成器生成时间轨迹
控制器跟踪轨迹
```

例如：

```text
RRT* 找到无碰路径
↓
轨迹平滑
↓
MPC / PD / 逆动力学控制跟踪
```

### 实现伪代码：RRT / RRT* 运动规划

适用场景：高维空间避障路径搜索、机械臂路径规划。

```text
function RRT_PLANNER(q_start, q_goal):
    tree = initialize_tree(q_start)

    for iter in range(max_iter):
        if random() < goal_sample_rate:
            q_rand = q_goal
        else:
            q_rand = sample_configuration_space()

        q_near = nearest_node(tree, q_rand)
        q_new = steer(q_near, q_rand, step_size)

        if is_collision_free(q_near, q_new):
            add_node(tree, q_new, parent=q_near)

            if distance(q_new, q_goal) < goal_tolerance:
                if is_collision_free(q_new, q_goal):
                    add_node(tree, q_goal, parent=q_new)
                    return extract_path(tree, q_goal)

    return failure
```

RRT* 多了重连优化：

```text
for each neighbor near q_new:
    if cost(parent_candidate) + edge_cost < cost(q_new):
        choose_better_parent(q_new)

for each neighbor near q_new:
    if cost(q_new) + edge_cost < cost(neighbor):
        rewire_neighbor_to_q_new()
```

实现要点：

- RRT 负责找可行路径，RRT* 逐步改善路径质量。
- 输出路径通常还需要平滑和时间参数化。
- 控制器负责跟踪规划器输出的轨迹。


### 实现伪代码：PRM 运动规划

适用场景：同一环境中多次查询路径。

```text
function BUILD_PRM_ROADMAP():
    nodes = []
    edges = []

    while len(nodes) < num_samples:
        q = sample_configuration_space()
        if not in_collision(q):
            nodes.append(q)

    for q in nodes:
        neighbors = find_k_nearest_neighbors(q, nodes, k)
        for q_neighbor in neighbors:
            if is_collision_free(q, q_neighbor):
                edges.append((q, q_neighbor, distance(q, q_neighbor)))

    return graph(nodes, edges)

function PRM_QUERY(graph, q_start, q_goal):
    connect q_start and q_goal to graph
    path = shortest_path_search(graph, q_start, q_goal)
    return path
```

实现要点：

- PRM 建图慢，但多次查询快。
- 适合固定环境、多起终点任务。
- 机械臂路径规划中常和碰撞检测库配合使用。


---

## 13. 腿足机器人常用控制算法

腿足机器人因为有浮动基和接触，控制更复杂。

---

### 13.1 ZMP 控制

ZMP = Zero Moment Point。

核心思想：

> 让零力矩点落在支撑多边形内，机器人就不容易倾倒。

适合：

- 传统双足机器人
- 平地行走
- 慢速步态

缺点：

- 偏保守
- 不适合强动态运动

---

### 13.2 Capture Point

Capture Point 描述机器人为了停下来，脚应该落在哪里。

线性倒立摆模型中：

$$
x_{cp}=x+\frac{\dot x}{\omega}
$$

其中：

$$
\omega=\sqrt{\frac{g}{z_c}}
$$

适合：

- 双足机器人步态调整
- 推扰恢复
- 落脚点规划

#### 实现伪代码：ZMP / Capture Point 控制

适用场景：双足机器人平衡、步态调整、推扰恢复。

```text
function CAPTURE_POINT_CONTROL(com_pos, com_vel, z_com):
    omega = sqrt(g / z_com)
    capture_point = com_pos + com_vel / omega

    if capture_point inside support_polygon:
        footstep_target = nominal_footstep
    else:
        footstep_target = project_capture_point_to_reachable_region(capture_point)

    return footstep_target
```

ZMP 检查：

```text
function ZMP_STABILITY_CHECK(zmp, support_polygon):
    if point_inside_polygon(zmp, support_polygon):
        stable = true
    else:
        stable = false
    return stable
```

实现要点：

- ZMP 更偏稳定性判据和慢速步态控制。
- Capture Point 更适合解释落脚点调整和推扰恢复。
- 实际人形机器人通常还会结合 MPC 和 WBC。


---

### 13.3 Centroidal Dynamics Control

质心动力学：

$$
\dot h_G =
\sum_i
\begin{bmatrix}
f_i \\
(r_i-r_G)\times f_i+\tau_i
\end{bmatrix}
+
\begin{bmatrix}
mg \\
0
\end{bmatrix}
$$

其中：

- $h_G$：质心动量
- $f_i$：接触力
- $r_i$：接触点位置
- $r_G$：质心位置

用途：

- 人形机器人平衡
- 四足机器人运动
- 质心轨迹跟踪
- 接触力优化

#### 实现伪代码：Centroidal MPC

适用场景：四足/人形机器人质心轨迹、接触力和步态控制。

```text
function CENTROIDAL_MPC_STEP(centroidal_state, contact_schedule, ref_traj):
    # State may include CoM position, CoM velocity, base orientation, angular velocity
    # Control may include contact forces at feet

    create variables X[0:N], F[0:N-1]

    cost = 0
    constraints = []
    constraints.append(X[0] == centroidal_state)

    for k in 0 to N-1:
        cost += norm_Q(X[k] - ref_traj[k])^2
        cost += norm_R(F[k])^2

        X_next = integrate_centroidal_dynamics(X[k], F[k], contact_schedule[k])
        constraints.append(X[k+1] == X_next)

        for foot in feet:
            if contact_schedule[k][foot] == stance:
                constraints.append(friction_cone_constraints(F[k][foot]))
            else:
                constraints.append(F[k][foot] == 0)

    solution = solve_qp_or_nlp(cost, constraints)
    contact_forces = solution.F[0]

    return contact_forces, solution
```

实现要点：

- Centroidal MPC 通常给出期望接触力或质心运动。
- 底层还需要 WBC 把接触力和任务转成关节力矩。
- 接触时序可以预先给定，也可以由更高层步态规划器提供。


---

### 13.4 Contact Force Optimization

优化接触力：

$$
\min_{f_i}
\|A f-b\|^2
$$

约束：

$$
f_z \ge 0
$$

$$
\sqrt{f_x^2+f_y^2}\le \mu f_z
$$

常用于：

- 四足机器人站立
- 步态控制
- 质心加速度跟踪
- WBC 前端

#### 实现伪代码：Contact Force Optimization

适用场景：四足/人形机器人站立、质心加速度跟踪、接触力分配。

优化变量：所有接触点力 $f$。

```text
function CONTACT_FORCE_QP(com_state, desired_com_acc, contact_positions):
    # Build centroidal mapping:
    # desired wrench = A f
    A = build_centroidal_force_mapping(contact_positions, com_position)
    b = mass * (desired_com_acc - gravity)

    # min 0.5 ||A f - b||_Q^2 + 0.5 ||f||_R^2
    H = transpose(A) * Q * A + R
    g = -transpose(A) * Q * b

    # friction cone approximation for each foot:
    # fz >= fz_min
    # |fx| <= mu fz
    # |fy| <= mu fz
    A_ineq, lower, upper = build_friction_pyramid_constraints(mu, fz_min, fz_max)

    f_star = solve_qp(H, g, A_ineq, lower, upper)
    return f_star
```

实现要点：

- 工程中常把摩擦锥近似成线性摩擦金字塔。
- 支撑脚才有接触力，摆动脚接触力应设为 0。
- 接触力优化常作为 WBC 或 MPC 的一部分。


---

## 14. 鲁棒控制与自适应控制

### 14.1 鲁棒控制

目标：

> 模型不准、扰动存在时，系统仍然稳定。

常见方法：

- $H_\infty$ 控制
- 滑模控制
- 鲁棒 MPC
- Tube MPC

---

### 14.2 滑模控制 Sliding Mode Control

定义滑模面：

$$
s=\dot e+\lambda e
$$

控制目标：

$$
s \rightarrow 0
$$

典型控制律包含符号函数：

$$
u = u_{eq}-K\text{sign}(s)
$$

优点：

- 抗扰动能力强
- 对模型不确定性鲁棒

缺点：

- 抖振 chattering
- 实际执行器不喜欢高频切换

#### 实现伪代码：滑模控制

适用场景：模型不确定、强扰动下的鲁棒控制入门。

```text
function SLIDING_MODE_CONTROL(x, xdot, x_des, xdot_des):
    error = x_des - x
    error_dot = xdot_des - xdot

    s = error_dot + lambda * error

    # sign(s) can cause chattering
    u_switch = K * sat(s / phi)
    u_eq = nominal_model_compensation(x, xdot, x_des, xdot_des)

    u = u_eq + u_switch
    return clamp(u, u_min, u_max)
```

其中 `sat` 是边界层饱和函数：

```text
function sat(y):
    if y > 1: return 1
    if y < -1: return -1
    return y
```

实现要点：

- 直接用 `sign(s)` 容易产生高频抖振。
- 工程上常用饱和函数或边界层降低 chattering。
- 执行器带宽有限时要谨慎。


---

### 14.3 自适应控制

目标：

> 模型参数未知时，在线估计参数并控制。

例如动力学参数：

$$
M(q)\ddot q+C(q,\dot q)\dot q+g(q)=Y(q,\dot q,\ddot q)\theta
$$

其中：

- $Y$：回归矩阵
- $\theta$：动力学参数

自适应控制会在线更新：

$$
\hat \theta
$$

适合：

- 负载变化
- 机械臂抓取未知物体
- 模型参数不准确

#### 实现伪代码：自适应控制

适用场景：动力学参数不确定、负载变化、模型误差较大。

```text
initialize theta_hat

function ADAPTIVE_CONTROL(q, qdot, q_des, qdot_des, qddot_des, dt):
    error = q_des - q
    error_dot = qdot_des - qdot

    s = error_dot + Lambda * error

    qdot_r = qdot_des + Lambda * error
    qddot_r = qddot_des + Lambda * error_dot

    Y = compute_dynamics_regressor(q, qdot, qdot_r, qddot_r)

    tau = Y * theta_hat + Kd * s

    # parameter adaptation law
    theta_hat_dot = Gamma * transpose(Y) * s
    theta_hat = theta_hat + theta_hat_dot * dt

    return clamp(tau, tau_min, tau_max), theta_hat
```

实现要点：

- 自适应控制依赖动力学线性参数化 $Y\theta$。
- 参数估计不一定收敛到真实物理参数，但控制误差可以收敛。
- 入门阶段重点理解结构，不建议一开始作为项目主线。


---

## 15. 学习型控制与强化学习

### 15.1 强化学习 RL

强化学习把控制问题建模为 MDP：

$$
(s_t,a_t,r_t,s_{t+1})
$$

目标是最大化累计回报：

$$
\max_\pi
\mathbb{E}
\left[
\sum_{t=0}^{T}\gamma^t r_t
\right]
$$

其中：

- $s_t$：状态
- $a_t$：动作
- $r_t$：奖励
- $\pi(a|s)$：策略
- $\gamma$：折扣因子

机器人常见算法：

- PPO
- SAC
- TD3
- DDPG
- Dreamer
- MPC + RL
- Imitation Learning
- Residual RL

#### 实现伪代码：强化学习控制训练

适用场景：高维复杂策略学习、接触丰富任务、传统模型难以精确建模的行为。

通用 actor-critic 训练框架：

```text
initialize policy network pi_theta(a | s)
initialize value network V_phi(s) or Q_phi(s, a)
initialize replay buffer or trajectory buffer

for iteration in range(num_iterations):
    trajectories = []

    for episode in range(num_episodes):
        s = env.reset()
        episode_data = []

        for t in range(max_steps):
            a = sample_action(pi_theta, s)
            s_next, reward, done, info = env.step(a)

            episode_data.append(s, a, reward, s_next, done)
            s = s_next

            if done:
                break

        trajectories.append(episode_data)

    advantages = estimate_advantage(trajectories, V_phi)
    update_policy_network(pi_theta, trajectories, advantages)
    update_value_network(V_phi, trajectories)

    evaluate_policy(pi_theta)
```

机器人控制常见执行结构：

```text
policy action = desired joint position / desired joint velocity / residual torque
low-level PD controller converts action to torque
simulator executes torque
reward evaluates tracking, energy, stability, smoothness
```

实现要点：

- 机器人 RL 常用低层 PD 执行动作，不一定直接输出力矩。
- reward 要同时考虑任务完成、能耗、平滑性、约束和安全性。
- 仿真训练需要大量环境随机化和稳定性验证。
- 对求职项目，RL 更适合作为高级拓展，不建议替代 QP/MPC 主线。


---

### 15.2 RL 和传统控制的区别

| 对比 | 传统控制 | 强化学习 |
|---|---|---|
| 依赖模型 | 通常依赖 | 可以不显式依赖 |
| 可解释性 | 强 | 较弱 |
| 约束处理 | 明确 | 较难 |
| 调参方式 | 权重/增益/模型 | reward / network / data |
| 泛化能力 | 依赖设计 | 依赖训练分布 |
| 工程安全性 | 较高 | 需要大量验证 |

---

### 15.3 机器人中更常见的组合方式

实际工程里，RL 往往不是完全替代传统控制，而是组合使用：

```text
RL 生成高层动作
PD 执行低层关节控制
```

或者：

```text
MPC 负责安全约束
RL 学习代价函数 / 残差控制
```

或者：

```text
模仿学习给初始策略
强化学习继续优化
```

---


## 16. 各算法之间的层级关系

```text
最底层执行：
  PID / PD / 电机控制

关节级轨迹跟踪：
  PD + feedforward
  重力补偿
  计算力矩控制

任务空间控制：
  Jacobian 伪逆
  Jacobian 转置
  操作空间控制

接触与交互：
  阻抗控制
  导纳控制
  混合位置-力控制

带约束控制：
  IK-QP
  逆动力学 QP
  WBC-QP

预测与最优控制：
  LQR
  iLQR / DDP
  MPC
  NMPC

腿足机器人高级控制：
  ZMP
  Capture Point
  Centroidal Dynamics
  Centroidal MPC
  Whole-body Control

学习型方法：
  RL
  Imitation Learning
  Residual Learning
```

---

## 17. 每类算法适合解决什么问题？

| 问题 | 常用算法 |
|---|---|
| 单关节位置跟踪 | PID / PD |
| 多关节轨迹跟踪 | PD + feedforward / computed torque |
| 末端位置控制 | IK / Jacobian 伪逆 / 操作空间控制 |
| 末端力控制 | 力控制 / 阻抗控制 / 导纳控制 |
| 有关节限位的 IK | IK-QP |
| 有力矩限制的控制 | 逆动力学 QP |
| 人形机器人多任务控制 | WBC-QP |
| 四足机器人站立和行走 | Centroidal MPC + WBC |
| 倒立摆稳定 | LQR / MPC |
| 非线性轨迹优化 | iLQR / DDP / NMPC |
| 接触丰富任务 | MPC / WBC / RL |
| 大规模策略学习 | PPO / SAC / 模仿学习 |

---

## 18. 推荐学习主线

当前阶段不需要所有算法同时深入，建议按这条线学习：

```text
第一阶段：基础控制
PD
重力补偿
计算力矩控制

第二阶段：运动学控制
FK
Jacobian
IK
微分 IK
Null-space 控制

第三阶段：动力学控制
逆动力学
操作空间控制
阻抗控制

第四阶段：优化控制
LQR
QP
OSQP
二连杆 QP 控制
WBC-QP

第五阶段：预测控制
线性 MPC
iLQR
NMPC

第六阶段：腿足机器人
ZMP
Capture Point
Centroidal Dynamics
Centroidal MPC
Whole-body Control

第七阶段：学习控制
强化学习
模仿学习
MPC + RL
```

### 从伪代码到项目代码的最小落地顺序

建议优先按以下顺序实现，因为它和机器人运动控制知识结构最一致：

```text
1. PD_CONTROLLER
   验证：单摆 / 二连杆关节角跟踪

2. GRAVITY_COMPENSATION_PD
   验证：机械臂低速轨迹，稳态误差下降

3. COMPUTED_TORQUE
   验证：关节轨迹误差和力矩曲线

4. DIFFERENTIAL_IK_DLS
   验证：末端轨迹跟踪，输出 qdot 曲线

5. QP_IK
   验证：关节限位和速度限幅是否满足

6. INVERSE_ACCELERATION_QP
   验证：末端加速度跟踪 + 力矩约束

7. LINEAR_MPC_STEP
   验证：滚动时域预测轨迹和控制输入

8. ILQR
   验证：cost 是否下降，rollout 是否逼近目标

9. WHOLE_BODY_CONTROL_QP
   验证：动力学等式、接触约束、摩擦锥和任务误差
```

每个算法实现后都建议至少输出：

```text
误差曲线：tracking_error.png
控制曲线：control_input.png
约束曲线：constraint_violation.png
仿真视频：demo.mp4 或 demo.gif
复现实验命令：README.md
```


---

## 19. 面试常见核心问题

需要能回答：

1. PID 和 PD 的区别是什么？
2. 为什么机器人控制常用 PD + feedforward？
3. 重力补偿有什么作用？
4. 计算力矩控制为什么可以把非线性系统变成近似线性误差系统？
5. Jacobian 的物理意义是什么？
6. $J^T F = \tau$ 表示什么？
7. IK、微分 IK、QP-IK 有什么区别？
8. 操作空间控制和关节空间控制有什么区别？
9. 阻抗控制和导纳控制有什么区别？
10. LQR 和 MPC 的关系是什么？
11. QP 在机器人控制里解决什么问题？
12. WBC 的优化变量通常有哪些？
13. MPC 为什么只执行第一个控制量？
14. iLQR 和 LQR 的关系是什么？
15. 强化学习和最优控制有什么关系？

---

## 20. 最核心总结

可以把机器人控制算法理解成四个层级：

```text
第一层：误差反馈
PD / PID
解决“偏了就拉回来”

第二层：模型补偿
重力补偿 / 逆动力学 / 计算力矩
解决“机器人本身有复杂动力学”

第三层：任务映射
Jacobian / IK / 操作空间控制
解决“末端任务怎么转成关节动作”

第四层：优化决策
LQR / QP / WBC / MPC / iLQR / NMPC
解决“多目标、多约束、预测未来的问题”
```

一句话总括：

> 传统控制负责稳定和跟踪；运动学控制负责从任务空间映射到关节空间；动力学控制负责把期望运动变成力矩；QP/WBC 负责处理多任务和约束；MPC/iLQR 负责预测未来并优化控制序列；RL 负责从数据中学习复杂策略。

---

## 21. 和项目实践的对应关系

对于仿真项目，可以按以下方式落地：

| 项目任务 | 对应算法 | 主要验证方式 |
|---|---|---|
| 单摆 / 二连杆 PD 跟踪 | PD / PID | 角度误差曲线、控制力矩曲线 |
| 二连杆重力补偿 | 重力补偿 + PD | 低速跟踪误差、稳态误差 |
| 二连杆逆动力学控制 | Computed Torque | 轨迹跟踪误差、力矩是否平滑 |
| 末端轨迹跟踪 | Jacobian / IK / OSC | 末端轨迹图、MuJoCo 视频 |
| 二连杆 QP 控制 | 逆加速度 QP / OSQP | 约束满足、末端误差 |
| 线性 MPC | LQR / QP / MPC | 滚动时域轨迹、控制输入 |
| iLQR demo | iLQR / DDP-lite | cost 收敛、rollout 轨迹 |
| 人形 / 四足入门 | WBC / Centroidal MPC | 接触力、质心误差、姿态稳定 |

---

## 22. 建议文件命名

如果放入你的机器人运动控制项目，可以保存为：

```text
docs/optimization_control/robot_control_algorithms_summary.md
```

或者：

```text
docs/control_notes/robot_control_algorithms_summary.md
```
