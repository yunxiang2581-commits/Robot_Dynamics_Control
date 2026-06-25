# walk_wbc 第七轮阅读笔记

## 1. 本轮目标

第七轮回答一个问题：

$$
\text{已知 } \ddot q_{\text{kin}}\text{，如何求接触力 }F\text{ 和关节力矩 }\tau\text{？}
$$

本轮关注：

- `WBC_priority::computeTau()`
- floating-base 动力学等式约束
- 接触力 / 接触力矩 QP 变量
- 最后从动力学反算关节力矩

R6 已经通过 `PriorityTasks::computeAll()` 得到运动学优先级解：

$$
\Delta q_{\text{kin}},\quad
\dot q_{\text{kin}},\quad
\ddot q_{\text{kin}}
$$

R7 不重新排序任务，而是在 R6 结果基础上做动力学一致性修正。

## 2. R6 和 R7 的分工

R6 解决的是任务协调：

$$
\text{task priority}
\rightarrow
\ddot q_{\text{kin}}
$$

它回答：

```text
身体任务、支撑脚任务、摆动脚任务、冗余关节任务之间如何按优先级协调？
```

但是它本质上是运动学 / 加速度层求解，不一定保证这个加速度一定能由地面接触力和关节力矩真实实现。

R7 解决的是动力学实现：

$$
\ddot q_{\text{kin}}
\rightarrow
F_{\text{opt}},\ \tau
$$

它回答：

```text
给定 R6 的期望加速度，如何找到接触力和关节力矩，
使 floating-base 动力学成立？
```

## 3. QP 变量

`computeTau()` 里的 QP 变量是：

$$
x
=
\begin{bmatrix}
\delta \ddot q_b\\
\delta F
\end{bmatrix}
\in \mathbb{R}^{18}
$$

其中：

$$
\delta \ddot q_b \in \mathbb{R}^{6}
$$

表示 floating base 的 6 维加速度修正。

$$
\delta F \in \mathbb{R}^{12}
$$

表示左右脚接触 wrench 的修正。

代码中：

```cpp
// QP problem contains joint torque, QP_nv=6+12, QP_nc=22;
```

这里注释里的 `joint torque` 容易误导。实际 QP 变量不是关节力矩，而是：

$$
x =
\begin{bmatrix}
\text{base acceleration correction}\\
\text{contact wrench correction}
\end{bmatrix}
$$

关节力矩是在 QP 求解之后由动力学方程反算出来的。

## 4. 为什么接触力是 12 维

这里的 \(F\) 不是单纯三维力，而是双脚的 6D contact wrench。

单只脚：

$$
F_{\text{foot}}
=
\begin{bmatrix}
f_x\\
f_y\\
f_z\\
\tau_x\\
\tau_y\\
\tau_z
\end{bmatrix}
\in \mathbb{R}^{6}
$$

前 3 维是接触力，后 3 维是足端接触力矩。

两只脚叠起来：

$$
F
=
\begin{bmatrix}
F_L\\
F_R
\end{bmatrix}
=
\begin{bmatrix}
f_{Lx}\\
f_{Ly}\\
f_{Lz}\\
\tau_{Lx}\\
\tau_{Ly}\\
\tau_{Lz}\\
f_{Rx}\\
f_{Ry}\\
f_{Rz}\\
\tau_{Rx}\\
\tau_{Ry}\\
\tau_{Rz}
\end{bmatrix}
\in \mathbb{R}^{12}
$$

代码里也按左右脚各 6 行拼接：

```cpp
Jfe = Eigen::MatrixXd::Zero(12, model_nv);
Jfe.block(0, 0, 6, model_nv) = robotState.J_l;
Jfe.block(6, 0, 6, model_nv) = robotState.J_r;
```

所以：

$$
J_{fe}
=
\begin{bmatrix}
J_L\\
J_R
\end{bmatrix}
\in \mathbb{R}^{12\times n_v}
$$

对应的接触广义力是：

$$
J_{fe}^{T}F \in \mathbb{R}^{n_v}
$$

维度检查：

$$
J_{fe}^{T}\in\mathbb{R}^{n_v\times 12},
\quad
F\in\mathbb{R}^{12},
\quad
J_{fe}^{T}F\in\mathbb{R}^{n_v}
$$

walking 时虽然只有一只脚支撑，但代码仍统一保留双脚 12 维，再通过后面的上下界约束把摆动脚接触量限制到接近 0。

## 5. selection matrix 的含义

假设广义速度维度是：

$$
n_v = 6+n_j
$$

前 6 维是 floating base，后面 \(n_j\) 维是关节。

### 5.1 \(S_f\)：取出 floating-base 方程

代码构造：

```cpp
Sf = Eigen::MatrixXd::Zero(6, model_nv);
Sf.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6);
```

数学上：

$$
S_f
=
\begin{bmatrix}
I_6 & 0
\end{bmatrix}
\in \mathbb{R}^{6\times n_v}
$$

所以 \(S_f y\) 就是取一个广义向量 \(y\) 的前 6 维 base 部分。

### 5.2 \(S_b^T\)：把 base 修正嵌回全身加速度

代码构造：

```cpp
St_qpV1 = Eigen::MatrixXd::Zero(model_nv, 6);
St_qpV1.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6);
```

数学上可理解为：

$$
S_b^T
=
\begin{bmatrix}
I_6\\
0
\end{bmatrix}
\in \mathbb{R}^{n_v\times 6}
$$

因此：

$$
S_b^T\delta\ddot q_b
=
\begin{bmatrix}
\delta\ddot q_b\\
0
\end{bmatrix}
$$

也就是只修正 floating base 加速度，不直接修正关节加速度。

## 6. 从完整动力学到 floating-base 等式

完整刚体动力学：

$$
M(q)\ddot q+h(q,\dot q)=J_{fe}^{T}F+S^{T}\tau
$$

其中：

- \(M(q)\)：质量矩阵，对应 `dyn_M`
- \(h(q,\dot q)\)：重力、科氏、离心等非线性项，对应 `dyn_Non`
- \(J_{fe}^{T}F\)：足端接触 wrench 产生的广义力
- \(S^{T}\tau\)：关节力矩映射到广义力

floating base 是欠驱动的，前 6 维没有电机力矩。因此取前 6 行后：

$$
S_fS^T\tau = 0
$$

所以 floating-base 动力学约束是：

$$
S_f
\left(
M\ddot q+h-J_{fe}^{T}F
\right)
=0
$$

物理意义是：

```text
base 自己没有电机；
base 的线加速度和角加速度必须由惯性、重力和接触力共同解释；
不能凭空产生一个 floating-base 动力学残差。
```

## 7. 把 QP 变量代入动力学等式

R7 定义：

$$
\ddot q_{\text{opt}}
=
\ddot q_{\text{kin}}
+
S_b^T\delta\ddot q_b
$$

$$
F_{\text{opt}}
=
F_{\text{ff}}
+
\delta F
$$

代入 floating-base 动力学：

$$
S_f
\left[
M
\left(
\ddot q_{\text{kin}}
+
S_b^T\delta\ddot q_b
\right)
+
h
-
J_{fe}^{T}
\left(
F_{\text{ff}}
+
\delta F
\right)
\right]
=0
$$

展开：

$$
S_fM\ddot q_{\text{kin}}
+
S_fMS_b^T\delta\ddot q_b
+
S_fh
-
S_fJ_{fe}^{T}F_{\text{ff}}
-
S_fJ_{fe}^{T}\delta F
=0
$$

把未知量放左边，把已知项放右边：

$$
\left[
S_fMS_b^T
\quad
-S_fJ_{fe}^{T}
\right]
\begin{bmatrix}
\delta\ddot q_b\\
\delta F
\end{bmatrix}
=
-S_fM\ddot q_{\text{kin}}
-S_fh
+S_fJ_{fe}^{T}F_{\text{ff}}
$$

这正好对应代码：

```cpp
eigen_qp_A1.block<6, 6>(0, 0) = Sf * dyn_M * St_qpV1;
eigen_qp_A1.block<6, 12>(0, 6) = -Sf * Jfe.transpose();

eqRes = -Sf * dyn_M * ddq_final_kin
        -Sf * dyn_Non
        +Sf * Jfe.transpose() * Fr_ff;
```

所以：

$$
A_1x=b_1
$$

其中：

$$
A_1
=
\begin{bmatrix}
S_fMS_b^T & -S_fJ_{fe}^{T}
\end{bmatrix}
$$

$$
x
=
\begin{bmatrix}
\delta\ddot q_b\\
\delta F
\end{bmatrix}
$$

$$
b_1
=
-S_fM\ddot q_{\text{kin}}
-S_fh
+S_fJ_{fe}^{T}F_{\text{ff}}
$$

## 8. 这个等式到底在修什么

如果直接把 R6 的结果和当前前馈接触力代进去：

$$
r_b
=
S_f
\left(
M\ddot q_{\text{kin}}
+
h
-
J_{fe}^{T}F_{\text{ff}}
\right)
$$

通常：

$$
r_b\ne 0
$$

这说明：

```text
R6 给出的运动学加速度 + 当前前馈接触力
不一定能解释 floating-base 动力学。
```

所以 R7 用两个量去修：

$$
\delta\ddot q_b
\quad\text{and}\quad
\delta F
$$

让：

$$
S_f
\left(
M\ddot q_{\text{opt}}
+
h
-
J_{fe}^{T}F_{\text{opt}}
\right)
=0
$$

成立。

可以把这一步理解成：

$$
\text{floating-base dynamic consistency repair}
$$

也就是把 R6 的运动学优先级结果修正成动力学可实现结果。

## 9. W 矩阵每一行的含义

`computeTau()` 里第二组约束来自接触 wrench 的可行域。

代码先为单脚构造 8 行约束，再复制到另一只脚：

```cpp
W(0, 0) = 1;
W(0, 2) = sqrt(2) / 2.0 * miu;
W(1, 0) = -1;
W(1, 2) = sqrt(2) / 2.0 * miu;
W(2, 1) = 1;
W(2, 2) = sqrt(2) / 2.0 * miu;
W(3, 1) = -1;
W(3, 2) = sqrt(2) / 2.0 * miu;
W.block<4, 4>(4, 2) = Eigen::MatrixXd::Identity(4, 4);
```

单脚 wrench 记为：

$$
F_{\text{foot}}
=
\begin{bmatrix}
f_x & f_y & f_z & \tau_x & \tau_y & \tau_z
\end{bmatrix}^T
$$

前 4 行是摩擦锥的线性近似：

```text
0:  fx + mu/sqrt(2) * fz
1: -fx + mu/sqrt(2) * fz
2:  fy + mu/sqrt(2) * fz
3: -fy + mu/sqrt(2) * fz
```

配合下界 0，可以理解为限制切向力不要超过法向力按摩擦系数给出的范围。

后 4 行来自：

```cpp
W.block<4, 4>(4, 2) = I;
```

因此对应：

```text
4: fz
5: tau_x
6: tau_y
7: tau_z
```

它们分别限制：

```text
法向力上下界
足端 roll / pitch / yaw 接触力矩上下界
```

双脚约束为：

```text
0 ~ 7:   左脚 wrench 约束
8 ~ 15:  右脚 wrench 约束
```

这里还有一个坐标系细节：

```cpp
W = W * Mw2b;
```

`Mw2b` 用脚底姿态 `Rfe.transpose()` 把世界系 wrench 转到脚/接触局部系里做约束。这样摩擦锥和足底力矩界限是在脚底局部坐标中解释的，而不是直接用世界系分量解释。

## 10. `f_low / f_upp` 在 stand 和 walk 下的区别

默认上下界先给双脚同样的接触能力：

```text
前 4 行摩擦锥表达式：
  low = 0
  upp = 1e10

fz：
  low = 10
  upp = 1400

tau_x / tau_y / tau_z：
  stand 使用 tau_low_stand_L / tau_upp_stand_L
  walk 使用 tau_low_walk_L / tau_upp_walk_L
```

站立时：

```text
左右脚都允许承受法向力和足端力矩。
```

走路时：

```text
支撑脚保留接触约束；
摆动脚的接触 wrench 被限制到接近 0。
```

具体地说，`LSt` 表示左脚支撑、右脚摆动。右脚对应约束行 `8 ~ 15`：

```cpp
f_upp(12) = 0;
f_upp(13) = 0;
f_upp(14) = 0;
f_upp(15) = 0;
f_low(12) = 0;
f_low(13) = 0;
f_low(14) = 0;
f_low(15) = 0;
f_low(8) = -1e-7;
f_low(9) = -1e-7;
f_low(10) = -1e-7;
f_low(11) = -1e-7;
```

这会把右脚的 `fz / tau_x / tau_y / tau_z` 固定为 0，并把摩擦锥前 4 行下界略微放到 `-1e-7`，避免数值上过死。

`RSt` 表示右脚支撑、左脚摆动，所以同样逻辑作用到左脚约束行 `0 ~ 7`。

## 11. 为什么约束右端要减去 `W * Fr_ff`

QP 的未知量不是完整接触 wrench，而是接触 wrench 修正量：

$$
F_{\text{opt}}=F_{\text{ff}}+\delta F
$$

接触不等式原本是：

$$
f_{\text{low}}
\le
W F_{\text{opt}}
\le
f_{\text{upp}}
$$

代入：

$$
f_{\text{low}}
\le
W(F_{\text{ff}}+\delta F)
\le
f_{\text{upp}}
$$

把已知的前馈项移到右边：

$$
f_{\text{low}}-WF_{\text{ff}}
\le
W\delta F
\le
f_{\text{upp}}-WF_{\text{ff}}
$$

对应代码：

```cpp
neqRes_low = f_low - W * Fr_ff;
neqRes_upp = f_upp - W * Fr_ff;
```

所以这一层约束的实际含义是：

```text
在 Fr_ff 的基础上，QP 可以调整 delta_F，
但最终 F_opt 必须仍然落在接触可行域里。
```

## 12. QP 总约束结构

最终 QP 约束矩阵是：

```text
A_final =
[
  A1    // 6 行 floating-base 动力学等式
  A2    // 16 行接触 wrench 不等式
]
```

维度为：

$$
A_{\text{final}}\in\mathbb{R}^{22\times18}
$$

前 6 行通过：

```cpp
lbA[0:6] = eqRes;
ubA[0:6] = eqRes;
```

强制成为等式。

后 16 行通过：

```cpp
lbA[6:] = neqRes_low;
ubA[6:] = neqRes_upp;
```

表达接触不等式。

因此：

```text
QP_nc = 22 = 6 + 16
QP_nv = 18 = 6 + 12
```

这也是 R7 最重要的维度检查。

## 13. H 矩阵权重的作用

目标函数是 qpOASES 标准形式：

$$
\frac{1}{2}x^THx+g^Tx
$$

代码里：

```cpp
qp_g[i] = 0;
```

所以目标主要是在惩罚：

```text
delta_ddq_b 不要太大
delta_F 不要太大
```

权重设置为：

```cpp
eigen_qp_H.block<6, 6>(0, 0) = Q2 * 2.0 * 1e7;
eigen_qp_H.block<12, 12>(6, 6) = Q1 * 2.0 * 1e1;
```

含义是：

```text
base 加速度修正的代价非常大；
接触 wrench 修正的代价相对小很多。
```

所以 QP 倾向于：

```text
尽量保留 R6 给出的 base 加速度，
优先通过调整接触 wrench 来满足 floating-base 动力学。
```

站立模式下还额外放大了几个接触力矩相关分量：

```cpp
eigen_qp_H(9,9) *= 100;
eigen_qp_H(10,10) *= 100;
eigen_qp_H(15,15) *= 100;
eigen_qp_H(16,16) *= 100;
```

这些索引位于 `delta_F` 部分，约束的是左右脚的部分接触力矩修正。直观理解是：站立时更不希望用很大的足端力矩修正来“硬拧”身体。

## 14. qpOASES 调用和求解结果

`computeTau()` 每次把初值设为 0：

```cpp
xOpt_iniGuess[i] = 0;
qp_g[i] = 0;
```

然后调用：

```cpp
QP_prob.init(
    qp_H,
    qp_g,
    qp_A,
    NULL,
    NULL,
    qp_lbA,
    qp_ubA,
    nWSR,
    &cpu_time,
    xOpt_iniGuess
);
```

这里没有显式变量上下界：

```text
lb = NULL
ub = NULL
```

主要限制都放在 `lbA <= A x <= ubA` 里。

如果返回成功：

```cpp
eigen_xOpt(i) = xOpt[i];
```

再拆成：

```cpp
eigen_ddq_Opt = ddq_final_kin;
eigen_ddq_Opt.block<6, 1>(0, 0) += eigen_xOpt.block<6, 1>(0, 0);
eigen_fr_Opt = Fr_ff + eigen_xOpt.block<12, 1>(6, 0);
```

也就是：

$$
\ddot q_{\text{opt}}
=
\ddot q_{\text{kin}}
+
\begin{bmatrix}
\delta\ddot q_b\\
0
\end{bmatrix}
$$

$$
F_{\text{opt}}
=
F_{\text{ff}}+\delta F
$$

## 15. 从 QP 结果反算关节力矩

QP 解完后，源码没有把关节力矩作为优化变量取出来，而是由完整动力学反算：

```cpp
tauRes = dyn_M * eigen_ddq_Opt
       + dyn_Non
       - Jfe.transpose() * eigen_fr_Opt;

tauJointRes = tauRes.block(6, 0, model_nv - 6, 1);
```

对应数学式：

$$
\tau_{\text{all}}
=
M\ddot q_{\text{opt}}
+
h
-
J_{fe}^{T}F_{\text{opt}}
$$

再取关节部分：

$$
\tau_{\text{joint}}
=
\tau_{\text{all}}[6:]
$$

这里为什么取 `[6:]`？

```text
前 6 维是 floating base，没有电机；
后 model_nv - 6 维才是真正关节电机力矩。
```

这一步完成了 R7 的核心目标：

```text
R6 的期望加速度
-> R7 的动力学一致性修正
-> 可送给关节层的 tauJointRes
```

## 16. 写回 DataBus 的一个细节

`dataBusWrite()` 里有一个容易忽略的覆盖：

```cpp
robotState.wbc_ddq_final = eigen_ddq_Opt;
...
robotState.wbc_delta_q_final = delta_q_final_kin;
robotState.wbc_dq_final = dq_final_kin;
robotState.wbc_ddq_final = ddq_final_kin;
```

也就是说：

```text
wbc_ddq_final 先被写成 QP 修正后的 eigen_ddq_Opt，
随后又被 ddq_final_kin 覆盖。
```

所以当前源码实际对外保留的是：

```text
wbc_delta_q_final = R6 运动学层 delta_q
wbc_dq_final      = R6 运动学层 dq
wbc_ddq_final     = R6 运动学层 ddq
wbc_tauJointRes   = R7 用 eigen_ddq_Opt / eigen_fr_Opt 反算出的关节力矩
wbc_FrRes         = R7 求出的最终接触 wrench
```

这解释了前面的问题：

```text
eigen_ddq_Opt 虽然没有作为 wbc_ddq_final 留给下游，
但它已经参与 tauJointRes 的计算。
```

在 `walk_wbc.cpp` 里，下游真正使用的是：

```cpp
pos_des = integrateDIY(RobotState.q, RobotState.wbc_delta_q_final);
RobotState.motors_pos_des = pos_des 的关节部分;
RobotState.motors_vel_des = RobotState.wbc_dq_final;
RobotState.motors_tor_des = RobotState.wbc_tauJointRes;
```

因此 R7 到 PVT 的接口是：

```text
WBC:
  R6 delta_q / dq
  R7 tauJointRes

PVT:
  tau_out = PD(pos_des, vel_des) + tauJointRes
```

对应 `PVT_Ctr::calMotorsPVT()`：

```cpp
tauDes =
    Kp * (motor_pos_des - motor_pos_cur)
  + Kd * (motor_vel_des - motor_vel)
  + motor_tor_des;
```

最后：

```text
PVT_Ctr::dataBusWrite()
-> RobotState.motors_tor_out
-> MJ_Interface::setMotorsTorque()
-> mj_data->ctrl
```

所以整条 R7 到执行端的链路是：

```text
computeTau()
-> tauJointRes
-> RobotState.wbc_tauJointRes
-> RobotState.motors_tor_des
-> PVT_Ctr::calMotorsPVT()
-> RobotState.motors_tor_out
-> mj_data->ctrl
```

## 17. R7 本轮结论

`WBC_priority::computeTau()` 做的不是“直接优化关节力矩”，而是：

```text
1. 固定 R6 给出的关节加速度部分；
2. 允许小幅修正 floating-base 加速度；
3. 允许修正双脚接触 wrench；
4. 用 floating-base 动力学等式保证欠驱动 base 动力学一致；
5. 用摩擦锥、法向力、足端力矩界限保证接触 wrench 合理；
6. 最后由完整动力学反算关节力矩 tauJointRes。
```

本轮读完后，R7 可以压缩成一句话：

```text
R7 把 R6 的运动学加速度解，修正成满足 floating-base 动力学和接触约束的关节前馈力矩。
```

## 18. 二刷补充：几个容易误读的实现细节

### 18.1 `Fr_ff` 在不同 demo 里的来源不一样

R7 的 QP 不是“从零开始找接触力”，而是在前馈接触 wrench 上做修正：

$$
F_{\text{opt}} = F_{\text{ff}} + \delta F
$$

源码里 `Fr_ff` 的来源有两种常见写法。

非 MPC 的 `walk_wbc.cpp` / `walk_wbc_joystick.cpp` 中，通常直接写死为：

```cpp
RobotState.Fr_ff << 0,0,370,0,0,0,
                    0,0,370,0,0,0;
```

这表示双脚各给一个大约 370N 的法向前馈。

而在 `walk_mpc_wbc.cpp` / `walk_mpc_wbc_joystick.cpp` 中，`Fr_ff` 会先由 MPC 写入：

```cpp
Data.Fr_ff = Ufe.block<12, 1>(0, 0);
```

所以 R7 的输入接口保持不变，但上游来源可以是手写前馈，也可以是 MPC 结果。

### 18.2 `Rfe` 只取一个脚底姿态

`computeTau()` 里构造坐标变换矩阵时：

```cpp
if (motionStateCur == DataBus::Stand)
{
    Rfe = fe_l_rot_cur_W;
}
else
{
    Rfe = stance_fe_rot_cur_W;
}
```

然后把同一个 `Rfe` 复制到 `Mw2b` 的 4 个 3x3 块里：

```cpp
Mw2b.block(0, 0, 3, 3) = Rfe.transpose();
Mw2b.block(3, 3, 3, 3) = Rfe.transpose();
Mw2b.block(6, 6, 3, 3) = Rfe.transpose();
Mw2b.block(9, 9, 3, 3) = Rfe.transpose();
```

这意味着：

```text
当前源码对双脚 wrench 约束使用的是同一个脚底姿态基准，
而不是左右脚分别单独变换。
```

读源码时要把这当成“当前实现选择”，不要默认它是左右脚各自最严格的做法。

### 18.3 QP 失败时的行为

这里有一个很实用的鲁棒性点。

代码先调用：

```cpp
res = QP_prob.init(...);
qpStatus = qpOASES::getSimpleStatus(res);
```

然后只有在成功时才把新解写回：

```cpp
if (res == qpOASES::SUCCESSFUL_RETURN)
    for (int i = 0; i < QP_nv; i++)
        eigen_xOpt(i) = xOpt[i];
```

所以如果 QP 失败，`eigen_xOpt` 不会被这次结果覆盖，后面仍会继续沿用旧值。

这带来一个实际后果：

```text
QP 失败时，tauJointRes 可能不会突然变成全新错误值，
而是更像沿用上一帧的优化结果。
```

当然这不是最理想的故障处理，但它比直接把未定义解写下去要稳一点。

### 18.4 `copy_Eigen_to_real_t()` 的拷贝顺序

矩阵传给 qpOASES 前，会经过一个行优先的手工拷贝：

```cpp
for (int i = 0; i < nRows; i++)
{
    for (int j = 0; j < nCols; j++)
    {
        target[count++] = isinf(source(i, j)) ? qpOASES::INFTY : source(i, j);
    }
}
```

这说明两个点：

1. `qpOASES` 这边吃的是线性数组，不是 Eigen 矩阵对象。
2. `isinf()` 会被转成 `qpOASES::INFTY`，所以某些无穷约束是通过这里传进去的。

这类细节虽然不改数学本身，但会影响“矩阵进去后到底长什么样”。

### 18.5 当前你应该怎么记 R7

如果把 R7 压成更工程化的一句，建议记成：

```text
R7 = 在前馈接触 wrench 上，修正 floating-base 加速度和接触力，
让 R6 的运动学解满足欠驱动动力学和接触约束，
再反算关节前馈力矩。
```

## 19. 逐段逐行复盘版

这一节按源码执行顺序复盘 `WBC_priority::computeTau()`，重点记录：

```text
代码段在构造什么数学对象
数据从哪里来
数据流向哪里
这一段和 R6 / R7 / PVT 的关系
```

### 19.1 QP 的变量不是关节力矩

`computeTau()` 里的 QP 变量是：

$$
x=
\begin{bmatrix}
\delta\ddot q_b\\
\delta F
\end{bmatrix}
\in\mathbb{R}^{18}
$$

其中：

$$
\delta\ddot q_b\in\mathbb{R}^{6}
$$

是 floating-base 的 6 维加速度修正。

$$
\delta F\in\mathbb{R}^{12}
$$

是双脚接触 wrench 修正。

所以这里的 `18` 不是关节数量，而是：

$$
18=6+12
$$

关节力矩不作为 QP 变量，后面会由完整动力学反算。

### 19.2 构造 floating-base 动力学等式 `A1`

源码构造：

$$
A_1=
\begin{bmatrix}
S_fMS_b^T & -S_fJ_{fe}^{T}
\end{bmatrix}
\in\mathbb{R}^{6\times18}
$$

对应约束：

$$
A_1x=b_1
$$

其中：

$$
b_1=
-S_fM\ddot q_{\text{kin}}
-S_fh
+S_fJ_{fe}^{T}F_{\text{ff}}
$$

这个等式来自 floating-base 动力学：

$$
S_f
\left(
M\ddot q_{\text{opt}}
+h
-J_{fe}^{T}F_{\text{opt}}
\right)=0
$$

R7 定义：

$$
\ddot q_{\text{opt}}
=
\ddot q_{\text{kin}}
+S_b^T\delta\ddot q_b
$$

$$
F_{\text{opt}}
=
F_{\text{ff}}+\delta F
$$

代入并整理，就得到：

$$
\begin{bmatrix}
S_fMS_b^T & -S_fJ_{fe}^{T}
\end{bmatrix}
\begin{bmatrix}
\delta\ddot q_b\\
\delta F
\end{bmatrix}
=
-S_fM\ddot q_{\text{kin}}
-S_fh
+S_fJ_{fe}^{T}F_{\text{ff}}
$$

这一段导入 `A1` 的数据是：

```text
Sf        固定选择矩阵，取 floating-base 前 6 行
dyn_M     当前质量矩阵 M(q)
St_qpV1   固定嵌入矩阵，把 6 维 base 修正嵌回 nv 维
Jfe       双脚 Jacobian，[J_l; J_r]
```

而这些不进入 `A1`，进入右端 `eqRes`：

```text
ddq_final_kin
dyn_Non
Fr_ff
```

数据流：

```text
Pin_KinDyn:
  q, dq -> dyn_M / dyn_Non / J_l / J_r

R6 computeDdq:
  task priority -> ddq_final_kin

demo / MPC:
  hand-written force or MPC -> Fr_ff

computeTau:
  dyn_M / dyn_Non / Jfe / ddq_final_kin / Fr_ff
  -> A1 / eqRes
```

### 19.3 构造接触 wrench 约束矩阵 `W`

单脚接触 wrench 是：

$$
F_{\text{foot}}
=
\begin{bmatrix}
f_x\\
f_y\\
f_z\\
\tau_x\\
\tau_y\\
\tau_z
\end{bmatrix}
\in\mathbb{R}^{6}
$$

双脚接触 wrench 是：

$$
F=
\begin{bmatrix}
F_L\\
F_R
\end{bmatrix}
\in\mathbb{R}^{12}
$$

源码先构造单脚约束矩阵：

$$
W_{\text{foot}}
=
\begin{bmatrix}
1 & 0 & \frac{\mu}{\sqrt{2}} & 0 & 0 & 0\\
-1 & 0 & \frac{\mu}{\sqrt{2}} & 0 & 0 & 0\\
0 & 1 & \frac{\mu}{\sqrt{2}} & 0 & 0 & 0\\
0 & -1 & \frac{\mu}{\sqrt{2}} & 0 & 0 & 0\\
0 & 0 & 1 & 0 & 0 & 0\\
0 & 0 & 0 & 1 & 0 & 0\\
0 & 0 & 0 & 0 & 1 & 0\\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}
\in\mathbb{R}^{8\times6}
$$

乘上单脚 wrench 后：

$$
W_{\text{foot}}F_{\text{foot}}
=
\begin{bmatrix}
f_x+\frac{\mu}{\sqrt{2}}f_z\\
-f_x+\frac{\mu}{\sqrt{2}}f_z\\
f_y+\frac{\mu}{\sqrt{2}}f_z\\
-f_y+\frac{\mu}{\sqrt{2}}f_z\\
f_z\\
\tau_x\\
\tau_y\\
\tau_z
\end{bmatrix}
$$

前 4 行是摩擦锥线性近似：

$$
|f_x|\le \frac{\mu}{\sqrt{2}}f_z
$$

$$
|f_y|\le \frac{\mu}{\sqrt{2}}f_z
$$

后 4 行用于限制：

$$
f_z,\quad \tau_x,\quad \tau_y,\quad \tau_z
$$

双脚矩阵先写成：

$$
W_{\text{raw}}
=
\begin{bmatrix}
W_{\text{foot}} & 0\\
0 & W_{\text{foot}}
\end{bmatrix}
\in\mathbb{R}^{16\times12}
$$

然后通过脚底姿态把世界系 wrench 转到接触局部系：

$$
M_{w2b}
=
\begin{bmatrix}
R_{fe}^{T} & 0 & 0 & 0\\
0 & R_{fe}^{T} & 0 & 0\\
0 & 0 & R_{fe}^{T} & 0\\
0 & 0 & 0 & R_{fe}^{T}
\end{bmatrix}
$$

最终：

$$
W=W_{\text{raw}}M_{w2b}
$$

所以 `W` 的作用是：

$$
f_{\text{low}}
\le
WF
\le
f_{\text{upp}}
$$

也就是把双脚接触 wrench 映射成 16 个接触约束量。

### 19.4 用行号控制哪只脚出力

因为：

$$
WF=
\begin{bmatrix}
W_{\text{foot}}F_L\\
W_{\text{foot}}F_R
\end{bmatrix}
$$

所以：

```text
0 ~ 7   行：左脚约束
8 ~ 15  行：右脚约束
```

每只脚 8 行依次是：

$$
\begin{bmatrix}
f_x+\frac{\mu}{\sqrt{2}}f_z\\
-f_x+\frac{\mu}{\sqrt{2}}f_z\\
f_y+\frac{\mu}{\sqrt{2}}f_z\\
-f_y+\frac{\mu}{\sqrt{2}}f_z\\
f_z\\
\tau_x\\
\tau_y\\
\tau_z
\end{bmatrix}
$$

`LSt` 表示左脚支撑、右脚摆动，因此右脚应该接近无接触：

$$
F_R\approx0
$$

源码通过设置右脚约束行来实现：

```text
右脚行号 8 ~ 15
其中 12 ~ 15 对应 f_Rz / tau_Rx / tau_Ry / tau_Rz
```

所以把第 12-15 行上下界都设成 0，可以得到：

$$
f_{Rz}=0
$$

$$
\tau_{Rx}=\tau_{Ry}=\tau_{Rz}=0
$$

再把第 8-11 行摩擦锥下界放松到：

$$
-10^{-7}
$$

可以让：

$$
f_{Rx}\approx0,\quad f_{Ry}\approx0
$$

因此：

$$
F_R\approx0
$$

`RSt` 时同理，只是限制左脚：

$$
F_L\approx0
$$

所以这里不是直接写 `F_L = 0` 或 `F_R = 0`，而是通过：

```text
F 的排列顺序
W 的行号
f_low / f_upp 的对应位置
```

来间接限制摆动脚不出力。

### 19.5 构造接触不等式 `A2`

最终接触 wrench 是：

$$
F_{\text{opt}}=F_{\text{ff}}+\delta F
$$

接触约束是：

$$
f_{\text{low}}
\le
WF_{\text{opt}}
\le
f_{\text{upp}}
$$

代入：

$$
f_{\text{low}}
\le
W(F_{\text{ff}}+\delta F)
\le
f_{\text{upp}}
$$

移项得到：

$$
f_{\text{low}}-WF_{\text{ff}}
\le
W\delta F
\le
f_{\text{upp}}-WF_{\text{ff}}
$$

因为 QP 变量是：

$$
x=
\begin{bmatrix}
\delta\ddot q_b\\
\delta F
\end{bmatrix}
$$

接触约束只作用在 \(\delta F\) 上，所以：

$$
A_2=
\begin{bmatrix}
0_{16\times6} & W
\end{bmatrix}
\in\mathbb{R}^{16\times18}
$$

上下界是：

$$
neqRes_{\text{low}}=f_{\text{low}}-WF_{\text{ff}}
$$

$$
neqRes_{\text{upp}}=f_{\text{upp}}-WF_{\text{ff}}
$$

### 19.6 组装完整 QP 数据

qpOASES 要求的问题形式是：

$$
\min_x
\frac{1}{2}x^THx+g^Tx
$$

约束为：

$$
l_A\le Ax\le u_A
$$

代码把约束矩阵拼成：

$$
A_{\text{final}}
=
\begin{bmatrix}
A_1\\
A_2
\end{bmatrix}
=
\begin{bmatrix}
S_fMS_b^T & -S_fJ_{fe}^{T}\\
0_{16\times6} & W
\end{bmatrix}
\in\mathbb{R}^{22\times18}
$$

约束下界：

$$
l_A=
\begin{bmatrix}
eqRes\\
neqRes_{\text{low}}
\end{bmatrix}
$$

约束上界：

$$
u_A=
\begin{bmatrix}
eqRes\\
neqRes_{\text{upp}}
\end{bmatrix}
$$

前 6 行上下界相等，因此是等式：

$$
A_1x=eqRes
$$

后 16 行是接触不等式：

$$
neqRes_{\text{low}}\le A_2x\le neqRes_{\text{upp}}
$$

目标函数中：

$$
H=
\begin{bmatrix}
2\cdot10^7I_6 & 0\\
0 & 2\cdot10^1I_{12}
\end{bmatrix}
$$

并且：

$$
g=0
$$

所以 QP 倾向于：

```text
尽量少改 base 加速度
相对更愿意调整接触 wrench
```

### 19.7 QP 解的含义

qpOASES 求出的解是：

$$
x^*=
\begin{bmatrix}
\delta\ddot q_b^*\\
\delta F^*
\end{bmatrix}
$$

于是得到修正后的加速度：

$$
\ddot q_{\text{opt}}
=
\ddot q_{\text{kin}}
+S_b^T\delta\ddot q_b^*
$$

也就是：

$$
\ddot q_{\text{opt}}
=
\begin{bmatrix}
\ddot q_{b,\text{kin}}+\delta\ddot q_b^*\\
\ddot q_{j,\text{kin}}
\end{bmatrix}
$$

注意：关节加速度部分不被 QP 修改。

修正后的接触 wrench 是：

$$
F_{\text{opt}}
=
F_{\text{ff}}+\delta F^*
$$

其中 wrench 表示：

$$
F_{\text{foot}}
=
\begin{bmatrix}
\text{3 维接触力}\\
\text{3 维接触力矩}
\end{bmatrix}
$$

双脚共 12 维。

### 19.8 反算关节力矩

完整动力学为：

$$
M\ddot q+h=J_{fe}^{T}F+S^T\tau
$$

移项：

$$
S^T\tau
=
M\ddot q+h-J_{fe}^{T}F
$$

代入 QP 后的结果：

$$
\tau_{\text{res}}
=
M\ddot q_{\text{opt}}
+h
-J_{fe}^{T}F_{\text{opt}}
$$

源码：

```cpp
tauRes = dyn_M * eigen_ddq_Opt + dyn_Non - Jfe.transpose() * eigen_fr_Opt;
```

这里的 `tauRes` 是完整广义维度：

$$
\tau_{\text{res}}\in\mathbb{R}^{n_v}
$$

前 6 维对应 floating base，不能由电机直接驱动；理想情况下，QP 已经让它接近 0。

真正的关节力矩是：

$$
\tau_{\text{joint}}
=
\tau_{\text{res}}[6:]
$$

源码：

```cpp
tauJointRes = tauRes.block(6, 0, model_nv - 6, 1);
```

数据流：

```text
tauJointRes
-> RobotState.wbc_tauJointRes
-> RobotState.motors_tor_des
-> PVT_Ctr
-> motors_tor_out
-> MuJoCo ctrl
```

### 19.9 R7 的最终一句话

R7 完成的是：

$$
\ddot q_{\text{kin}},\ F_{\text{ff}}
\rightarrow
QP(\delta\ddot q_b,\delta F)
\rightarrow
\ddot q_{\text{opt}},\ F_{\text{opt}}
\rightarrow
\tau_{\text{joint}}
$$

也就是：

```text
R6 负责“任务上想怎么动”；
R7 负责“这个动作在动力学和接触约束下如何实现”；
QP 不直接优化关节力矩；
关节力矩由修正后的 ddq_opt 和 F_opt 反算得到。
```
