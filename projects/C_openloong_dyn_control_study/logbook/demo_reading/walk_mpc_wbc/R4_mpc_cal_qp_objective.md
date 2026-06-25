# R4: MPC::cal 目标函数与 H/c 推导

## 1. 本轮目标

本轮继续阅读：

```text
external/open_source_repos/OpenLoong-Dyn-Control/algorithm/mpc.cpp
```

中的：

```cpp
void MPC::cal()
```

重点覆盖第 269-285 行：

```cpp
Eigen::Matrix<double, nu * ch, 1> delta_U;
delta_U.setZero();
for (int i = 0; i < ch; i++)
{
    if (legState[i] == DataBus::LSt)
        delta_U(nu * i + 2) = m * g;
    else if (legState[i] == DataBus::RSt)
        delta_U(nu * i + 8) = m * g;
    else
    {
        delta_U(nu * i + 2) = 0.5 * m * g;
        delta_U(nu * i + 8) = 0.5 * m * g;
    }
}

H = 2 * (Bqp.transpose() * L * Bqp + alpha * K)
    + 1e-10 * Eigen::MatrixXd::Identity(nx * mpc_N, nx * mpc_N);
c = 2 * Bqp.transpose() * L * (Aqp * X_cur - Xd)
    + 2 * alpha * K * delta_U;
```

这一段做的事是：把 MPC 的“跟踪目标 + 控制代价”整理成 qpOASES 需要的标准 QP 形式：

$$
\min_U \frac{1}{2}U^T H U + c^T U
$$

## 2. 接上上一轮：预测模型已经有了

上一轮 R3 已经得到 10 步预测模型：

$$
X=A_{qp}X_{cur}+B_{qp}U
$$

其中：

$$
X\in\mathbb{R}^{120}
$$

因为：

$$
mpc\_N=10,\quad nx=12
$$

所以：

$$
120=10\times12
$$

优化变量：

$$
U\in\mathbb{R}^{39}
$$

因为：

$$
ch=3,\quad nu=13
$$

所以：

$$
39=3\times13
$$

也就是：

$$
U=
\begin{bmatrix}
u_0\\
u_1\\
u_2
\end{bmatrix}
$$

每个控制步：

$$
u_k=
\begin{bmatrix}
f_L\\
\tau_L\\
f_R\\
\tau_R\\
u_g
\end{bmatrix}
\in\mathbb{R}^{13}
$$

前 12 维是双脚 wrench，第 13 维是 MPC 内部竖直动力学输入。

## 3. MPC 想优化什么

MPC 的目标可以理解成两件事：

```text
1. 未来状态 X 尽量接近期望轨迹 Xd
2. 控制输入 U 不要太大，并且围绕合理的重力支撑初值
```

所以原始代价可以写成：

$$
J(U)
=
(X-X_d)^T L (X-X_d)
+\alpha (U+\Delta U)^T K (U+\Delta U)
$$

其中：

$$
L\in\mathbb{R}^{120\times120}
$$

是状态误差权重矩阵。

$$
K\in\mathbb{R}^{39\times39}
$$

是输入权重矩阵。

$$
\alpha
$$

是输入代价整体权重，源码里由 `u_weight` 传入：

```cpp
alpha = u_weight;
```

## 4. delta_U 是什么

源码：

```cpp
Eigen::Matrix<double, nu * ch, 1> delta_U;
delta_U.setZero();
```

定义：

$$
\Delta U\in\mathbb{R}^{39}
$$

它不是最终控制输出，而是输入代价里的偏置项。

代价写的是：

$$
(U+\Delta U)^TK(U+\Delta U)
$$

如果只看这个输入代价，最小值会倾向于：

$$
U=-\Delta U
$$

因为源码里：

$$
g=-9.8
$$

所以：

$$
mg<0
$$

当：

```cpp
delta_U(nu * i + 2) = 0.5 * m * g;
```

时，优化会倾向于：

$$
U(nu\cdot i+2)\approx -0.5mg
$$

而：

$$
-0.5mg>0
$$

这正好表示左脚竖直方向向上的半体重支撑力。

## 5. 不同步态下 delta_U 怎么填

### 5.1 LSt：左脚支撑

源码：

```cpp
if (legState[i] == DataBus::LSt)
    delta_U(nu * i + 2) = m * g;
```

第 `nu*i+2` 维是第 `i` 个控制步的左脚竖直力：

$$
f_{Lz}
$$

因为输入代价倾向于：

$$
U=-\Delta U
$$

所以：

$$
f_{Lz}\approx -mg
$$

也就是左脚承担全部体重。

### 5.2 RSt：右脚支撑

源码：

```cpp
else if (legState[i] == DataBus::RSt)
    delta_U(nu * i + 8) = m * g;
```

第 `nu*i+8` 维是右脚竖直力：

$$
f_{Rz}
$$

所以：

$$
f_{Rz}\approx -mg
$$

右脚承担全部体重。

### 5.3 DSt：双脚支撑

源码：

```cpp
else
{
    delta_U(nu * i + 2) = 0.5 * m * g;
    delta_U(nu * i + 8) = 0.5 * m * g;
}
```

输入代价倾向于：

$$
f_{Lz}\approx -0.5mg
$$

$$
f_{Rz}\approx -0.5mg
$$

即左右脚各承担一半体重。

## 6. 把状态误差代价展开

预测模型：

$$
X=A_{qp}X_{cur}+B_{qp}U
$$

定义无控制时的预测误差：

$$
e_0=A_{qp}X_{cur}-X_d
$$

则：

$$
X-X_d=e_0+B_{qp}U
$$

状态代价：

$$
J_x=(X-X_d)^TL(X-X_d)
$$

代入：

$$
J_x=(e_0+B_{qp}U)^TL(e_0+B_{qp}U)
$$

展开：

$$
J_x
=
U^TB_{qp}^TLB_{qp}U
+2U^TB_{qp}^TLe_0
+e_0^TLe_0
$$

其中最后一项：

$$
e_0^TLe_0
$$

和优化变量 \(U\) 无关，所以对求解 QP 没影响。

## 7. 把输入代价展开

输入代价：

$$
J_u=\alpha(U+\Delta U)^TK(U+\Delta U)
$$

展开：

$$
J_u
=
\alpha U^TKU
+2\alpha U^TK\Delta U
+\alpha\Delta U^TK\Delta U
$$

其中：

$$
\alpha\Delta U^TK\Delta U
$$

和 \(U\) 无关，也可以忽略。

## 8. 合并成 QP 标准形式

保留和 \(U\) 有关的项：

$$
J(U)
=
U^T(B_{qp}^TLB_{qp}+\alpha K)U
+2U^TB_{qp}^TL(A_{qp}X_{cur}-X_d)
+2\alpha U^TK\Delta U
+const
$$

qpOASES 使用的标准形式是：

$$
\min_U \frac{1}{2}U^THU+c^TU
$$

所以取：

$$
H=2(B_{qp}^TLB_{qp}+\alpha K)
$$

$$
c=2B_{qp}^TL(A_{qp}X_{cur}-X_d)+2\alpha K\Delta U
$$

对应源码：

```cpp
H = 2 * (Bqp.transpose() * L * Bqp + alpha * K) + ...
c = 2 * Bqp.transpose() * L * (Aqp * X_cur - Xd)
    + 2 * alpha * K * delta_U;
```

## 9. H 和 c 的维度

先看：

$$
B_{qp}\in\mathbb{R}^{120\times39}
$$

$$
L\in\mathbb{R}^{120\times120}
$$

所以：

$$
B_{qp}^TLB_{qp}
\in
\mathbb{R}^{39\times39}
$$

又因为：

$$
K\in\mathbb{R}^{39\times39}
$$

所以：

$$
H\in\mathbb{R}^{39\times39}
$$

再看：

$$
A_{qp}X_{cur}-X_d\in\mathbb{R}^{120}
$$

所以：

$$
B_{qp}^TL(A_{qp}X_{cur}-X_d)
\in
\mathbb{R}^{39}
$$

并且：

$$
K\Delta U\in\mathbb{R}^{39}
$$

所以：

$$
c\in\mathbb{R}^{39}
$$

## 10. 源码里 1e-10 Identity 的注意点

源码写的是：

```cpp
H = 2 * (Bqp.transpose() * L * Bqp + alpha * K)
    + 1e-10 * Eigen::MatrixXd::Identity(nx * mpc_N, nx * mpc_N);
```

从数学维度看，`H` 是：

$$
39\times39
$$

因此正则项通常也应该是：

$$
I_{39}
$$

也就是：

```cpp
Eigen::MatrixXd::Identity(nu * ch, nu * ch)
```

源码这里写成：

$$
I_{120}
$$

因为：

$$
nx\cdot mpc\_N=120
$$

从严格维度上看是不匹配的。学习时先记住：这行的意图是给 `H` 加一个极小正则项，让 QP 数值更稳定；但维度写法需要小心检查实际编译版本和 Eigen 表达式行为。

## 11. 这一段的最终结果

第 269-285 行最终得到两个 QP 目标函数对象：

$$
H\in\mathbb{R}^{39\times39}
$$

$$
c\in\mathbb{R}^{39}
$$

它们定义了优化目标：

$$
\boxed{
\min_U \frac{1}{2}U^THU+c^TU
}
$$

其中：

```text
H: 二次项，描述控制 U 对未来状态误差和输入大小的影响
c: 一次项，描述当前状态离目标轨迹的偏差，以及重力支撑偏置
```

下一轮 R5 接着看：

```text
As, u_low/u_up, lbA/ubA, qpOASES, Ufe, X_cal
```

