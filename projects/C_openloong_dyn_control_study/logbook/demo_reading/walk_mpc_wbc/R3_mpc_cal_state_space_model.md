# R3: MPC::cal 状态空间模型与预测矩阵

## 1. 本轮目标

本轮阅读：

```text
external/open_source_repos/OpenLoong-Dyn-Control/algorithm/mpc.cpp
```

中的：

```cpp
void MPC::cal()
```

重点只覆盖状态空间模型部分：

1. `EN` 开关对 `cal()` 的影响。
2. 连续时间单刚体模型 `Ac / Bc` 如何构造。
3. 离散模型 `A / B` 如何得到。
4. `Aqp / Bqp` 如何把单步模型展开成 10 步预测。
5. 为什么状态预测是 10 步，但控制优化变量只有 3 步。

本轮不展开摩擦锥、足底力矩约束和 qpOASES 求解，它们放到后续 R4/R5。

## 2. 函数整体结构

源码结构：

```cpp
void MPC::cal()
{
    if (EN)
    {
        // qp pre
        ...
    }
    else
    {
        Ufe.setZero();
        Ufe(2) = -0.5*m*g;
        Ufe(8) = -0.5*m*g;
        Ufe(12) = m*g;
        Ufe_pre.setZero();
    }
}
```

`EN` 是 MPC 是否启用的标志：

```text
EN == true   构造并求解完整 MPC-QP
EN == false  不求 QP，只给默认站立支撑力
```

当 `EN == false`：

```cpp
Ufe(2) = -0.5*m*g;
Ufe(8) = -0.5*m*g;
Ufe(12) = m*g;
```

由于源码里：

$$
g=-9.8
$$

所以：

$$
-0.5mg>0
$$

表示左右脚竖直方向各承担一半体重：

$$
f_{Lz}=f_{Rz}=-0.5mg
$$

第 13 维：

$$
u_g=mg
$$

用于 MPC 内部竖直方向重力项，不属于 WBC 的 12 维足端 wrench。

## 3. 状态与输入定义

MPC 状态：

$$
x
=
\begin{bmatrix}
rpy\\
p\\
\omega\\
v
\end{bmatrix}
\in\mathbb{R}^{12}
$$

展开：

```text
0..2    rpy
3..5    position p
6..8    angular velocity omega
9..11   linear velocity v
```

MPC 输入：

$$
u
=
\begin{bmatrix}
f_L\\
\tau_L\\
f_R\\
\tau_R\\
u_g
\end{bmatrix}
\in\mathbb{R}^{13}
$$

展开：

```text
0..2     f_L
3..5     tau_L
6..8     f_R
9..11    tau_R
12       u_g
```

其中前 12 维是双脚接触 wrench，第 13 维是 MPC 内部的竖直重力/偏置输入。

## 4. 第 223-228 行：构造连续状态矩阵 Ac 和离散 A

源码：

```cpp
for (int i = 0; i < mpc_N; i++)
{
    Ac[i].block<3, 3>(0, 6) = R_curz[i].transpose();
    Ac[i].block<3, 3>(3, 9) = Eigen::MatrixXd::Identity(3, 3);
    A[i] = Eigen::MatrixXd::Identity(nx, nx) + dt * Ac[i];
}
```

连续模型形式：

$$
\dot{x}=A_cx+B_cu
$$

这一段只构造 `Ac`。

`Ac` 填了两个块：

1. 角速度影响姿态变化：

$$
\dot{rpy}=R_z^T\omega
$$

2. 线速度影响位置变化：

$$
\dot{p}=v
$$

块结构：

$$
A_c=
\begin{bmatrix}
0 & 0 & R_z^T & 0\\
0 & 0 & 0 & I\\
0 & 0 & 0 & 0\\
0 & 0 & 0 & 0
\end{bmatrix}
$$

每个块都是 \(3\times3\)。

设：

$$
c=\cos(yaw),
\quad
s=\sin(yaw)
$$

则：

$$
R_z^T
=
\begin{bmatrix}
c & s & 0\\
-s & c & 0\\
0 & 0 & 1
\end{bmatrix}
$$

因此 `Ac` 中非零行可以理解为：

$$
\dot{roll}=c\omega_x+s\omega_y
$$

$$
\dot{pitch}=-s\omega_x+c\omega_y
$$

$$
\dot{yaw}=\omega_z
$$

$$
\dot{x}=v_x,\quad
\dot{y}=v_y,\quad
\dot{z}=v_z
$$

离散化使用一阶欧拉：

$$
x_{k+1}=x_k+dt\dot{x}_k
$$

所以：

$$
A_i=I+dtA_{c,i}
$$

## 5. 第 229-242 行：构造连续输入矩阵 Bc 和离散 B

源码：

```cpp
for (int i = 0; i < mpc_N; i++)
{
    pf2comi[i] = pf2com;
    Eigen::Matrix3d Ic_W_inv;
    Ic_W_inv = (R_curz[i] * Ic * R_curz[i].transpose()).inverse();

    Bc[i].block<3, 3>(6, 0) = Ic_W_inv * CrossProduct_A(pf2comi[i].block<3, 1>(0, 0));
    Bc[i].block<3, 3>(6, 3) = Ic_W_inv;
    Bc[i].block<3, 3>(6, 6) = Ic_W_inv * CrossProduct_A(pf2comi[i].block<3, 1>(3, 0));
    Bc[i].block<3, 3>(6, 9) = Ic_W_inv;
    Bc[i].block<3, 3>(9, 0) = Eigen::MatrixXd::Identity(3, 3) / m;
    Bc[i].block<3, 3>(9, 6) = Eigen::MatrixXd::Identity(3, 3) / m;
    Bc[i]((nx - 1), (nu - 1)) = 1.0 / m;
    B[i] = dt * Bc[i];
}
```

`Bc` 描述输入如何影响状态导数：

$$
\dot{x}=A_cx+B_cu
$$

核心动力学是单刚体模型：

$$
I_W\dot{\omega}
=
r_L\times f_L+\tau_L+r_R\times f_R+\tau_R
$$

$$
m\dot{v}
=
f_L+f_R+u_g e_z
$$

其中：

$$
r_L=p_L-p_{CoM}
$$

$$
r_R=p_R-p_{CoM}
$$

$$
I_W=R_z I_c R_z^T
$$

源码先算：

$$
I_W^{-1}=(R_z I_c R_z^T)^{-1}
$$

然后填入 `Bc`。

### 5.1 左右脚力对角速度的影响

左脚力：

```cpp
Bc[i].block<3, 3>(6, 0) =
    Ic_W_inv * CrossProduct_A(r_L);
```

右脚力：

```cpp
Bc[i].block<3, 3>(6, 6) =
    Ic_W_inv * CrossProduct_A(r_R);
```

数学上：

$$
r\times f=[r]_\times f
$$

所以：

$$
\dot{\omega}
=
I_W^{-1}[r_L]_\times f_L
+
I_W^{-1}[r_R]_\times f_R
+\cdots
$$

### 5.2 左右脚力矩对角速度的影响

源码：

```cpp
Bc[i].block<3, 3>(6, 3) = Ic_W_inv;
Bc[i].block<3, 3>(6, 9) = Ic_W_inv;
```

对应：

$$
\dot{\omega}
=
I_W^{-1}\tau_L
+
I_W^{-1}\tau_R
+\cdots
$$

### 5.3 左右脚力对线速度的影响

源码：

```cpp
Bc[i].block<3, 3>(9, 0) = I / m;
Bc[i].block<3, 3>(9, 6) = I / m;
```

对应：

$$
\dot{v}
=
\frac{1}{m}f_L
+
\frac{1}{m}f_R
+\cdots
$$

### 5.4 第 13 维输入对竖直加速度的影响

源码：

```cpp
Bc[i](11, 12) = 1.0 / m;
```

即：

$$
\dot{v}_z
+=
\frac{1}{m}u_g
$$

若后面把：

$$
u_g=mg
$$

则竖直方向可写成：

$$
\dot{v}_z
=
\frac{1}{m}(f_{Lz}+f_{Rz}+mg)
$$

这就是重力项进入 MPC 线性模型的方式。

### 5.5 Bc 的块结构

按状态行和输入列写：

$$
B_c=
\begin{bmatrix}
0 & 0 & 0 & 0 & 0\\
0 & 0 & 0 & 0 & 0\\
I_W^{-1}[r_L]_\times & I_W^{-1} & I_W^{-1}[r_R]_\times & I_W^{-1} & 0\\
\frac{1}{m}I & 0 & \frac{1}{m}I & 0 & b_g
\end{bmatrix}
$$

其中：

$$
b_g=
\begin{bmatrix}
0\\
0\\
1/m
\end{bmatrix}
$$

离散化：

$$
B_i=dtB_{c,i}
$$

最终单步离散模型是：

$$
x_{k+1}=A_ix_k+B_iu_k
$$

## 6. 第 243-247 行：构造 Aqp

源码：

```cpp
for (int i = 0; i < mpc_N; i++)
    Aqp.block<nx, nx>(i * nx, 0) = Eigen::MatrixXd::Identity(nx, nx);
for (int i = 0; i < mpc_N; i++)
    for (int j = 0; j < i + 1; j++)
        Aqp.block<nx, nx>(i * nx, 0) = A[j] * Aqp.block<nx, nx>(i * nx, 0);
```

`Aqp` 维度：

$$
A_{qp}\in\mathbb{R}^{120\times12}
$$

它负责把当前状态传播成未来 10 步状态中“由当前状态贡献的部分”。

单步：

$$
x_1=A_0x_0+B_0u_0
$$

只看当前状态贡献：

$$
x_1\leftarrow A_0x_0
$$

两步：

$$
x_2\leftarrow A_1A_0x_0
$$

三步：

$$
x_3\leftarrow A_2A_1A_0x_0
$$

所以：

$$
A_{qp}
=
\begin{bmatrix}
A_0\\
A_1A_0\\
A_2A_1A_0\\
\vdots\\
A_9A_8\cdots A_0
\end{bmatrix}
$$

后面会出现在：

$$
A_{qp}X_{cur}-X_d
$$

表示“不考虑未来控制输入时，从当前状态自然预测到未来，与目标轨迹之间的误差”。

## 7. 第 249-267 行：构造 Bqp

源码：

```cpp
for (int i = 0; i < mpc_N; i++)
    for (int j = 0; j < i + 1; j++)
        Aqp1.block<nx, nx>(i * nx, j * nx) = Eigen::MatrixXd::Identity(nx, nx);
for (int i = 1; i < mpc_N; i++)
    for (int j = 0; j < i; j++)
        for (int k = j + 1; k < (i + 1); k++)
            Aqp1.block<nx, nx>(i * nx, j * nx) = A[k] * Aqp1.block<nx, nx>(i * nx, j * nx);

for (int i = 0; i < mpc_N; i++)
    Bqp1.block<nx, nu>(i * nx, i * nu) = B[i];
Eigen::MatrixXd Bqp11 = Eigen::MatrixXd::Zero(nu * mpc_N, nu * ch);
Bqp11.setZero();
Bqp11.block<nu * ch, nu * ch>(0, 0) = Eigen::MatrixXd::Identity(nu * ch, nu * ch);
for (int i = 0; i < (mpc_N - ch); i++)
    Bqp11.block<nu, nu>(nu * ch + i * nu, nu * (ch - 1)) = Eigen::MatrixXd::Identity(nu, nu);

Eigen::MatrixXd B_tmp = Eigen::MatrixXd::Zero(nx * mpc_N, nu * ch);
B_tmp = Bqp1 * Bqp11;
Bqp = Aqp1 * B_tmp;
```

这一段构造最终：

$$
B_{qp}\in\mathbb{R}^{120\times39}
$$

它描述未来 3 个输入如何影响未来 10 个状态。

### 7.1 Aqp1：输入影响向未来传播

`Aqp1` 是：

$$
A_{qp1}\in\mathbb{R}^{120\times120}
$$

它是下三角块矩阵。前 4 步可写为：

$$
A_{qp1}
=
\begin{bmatrix}
I & 0 & 0 & 0\\
A_1 & I & 0 & 0\\
A_2A_1 & A_2 & I & 0\\
A_3A_2A_1 & A_3A_2 & A_3 & I
\end{bmatrix}
$$

含义是：某一步输入造成的状态增量，会被后续 `A` 矩阵继续传播到更远未来。

### 7.2 Bqp1：每步输入先作用到对应状态

`Bqp1` 是：

$$
B_{qp1}\in\mathbb{R}^{120\times130}
$$

它是块对角矩阵：

$$
B_{qp1}
=
\begin{bmatrix}
B_0 & 0 & \cdots & 0\\
0 & B_1 & \cdots & 0\\
\vdots & \vdots & \ddots & \vdots\\
0 & 0 & \cdots & B_9
\end{bmatrix}
$$

如果优化 10 步输入：

$$
U_{10}
=
\begin{bmatrix}
u_0\\
u_1\\
\vdots\\
u_9
\end{bmatrix}
\in\mathbb{R}^{130}
$$

那么输入贡献可写成：

$$
A_{qp1}B_{qp1}U_{10}
$$

但本源码不直接优化 10 步输入。

### 7.3 Bqp11：把 3 步优化输入扩展为 10 步输入

源码中：

```cpp
ch = 3;
mpc_N = 10;
```

所以优化变量是：

$$
U
=
\begin{bmatrix}
u_0\\
u_1\\
u_2
\end{bmatrix}
\in\mathbb{R}^{39}
$$

`Bqp11` 的作用是把 3 步输入扩展成 10 步输入：

$$
U_{10}
=
B_{qp11}U
$$

扩展规则：

$$
U_{10}
=
\begin{bmatrix}
u_0\\
u_1\\
u_2\\
u_2\\
u_2\\
\vdots\\
u_2
\end{bmatrix}
$$

也就是：

```text
预测状态看 10 步；
控制只优化 3 步；
第 4 到第 10 步输入假设继续沿用 u2。
```

块矩阵形式：

$$
B_{qp11}
=
\begin{bmatrix}
I & 0 & 0\\
0 & I & 0\\
0 & 0 & I\\
0 & 0 & I\\
\vdots & \vdots & \vdots\\
0 & 0 & I
\end{bmatrix}
$$

每个块是 \(13\times13\)。

### 7.4 最终 Bqp

源码：

```cpp
B_tmp = Bqp1 * Bqp11;
Bqp = Aqp1 * B_tmp;
```

数学上：

$$
B_{qp}=A_{qp1}B_{qp1}B_{qp11}
$$

维度：

$$
A_{qp1}: 120\times120
$$

$$
B_{qp1}: 120\times130
$$

$$
B_{qp11}: 130\times39
$$

所以：

$$
B_{qp}: 120\times39
$$

## 8. 10 步状态预测与 3 步控制优化

这里有两个窗口：

```text
mpc_N = 10  状态预测窗口
ch = 3      控制优化窗口
```

10 步状态预测：

$$
X
=
\begin{bmatrix}
x_1\\
x_2\\
\vdots\\
x_{10}
\end{bmatrix}
\in\mathbb{R}^{120}
$$

3 步控制优化：

$$
U
=
\begin{bmatrix}
u_0\\
u_1\\
u_2
\end{bmatrix}
\in\mathbb{R}^{39}
$$

不是两个 MPC，而是同一个 MPC 里的两个窗口：

```text
10 步: 我往未来看多远；
3 步: 我这次真正优化几个未来控制输入。
```

如果优化 10 步输入，变量维度是：

$$
13\times10=130
$$

现在只优化 3 步，变量维度是：

$$
13\times3=39
$$

这样 QP 更小，更适合实时控制。

## 9. 最终预测模型

到这里，`cal()` 已经构造出：

$$
X=A_{qp}X_{cur}+B_{qp}U
$$

其中：

$$
X\in\mathbb{R}^{120}
$$

$$
X_{cur}\in\mathbb{R}^{12}
$$

$$
U\in\mathbb{R}^{39}
$$

这句话的含义是：

```text
未来 10 步身体状态
=
当前状态自然传播的结果
+
未来 3 步接触输入造成的影响
```

这 10 个状态不是传感器直接给的，而是从当前状态出发，用单刚体动力学递推预测出来的：

$$
x_{k+1}=A_kx_k+B_ku_k
$$

MPC 真正优化的变量不是状态 `X`，而是输入 `U`。状态 `X` 会随着选择的 `U` 改变。

## 10. 与下一轮的衔接

下一步 `cal()` 会构造：

```cpp
delta_U
H
c
```

把目标函数整理成 qpOASES 的标准 QP：

$$
\min_U
\frac{1}{2}U^THU+c^TU
$$

本轮到此为止，已经得到 R4/R5 所需的预测模型：

$$
X=A_{qp}X_{cur}+B_{qp}U
$$

