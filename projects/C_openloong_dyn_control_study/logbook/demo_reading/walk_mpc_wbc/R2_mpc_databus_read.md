# R2: MPC::dataBusRead 输入与状态整理

## 1. 本轮目标

本轮阅读：

```text
external/open_source_repos/OpenLoong-Dyn-Control/algorithm/mpc.cpp
```

中的：

```cpp
void MPC::dataBusRead(DataBus &Data)
```

本轮只回答：

1. MPC 当前状态 `X_cur` 是如何从 `DataBus` 整理出来的？
2. 期望轨迹 `Xd` 从哪里来，如何滚动更新？
3. 足端、base/CoM、惯量、支撑腿状态这些几何量如何准备？
4. `R_w2f` 为什么要算，它后面服务于什么约束？

这一轮不进入 `MPC::cal()` 的 QP 求解细节，只把 MPC 的输入准备讲清楚。

## 2. 函数入口与整体作用

源码：

```cpp
void MPC::dataBusRead(DataBus &Data)
{
    // set value
    ...
}
```

`DataBus` 是整个 demo 的共享状态总线。`MPC::dataBusRead()` 的作用是把外部机器人状态整理成 MPC 内部需要的变量：

```text
DataBus
-> X_cur
-> Xd
-> R_cur / R_curz
-> pCoM / pe / pf2com
-> Ic
-> legState[]
-> R_f2w / R_w2f
```

后面的 `MPC::cal()` 会使用这些量构造单刚体预测模型和 QP。

## 3. 第 135-138 行：构造当前状态 X_cur

源码：

```cpp
X_cur.block<3, 1>(0, 0) = Data.base_rpy;
X_cur.block<3, 1>(3, 0) = Data.q.block<3, 1>(0, 0);
X_cur.block<3, 1>(6, 0) = Data.dq.block<3, 1>(3, 0);
X_cur.block<3, 1>(9, 0) = Data.dq.block<3, 1>(0, 0);
```

MPC 的状态是 12 维：

$$
X_{cur}
=
\begin{bmatrix}
rpy\\
p\\
\omega\\
v
\end{bmatrix}
\in \mathbb{R}^{12}
$$

展开为：

$$
X_{cur}
=
\begin{bmatrix}
roll\\
pitch\\
yaw\\
x\\
y\\
z\\
\omega_x\\
\omega_y\\
\omega_z\\
v_x\\
v_y\\
v_z
\end{bmatrix}
$$

对应关系：

```text
X_cur[0:2]   = Data.base_rpy
X_cur[3:5]   = Data.q[0:2]   = base position
X_cur[6:8]   = Data.dq[3:5]  = base angular velocity
X_cur[9:11]  = Data.dq[0:2]  = base linear velocity
```

这里做的是接口转换。`DataBus` 里的 `q/dq` 更接近浮动基机器人广义坐标：

```text
q  = [base position, base quaternion, joint positions]
dq = [base linear velocity, base angular velocity, joint velocities]
```

而 MPC 只关心单刚体 base 状态：

```text
X_cur = [rpy, position, angular velocity, linear velocity]
```

所以这几行把浮动基状态重排成 MPC 的 12 维状态。

## 4. 第 139-152 行：MPC 启用时滚动更新 Xd

源码：

```cpp
if (EN)
{
    for (int i = 0; i < (mpc_N - 1); i++)
        Xd.block<nx, 1>(nx * i, 0) = Xd.block<nx, 1>(nx * (i + 1), 0);

    for (int j = 0; j < 3; j++)
        Xd(nx * (mpc_N - 1) + j) = Data.js_eul_des(j);
    for (int j = 0; j < 3; j++)
        Xd(nx * (mpc_N - 1) + 3 + j) = Data.js_pos_des(j);
    for (int j = 0; j < 3; j++)
        Xd(nx * (mpc_N - 1) + 6 + j) = Data.js_omega_des(j);
    for (int j = 0; j < 3; j++)
        Xd(nx * (mpc_N - 1) + 9 + j) = Data.js_vel_des(j);
}
```

`Xd` 是 MPC 的 10 步期望状态窗口：

$$
X_d
=
\begin{bmatrix}
x_{d,0}\\
x_{d,1}\\
\vdots\\
x_{d,9}
\end{bmatrix}
\in \mathbb{R}^{120}
$$

每个期望状态：

$$
x_d
=
\begin{bmatrix}
rpy_d\\
p_d\\
\omega_d\\
v_d
\end{bmatrix}
\in \mathbb{R}^{12}
$$

第一个循环做滚动窗口：

```text
Xd[0] <- Xd[1]
Xd[1] <- Xd[2]
...
Xd[8] <- Xd[9]
```

然后最后一格写入最新上层目标：

```text
Xd[108:110] = Data.js_eul_des
Xd[111:113] = Data.js_pos_des
Xd[114:116] = Data.js_omega_des
Xd[117:119] = Data.js_vel_des
```

这些 `Data.js_*_des` 主要由 `JoyStickInterpreter` 和上层命令写入 `RobotState`。链路是：

```text
JoyStickInterpreter / 上层目标
-> DataBus.js_*_des
-> MPC::dataBusRead()
-> Xd
```

所以 `Xd` 不是 MPC 自己凭空生成的轨迹，而是从上层目标维护出来的 10 步参考窗口。

## 5. 第 153-174 行：MPC 未启用时 Xd 等于当前状态

源码：

```cpp
else
{
    for (int i = 0; i < mpc_N; i++)
    {
        for (int j = 0; j < 3; j++)
            Xd(nx * i + j) = X_cur(j);
        for (int j = 0; j < 3; j++)
            Xd(nx * i + 3 + j) = X_cur(3 + j);
        for (int j = 0; j < 3; j++)
            Xd(nx * i + 6 + j) = X_cur(6 + j);
        for (int j = 0; j < 3; j++)
            Xd(nx * i + 9 + j) = X_cur(9 + j);
    }
}
```

当 `EN == false`，MPC 没启用。此时源码把 10 个期望状态都设成当前状态：

$$
x_{d,i}=X_{cur},
\quad i=0,\dots,9
$$

也就是：

$$
X_d
=
\begin{bmatrix}
X_{cur}\\
X_{cur}\\
\vdots\\
X_{cur}
\end{bmatrix}
$$

这样做的意义是：未启用 MPC 时，不让参考轨迹突然跳到 joystick 目标，避免一打开 MPC 就产生很大的状态误差。

## 6. 第 176-180 行：当前姿态矩阵 R_cur 与 yaw 矩阵 R_curz

源码：

```cpp
R_cur = eul2Rot(X_cur(0), X_cur(1), X_cur(2)); // Data.base_rot;
for (int i = 0; i < mpc_N; i++)
{
    R_curz[i] = Rz3(X_cur(2));
}
```

`R_cur` 是完整姿态旋转矩阵：

$$
R_{cur}=R(roll,pitch,yaw)
$$

`R_curz[i]` 只取 yaw：

$$
R_{curz}[i]=R_z(yaw)
$$

并且当前源码中 10 个预测步都使用同一个当前 yaw。

这样做是单刚体 MPC 的简化：水平行走中主要用 yaw 描述机器人朝向，roll/pitch 不进入这些简化旋转块。

后面 `R_curz` 会用于：

1. `set_weight()` 中旋转状态/输入权重。
2. `cal()` 中构造姿态动力学：

$$
\dot{rpy}=R_z^T\omega
$$

3. `cal()` 中把惯量转到世界/航向系：

$$
I_W=R_z I_c R_z^T
$$

## 7. 第 181-188 行：脚端几何量 pCoM / pe / pf2com

源码：

```cpp
pCoM = X_cur.block<3, 1>(3, 0);
pe.block<3, 1>(0, 0) = Data.fe_l_pos_W;
pe.block<3, 1>(3, 0) = Data.fe_r_pos_W;

pf2com.block<3, 1>(0, 0) = pe.block<3, 1>(0, 0) - pCoM;
pf2com.block<3, 1>(3, 0) = pe.block<3, 1>(3, 0) - pCoM;
pf2comd.block<3, 1>(0, 0) = pe.block<3, 1>(0, 0) - Xd.block<3, 1>(3, 0);
pf2comd.block<3, 1>(3, 0) = pe.block<3, 1>(3, 0) - Xd.block<3, 1>(3, 0);
```

这里的 `pCoM` 名字叫 CoM，但源码实际取的是 base 位置：

$$
p_{CoM}\approx p_{base}=X_{cur}[3:5]
$$

`pe` 是左右脚端世界系位置拼起来的 6 维向量：

$$
p_e
=
\begin{bmatrix}
p_L\\
p_R
\end{bmatrix}
=
\begin{bmatrix}
p_{Lx}\\
p_{Ly}\\
p_{Lz}\\
p_{Rx}\\
p_{Ry}\\
p_{Rz}
\end{bmatrix}
$$

其中：

```text
p_L = Data.fe_l_pos_W
p_R = Data.fe_r_pos_W
```

`pf2com` 实际计算的是脚相对 base/CoM 的向量：

$$
r_L=p_L-p_{CoM}
$$

$$
r_R=p_R-p_{CoM}
$$

拼成：

$$
pf2com
=
\begin{bmatrix}
r_L\\
r_R
\end{bmatrix}
$$

这个向量后面进入角动量方程：

$$
I\dot{\omega}
=
r_L\times f_L+\tau_L+r_R\times f_R+\tau_R
$$

`pf2comd` 使用的是脚相对期望位置：

$$
pf2comd
=
\begin{bmatrix}
p_L-p_d\\
p_R-p_d
\end{bmatrix}
$$

但当前 `mpc.cpp` 后面基本没有使用 `pf2comd`，可理解为预留变量或历史遗留变量。

## 8. 第 190-191 行：固定单刚体惯量 Ic

源码：

```cpp
// Ic = Data.inertia;
Ic << 12.61, 0, 0.37, 0, 11.15, 0.01, 0.37, 0.01, 2.15;
```

惯量矩阵：

$$
I_c
=
\begin{bmatrix}
12.61 & 0 & 0.37\\
0 & 11.15 & 0.01\\
0.37 & 0.01 & 2.15
\end{bmatrix}
$$

理论上可以从：

```cpp
Data.inertia
```

读取全身当前惯量，但源码选择固定惯量。这是单刚体 MPC 的常见工程近似：

```text
真实全身惯量随姿态和关节变化；
MPC 为了实时性和稳定性，使用固定整体惯量近似。
```

后面会用：

$$
I_W^{-1}=(R_z I_c R_z^T)^{-1}
$$

计算脚底力/力矩对角速度变化的影响。

## 9. 第 193-205 行：预测未来支撑腿状态 legState

源码：

```cpp
legStateCur = Data.legState;
legStateNext = Data.legStateNext;
for (int i = 0; i < mpc_N; i++)
{
    double aa;
    aa = i * dt / 0.4;
    double phip;
    phip = Data.phi + aa;
    if (phip > 1)
        legState[i] = legStateNext;
    else
        legState[i] = legStateCur;
}
```

`Data.legState` 是当前支撑状态，可能是：

```text
LSt: left stance
RSt: right stance
DSt: double/default stance
```

`Data.legStateNext` 是下一支撑状态。

`Data.phi` 是当前步态相位：

$$
\phi\in[0,1]
$$

表示当前这一步已经走到多少比例：

```text
phi = 0    当前这一步刚开始
phi = 0.5  当前这一步走到一半
phi = 1    当前这一步结束，准备换脚
```

源码预测第 `i` 个 MPC 点时的相位：

$$
\phi_i
=
\phi+\frac{i\cdot dt}{0.4}
$$

这里 `0.4` 是假定一步周期 0.4 秒。若：

$$
\phi_i>1
$$

说明预测到该时刻时当前这一步已经结束，于是：

$$
legState[i]=legStateNext
$$

否则：

$$
legState[i]=legStateCur
$$

所以这一块不是计算总共走了多少步，而是在判断：

```text
当前这一脚在未来预测窗口内会不会结束；
如果结束，未来对应时刻使用下一支撑脚状态。
```

这个数组后面会影响：

1. 哪只脚能出力。
2. 哪只脚的 wrench 必须为 0。
3. 默认重力支撑力分配给哪只脚。

## 10. 第 207-215 行：选择接触坐标系 R_f2w / R_w2f

源码：

```cpp
Eigen::Matrix<double, 3, 3> R_slop;
R_slop = eul2Rot(Data.slop(0), Data.slop(1), Data.slop(2));
if (legStateCur == DataBus::RSt)
    R_f2w = Data.fe_r_rot_W;
else if (legStateCur == DataBus::LSt)
    R_f2w = Data.fe_l_rot_W;
else
    R_f2w = R_slop;
R_w2f = R_f2w.transpose();
```

这块用于选择当前接触坐标系。

如果当前右脚支撑：

$$
R_{f2w}=R_{rightfoot}^{world}
$$

如果当前左脚支撑：

$$
R_{f2w}=R_{leftfoot}^{world}
$$

否则使用坡面/地形姿态：

$$
R_{f2w}=R_{slop}
$$

再取转置：

$$
R_{w2f}=R_{f2w}^T
$$

它后面用于把世界系接触力转到足底坐标系：

$$
f_F=R_{w2f}f_W
$$

这样摩擦锥约束才能写成足底坐标系下的：

$$
|f_x|\le\mu f_z
$$

$$
|f_y|\le\mu f_z
$$

如果脚底有姿态或地面有坡度，接触法向不一定等于世界 z 轴，所以必须有这个坐标系转换。

## 11. 本轮变量去向总结

`dataBusRead()` 准备的变量后面主要这样用：

```text
X_cur
  -> Aqp * X_cur
  -> 状态预测起点

Xd
  -> Aqp * X_cur - Xd
  -> QP 目标函数里的状态误差

R_curz
  -> Ac 姿态动力学
  -> Ic_W_inv
  -> L/K 权重旋转

pCoM + pe
  -> pf2com
  -> CrossProduct_A(r)
  -> Bc

Ic
  -> Ic_W_inv
  -> Bc

legState[]
  -> delta_U
  -> 输入上下界 u_low/u_up
  -> 支撑脚接触约束

R_f2w / R_w2f
  -> 摩擦锥约束
  -> 足底力矩约束
```

一句话总结：

```text
MPC::dataBusRead()
不是求解 MPC，
而是把 DataBus 里的机器人状态整理成 cal() 所需的状态、目标、几何和接触信息。
```

