# R5: MPC::cal 约束矩阵、qpOASES 求解与 X_cal

## 1. 本轮目标

本轮继续阅读：

```text
external/open_source_repos/OpenLoong-Dyn-Control/algorithm/mpc.cpp
```

中的：

```cpp
void MPC::cal()
```

重点覆盖第 287-515 行：

```text
Asfr     摩擦锥约束
Astxy    足底 x/y 力矩约束
Astz     足底 z/yaw 力矩约束
As       总约束矩阵
u_low/u_up
lbA/ubA
qpOASES 求解
Ufe
dX_cal / delta_X / X_cal
EN == false 默认输出
```

这一轮的主线是：目标函数 `H/c` 已经有了，现在要给 QP 加物理约束，并调用 qpOASES 求出最优接触力序列。

## 2. QP 总公式

到 R4 为止，目标函数已经整理成：

$$
\min_U \frac{1}{2}U^THU+c^TU
$$

本轮加入约束以后，完整 QP 是：

$$
\boxed{
\begin{aligned}
\min_U \quad & \frac{1}{2}U^THU+c^TU\\
\text{s.t.}\quad
& u_{low}\le U\le u_{up}\\
& lbA\le AsU\le ubA
\end{aligned}
}
$$

其中：

$$
U\in\mathbb{R}^{39}
$$

因为：

$$
U=
\begin{bmatrix}
u_0\\
u_1\\
u_2
\end{bmatrix},
\quad
u_k\in\mathbb{R}^{13}
$$

而：

$$
As\in\mathbb{R}^{96\times39}
$$

因为每个控制步有：

$$
nc=32
$$

行约束，控制窗口：

$$
ch=3
$$

所以：

$$
96=32\times3
$$

## 3. 约束行数来源

`mpc.h` 中常量：

```cpp
const uint16_t ncfr_single = 4;
const uint16_t ncfr = ncfr_single * 2;

const uint16_t ncstxya = 1;
const uint16_t ncstxy_single = ncstxya * 4;
const uint16_t ncstxy = ncstxy_single * 2;

const uint16_t ncstza = 2;
const uint16_t ncstz_single = ncstza * 4;
const uint16_t ncstz = ncstz_single * 2;
const uint16_t nc = ncfr + ncstxy + ncstz;
```

逐项看：

$$
ncfr\_single=4
$$

单脚摩擦锥 4 行。

$$
ncfr=8
$$

双脚摩擦锥 8 行。

$$
ncstxy\_single=4
$$

单脚 x/y 力矩约束 4 行。

$$
ncstxy=8
$$

双脚 x/y 力矩约束 8 行。

$$
ncstz\_single=8
$$

单脚 z/yaw 力矩约束 8 行。

$$
ncstz=16
$$

双脚 z/yaw 力矩约束 16 行。

所以单个控制步总约束行数：

$$
nc=8+8+16=32
$$

三步控制窗口总行数：

$$
nc\cdot ch=32\times3=96
$$

## 4. Asfr：摩擦锥约束

源码：

```cpp
Eigen::Matrix<double, ncfr_single, 3> Asfr111, Asfr11;
Eigen::Matrix<double, ncfr, nu> Asfr1;
Eigen::Matrix<double, ncfr * ch, nu * ch> Asfr;
...
Asfr111 << -1.0, 0.0, -1.0 / sqrt(2.0) * miu,
     1.0, 0.0, -1.0 / sqrt(2.0) * miu,
     0.0, -1.0, -1.0 / sqrt(2.0) * miu,
     0.0, 1.0, -1.0 / sqrt(2.0) * miu;
Asfr11 = Asfr111 * R_w2f;
Asfr1.block<ncfr_single, 3>(0, 0) = Asfr11;
Asfr1.block<ncfr_single, 3>(ncfr_single, 6) = Asfr11;
for (int i = 0; i < ch; i++)
    Asfr.block<ncfr, nu>(ncfr * i, i * nu) = Asfr1;
```

`Asfr111` 是单脚 4 行摩擦锥：

$$
Asfr111\in\mathbb{R}^{4\times3}
$$

它作用在脚底坐标系下的接触力：

$$
f_F=
\begin{bmatrix}
f_x\\f_y\\f_z
\end{bmatrix}
$$

约束形式：

$$
Asfr111 f_F\le0
$$

展开近似为：

$$
|f_x|\le\frac{\mu}{\sqrt{2}}f_z
$$

$$
|f_y|\le\frac{\mu}{\sqrt{2}}f_z
$$

这是把圆锥摩擦锥近似成四棱锥。

为什么要乘：

```cpp
Asfr11 = Asfr111 * R_w2f;
```

因为优化变量里的接触力通常按世界系表达，而摩擦锥最好在脚底坐标系判断。脚底坐标系的 z 轴就是接触法向，x/y 是切向，所以：

$$
f_F=R_{w2f}f_W
$$

因此：

$$
Asfr111R_{w2f}f_W\le0
$$

即：

$$
Asfr11=Asfr111R_{w2f}
$$

`Asfr1` 是单个控制步的双脚摩擦约束：

$$
Asfr1\in\mathbb{R}^{8\times13}
$$

排列为：

$$
\begin{bmatrix}
\text{left friction 4 rows}\\
\text{right friction 4 rows}
\end{bmatrix}
$$

`Asfr` 是 3 个控制步的摩擦约束：

$$
Asfr\in\mathbb{R}^{24\times39}
$$

## 5. Astxy：足底 x/y 力矩约束

源码主要结构：

```cpp
Eigen::Matrix<double, ncstxya, 6> Astxy_r[4];
Eigen::Matrix<double, ncstxy_single, 6> Astxy11;
Eigen::Matrix<double, ncstxy, nu> Astxy1;
Eigen::Matrix<double, ncstxy * ch, nu * ch> Astxy;
```

维度是：

$$
Astxy\_r[i]\in\mathbb{R}^{1\times6}
$$

$$
Astxy11\in\mathbb{R}^{4\times6}
$$

$$
Astxy1\in\mathbb{R}^{8\times13}
$$

$$
Astxy\in\mathbb{R}^{24\times39}
$$

它约束的是单脚 wrench：

$$
w=
\begin{bmatrix}
f\\
\tau
\end{bmatrix}
\in\mathbb{R}^6
$$

物理含义是：限制足底 x/y 方向力矩，使等效压力中心 CoP 不要跑出脚底支撑区域。

简单说，脚掌能提供的力矩不是无限大。接触力的作用点如果跑到脚掌外，脚就会翻转或打滑。

所以 `Astxy` 可以理解为：

```text
足底支撑区域 / CoP 约束
```

每只脚 4 行，对应脚底前后左右 4 个边界。

## 6. Astz：足底 z/yaw 力矩约束

源码：

```cpp
Eigen::Matrix<double, ncstza, 6> Astz_r[4];
Eigen::Matrix<double, ncstz_single, 6> Astz11;
Eigen::Matrix<double, ncstz, nu> Astz1;
Eigen::Matrix<double, ncstz * ch, nu * ch> Astz;
```

维度：

$$
Astz\_r[i]\in\mathbb{R}^{2\times6}
$$

$$
Astz11\in\mathbb{R}^{8\times6}
$$

$$
Astz1\in\mathbb{R}^{16\times13}
$$

$$
Astz\in\mathbb{R}^{48\times39}
$$

核心源码：

```cpp
for (int i = 0; i < 4; i++)
{
    Astz_r[i].block<1, 3>(0, 0) =
        -sqrt(p[i](0) * p[i](0) + p[i](1) * p[i](1) + p[i](2) * p[i](2))
        * miu * Eigen::Matrix<double, 1, 3>(0.0, 0.0, 1.0) * R_w2f;
    Astz_r[i].block<1, 3>(0, 3) =
        Eigen::Matrix<double, 1, 3>(0.0, 0.0, 1.0) * R_w2f;
    Astz_r[i].block<1, 3>(1, 0) = Astz_r[i].block<1, 3>(0, 0);
    Astz_r[i].block<1, 3>(1, 3) = -1 * Astz_r[i].block<1, 3>(0, 3);
    Astz11.block<ncstza, 6>(i * ncstza, 0) = Astz_r[i];
}
```

每个足底边界点 `p[i]` 生成 2 行：

$$
\begin{bmatrix}
-\mu r_i n^T & n^T\\
-\mu r_i n^T & -n^T
\end{bmatrix}
$$

其中：

$$
n^T=[0,0,1]R_{w2f}
$$

$$
r_i=\|p_i\|
$$

物理含义近似为：

$$
|\tau_z|\le\mu r_i f_z
$$

也就是：脚底绕法向的 yaw 扭矩不能无限大，它受摩擦系数、接触法向力、脚底有效半径限制。

如果没有这个约束，MPC 可能会算出一个物理上不现实的“脚底强行拧地”的 yaw 力矩。

## 7. As：总约束矩阵

源码：

```cpp
As.block<ncfr * ch, nu * ch>(0, 0) = Asfr;
As.block<ncstxy * ch, nu * ch>(ncfr * ch, 0) = Astxy;
As.block<ncstz * ch, nu * ch>(ncfr * ch + ncstxy * ch, 0) = Astz;
```

所以：

$$
As=
\begin{bmatrix}
Asfr\\
Astxy\\
Astz
\end{bmatrix}
$$

维度：

$$
Asfr\in\mathbb{R}^{24\times39}
$$

$$
Astxy\in\mathbb{R}^{24\times39}
$$

$$
Astz\in\mathbb{R}^{48\times39}
$$

所以：

$$
As\in\mathbb{R}^{96\times39}
$$

约束形式：

$$
lbA\le AsU\le ubA
$$

## 8. Guess_value 与 u_low/u_up

源码从第 383 行开始：

```cpp
Eigen::Matrix<double, nu * ch, 1> Guess_value;
Guess_value.setZero();
for (int i = 0; i < ch; i++)
{
    ...
}
```

`Guess_value` 是 qpOASES 的初始猜测：

$$
Guess\_value\in\mathbb{R}^{39}
$$

`u_low/u_up` 是优化变量的上下界：

$$
u_{low}\le U\le u_{up}
$$

### 8.1 DSt：双脚支撑

源码：

```cpp
Guess_value(i * nu + 2) = -0.5 * m * g;
Guess_value(i * nu + 8) = -0.5 * m * g;
Guess_value(i * nu + 12) = m * g;
```

含义：

$$
f_{Lz}=f_{Rz}=-0.5mg
$$

左右脚各承担一半体重。

并且左右脚 6D wrench 都允许在 `min/max` 范围内变化。

### 8.2 LSt：左脚支撑

源码：

```cpp
Guess_value(i * nu + 2) = -m * g;
Guess_value(i * nu + 8) = 0.0;
Guess_value(i * nu + 12) = m * g;
```

含义：

$$
f_{Lz}=-mg
$$

$$
f_{Rz}=0
$$

左脚承担全部体重，右脚摆动。

右脚 wrench 被固定为 0：

```cpp
u_low(i * nu + j + nu / 2) = 0.0;
u_up(i * nu + j + nu / 2) = 0.0;
```

注意这里：

$$
nu/2=6
$$

所以 `j+6` 正好对应右脚 wrench 的 6 维。

### 8.3 RSt：右脚支撑

源码：

```cpp
Guess_value(i * nu + 2) = 0.0;
Guess_value(i * nu + 8) = -m * g;
Guess_value(i * nu + 12) = m * g;
```

含义：

$$
f_{Lz}=0
$$

$$
f_{Rz}=-mg
$$

右脚承担全部体重，左脚摆动。

左脚 wrench 被固定为 0。

### 8.4 第 13 维被固定

三种状态中都有：

```cpp
u_low(i * nu + 12) = m * g;
u_up(i * nu + 12) = m * g;
```

所以第 13 维不是自由优化，而是被固定为：

$$
u_g=mg
$$

它通过：

```cpp
Bc[i](11, 12) = 1.0 / m;
```

作用到竖直速度导数：

$$
\dot v_z+=\frac{1}{m}u_g
$$

## 9. lbA/ubA：根据步态打开约束行

源码：

```cpp
Eigen::Matrix<double, nc * ch, 1> lbA, ubA, one_ch_1;
one_ch_1.setOnes();
lbA = -1e7 * one_ch_1;
ubA = 1e7 * one_ch_1;
```

默认：

$$
lbA=-10^7
$$

$$
ubA=10^7
$$

也就是默认约束基本不生效。

后面通过：

```cpp
ubA.block<...>(...).setZero();
```

把某些行的上界设为 0，使它们变成真正的不等式：

$$
AsU\le0
$$

### 9.1 DSt：双脚支撑

源码：

```cpp
ubA.block<ncfr, 1>(ncfr * i, 0).setZero();
ubA.block<ncstxy, 1>(ncfr * ch + ncstxy * i, 0).setZero();
ubA.block<ncstz, 1>(ncfr * ch + ncstxy * ch + ncstz * i, 0).setZero();
```

打开第 `i` 个控制步的双脚约束：

$$
8+8+16=32
$$

也就是该步所有接触约束都生效。

### 9.2 LSt：左脚支撑

源码：

```cpp
ubA.block<ncfr_single, 1>(ncfr * i, 0).setZero();
ubA.block<ncstxy_single, 1>(ncfr * ch + ncstxy * i, 0).setZero();
ubA.block<ncstz_single, 1>(ncfr * ch + ncstxy * ch + ncstz * i, 0).setZero();
```

打开左脚约束：

$$
4+4+8=16
$$

右脚是摆动脚，右脚 wrench 已经由 `u_low/u_up` 固定为 0，所以右脚约束不需要打开。

### 9.3 RSt：右脚支撑

源码：

```cpp
ubA.block<ncfr_single, 1>(ncfr * i + ncfr_single, 0).setZero();
ubA.block<ncstxy_single, 1>(ncfr * ch + ncstxy * i + ncstxy_single, 0).setZero();
ubA.block<ncstz_single, 1>(ncfr * ch + ncstxy * ch + ncstz * i + ncstz_single, 0).setZero();
```

打开右脚约束：

$$
4+4+8=16
$$

这里多出来的：

$$
ncfr\_single,\quad ncstxy\_single,\quad ncstz\_single
$$

都是为了跳过左脚约束行，定位到右脚约束行。

## 10. Eigen 矩阵转 qpOASES 普通数组

源码：

```cpp
copy_Eigen_to_real_t(qp_H, H, nu * ch, nu * ch);
copy_Eigen_to_real_t(qp_c, c, nu * ch, 1);
copy_Eigen_to_real_t(qp_As, As, nc * ch, nu * ch);
copy_Eigen_to_real_t(qp_lbA, lbA, nc * ch, 1);
copy_Eigen_to_real_t(qp_ubA, ubA, nc * ch, 1);
copy_Eigen_to_real_t(qp_lu, u_low, nu * ch, 1);
copy_Eigen_to_real_t(qp_uu, u_up, nu * ch, 1);
copy_Eigen_to_real_t(xOpt_iniGuess, Guess_value, nu * ch, 1);
```

Eigen 矩阵是“懂线性代数”的对象，可以直接写：

```cpp
Bqp.transpose() * L * Bqp
```

普通数组只是连续内存里的数字，qpOASES 的接口需要这种 C 风格数组：

```cpp
qpOASES::real_t*
```

所以这里的复制只是数据格式转换，不改变数学含义。

## 11. qpOASES 求解

源码：

```cpp
res = QP.init(qp_H, qp_c, qp_As,
              qp_lu, qp_uu,
              qp_lbA, qp_ubA,
              nWSR, &cpu_time,
              xOpt_iniGuess);
```

它求解的就是：

$$
\boxed{
\begin{aligned}
\min_U \quad & \frac{1}{2}U^THU+c^TU\\
\text{s.t.}\quad
& u_{low}\le U\le u_{up}\\
& lbA\le AsU\le ubA
\end{aligned}
}
$$

求解结果状态：

```cpp
qp_Status = qpOASES::getSimpleStatus(res);
qp_nWSR = nWSR;
qp_cpuTime = cpu_time;
```

如果失败：

```cpp
printf("failed!!!!!!!!!!!!!\n");
```

这份源码没有复杂 fallback，只打印失败信息。

## 12. Ufe：最优控制序列

源码：

```cpp
qpOASES::real_t xOpt[nu * ch];
QP.getPrimalSolution(xOpt);
if (qp_Status == 0)
{
    for (int i = 0; i < nu * ch; i++)
        Ufe(i) = xOpt[i];
}
```

`xOpt` 是 qpOASES 求出的最优解：

$$
xOpt=U^*
$$

写回 Eigen 变量：

$$
Ufe=U^*
$$

也就是：

$$
Ufe=
\begin{bmatrix}
u_0^*\\
u_1^*\\
u_2^*
\end{bmatrix}
\in\mathbb{R}^{39}
$$

当前控制周期真正使用的是：

$$
u_0^*
$$

这就是 MPC 的滚动优化思想：预测未来多步，只执行第一步。

## 13. dX_cal：用 u0 计算当前状态导数

源码：

```cpp
dX_cal = Ac[0] * X_cur + Bc[0] * Ufe.block<nu, 1>(0, 0);
```

这里取的是：

$$
Ufe.block<nu,1>(0,0)=u_0^*
$$

所以：

$$
\dot X_{cal}
=
A_cX_{cur}+B_cu_0^*
$$

状态顺序：

$$
X=
\begin{bmatrix}
rpy\\
p\\
\omega\\
v
\end{bmatrix}
$$

所以：

$$
dX_{cal}[6:8]=\dot\omega
$$

$$
dX_{cal}[9:11]=\dot v
$$

## 14. delta_X：二阶积分修正

源码：

```cpp
Eigen::Matrix<double, nx, 1> delta_X;
delta_X.setZero();
for (int i = 0; i < 3; i++)
{
    delta_X(i) = 0.5 * dX_cal(i + 6) * dt * dt;
    delta_X(i + 3) = 0.5 * dX_cal(i + 9) * dt * dt;
    delta_X(i + 6) = dX_cal(i + 6) * dt;
    delta_X(i + 9) = dX_cal(i + 9) * dt;
}
```

假设一个很小时间步 `dt` 内角加速度和线加速度近似常数：

$$
\dot\omega=\alpha
$$

$$
\dot v=a
$$

则匀加速积分为：

$$
\omega_{k+1}=\omega_k+\alpha dt
$$

$$
v_{k+1}=v_k+a dt
$$

$$
rpy_{k+1}=rpy_k+\omega_kdt+\frac{1}{2}\alpha dt^2
$$

$$
p_{k+1}=p_k+v_kdt+\frac{1}{2}a dt^2
$$

源码里的 `Aqp * X_cur + Bqp * Ufe` 已经包含一阶离散预测，`delta_X` 额外补入：

$$
\Delta X=
\begin{bmatrix}
\frac{1}{2}\dot\omega dt^2\\
\frac{1}{2}\dot v dt^2\\
\dot\omega dt\\
\dot v dt
\end{bmatrix}
$$

对应代码：

```text
delta_X(0..2)   = 0.5 * dot_omega * dt^2
delta_X(3..5)   = 0.5 * dot_v     * dt^2
delta_X(6..8)   = dot_omega * dt
delta_X(9..11)  = dot_v     * dt
```

注意：从严格离散系统角度看，速度层的增量通常已经通过 `B` 进入预测模型；这里再加一次属于源码作者额外做的显式积分修正。学习时可以记成：

```text
标准线性离散预测 + 基于当前 u0 的匀加速补偿
```

## 15. X_cal：取 10 步预测里的第一步状态

源码：

```cpp
X_cal = (Aqp * X_cur + Bqp * Ufe).block<nx, 1>(nx * 0, 0) + delta_X;
```

先算：

$$
X_{pred}=A_{qp}X_{cur}+B_{qp}Ufe
$$

其中：

$$
X_{pred}\in\mathbb{R}^{120}
$$

它包含：

$$
X_{pred}=
\begin{bmatrix}
x_1\\
x_2\\
\vdots\\
x_{10}
\end{bmatrix}
$$

每个：

$$
x_i\in\mathbb{R}^{12}
$$

源码：

```cpp
.block<nx, 1>(nx * 0, 0)
```

等价于：

```cpp
.block<12, 1>(0, 0)
```

也就是取第一个未来状态：

$$
x_1
$$

所以：

$$
X_{cal}=x_1+\Delta X
$$

即：

$$
\boxed{
X_{cal}
=
(A_{qp}X_{cur}+B_{qp}Ufe)_{0:12}
+
\Delta X
}
$$

## 16. 三步控制和十步状态的关系

这里容易混：

```text
Ufe 是 3 步控制输出
Xpred 是 10 步状态预测
```

控制：

$$
Ufe=
\begin{bmatrix}
u_0\\
u_1\\
u_2
\end{bmatrix}
\in\mathbb{R}^{39}
$$

状态：

$$
X_{pred}=
\begin{bmatrix}
x_1\\
x_2\\
\vdots\\
x_{10}
\end{bmatrix}
\in\mathbb{R}^{120}
$$

源码通过 `Bqp11` 把 3 步控制扩展成 10 步使用：

$$
u_0,u_1,u_2,u_2,u_2,\dots,u_2
$$

所以可以预测 10 个状态，但优化变量仍然只有 39 维。

当前周期实际执行：

$$
u_{\text{apply}}=u_0^*
$$

当前周期用于记录/观察的下一步状态：

$$
X_{cal}=x_1+\Delta X
$$

## 17. Ufe_pre 与 QP.reset

源码：

```cpp
Ufe_pre = Ufe.block<nu, 1>(0, 0);
QP.reset();
```

保存当前第一步控制：

$$
Ufe\_pre=u_0^*
$$

然后重置 qpOASES。

这份源码每次 `cal()` 都使用：

```cpp
QP.init(...)
```

而不是 `hotstart(...)`。所以实现简单，但没有充分利用相邻 MPC 周期之间 QP 很相似这个特点。

## 18. EN == false 时的默认输出

源码：

```cpp
else{
    Ufe.setZero();
    Ufe(2) = -0.5*m*g;
    Ufe(8) = -0.5*m*g;
    Ufe(12) = m*g;
    Ufe_pre.setZero();
}
```

当 MPC 不启用时，不求 QP，直接给第 0 个控制步默认支撑力。

因为：

$$
g=-9.8
$$

所以：

$$
f_{Lz}=Ufe(2)=-0.5mg>0
$$

$$
f_{Rz}=Ufe(8)=-0.5mg>0
$$

左右脚各承担一半体重。

第 13 维：

$$
u_g=Ufe(12)=mg
$$

竖直方向动力学近似：

$$
\dot v_z=
\frac{1}{m}(f_{Lz}+f_{Rz}+u_g)
$$

代入：

$$
\dot v_z
=
\frac{1}{m}(-0.5mg-0.5mg+mg)=0
$$

所以默认输出的目的就是让机器人不因为没有 MPC 解而自由下落。

## 19. 本轮总结

这一轮把 `MPC::cal()` 的后半部分读完了。

最终 QP：

$$
\boxed{
\begin{aligned}
\min_U \quad & \frac{1}{2}U^THU+c^TU\\
\text{s.t.}\quad
& u_{low}\le U\le u_{up}\\
& lbA\le AsU\le ubA
\end{aligned}
}
$$

优化变量：

$$
U=Ufe\in\mathbb{R}^{39}
$$

其中：

$$
Ufe=
\begin{bmatrix}
u_0^*\\
u_1^*\\
u_2^*
\end{bmatrix}
$$

接触约束：

$$
As=
\begin{bmatrix}
Asfr\\
Astxy\\
Astz
\end{bmatrix}
$$

当前周期真正执行：

$$
u_0^*
$$

下一轮最自然读：

```cpp
void MPC::dataBusWrite(DataBus &Data)
```

因为它会把：

```cpp
Data.Fr_ff = Ufe.block<12, 1>(0, 0);
```

写给 WBC。也就是把 MPC 的最优接触 wrench 前馈真正接到 WBC。

