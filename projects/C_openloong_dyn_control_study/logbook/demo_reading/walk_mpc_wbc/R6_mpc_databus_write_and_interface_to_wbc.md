# R6: MPC::dataBusWrite 与 MPC 到 WBC 的接口

## 1. 本轮目标

本轮阅读：

```text
external/open_source_repos/OpenLoong-Dyn-Control/algorithm/mpc.cpp
```

中的：

```cpp
void MPC::dataBusWrite(DataBus &Data)
```

重点看 MPC 求解结果如何写回 `DataBus`，尤其是：

```cpp
Data.Fr_ff = Ufe.block<12, 1>(0, 0);
```

这行是 MPC 和 WBC 真正接起来的接口。

前面几轮已经得到：

$$
Ufe=
\begin{bmatrix}
u_0^*\\
u_1^*\\
u_2^*
\end{bmatrix}
\in\mathbb{R}^{39}
$$

其中每个控制步：

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

`dataBusWrite()` 做的事就是把这些 MPC 内部量写到 `RobotState/DataBus`，让后面的 WBC、logger、调试代码可以读取。

## 2. 源码总览

源码：

```cpp
void MPC::dataBusWrite(DataBus &Data)
{
    Data.Xd = Xd;
    Data.X_cur = X_cur;
    Data.fe_react_tau_cmd = Ufe;
    Data.X_cal = X_cal;
    Data.dX_cal = dX_cal;

    Data.qp_nWSR_MPC = nWSR;
    Data.qp_cpuTime_MPC = cpu_time;
    Data.qpStatus_MPC = qp_Status;

    Data.Fr_ff = Ufe.block<12, 1>(0, 0);

    double k = 5;
    Data.des_ddq.block<2, 1>(0, 0) << dX_cal(9), dX_cal(10);

    Data.des_ddq(5) = k * (Xd(6 + 2) - Data.dq(5));

    Data.des_dq.block<3, 1>(0, 0) << Xd(9 + 0), Xd(9 + 1), Xd(9 + 2);
    Data.des_dq.block<2, 1>(3, 0) << 0.0, 0.0;
    Data.des_dq(5) = Xd(6 + 2);

    Data.des_delta_q.block<2, 1>(0, 0) = Data.des_dq.block<2, 1>(0, 0) * dt;
    Data.des_delta_q(5) = Data.des_dq(5) * dt;

    Data.base_rpy_des << 0.005, 0.00, Xd(2);
    Data.base_pos_des << Xd(3 + 0), Xd(3 + 1), Xd(3 + 2);
}
```

可以分成三类输出：

```text
1. MPC 调试/记录量
2. 给 WBC 的接触力前馈 Fr_ff
3. 给 WBC 的 base 运动目标
```

## 3. 写出 MPC 调试和记录量

源码：

```cpp
Data.Xd = Xd;
Data.X_cur = X_cur;
Data.fe_react_tau_cmd = Ufe;
Data.X_cal = X_cal;
Data.dX_cal = dX_cal;
```

### 3.1 Data.Xd

`Xd` 是 MPC 的未来 10 步期望状态：

$$
X_d=
\begin{bmatrix}
x_{d,1}\\
x_{d,2}\\
\vdots\\
x_{d,10}
\end{bmatrix}
\in\mathbb{R}^{120}
$$

每个状态：

$$
x_d=
\begin{bmatrix}
rpy_d\\
p_d\\
\omega_d\\
v_d
\end{bmatrix}
\in\mathbb{R}^{12}
$$

所以：

```cpp
Data.Xd = Xd;
```

主要用于记录、调试或外部查看 MPC 当时跟踪的目标。

### 3.2 Data.X_cur

`X_cur` 是 MPC 当前使用的状态：

$$
X_{cur}=
\begin{bmatrix}
rpy\\
p\\
\omega\\
v
\end{bmatrix}
\in\mathbb{R}^{12}
$$

所以：

```cpp
Data.X_cur = X_cur;
```

写出的是 MPC 当前这次计算所认为的机器人状态。

### 3.3 Data.fe_react_tau_cmd

源码：

```cpp
Data.fe_react_tau_cmd = Ufe;
```

这里写出完整的 MPC 控制序列：

$$
Ufe=
\begin{bmatrix}
u_0^*\\
u_1^*\\
u_2^*
\end{bmatrix}
\in\mathbb{R}^{39}
$$

名字 `fe_react_tau_cmd` 容易让人以为只是某一步足端反力，但这里实际写进去的是完整 39 维 `Ufe`。

在 `walk_mpc_wbc` 里，WBC 主要使用后面的 `Fr_ff`。`fe_react_tau_cmd` 更像是完整 MPC 解的记录量，也可能被其他 demo 使用。

### 3.4 Data.X_cal

源码：

```cpp
Data.X_cal = X_cal;
```

`X_cal` 是 MPC 预测出来的下一步状态估计：

$$
X_{cal}
=
(A_{qp}X_{cur}+B_{qp}Ufe)_{0:12}
+
\Delta X
$$

也就是从 10 步预测状态里取第一步：

$$
x_1
$$

再加源码额外构造的二阶积分修正。

### 3.5 Data.dX_cal

源码：

```cpp
Data.dX_cal = dX_cal;
```

`dX_cal` 是根据当前状态和当前第一步最优输入算出的连续状态导数：

$$
dX_{cal}
=
A_cX_{cur}+B_cu_0^*
$$

状态顺序是：

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

其中 \(dX_{cal}(9), dX_{cal}(10)\) 后面会被写成 base 水平方向期望加速度。

## 4. 写出 QP 求解状态

源码：

```cpp
Data.qp_nWSR_MPC = nWSR;
Data.qp_cpuTime_MPC = cpu_time;
Data.qpStatus_MPC = qp_Status;
```

这三个量用于观察 qpOASES 求解情况：

```text
qp_nWSR_MPC     工作集迭代相关计数
qp_cpuTime_MPC  求解耗时
qpStatus_MPC    求解状态
```

前面 `cal()` 中：

```cpp
qp_Status = qpOASES::getSimpleStatus(res);
```

并且只有：

```cpp
if (qp_Status == 0)
```

时才把 `xOpt` 写回 `Ufe`。

所以学习时可以先粗略记：

$$
qp\_Status=0
$$

表示 QP 成功。

## 5. 最关键接口：Fr_ff

源码：

```cpp
Data.Fr_ff = Ufe.block<12, 1>(0, 0);
```

这是本轮最重要的一行。

MPC 输出：

$$
Ufe=
\begin{bmatrix}
u_0^*\\
u_1^*\\
u_2^*
\end{bmatrix}
\in\mathbb{R}^{39}
$$

第 0 个控制步：

$$
u_0^*=
\begin{bmatrix}
f_L\\
\tau_L\\
f_R\\
\tau_R\\
u_g
\end{bmatrix}
\in\mathbb{R}^{13}
$$

其中前 12 维是双脚 6D wrench：

$$
\begin{bmatrix}
f_L\\
\tau_L\\
f_R\\
\tau_R
\end{bmatrix}
\in\mathbb{R}^{12}
$$

第 13 维：

$$
u_g
$$

是 MPC 内部竖直动力学输入，不是足端 wrench。

所以：

```cpp
Ufe.block<12, 1>(0, 0)
```

表示：

$$
Fr_{ff}=u_0^*[0:12]
$$

也就是：

$$
Fr_{ff}=
\begin{bmatrix}
f_L\\
\tau_L\\
f_R\\
\tau_R
\end{bmatrix}
\in\mathbb{R}^{12}
$$

这里同时体现两个 MPC 思想：

```text
1. 只取 u0，因为滚动优化每次只执行第一步。
2. 只取前 12 维，因为 WBC 需要的是双脚 wrench，不需要第 13 维 ug。
```

## 6. WBC 如何使用 Fr_ff

在 `wbc_priority.cpp` 中可以看到：

```cpp
Fr_ff = robotState.Fr_ff;
```

然后进入动力学等式项：

```cpp
eqRes = -Sf * dyn_M * ddq_final_kin
        - Sf * dyn_Non
        + Sf * Jfe.transpose() * Fr_ff;
```

这说明 MPC 输出的双脚 wrench 前馈，会通过：

$$
J_{fe}^TFr_{ff}
$$

变成广义力贡献，进入 WBC 的动力学约束。

后面还有：

```cpp
eigen_fr_Opt = Fr_ff + eigen_xOpt.block<12, 1>(6, 0);
```

所以 WBC 不是简单照抄 MPC 的接触力，而是在 MPC 前馈基础上优化一个修正量：

$$
F_{WBC}=F_{MPC}+\Delta F_{WBC}
$$

直观关系：

```text
MPC:  根据单刚体模型，算一个合理的双脚接触力前馈
WBC:  在全身动力学和任务约束下，对这个力做修正并求关节输出
```

## 7. 写 des_ddq：期望加速度

源码：

```cpp
double k = 5;
Data.des_ddq.block<2, 1>(0, 0) << dX_cal(9), dX_cal(10);
```

因为：

$$
dX_{cal}[9:11]=\dot v
$$

所以：

$$
dX_{cal}(9)=\dot v_x
$$

$$
dX_{cal}(10)=\dot v_y
$$

写入：

$$
des\_ddq[0:1]=
\begin{bmatrix}
\dot v_x\\
\dot v_y
\end{bmatrix}
$$

也就是把 MPC 算出的水平加速度作为 base x/y 方向的期望加速度。

## 8. 写 yaw 期望加速度

源码：

```cpp
Data.des_ddq(5) = k * (Xd(6 + 2) - Data.dq(5));
```

索引：

$$
Xd(6+2)=Xd(8)
$$

MPC 状态：

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
Xd(8)=\omega_{z,d}
$$

`Data.dq(5)` 对应浮动基 yaw 角速度。

因此：

$$
des\_ddq(5)
=
k(\omega_{z,d}-\omega_z)
$$

其中：

$$
k=5
$$

这相当于一个简单的 yaw 角速度 P 反馈，把当前 yaw 角速度拉向期望 yaw 角速度。

## 9. 写 des_dq：期望速度

源码：

```cpp
Data.des_dq.block<3, 1>(0, 0) << Xd(9 + 0), Xd(9 + 1), Xd(9 + 2);
Data.des_dq.block<2, 1>(3, 0) << 0.0, 0.0;
Data.des_dq(5) = Xd(6 + 2);
```

因为：

$$
Xd[9:11]=v_d
$$

所以：

$$
des\_dq[0:2]=
\begin{bmatrix}
v_{x,d}\\
v_{y,d}\\
v_{z,d}
\end{bmatrix}
$$

接着：

$$
des\_dq[3:4]=
\begin{bmatrix}
0\\
0
\end{bmatrix}
$$

表示 roll/pitch 角速度期望为 0。

最后：

$$
des\_dq[5]=Xd(8)=\omega_{z,d}
$$

所以 base 部分可写成：

$$
des\_dq=
\begin{bmatrix}
v_{x,d}\\
v_{y,d}\\
v_{z,d}\\
0\\
0\\
\omega_{z,d}
\end{bmatrix}
$$

## 10. 写 des_delta_q：期望位移增量

源码：

```cpp
Data.des_delta_q.block<2, 1>(0, 0) =
    Data.des_dq.block<2, 1>(0, 0) * dt;
Data.des_delta_q(5) = Data.des_dq(5) * dt;
```

这是把速度乘以时间步，得到小的位移增量：

$$
\Delta q_{des}=dq_{des}\cdot dt
$$

但源码只写了：

$$
x,\ y,\ yaw
$$

即：

$$
\Delta x=v_xdt
$$

$$
\Delta y=v_ydt
$$

$$
\Delta yaw=\omega_zdt
$$

没有写 z、roll、pitch 的增量。

## 11. 写 base_rpy_des 与 base_pos_des

源码：

```cpp
Data.base_rpy_des << 0.005, 0.00, Xd(2);
Data.base_pos_des << Xd(3 + 0), Xd(3 + 1), Xd(3 + 2);
```

`Xd(2)` 是期望 yaw：

$$
Xd(2)=yaw_d
$$

所以：

$$
base\_rpy\_des=
\begin{bmatrix}
0.005\\
0\\
yaw_d
\end{bmatrix}
$$

这里 roll 期望不是 0，而是写死为：

$$
roll_d=0.005
$$

这是一个工程经验偏置。可能用于行走时姿态补偿，也可能是实验调出来的数值。

位置期望：

$$
base\_pos\_des=
\begin{bmatrix}
Xd(3)\\
Xd(4)\\
Xd(5)
\end{bmatrix}
=
\begin{bmatrix}
x_d\\
y_d\\
z_d
\end{bmatrix}
$$

这会被 WBC 的 base task 使用。

## 12. dataBusWrite 的整体作用

`dataBusWrite()` 是 MPC 输出到系统总线的接口层。

可以概括为：

```text
MPC 内部结果
    ↓
DataBus / RobotState
    ↓
WBC / logger / debug
```

三类输出：

```text
1. 调试和记录：
   Xd, X_cur, fe_react_tau_cmd, X_cal, dX_cal, qp status

2. 接触力前馈：
   Fr_ff = Ufe.block<12,1>(0,0)

3. base 运动目标：
   des_ddq, des_dq, des_delta_q, base_rpy_des, base_pos_des
```

最重要链路：

$$
Ufe\in\mathbb{R}^{39}
\Rightarrow
u_0^*\in\mathbb{R}^{13}
\Rightarrow
Fr_{ff}=u_0^*[0:12]\in\mathbb{R}^{12}
$$

也就是：

```text
MPC 预测未来 3 步接触输入；
当前周期只使用第 0 步；
WBC 只需要其中双脚 wrench 的前 12 维。
```

## 13. enable / disable / get_ENA

源码：

```cpp
void MPC::enable()
{
    EN = true;
}
void MPC::disable()
{
    EN = false;
}

bool MPC::get_ENA()
{
    return EN;
}
```

`EN` 是 MPC 是否启用的开关。

在 `cal()` 中：

```cpp
if (EN)
{
    // 构造并求解 MPC-QP
}
else
{
    // 不求 QP，给默认双脚支撑力
}
```

所以：

```text
enable()   开启 MPC QP 求解
disable()  关闭 MPC QP，使用默认支撑力
get_ENA()  查询当前 MPC 是否启用
```

## 14. copy_Eigen_to_real_t

源码：

```cpp
void MPC::copy_Eigen_to_real_t(qpOASES::real_t *target,
                               Eigen::MatrixXd source,
                               int nRows,
                               int nCols)
{
    int count = 0;

    // Strange Behavior: Eigen matrix matrix(count) is stored by columns (not rows)
    // real_t is stored by rows, same to C array
    for (int i = 0; i < nRows; i++)
    {
        for (int j = 0; j < nCols; j++)
        {
            target[count] = source(i, j);
            count++;
        }
    }
}
```

这个函数把 Eigen 矩阵复制成 qpOASES 接收的普通数组。

Eigen 矩阵适合写数学：

```cpp
Bqp.transpose() * L * Bqp
```

qpOASES 接口需要 C 风格连续数组：

```cpp
qpOASES::real_t*
```

所以这里做的是数据格式转换：

$$
\text{Eigen matrix}
\Rightarrow
\text{real\_t array}
$$

不改变矩阵的数学含义。

## 15. R6 总结

这一轮读完后，MPC 模块的闭环就完整了：

```text
dataBusRead()
    从 DataBus 读取当前状态、目标、脚位置、接触状态

cal()
    构造单刚体 MPC-QP，并求出 Ufe

dataBusWrite()
    把 Ufe、Fr_ff、base 目标和调试量写回 DataBus
```

其中 MPC 到 WBC 的核心接口是：

$$
\boxed{
Data.Fr\_ff = Ufe.block<12,1>(0,0)
}
$$

后续进入 WBC 时，要重点追踪：

```cpp
Fr_ff = robotState.Fr_ff;
```

以及：

```cpp
Jfe.transpose() * Fr_ff
```

这就是 MPC 接触力前馈进入全身动力学 QP 的位置。

