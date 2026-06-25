# R7: walk_wbc 与 walk_mpc_wbc 的完整对比和面试总结

## 1. 本轮目标

这一轮不再细抠单个函数，而是把两条完整链路放在一起对比：

```text
walk_wbc
walk_mpc_wbc
```

重点回答三个问题：

1. 两个 demo 的控制主线哪里相同，哪里不同？
2. MPC 在 `walk_mpc_wbc` 里到底插到了哪一层？
3. 面试时如何用一句话讲清楚这个项目？

## 2. 先给一句话结论

`walk_wbc` 是一个典型的 **WBC + PVT 行走闭环**；

`walk_mpc_wbc` 是在它前面插了一层 **MPC 接触力前馈与 base 预测层**。

可以把两者记成：

$$
walk\_wbc:
\quad
参考生成 \rightarrow WBC \rightarrow PVT \rightarrow 电机
$$

$$
walk\_mpc\_wbc:
\quad
参考生成 \rightarrow MPC \rightarrow WBC \rightarrow PVT \rightarrow 电机
$$

所以 `walk_mpc_wbc` 不是重写控制框架，而是在原有 `walk_wbc` 的基础上把“接触力规划”往前提了一层。

## 3. 两条主线的总对比

| 维度 | walk_wbc | walk_mpc_wbc |
|---|---|---|
| 核心思想 | WBC 直接跟踪步态和 base 目标 | MPC 先算接触力前馈，再交给 WBC |
| 参考来源 | JoyStickInterpreter + GaitScheduler + FootPlacement | 同样有这三层，但 MPC 进一步吸收状态、步态和目标 |
| 预测层 | 没有显式 QP 预测层 | 有 10 步状态预测 + 3 步控制优化 |
| WBC 输入 | 直接读 base/脚步/任务目标 | 额外读 `Fr_ff`、`Xd`、`X_cal`、`dX_cal` |
| 输出到电机 | PVT 生成最终 torque | 同样经 WBC + PVT 输出 torque |
| 控制风格 | 更像“任务驱动的全身控制” | 更像“预测控制 + 全身控制” |
| 调试重点 | 任务层和 WBC 约束 | MPC 预测、QP、约束、前馈接口 |

## 4. 共同主线

两者都保留了这条底层闭环：

```text
MuJoCo
-> MJ_Interface
-> DataBus
-> Pin_KinDyn
-> gait / foot placement
-> WBC
-> PVT
-> MuJoCo
```

所以两者共享很多底座：

$$
q,\ dq,\ J,\ dJ,\ M,\ h,\ p_{CoM}
$$

也都需要：

$$
floating-base
$$

$$
WBC
$$

$$
PVT
$$

真正的区别不是“换了一套机器人控制器”，而是 **在同一个 WBC 闭环前面加入了 MPC 外环**。

## 5. walk_wbc 的职责边界

`walk_wbc` 的特点是：

```text
高层目标 -> 任务生成 -> WBC -> PVT
```

它的逻辑更直接：

1. `JoyStickInterpreter` 把摇杆速度变成 base 目标。
2. `GaitScheduler` 决定哪只脚支撑、哪只脚摆动。
3. `FootPlacement` 生成摆脚目标。
4. `WBC_priority` 把这些目标变成 `ddq / tau`。
5. `PVT_Ctr` 把关节层结果发给仿真。

它更像是：

```text
“我要往哪走”
-> “我给你一个摆脚和 base 目标”
-> “WBC 负责让身体真的这样动起来”
```

## 6. walk_mpc_wbc 的职责边界

`walk_mpc_wbc` 在原来的任务层和 WBC 之间插入了 MPC。

它的逻辑变成：

```text
高层目标 / 步态状态
-> MPC 预测与优化
-> WBC
-> PVT
```

MPC 读入的关键量是：

$$
X_{cur},\ X_d,\ legState,\ p_{foot},\ p_{CoM}
$$

MPC 输出的关键量是：

$$
Ufe,\ Fr_{ff},\ X_{cal},\ dX_{cal},\ base\_pos_{des},\ base\_rpy_{des}
$$

这意味着 `walk_mpc_wbc` 比 `walk_wbc` 多了一个问题：

```text
不是只问“当前怎么控”，而是先问“未来几步最合理的接触力是什么”。
```

## 7. 数据流差异

### 7.1 walk_wbc

```text
Joystick / gait / foot placement
-> WBC
-> PVT
-> actuator
```

### 7.2 walk_mpc_wbc

```text
Joystick / gait / foot placement
-> MPC
-> Fr_ff / X_cal / base_des
-> WBC
-> PVT
-> actuator
```

从变量上看，`walk_mpc_wbc` 多了这些 MPC 专属量：

$$
X_{cur},\ X_d,\ X_{cal},\ dX_{cal},\ Ufe,\ Fr_{ff},\ qpStatus
$$

而 `walk_wbc` 主要围绕：

$$
base\_pos_{des},\ base\_rpy_{des},\ swing\_fe\_pos_{des},\ legState
$$

## 8. 两者最本质的差别

### 8.1 预测 vs 非预测

`walk_wbc`：

```text
任务生成后，直接交给 WBC
```

`walk_mpc_wbc`：

```text
先在预测窗口里优化接触力
再把结果交给 WBC
```

### 8.2 力的来源

`walk_wbc` 的接触力更多来自 WBC 的动力学平衡和任务约束；

`walk_mpc_wbc` 先由 MPC 计算一个前馈接触 wrench：

$$
Fr_{ff}
$$

然后 WBC 在它基础上继续做修正：

$$
F_{WBC}
=
F_{MPC}

\Delta F
$$

### 8.3 控制风格

`walk_wbc` 更偏向：

```text
任务优先级 + 动力学约束
```

`walk_mpc_wbc` 更偏向：

```text
预测优化 + 任务优先级 + 动力学约束
```

## 9. 关键接口变量

`walk_mpc_wbc` 最关键的接口是：

$$
Fr_{ff} = Ufe[0:12]
$$

它进入 WBC 后会出现在：

$$
J_{fe}^T Fr_{ff}
$$

也就是 floating-base 动力学平衡里。

所以如果面试官问你：

```text
MPC 输出到底去哪了？
```

你应该回答：

```text
MPC 输出的是双脚 wrench 前馈 Fr_ff，
它被 WBC 读入后，作为动力学平衡项参与全身 QP，
最后再被 PVT 转成电机力矩。
```

## 10. 为什么 walk_mpc_wbc 更“完整”

因为它把整个 walking 闭环拆成了三层：

```text
1. 高层参考与步态层
2. MPC 外环预测层
3. WBC / PVT 执行层
```

这样更接近真实机器人系统的工程结构：

$$
\text{高层意图}
\rightarrow
\text{接触规划}
\rightarrow
\text{全身动力学执行}
\rightarrow
\text{关节/电机输出}
$$

## 11. 面试时怎么说

### 11.1 30 秒版本

我系统阅读了 OpenLoong 的 `walk_wbc` 和 `walk_mpc_wbc` 两套行走 demo。`walk_wbc` 是标准的任务优先级 WBC 闭环，`walk_mpc_wbc` 在 WBC 前面增加了一个基于单刚体动力学的 MPC 外环，用来预测并优化双脚接触 wrench，再把 `Fr_ff` 送入 WBC 做全身动力学控制。

### 11.2 2 分钟版本

`walk_wbc` 主要由状态估计、Pinocchio 运动学动力学、步态生成、摆脚规划、WBC 和 PVT 构成。它的特点是任务生成后直接交给 WBC 跟踪。`walk_mpc_wbc` 则在任务层和 WBC 之间增加了 MPC：MPC 用 10 步状态预测、3 步控制优化，输出双脚接触力前馈 `Fr_ff` 和 base 预测量，再交给 WBC 进行全身任务和动力学求解。这样做的优势是接触力更有前瞻性，WBC 的输入也更接近物理可行的力分配。

### 11.3 面试官追问时的关键词

```text
floating-base
state estimation
Pinocchio Jacobian / dynamics
gait scheduler
foot placement
MPC 3-step control / 10-step prediction
Fr_ff
WBC QP
PVT torque output
```

## 12. 常见追问回答

### 12.1 为什么要加 MPC？

因为 `walk_wbc` 的任务生成是局部的，更多依赖当前步态和当前任务约束；MPC 能提前看未来几步，先规划接触力和 base 趋势，减少 WBC 的“临场救火”压力。

### 12.2 为什么 MPC 不直接替代 WBC？

因为 MPC 用的是简化的单刚体模型，只适合规划和接触力前馈；WBC 才负责完整 floating-base 动力学、关节约束、冗余任务和精细控制。

### 12.3 为什么还要 PVT？

因为 WBC 输出的是关节层参考和力矩解，而真实电机或仿真接口通常还需要更稳定的伺服整形、限幅和前馈叠加，这一层由 PVT 完成。

### 12.4 如果只说一个区别，是什么？

就是：

$$
walk\_wbc:
\text{WBC 直接接任务}
$$

$$
walk\_mpc\_wbc:
\text{MPC 先给 WBC 一个更合理的接触力前馈}
$$

## 13. 最后的项目总结

如果把这两个 demo 合在一起看，它们其实展示了一个很完整的人形机器人行走控制栈：

```text
状态估计
-> 步态与落脚点
-> MPC 预测优化
-> WBC 全身动力学
-> PVT 关节执行
```

这也是我在面试里最适合讲的主线。

一句话收尾：

$$
\boxed{
walk\_wbc\ \text{解决“怎么走”}
\quad
walk\_mpc\_wbc\ \text{解决“先算好怎么走得更稳”}
}
$$

