# Roadmap: 奔跑 / 边走边跳 / 跨栏 扩展分析

## 0. 本文目标

在 OpenLoong-Dyn-Control 现有代码基础上，评估新增以下三个动作的可行性、缺口和推进顺序：

```text
奔跑 (running)
边走边跳 / 边跑边跳 (walk+jump / run+jump)
跨栏 (hurdle)
```

原则：先写从源码确认的事实，再写扩展判断。

---

## 1. OpenLoong 现有能力边界（源码确认）

以下均来自实际读码，不是印象。

### 1.1 MPC —— `algorithm/mpc.h`

```text
模型:     单刚体 (SRBM)
状态 nx:  12  (pos / rpy / vel / omega)
输入 nu:  13  (双足各 6 维接触力 + 1 个重力项)
预测:     mpc_N = 10
控制:     ch = 3
求解器:   qpOASES
约束:     摩擦锥 (ncfr) + 支撑面 xy 约束 (ncstxy) + 支撑面 z 约束 (ncstz)
```

判断：SRBM 内核是通用的，奔跑/跳跃都能复用，真正要改的是"接触约束在飞行相如何归零"和"参考轨迹如何生成"。

### 1.2 GaitScheduler —— `algorithm/gait_scheduler.h`

```text
摆动时间:  tSwing = 0.4 (固定)
触地判断:  Fz 阈值 = 100 N
腿态:      双腿交替，始终至少一腿支撑
接口:      start / stop / step，无速度自适应的步频/相位调整
```

判断：这是"行走假设"最强的模块。奔跑必须打破"始终至少一腿支撑"这条隐含前提。

### 1.3 FootPlacement —— `algorithm/foot_placement.h`

```text
落脚策略:  Raibert (kp_vx / kp_vy / kp_wz)
抬腿高度:  stepHeight = 0.1 (固定)
速度跟踪:  支持，配合 joystick_interpreter
落点约束:  无（只跟速度，不能指定落在哪）
```

判断：能跟速度指令，但不能接受"落点位置约束"，跨栏时这是硬缺口。

### 1.4 JumpScheduler —— `algorithm/jump_scheduler.h`

```text
状态机:   Prep / PushOff / PreJump / FlightUp / FlightDown / Landing
触发:     时间触发 (startJumpingTime = 8.5)
形态:     原地单次跳
飞行相:   腿部 IK + pitch 补偿（不是姿态 MPC）
钩子:     MpcWeightMode (None / PushOff / Landing) 可扩展
```

判断：已经处理过"飞行相接触力归零"，这是奔跑能直接复用的最大资产。但目前是"单次、原地、时间触发"，不能周期化。

### 1.5 核心结论

```text
OpenLoong 当前是「准静态周期步态 + 单次爆发跳跃」两套独立框架。
它不是「周期性动态步态」框架。
GaitScheduler 与 JumpScheduler 互不知道对方存在。
```

---

## 2. 三个目标各自的缺口

### 2.1 奔跑 (running) —— 缺「飞行相 + 步态自适应」

本质区别：**行走始终至少一腿触地；奔跑存在双脚离地的飞行相。**

| 模块 | 现状 | 奔跑需要 |
|------|------|---------|
| GaitScheduler | 固定 tSwing，Fz 判触地 | 支持 flight phase（双腿无接触），步频随速度变 |
| MPC 接触约束 | 假设支撑腿有接触力 | 飞行相接触力 = 0，约束与参考轨迹要切换 |
| FootPlacement | stepHeight = 0.1 固定 | 高速下前伸量、抬腿高度都要显著增大 |
| StateEst | 靠触地做零速修正 | 飞行相无触地事件，估计器要顶住 |

可复用资产：jump_mpc 里"飞行相接触力归零"的经验可直接反哺。奔跑 ≈ 周期性的小跳跃。

### 2.2 边走边跳 / 边跑边跳 —— 缺「调度器融合」

难点不在动力学，在**两个状态机如何共存**：

```text
现状: GaitScheduler 和 JumpScheduler 是两条独立的线，
      walk_* demo 用前者，jump_mpc 用后者。
```

需要一个上层协调器决定控制权交接：

- 起跳前选合适的步态相位（不能在单脚刚抬腿时起跳）
- 落地后重新进入步态，而不是回到 jump 的原地站立恢复
- MPC 参考轨迹和权重在两种模式间平滑切换（`MpcWeightMode` 是现成钩子）

这是三个目标里**工程整合量最大**的，但动力学上不需要新东西。

### 2.3 跨栏 (hurdle) —— 缺「环境感知 + 轨迹规划」

跨栏 = 边走边跳 + 障碍物位置约束。额外缺：

```text
- 障碍物位置/高度输入（当前系统无环境感知）
- 落脚点需避开障碍（FootPlacement 现不接受落点约束）
- 起跳时机/高度按栏距和栏高计算（当前 jump_z 是常数）
```

跨栏是最终目标，包含前两者，不建议提前碰。

---

## 3. 依赖关系与推进顺序

```text
奔跑 (新增飞行相步态)
   │
   ├──► 边走边跳 (调度器融合，复用跳跃 + 步态)
   │        │
   └────┴──► 跨栏 (加环境感知 + 落点约束)
```

难度排序（低 → 高）：

```text
奔跑  <  边走边跳  <  跨栏
```

注意：奔跑的"飞行相步态"与边走边跳的"调度器融合"是两个不同维度，不是纯线性递进。若更想先啃"多状态机融合"，也可先做低速小跳版的边走边跳，避开奔跑的高速状态估计问题。

---

## 4. 对外部仓库策略的影响

| 目标 | 最该参考的仓库 | 看什么 |
|------|--------------|--------|
| 奔跑飞行相 | Learning_MPC_Jumping | 飞行相接触力归零、阶段切换（与 jump_mpc 同源） |
| 奔跑通用 MPC | Quadruped-PyMPC | 周期步态下的 SRBM MPC 参考轨迹生成 |
| 飞行姿态 | olympus_mpc_demo | 飞行相身体姿态控制（跨栏用得上） |

结论：现在只下 `Learning_MPC_Jumping` 是对的 —— 它同时喂养"奔跑"和"边走边跳"两个近期目标。另两个推迟到跨栏阶段。

---

## 5. 一句话结论

```text
近期路线: jump_mpc 收尾 → 奔跑(飞行相步态) → 边走边跳(调度器融合) → 跨栏
外部仓库: 现在只下 Learning_MPC_Jumping
最大的坑: 不是动力学，是 GaitScheduler 与 JumpScheduler 两个状态机如何融合
```

---

## 6. 下一步追踪点

- [ ] 深入读 GaitScheduler 与 JumpScheduler 的实际交互点，评估融合的具体改动量
- [ ] 从 jump_mpc 提炼"飞行相接触力归零"的可复用接口
- [ ] 下载 Learning_MPC_Jumping 后，对照其阶段划分，回填本文的奔跑章节
