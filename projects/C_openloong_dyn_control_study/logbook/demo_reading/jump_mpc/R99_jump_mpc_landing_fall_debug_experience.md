# R2: jump_mpc 落地摔倒排障与经验复盘

## 1. 本轮目标

这一轮不是继续读 `jump_mpc.cpp` 的每一行，而是复盘一次真实排障过程：

```text
jump_mpc 起跳后落地摔倒
-> 怀疑 noVNC / 显示问题
-> 排除显示层
-> 沿 MuJoCo 状态到 MPC 输入链路排查
-> 用临时影子副本验证 base 状态写回问题
```

本轮要沉淀三类经验：

1. 后续继续调 `jump_mpc` 时，先检查哪些状态链路。
2. 遇到机器人控制 demo 摔倒时，如何组织证据和最小实验。
3. 求职面试中，如何把这次经历讲成一个“系统级调试案例”。

## 2. 故障现象

现象是：

```text
jump_mpc 可以运行并起跳，
但落地阶段姿态快速失控，
同时运行日志出现 QP working-set failure。
```

最初容易怀疑 noVNC、WSLg、OpenGL 或显示环境，因为 demo 是通过容器和远程可视化跑起来的。

但这次排查的关键判断是：

```text
画面只是观察窗口，
摔倒本身更可能来自控制链路中的状态不一致。
```

也就是说，不先调显示，不先调 MPC 权重，而是先检查控制器看到的状态和 MuJoCo 真实状态是不是同一个东西。

## 3. 控制链路回顾

`jump_mpc` 的核心链路可以简化成：

```text
MuJoCo
-> MJ_Interface::updateSensorValues()
-> MJ_Interface::dataBusWrite()
-> DataBus::updateQ()
-> Pin_KinDyn
-> jump_state 状态机
-> MPC::dataBusRead()
-> MPC::cal()
-> DataBus / PVT_Ctr
-> mj_data->ctrl
```

这里最重要的一点是：

```text
MPC 并不直接读 MuJoCo。
MPC 读的是 DataBus 里的 q / dq / base 状态。
```

如果 MuJoCo 里的机身已经起跳、下落、翻转，但 `DataBus` 里的浮动基位置和速度仍然接近初值，那么 MPC 就会基于错误状态求接触力。

这类问题不属于“参数没调好”，而是底层状态链路不一致。

## 4. 最可疑的问题点

排查时最高优先级的问题是：

```cpp
// busIn.basePos[0] = basePos[0];
// busIn.basePos[1] = basePos[1];
// busIn.basePos[2] = basePos[2];
// busIn.baseLinVel[0] = baseLinVel[0];
// busIn.baseLinVel[1] = baseLinVel[1];
// busIn.baseLinVel[2] = baseLinVel[2];
```

这些语句位于 `MJ_interface::dataBusWrite()` 附近。

含义是：

```text
MJ_Interface 内部虽然从 MuJoCo 读取了 basePos / baseLinVel，
但没有把它们写回 DataBus。
```

直接后果：

```text
MuJoCo 真实 base: 正在起跳、落地、运动
DataBus basePos: 仍然接近 [0, 0, 0]
DataBus baseLinVel: 也没有真实线速度
DataBus::updateQ(): 用错误 basePos / baseLinVel 组装 q / dq
MPC::dataBusRead(): 用错误 q / dq 作为当前 CoM / base 状态
```

这会影响：

- `q[0:3]` 的浮动基位置。
- `dq[0:3]` 的浮动基线速度。
- Pinocchio 中脚端世界坐标和动力学项。
- MPC 对 CoM 高度、速度、接触力的判断。
- 落地恢复阶段重新启用 MPC 时的初始状态。

## 5. 第二个嫌疑点：落地接触切换

`jump_mpc.cpp` 中落地状态切换依赖：

```cpp
if (FLest(2) > 1000 && FRest(2) > 1000) {
    jump_state = 5;
}
```

这里的 `FLest / FRest` 是通过动力学反推得到的接触力估计，不是 MuJoCo 真实接触传感器。

这会带来两个风险：

1. 落地冲击很大时，估计力可能有延迟、尖峰或误判。
2. 一旦切到 `jump_state == 5`，MPC 重新接管；如果此时 base 状态还是错的，QP 很容易进入不可行或数值病态。

所以当时的判断不是“只改触地阈值”，而是：

```text
先修正底层 base 状态链路，
再考虑接触切换和 MPC 参数。
```

## 6. 最小验证策略

用户要求是：

```text
尝试，不要改动源文件
```

所以采用了“影子实验”：

```text
原始 OpenLoong 源码不动
-> 复制一份临时 worktree 到 outputs/docker_reproduction
-> 只在临时副本中恢复 basePos / baseLinVel 写入 DataBus
-> 不改 MPC 权重
-> 不改 QP
-> 不改 jump_state 时序
-> 不改接触阈值
-> 只重建受影响对象和 jump_mpc 可执行文件
-> 用同样 demo 对比日志
```

这个实验的输入是：

```text
MuJoCo 读取到的 basePos / baseLinVel
```

实验输出是：

```text
DataBus 中真实浮动基位置和速度
Pinocchio / MPC 看到的 q / dq
datalog.log 中 gpsVal / rpyVal / QP 日志变化
```

数学逻辑是否变化：

```text
没有变化。
MPC 目标、QP 约束、权重、跳跃状态机、接触阈值都没有改。
只修正了状态输入链路。
```

这一步的可能风险：

```text
basePos / baseLinVel 是底层状态源。
修复后不只影响 jump_mpc，也可能影响 walk_wbc、walk_mpc_wbc、动力学项、脚端世界坐标和状态估计逻辑。
正式合入前必须跑多 demo 回归。
```

## 7. 对照实验结果

### 7.1 Baseline：修复前

baseline 日志：

```text
projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_demo_runs/20260630_213842_jump_mpc_novnc/runtime_record/datalog.log
```

运行日志：

```text
projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_demo_runs/20260630_213842_jump_mpc_novnc/logs/02_jump_mpc_runtime.log
```

关键结果：

```text
运行到 t = 13.005s
gpsVal first = [0, 0, 0]
gpsVal last  = [0, 0, 0]
gpsVal min/max 全程为 0
```

这说明 `DataBus` 记录到的 base position 没有随机器人运动变化。

姿态发散过程：

```text
t = 8.500s: rpy = [ 0.005993, -0.074887,  0.008743]
t = 9.000s: rpy = [ 0.013236,  0.004267,  0.007110]
t = 9.200s: rpy = [-0.414359,  0.538040, -0.167253]
t = 9.500s: rpy = [ 2.951617,  0.217308, -2.409265]
t = 13.000s: rpy = [-0.089892, 1.466552, 1.600454]
```

最大姿态量：

```text
max |roll|  = 3.140634 rad
max |pitch| = 1.487173 rad
max |yaw|   = 3.622764 rad
```

QP 日志出现：

```text
ERROR: Maximum number of working set recalculations performed
failed!!!!!!!!!!!!!
```

这说明 MPC/QP 在某些时刻出现了不可行、数值病态或状态跳变过大的情况。

### 7.2 Shadow experiment：修复 base 状态写回后

临时实验副本：

```text
projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_jump_mpc_basepos_exp/20260701_001306/worktree/OpenLoong-Dyn-Control
```

修复实验日志：

```text
projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_jump_mpc_basepos_exp/foreground_exp_20260701_010857/runtime_record/datalog.log
```

关键结果：

```text
运行到 t = 13.005s
gpsVal first = [0, 0, 1.2]
gpsVal last  = [0.005856, -0.018688, 1.097447]
gpsVal z min/max = 0.837104 / 1.2
```

这说明 `DataBus` 终于记录到了真实的 base 高度变化。

姿态结果：

```text
t = 8.500s: rpy = [0.005993, -0.074887, 0.008743]
t = 8.970s: rpy = [0.017271,  0.242403, 0.003235]
t = 9.000s: rpy = [0.018854,  0.212625, 0.006704]
t = 9.200s: rpy = [0.010242, -0.118111, 0.007209]
t = 9.500s: rpy = [0.001491,  0.020526, 0.006272]
t = 13.000s: rpy = [0.006673, 0.000788, 0.002993]
```

最大姿态量：

```text
max |roll|  = 0.019432 rad
max |pitch| = 0.242403 rad
max |yaw|   = 0.019302 rad
```

其中最大值 `0.242403 rad` 对应的是 `pitch`，也就是俯仰角，约等于 `13.9 deg`。

修复实验没有再看到 QP working-set failure。

## 8. 结论

这次实验支持一个明确结论：

```text
jump_mpc 落地摔倒的首要问题不是 noVNC 或显示环境，
而是 MuJoCo 读到的 basePos / baseLinVel 没有写回 DataBus，
导致 MPC / Pinocchio / DataBus 使用了错误的浮动基状态。
```

为什么这个结论可信：

1. baseline 中 `gpsVal` 全程为零，但机器人实际已经起跳和落地。
2. baseline 在落地后 `rpyVal` 快速发散。
3. baseline 出现 QP working-set failure。
4. shadow experiment 只恢复 base 状态写回，不改控制数学和参数。
5. 修复后 `gpsVal` 变成真实 base 位置，姿态不再发散，QP failure 消失。

这不是最终正式合入结论，但足以说明：

```text
在继续调 MPC 权重、接触阈值、起跳高度之前，
必须先保证状态链路正确。
```

## 9. 后续调整检查清单

以后继续调 `jump_mpc`，建议按这个顺序：

1. 先确认 `DataBus` 状态是否可信。
   - `basePos` 是否随 MuJoCo base 实际运动变化。
   - `baseLinVel` 是否有合理的起跳、下落速度。
   - `q[0:3]` 和 `dq[0:3]` 是否来自真实浮动基状态。

2. 再确认日志列。
   - `rpyVal=dataRec(:,157:159)`。
   - `gpsVal=dataRec(:,160:162)`。
   - `Ufe=dataRec(:,175:186)`。
   - 对比 `gpsVal.z`、`rpyVal.pitch`、QP 报错时间点。

3. 再检查落地切换。
   - `FLest(2) > 1000 && FRest(2) > 1000` 是否过于依赖反推力。
   - 是否需要加入接触持续时间、足端高度、base 垂向速度等辅助条件。
   - 是否需要避免触地瞬间立刻全量重新启用 MPC。

4. 最后才调 MPC 参数。
   - 权重。
   - 接触力上下界。
   - 恢复阶段目标高度。
   - 起跳速度和加速时间。

5. 每次只改一个变量。
   - 记录输入变化。
   - 记录输出日志。
   - 记录最大 `rpy`、`gpsVal.z` 范围、QP failure 次数。
   - 保留 baseline 和实验路径。

## 10. 对机器人控制调试的通用经验

### 10.1 不要先怀疑最显眼的层

GUI / noVNC / OpenGL 最显眼，但未必是根因。

机器人摔倒时，优先问：

```text
控制器看到的状态，和仿真器真实状态一致吗？
```

### 10.2 调参数之前先验状态

MPC、WBC、QP 这类模块对输入状态非常敏感。

如果 `q / dq / base pose / base velocity / contact state` 错了，调权重通常只是在错误输入上做补偿。

### 10.3 影子副本比直接改源码更适合验证假设

这次没有直接改原始源码，而是在 `outputs` 下复制临时 worktree。

好处是：

- 原始代码保持干净。
- 实验变量可控。
- 失败了也不会污染主线。
- 可以明确告诉别人“我只改了这个状态写回点”。

### 10.4 日志要能回答因果问题

这次关键不是“看起来站稳了”，而是能回答：

```text
为什么怀疑 base 状态？
修复前 base 状态是什么？
修复后 base 状态是什么？
姿态发散是否消失？
QP failure 是否消失？
控制数学有没有变化？
```

这些问题都能从 `datalog.log` 和 runtime log 中得到证据。

### 10.5 对强接触切换要特别谨慎

跳跃落地不是平滑阶段，具有：

- 冲击大。
- 接触状态突变。
- 速度方向变化。
- QP 可行域快速变化。
- MPC 重新接管时输入容易跳变。

所以落地阶段要特别关注：

```text
base height
base vertical velocity
pitch / roll
contact force estimate
contact debounce
MPC enable timing
```

## 11. 求职项目表达

### 11.1 简历项目 bullet

可以写成：

```text
- 复现并调试 OpenLoong 人形机器人 jump_mpc 跳跃 demo，定位落地失稳问题并通过日志对照验证根因：MuJoCo basePos/baseLinVel 未写回 DataBus，导致 Pinocchio q/dq 与 MPC 输入状态不一致；在不修改 MPC 权重和接触逻辑的前提下，通过影子副本最小修复使姿态峰值由约 3.62 rad 降至 0.24 rad，并消除 QP working-set failure。
```

如果简历需要更短：

```text
- 调试 OpenLoong jump_mpc 落地失稳问题，定位 MuJoCo-DataBus 浮动基状态链路缺失，设计影子副本验证实验，使落地姿态由发散恢复到稳定，并建立可复现实验日志。
```

### 11.2 面试 1 分钟讲法

可以这样讲：

```text
我在复现 OpenLoong 的 jump_mpc 跳跃 demo 时遇到落地摔倒。最初看起来像 noVNC 或显示环境问题，但我没有先调显示，而是沿控制链路检查 MuJoCo 到 DataBus、Pinocchio q/dq、再到 MPC 输入的状态一致性。日志显示 gpsVal/base_pos 全程为零，但机器人实际已经起跳和落地，同时 rpy 在 9 秒后快速发散，QP 也报 working-set failure。之后我发现 MJ_Interface 里 basePos/baseLinVel 写回 DataBus 的代码被注释掉了，于是在 outputs 下做了一个临时影子副本，只恢复这 6 行状态写回，不改 MPC 权重、QP、跳跃时序和接触阈值。对照实验显示，修复后 base 高度从 1.2m 到落地约 0.85m 的变化能被记录，最大姿态扰动约 0.24 rad，不再出现 QP failure。这次经历让我更明确：机器人控制调参前，必须先验证状态链路和接触切换是否一致。
```

### 11.3 面试追问：为什么不是直接调 MPC？

回答思路：

```text
因为 MPC 的输入状态已经错了。
如果 base position 和 linear velocity 没有写入 DataBus，
那么 MPC 看到的 CoM 状态不是仿真器里的真实状态。
这时调权重只是补偿错误输入，不能稳定解决问题。
所以我先做状态链路最小修复，再考虑后续参数优化。
```

### 11.4 面试追问：怎么证明只改这个有效？

回答思路：

```text
我用影子副本做了单变量实验。
原始源码不动，只在临时副本恢复 basePos/baseLinVel 写回。
实验前后保持 MPC、QP、jump_state、接触阈值都不变。
然后对比同一 demo 的 datalog：
修复前 gpsVal 全零、rpy 最大约 3.62 rad、QP failure 重复出现；
修复后 gpsVal 变成真实 base 位置、rpy 最大约 0.24 rad、QP failure 消失。
```

### 11.5 面试追问：这个修复有什么风险？

回答思路：

```text
basePos/baseLinVel 是底层状态源。
恢复写回后，不只影响 jump_mpc，也可能影响 walk_wbc、walk_mpc_wbc、Pinocchio 动力学项、脚端世界坐标和状态估计。
所以我不会直接说“可以无脑合入”，而是会安排 walk_wbc、walk_mpc_wbc、jump_mpc 的回归验证，并检查是否有模块原本依赖 StateEst 覆盖这些状态。
```

## 12. 后续可继续做的事情

建议下一步分两条线：

1. 正式修复线。
   - 在原始源码中恢复 `basePos / baseLinVel` 写回。
   - 明确注释为什么 jump_mpc 需要真实浮动基状态。
   - 跑 `walk_wbc`、`walk_mpc_wbc`、`jump_mpc` 回归。

2. 落地鲁棒性线。
   - 检查 `FLest / FRest` 的触地判定是否需要防抖。
   - 在 `jump_state == 5` 重新启用 MPC 前加入过渡期。
   - 记录 `baseLinVel.z`、`gpsVal.z`、`pitch` 和 `Ufe.z` 的时间曲线。
   - 分析落地瞬间 QP 约束是否过紧。

## 13. 本轮一句话总结

```text
这次 jump_mpc 落地摔倒的关键收获是：
不要把动态失稳先当成显示问题或参数问题，
而要先验证仿真器、状态总线、动力学模型和 MPC 输入之间的一致性。
```
