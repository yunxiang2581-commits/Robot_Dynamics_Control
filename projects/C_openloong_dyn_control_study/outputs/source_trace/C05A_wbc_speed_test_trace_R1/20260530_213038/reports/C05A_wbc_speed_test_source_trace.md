# C05A wbc_speed_test Source Trace

## 1. 本轮目标和边界

本轮目标是基于 C04-FIX-RUNTIME-R2 已经 PASS 的 `wbc_speed_test`，做源码级追踪：主循环、WBC 输入输出、DataBus 读写关系、stdout 输出含义，以及 `record/datalog.log` 每列含义。

本轮输入是已构建的 R2 worktree 源码和 C04 runtime 证据。本轮输出只写入：

```text
projects/C_openloong_dyn_control_study/outputs/source_trace/C05A_wbc_speed_test_trace_R1/20260530_213038/
```

边界：

- 未修改 `external/open_source_repos/OpenLoong-Dyn-Control/`。
- 未修改 R2 worktree 源码。
- 未运行 `walk_wbc`。
- 未运行 `walk_mpc_wbc`。
- 未打开 GUI viewer。
- 未生成 MP4。
- 未执行 `git add` / `git commit` / `git push`。

数学逻辑没有变化。本轮只是源码追踪与运行证据解释。

## 2. 输入源码和 runtime 证据

优先源码根目录：

```text
projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827/worktree/OpenLoong-Dyn-Control/
```

关键 runtime 证据：

```text
projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_wbc_speed_test_runtime_fix_R2/20260530_184509/logs/02_wbc_speed_test_stdout.log
projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_wbc_speed_test_runtime_fix_R2/20260530_184509/logs/03_wbc_speed_test_stderr.log
projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_wbc_speed_test_runtime_fix_R2/20260530_184509/artifacts/datalog.log
projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_wbc_speed_test_runtime_fix_R2/20260530_184509/runtime_record/matlabReadDataScript.txt
```

已确认：

```text
wbc_speed_test binary: exists
exit code: 0
stderr lines: 0
stdout lines: 10001
datalog lines: 10000
datalog columns: 84
```

## 3. wbc_speed_test 的定位

`wbc_speed_test` 的源码入口是：

```text
demo/walk_wbc_speed_test.cpp
```

它不是 viewer demo。

源码中没有加载 MuJoCo XML scene，没有创建 MuJoCo viewer，也没有真实仿真步进。它使用一组固定的电机位置、速度、base 姿态、base 位置和传感器样本，循环执行 kinematics/dynamics、gait/foot-placement、WBC、PVT、logging。

一句话定位：

```text
wbc_speed_test 是一个无 viewer、无 MuJoCo 闭环的 WBC/PVT 数值计算与循环耗时 benchmark。
```

## 4. main 初始化阶段

主入口 `demo/walk_wbc_speed_test.cpp:21-31` 创建这些对象：

| 对象 | 构造 | 作用 |
|---|---|---|
| `Pin_KinDyn` | `Pin_KinDyn("../models/AzureLoong.urdf")` | Pinocchio 运动学/动力学求解器 |
| `DataBus` | `DataBus(kinDynSolver.model_nv)` | 模块之间共享状态的黑板 |
| `WBC_priority` | `WBC_priority(model_nv, 18, 22, 0.7, timestep)` | WBC 优先级任务 + QP |
| `GaitScheduler` | `GaitScheduler(0.4, timestep)` | 步态相位和支撑腿状态 |
| `PVT_Ctr` | `PVT_Ctr(timestep, "../common/joint_ctrl_config.json")` | 关节 PVT/PD 力矩输出 |
| `FootPlacement` | `FootPlacement` | 摆动脚目标位置 |
| `JoyStickInterpreter` | `JoyStickInterpreter(timestep)` | 速度命令转 base 目标 |
| `DataLogger` | `DataLogger("../record/datalog.log")` | 写 datalog 和 Matlab 读取脚本 |

`Pin_KinDyn` 构造函数中使用：

```cpp
pinocchio::JointModelFreeFlyer root_joint;
pinocchio::urdf::buildModel(urdf_pathIn, root_joint, model_biped);
pinocchio::urdf::buildModel(urdf_pathIn, model_biped_fixed);
```

这说明该 benchmark 同时维护：

- 浮动基模型：用于人形整体动力学，`q` 是 base position + quaternion + joints，`dq` 是 base velocity + joint velocities。
- 固定基模型：用于部分 body-frame 足端/手端运动学计算。

这与项目主线中的“固定基 vs 浮动基”“`nq` vs `nv`”直接相关：这里 `DataBus.q` 长度是 `model_nv + 1`，因为浮动基姿态用四元数存储，`nq = nv + 1`。

## 5. LoopNum=10000 主循环

主循环：

```text
demo/walk_wbc_speed_test.cpp:97-226
```

核心顺序：

1. 写入固定传感器/电机样本到 `RobotState`。
2. `RobotState.updateQ()` 由传感器样本组装 `q` / `dq`。
3. `Pin_KinDyn` 读取 DataBus，计算 Jacobian 与动力学项，再写回 DataBus。
4. `JoyStickInterpreter` 写 desired base command。
5. `GaitScheduler` 写步态相位、支撑腿、stance/swing 相关状态。
6. `FootPlacement` 写摆动脚目标。
7. main 写 WBC 输入：`Fr_ff`、`des_delta_q`、`des_dq`、`des_ddq`、base target。
8. `WBC_priority` 计算 `ddq`、接触力修正和关节力矩。
9. `PVT_Ctr` 计算电机输出力矩。
10. 计时、写 datalog、打印 stdout。

完整调用顺序表见：

```text
tables/wbc_speed_test_call_sequence.csv
```

## 6. WBC 输入输出

WBC 入口：

```text
demo/walk_wbc_speed_test.cpp:156-184
algorithm/wbc_priority.cpp:99-196
algorithm/wbc_priority.cpp:198-395
algorithm/wbc_priority.cpp:397-729
```

main 中显式设置的 WBC 输入包括：

- `RobotState.Fr_ff`：12 维足端 feed-forward wrench，左右脚各 6 维。
- `RobotState.des_ddq`：期望广义加速度。
- `RobotState.des_dq`：期望广义速度。
- `RobotState.des_delta_q`：期望增量配置。
- `RobotState.base_rpy_des`：期望 base 姿态。
- `RobotState.base_pos_des(2)`：期望 base 高度。

`WBC_priority::dataBusRead()` 还从 DataBus 读取：

- 足端/手端位置姿态。
- `J_l`、`J_r`、`J_base`、`Jcom_W` 等 Jacobian。
- `dyn_M`、`dyn_M_inv`、`dyn_Ag`、`dyn_dAg`、`dyn_Non`。
- 当前 `q`、`dq`。
- `legState`、`motionState`。

`computeDdq()` 负责优先级任务解算：

- Walk 任务树包含 `static_Contact`、`PosRot`、`SwingLeg`、`RedundantJoints`、`HandTrackJoints` 等。
- Stand 任务树包含 `static_Contact`、`CoMXY_HipRPY`、`Pz`、`HandTrackJoints`、`HeadRP` 等。
- 当前 benchmark 初始 `motionState` 为 `Stand`，除非后续逻辑改变，否则走 stand task tree。

`computeTau()` 负责 QP：

- QP 变量维度 `QP_nv=18`，可以理解为 floating-base 加速度修正 6 维 + 接触 wrench 修正 12 维。
- QP 约束维度 `QP_nc=22`，包括 6 维 floating-base dynamics equality 和 16 维接触摩擦/力矩不等式。
- 输出 `eigen_fr_Opt`、`tauJointRes`、`qpStatus`、`nWSR`、`cpu_time`。

`WBC_priority::dataBusWrite()` 写回：

- `RobotState.wbc_ddq_final`
- `RobotState.wbc_tauJointRes`
- `RobotState.wbc_FrRes`
- `RobotState.wbc_delta_q_final`
- `RobotState.wbc_dq_final`
- `RobotState.qp_status`
- `RobotState.qp_nWSR`
- `RobotState.qp_cpuTime`

重要限制：本 demo 没有把这些 WBC 输出写入 datalog，也没有打印 QP status。因此 C05A 可以确认源码链路，但不能从现有 datalog 统计 QP 成功率。

## 7. PVT 输入输出

PVT 调用位置：

```text
demo/walk_wbc_speed_test.cpp:193-201
common/PVT_ctrl.cpp:49-91
```

`PVT_Ctr::dataBusRead()` 读取：

- `motors_pos_cur`
- `motors_vel_cur`
- `motors_pos_des`
- `motors_vel_des`
- `motors_tor_des`

`calMotorsPVT()` 计算：

```text
tauDes = Kp * (pos_des - pos_cur) + Kd * (vel_des - vel_cur) + feedforward_torque
```

然后经过一阶低通滤波、最大力矩限幅和 gear ratio，写出：

- `busIn.motors_tor_out`
- `busIn.motors_tor_cur`

关键注意点：`walk_wbc_speed_test.cpp:188-191` 中将 WBC 输出转换为电机期望位置/速度/力矩的代码被注释掉了：

```cpp
// RobotState.motors_pos_des = ...
// RobotState.motors_vel_des = ...
// RobotState.motors_tor_des = ...
```

因此在这个 speed test 中，PVT 更多是在固定输入状态和默认 desired command 下运行，而不是完整使用 WBC 结果驱动电机命令。这一点对理解它和 `walk_wbc` 的差异很重要。

## 8. DataLogger 输出

DataLogger 注册位置：

```text
demo/walk_wbc_speed_test.cpp:74-85
```

每轮写入位置：

```text
demo/walk_wbc_speed_test.cpp:208-219
```

`datalog.log`：

- 10000 行。
- 84 列。
- 逗号分隔。
- 无 header。
- 字段范围由 `runtime_record/matlabReadDataScript.txt` 和源码 `addIterm()` 确认。

逐列解析见：

```text
tables/wbc_speed_test_datalog_columns.csv
reports/C05A_wbc_speed_test_datalog_columns.md
```

当前 datalog 记录的是样本状态和循环耗时，不记录 WBC QP status 或 WBC 输出力矩/接触力。

## 9. stdout 输出

stdout 结构：

```text
10000 行 Execution time: <sec> sec.
1 行 loop time recorded to the last column of record/datalog.log
```

每轮 `Execution time` 与 datalog 第 84 列 `runTime` 同源。

统计值：

```text
count      10000
min_sec    0.000480524
mean_sec   0.0005172386373
median_sec 0.0005050585
p95_sec    0.000560175
max_sec    0.003082548
first_sec  0.003082548
last_sec   0.000495746
```

专项分析见：

```text
reports/C05A_wbc_speed_test_stdout_analysis.md
```

## 10. 当前已经吃透的点

- `wbc_speed_test` 是 non-viewer speed benchmark，不是 MuJoCo viewer demo。
- 主循环固定为 `LoopNum=10000`。
- C04 的 stdout/datalog 行数和源码循环完全对齐。
- DataBus 是模块间共享状态黑板。
- Pin_KinDyn 负责浮动基/固定基 Pinocchio 模型、Jacobian、frame pose、动力学项。
- WBC_priority 负责优先级任务 `computeDdq()` 和接触/力矩 QP `computeTau()`。
- PVT_Ctr 负责关节级 PD/PVT 力矩输出。
- DataLogger 的 84 列 schema 已经解析。
- stdout 的 10001 行来源已明确。

## 11. 当前仍不确定的点

- `fL` / `fR` 的 x/y 分量精确定义和单位：`UNKNOWN`。
- `FootPlacement` 在 `legState=DSt` 时输出语义：`NEEDS_WALK_WBC_CONFIRMATION`。
- `wbc_speed_test` 中 PVT 没有直接使用 WBC 输出驱动 desired motor command，这是否只是 benchmark 简化：`NEEDS_WALK_WBC_CONFIRMATION`。
- datalog 不含 QP status，因此不能用现有 datalog 判断 WBC QP 成功率。
- stdout/datalog 的 runtime 不是纯 WBC 或纯 QP 时间，因为计时窗口包含多模块循环，并且后续轮次会包含上一轮日志/打印开销。

## 12. 下一步建议

建议优先级：

1. `C05A-2 DataLogger/WBC instrumentation design`：设计不改官方源码的解析或 overlay 方案，确认是否需要复制 worktree 增加 QP status / WBC 输出字段。
2. `C05B walk_wbc 主循环源码追踪`：对比 `walk_wbc` 中 MuJoCo、MJ_Interface、DataBus、WBC、PVT 的真实闭环。
3. `C06 walk_wbc GUI/runtime observation`：规划 X11/OpenGL/viewer 截图或录屏方案，但应在 C05B 源码链路清楚后再做。
