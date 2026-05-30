# C05A wbc_speed_test datalog Columns

## 1. 输入

datalog 输入文件：

```text
projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_wbc_speed_test_runtime_fix_R2/20260530_184509/artifacts/datalog.log
```

运行时自动生成的字段脚本：

```text
projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_wbc_speed_test_runtime_fix_R2/20260530_184509/runtime_record/matlabReadDataScript.txt
```

源码依据：

```text
demo/walk_wbc_speed_test.cpp:74-85
demo/walk_wbc_speed_test.cpp:208-219
common/data_logger.cpp:22-51
common/data_logger.cpp:114-122
```

## 2. 文件结构

验证结果：

```text
datalog lines: 10000
separator: comma
columns per row: 84
```

`LoopNum=10000`，所以 datalog 每一轮写一行，行数和主循环对齐。

## 3. DataLogger 如何定义列

`DataLogger::addIterm(name, len)` 按注册顺序分配列区间：

```cpp
recItemStartCol.push_back(colCout);
recItemEndCol.push_back(colCout+len-1);
colCout+=len;
```

`finishItermAdding()` 会写出 `matlabReadDataScript.txt`，内容为：

```matlab
motors_pos_cur=dataRec(:,1:31);
motors_vel_cur=dataRec(:,32:62);
rpy=dataRec(:,63:65);
fL=dataRec(:,66:68);
fR=dataRec(:,69:71);
basePos=dataRec(:,72:74);
baseLinVel=dataRec(:,75:77);
baseAcc=dataRec(:,78:80);
baseAngVel=dataRec(:,81:83);
runTime=dataRec(:,84:84);
```

`finishLine()` 使用：

```cpp
tmpStr = fmt::format("{:.6e}", fmt::join(recValue, ","));
LOG_INFO(dl, "{}", tmpStr);
```

所以 datalog 没有表头，列定义必须来自 `addIterm()` 顺序和 `matlabReadDataScript.txt`。

## 4. 列组总览

| 列范围 | 字段 | 长度 | 源码记录位置 | 含义 |
|---|---:|---:|---|---|
| 1-31 | `motors_pos_cur` | 31 | `demo/walk_wbc_speed_test.cpp:209` | 31 个电机当前位置 |
| 32-62 | `motors_vel_cur` | 31 | `demo/walk_wbc_speed_test.cpp:210` | 31 个电机当前速度 |
| 63-65 | `rpy` | 3 | `demo/walk_wbc_speed_test.cpp:211` | base roll/pitch/yaw |
| 66-68 | `fL` | 3 | `demo/walk_wbc_speed_test.cpp:212` | 左脚力/传感器样本，当前固定为 0 |
| 69-71 | `fR` | 3 | `demo/walk_wbc_speed_test.cpp:213` | 右脚力/传感器样本，当前固定为 0 |
| 72-74 | `basePos` | 3 | `demo/walk_wbc_speed_test.cpp:214` | base position |
| 75-77 | `baseLinVel` | 3 | `demo/walk_wbc_speed_test.cpp:215` | base linear velocity |
| 78-80 | `baseAcc` | 3 | `demo/walk_wbc_speed_test.cpp:216` | base acceleration |
| 81-83 | `baseAngVel` | 3 | `demo/walk_wbc_speed_test.cpp:217` | base angular velocity |
| 84 | `runTime` | 1 | `demo/walk_wbc_speed_test.cpp:218` | 当前循环 duration，单位秒 |

详细逐列表：

```text
tables/wbc_speed_test_datalog_columns.csv
```

## 5. 31 个电机列顺序

电机顺序来自 `common/PVT_ctrl.h:58-64` 与 `algorithm/pino_kin_dyn.h:29-35`：

```text
J_arm_l_01 ... J_arm_l_07
J_arm_r_01 ... J_arm_r_07
J_head_yaw
J_head_pitch
J_waist_pitch
J_waist_roll
J_waist_yaw
J_hip_l_roll
J_hip_l_yaw
J_hip_l_pitch
J_knee_l_pitch
J_ankle_l_pitch
J_ankle_l_roll
J_hip_r_roll
J_hip_r_yaw
J_hip_r_pitch
J_knee_r_pitch
J_ankle_r_pitch
J_ankle_r_roll
```

因此：

- `motors_pos_cur` 的 1-31 列按此顺序解释为关节位置，单位通常为 rad。
- `motors_vel_cur` 的 32-62 列按同一顺序解释为关节速度，单位通常为 rad/s。

这里对单位给 `MEDIUM` confidence：机器人关节角/角速度的物理惯例很明确，但源码没有在 datalog schema 中显式写单位。

## 6. 哪些列含义明确

高置信字段：

- `rpy`：源码逐项赋值，单位 rad。
- `basePos`：源码逐项赋值，单位 m。
- `baseLinVel`：源码逐项赋值，单位 m/s。
- `baseAcc`：源码逐项赋值，单位 m/s^2。
- `baseAngVel`：源码逐项赋值，单位 rad/s。
- `runTime`：源码明确来自 `duration.count()`，单位 sec。

中置信字段：

- `motors_pos_cur` / `motors_vel_cur`：列范围和关节顺序明确；单位依赖关节语义。
- `fL` / `fR`：源码只记录 3 个分量，其中 `GaitScheduler` 将 `fL[2]`、`fR[2]` 当成竖直力使用；x/y 的精确物理定义需要结合传感器接口或 `walk_wbc` runtime 再确认。

## 7. 哪些列仍不明确

- `fL.x/fL.y/fR.x/fR.y` 的单位和坐标系：`UNKNOWN`。
- `fL/fR` 是否在真实 viewer demo 中代表 3D force 还是某种简化接触量：`NEEDS_WALK_WBC_CONFIRMATION`。
- datalog 没有记录 WBC 的 `qp_status`、`wbc_tauJointRes`、`wbc_FrRes`，所以不能只靠 datalog 评估 WBC QP 成功率或接触力。

## 8. 是否适合后续 metrics parser

适合做一个基础 metrics parser，原因：

- 行数和 LoopNum 对齐。
- 每行列数固定为 84。
- 分隔符为逗号。
- `matlabReadDataScript.txt` 提供机器可读的列范围。
- 最后一列 `runTime` 可直接用于 speed smoke 统计。

但第一版 parser 应明确限制：

- 它只能解析本 demo 当前注册的字段。
- 它不能推断 QP status。
- 它不能验证接触力约束，因为 WBC contact force 没有写入 datalog。
- 它不能当作 MuJoCo 闭环轨迹，因为 `wbc_speed_test` 没有加载 MuJoCo viewer 或 XML scene。

## 9. 当前解析完成度

逐列结构解析完成度：`HIGH`。

物理单位解析完成度：`MEDIUM`。

WBC 内部输出覆盖度：`LOW`，因为本 demo datalog 没记录 WBC 输出。
