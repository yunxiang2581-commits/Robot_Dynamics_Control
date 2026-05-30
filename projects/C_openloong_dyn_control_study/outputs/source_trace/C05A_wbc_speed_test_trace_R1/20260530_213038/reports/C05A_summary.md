# C05A Summary

## 1. 输出目录

```text
projects/C_openloong_dyn_control_study/outputs/source_trace/C05A_wbc_speed_test_trace_R1/20260530_213038/
```

## 2. 生成文件列表

Reports:

```text
reports/C05A_wbc_speed_test_source_trace.md
reports/C05A_wbc_speed_test_datalog_columns.md
reports/C05A_wbc_speed_test_stdout_analysis.md
reports/C05A_summary.md
```

Tables:

```text
tables/wbc_speed_test_call_sequence.csv
tables/wbc_speed_test_module_io_table.csv
tables/wbc_speed_test_datalog_columns.csv
```

Diagram:

```text
diagrams/wbc_speed_test_pipeline.mmd
```

Logs and commands:

```text
logs/00_wbc_speed_test_source_candidates.log
logs/01_wbc_speed_test_rg_trace.log
logs/02_stdout_head_sample.log
logs/03_stdout_tail_sample.log
logs/04_git_status_short.log
logs/05_git_diff_stat_before_c05a.log
logs/06_input_wbc_speed_test_binary.log
logs/07_input_c04_logs_listing.log
logs/08_input_c04_artifacts_listing.log
logs/09_datalog_runtime_stats.log
commands/C05A_wbc_speed_test_source_trace_commands.sh
```

## 3. wbc_speed_test 一句话定位

`wbc_speed_test` 是一个无 MuJoCo viewer、无 MuJoCo XML scene 的 WBC/PVT 数值计算与循环耗时 benchmark；它复用固定传感器/电机状态循环 10000 次，而不是做完整仿真闭环。

## 4. 主循环核心顺序

核心顺序是：

```text
fixed sample state
-> DataBus.updateQ
-> Pin_KinDyn FK/Jacobian/dynamics
-> JoyStickInterpreter desired command
-> GaitScheduler
-> FootPlacement
-> WBC_priority computeDdq/computeTau
-> PVT_Ctr calMotorsPVT
-> chrono duration
-> DataLogger datalog.log
-> stdout Execution time
```

更详细的表格见：

```text
tables/wbc_speed_test_call_sequence.csv
```

## 5. datalog 列解析完成度

结构解析完成度：`HIGH`。

已确认：

- `datalog.log` 有 10000 行。
- 每行 84 列。
- 分隔符为逗号。
- 第 84 列 `runTime` 与 stdout 的 `Execution time` 同源。
- `matlabReadDataScript.txt` 明确给出字段范围。

逐列 CSV 见：

```text
tables/wbc_speed_test_datalog_columns.csv
```

限制：当前 datalog 不记录 `qp_status`、`wbc_tauJointRes`、`wbc_FrRes`，因此不能用它直接统计 WBC QP 成功率或接触力约束。

## 6. stdout 解析完成度

解析完成度：`HIGH`。

已确认：

- stdout 总行数为 10001。
- 10000 行来自循环内 `printf("Execution time: ...")`。
- 最后一行来自循环后的 `std::cout` 提示。
- 10000 个 `Execution time` 与 `LoopNum=10000` 对齐。

限制：该耗时不是纯 WBC/QP benchmark，后续循环会混入上一轮 logging/printf 开销。

## 7. 当前最大不确定点

最大不确定点是：`wbc_speed_test` 只做 non-viewer speed benchmark，它没有记录 WBC 内部输出字段，也注释掉了 WBC 结果到电机 desired command 的直接转换代码。因此它能证明 WBC/PVT 计算链路能跑通，但不能代表 `walk_wbc` 的完整 MuJoCo 闭环语义。

具体不确定点：

- `fL/fR` 的 x/y 分量单位和坐标系仍为 `UNKNOWN`。
- `FootPlacement` 在 `legState=DSt` 下的物理语义需要 `walk_wbc` 对照。
- `PVT_Ctr` 在本 benchmark 中是否有意不使用 WBC 输出，需要 `walk_wbc` 主循环对照。
- 纯 WBC/QP 时间需要额外分段计时，目前只有整体 loop runtime。

## 8. 下一步建议

建议路线：

1. 如果 datalog 列含义仍需更细：做 `C05A-2 DataLogger/WBC runtime field design`，设计如何记录 QP status、WBC torque、WBC contact force。
2. 如果 `wbc_speed_test` 已清楚：做 `C05B walk_wbc 主循环源码追踪`，对比真实 MuJoCo viewer 闭环。
3. 如果想做运行验证：做 `C06 walk_wbc GUI/runtime observation`，规划 X11/OpenGL/截图或录屏。

更稳顺序是先做 `C05B`，再做 `C06`。

## 9. 边界确认

本轮确认：

- 未修改 `external/open_source_repos/OpenLoong-Dyn-Control/` 官方源码。
- 未修改 R2 worktree 源码。
- 未运行 `walk_wbc`。
- 未运行 `walk_mpc_wbc`。
- 未打开 GUI viewer。
- 未生成 MP4。
- 未执行 `git add`。
- 未执行 `git commit`。
- 未执行 `git push`。
