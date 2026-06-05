# C05B Summary

## 1. 输出目录

```text
projects/C_openloong_dyn_control_study/outputs/source_trace/C05B_walk_wbc_trace_R1/20260603_000000/
```

## 2. 本轮一句话结论

`walk_wbc` 是 `wbc_speed_test` 之后更接近真实控制链的入口：它使用 MuJoCo sensor/state 作为输入，经过 StateEst、Pin_KinDyn、GaitScheduler、FootPlacement、WBC_priority、PVT_Ctr，最后把 `motors_tor_out` 写回 `mj_data->ctrl`。

## 3. 关键文件

```text
external/open_source_repos/OpenLoong-Dyn-Control/demo/walk_wbc.cpp
external/open_source_repos/OpenLoong-Dyn-Control/sim_interface/MJ_interface.cpp
external/open_source_repos/OpenLoong-Dyn-Control/sim_interface/MJ_interface.h
external/open_source_repos/OpenLoong-Dyn-Control/common/data_bus.h
external/open_source_repos/OpenLoong-Dyn-Control/algorithm/wbc_priority.cpp
external/open_source_repos/OpenLoong-Dyn-Control/common/PVT_ctrl.cpp
```

## 4. 生成文件

Reports:

```text
reports/C05B_walk_wbc_source_trace.md
reports/C05B_summary.md
```

Tables:

```text
tables/walk_wbc_call_sequence.csv
tables/walk_wbc_module_io_table.csv
```

Diagram:

```text
diagrams/walk_wbc_pipeline.mmd
```

## 5. 最大新增理解

- `walk_wbc` 有真实 MuJoCo 闭环：`mj_step` -> sensor/state -> controller -> torque -> `mj_data->ctrl`。
- `StateEst` 在真实闭环中很重要，因为 `MJ_Interface::dataBusWrite()` 没有直接写入 base position / base linear velocity。
- `WBC_priority` 会写出 `qp_status`、`wbc_tauJointRes`、`wbc_FrRes`，但 `walk_wbc` 当前 datalog 没记录这些字段。
- `wbc_speed_test` 中被注释的 WBC 输出转关节命令，在 `walk_wbc` 中是启用的。

## 6. 下一步建议

建议继续 `C05C MJ_Interface / StateEst / contact force trace`：

- 追踪 `fL/fR` 的来源。
- 追踪 base position / base velocity 在 `StateEst` 里的写入路径。
- 判断 `walk_wbc` datalog 是否足够支持后续 metrics。

随后再进入 `C06 walk_wbc GUI/runtime observation`。

## 7. 边界

- 本轮未修改官方源码。
- 本轮未运行 GUI demo。
- 本轮未修改仿真或控制参数。
