# B02 双连杆 MPC Tracking

B02 是 Project B 从“标量关节残差”进入“任务空间跟踪残差”的任务：

```text
B01: q -> q_target(t)
B02: p_ee(q) -> p_target(t)
```

它的目标是把 target trajectory、actual end-effector trajectory、tracking error、MPC step cost 和可复现输出串成一个完整闭环 demo。

## 1. 模型与状态

B02 使用平面二连杆机械臂：

```text
joint1: shoulder hinge
joint2: elbow hinge
link1 length: l1
link2 length: l2
```

状态：

```text
x = [q1, q2, dq1, dq2]
```

控制：

```text
u = [tau1, tau2]
```

末端位置：

```text
p_ee = [x_ee, y_ee]
```

手写 FK 参考：

```text
x_ee = l1 cos(q1) + l2 cos(q1 + q2)
y_ee = l1 sin(q1) + l2 sin(q1 + q2)
```

B02 第一版优先使用 MuJoCo site 读取末端位置，并可与手写 FK 做对照。

## 2. 目标轨迹与 Cost

目标轨迹：

```text
p_target(t) = [x_target(t), y_target(t)]
```

当前支持：

- `fixed`
- `circle`
- `figure8`
- `sinusoidal`
- `custom`

跟踪误差：

```text
error_k = ||actual_xy[k] - target_xy[k]||
```

基础 horizon cost：

```text
J = sum_k w_ee * ||p_ee(q_k) - p_target(t_k)||^2
  + sum_k w_dq * ||dq_k||^2
  + sum_k w_tau * ||tau_k||^2
  + w_terminal * ||p_ee(q_H) - p_target(t_H)||^2
```

## 3. 控制与可视化边界

B02 当前保持清晰职责分工：

- `controller`：输出当前控制 `u`，保存 `last_plan["best_cost"]`。
- `runner`：组织 env / planner / controller，并写出输出。
- `trajectory_logger`：记录统一 tracking log。
- `marker_renderer`：规划每帧 target/history/future/actual/error marker。
- `video_renderer`：消费 raw frames 与 tracking log，导出 marked MP4。

marker 绘制逻辑不进入 controller。

tracking log 一行一步，核心字段：

```text
step, time, q1, q2, dq1, dq2,
target_x, target_y, actual_x, actual_y,
error_norm, u1, u2, mpc_cost
```

第 `k` 帧时序规则：

```text
current_target = target_xy[k]
target_history = target_xy[:k+1]
target_future  = target_xy[k:]
current_actual = actual_xy[k]
error_line     = current_actual -> current_target
```

## 4. 可视化模式

- `overlay`：在原始 MuJoCo 视频帧上做 2D 后处理叠加。
- `scene`：在 marked-video 渲染阶段向 MuJoCo scene 中加入 marker。
- `hybrid`：组合 3D scene marker 和 2D overlay 文本。

显示内容包括：

- 当前目标点
- 目标历史轨迹
- 未来目标预览
- 当前实际末端位置
- 误差连线
- `time / error_norm / mpc_cost`

## 5. 输出文件

输出目录：

```text
outputs/runs/B02_two_link_mpc_tracking_demo/<run_id>/
```

典型输出：

```text
videos/B02_two_link_mpc_tracking_demo.mp4
videos/B02_two_link_mpc_tracking_marked.mp4
logs/B02_control_log.txt
logs/B02_tracking_log.csv
metrics/B02_metrics.csv
figures/B02_xy_target_vs_actual.png
figures/B02_tracking_error_time.png
figures/B02_control_input_time.png
```

## 6. Benchmark 与 Regression

B02 当前有两套稳定基线，不能混用。

### B02-current-10Nm

正式 benchmark / 交付验收基线。

配置：

```text
configs/B02_two_link_mpc.yaml
```

关键参数：

```yaml
simulation:
  num_steps: 1000

mpc:
  horizon: 16
  num_candidates: 768
  torque_limit: 10.0

cost:
  ee_weight: 80.0
  dq_weight: 0.1
  torque_weight: 0.002
  terminal_weight: 10.0
```

正式验收目标：

| 指标 | 目标 |
| --- | --- |
| `final_ee_error` | `<= 0.04` |
| `mean_ee_error` | `<= 0.03` |
| `max_ee_error` | `<= 0.14` |
| `max_abs_applied_torque` | `<= 10.0` |
| `max_abs_command_torque` | `<= 10.0` |
| `runtime_per_control_step` | `<= 0.30` |

参考通过 run：

```text
run_id = tuning_delivery_run_2
```

结果：

| 指标 | 数值 |
| --- | --- |
| `final_ee_error` | `0.0114` |
| `mean_ee_error` | `0.0202` |
| `max_ee_error` | `0.1394` |
| `max_abs_applied_torque` | `9.990` |
| `max_abs_command_torque` | `9.990` |
| `runtime_per_control_step` | `0.285` |

结论：

```text
B02-current-10Nm: ACCEPTED / PASSED
```

### B02-regression-light

日常快速健康检查基线，不替代正式 benchmark。

配置：

```text
configs/B02_two_link_mpc_regression.yaml
```

关键参数：

```yaml
simulation:
  num_steps: 100

mpc:
  horizon: 8
  num_candidates: 64
  torque_limit: 10.0
```

用途：

- 快速回归
- B03 前验证
- 检查 video / figures / metrics / logs schema 是否仍然完整

### 历史 2Nm 基线

`B02-benchmark-2Nm` 只保留历史意义。当前仓库已使用：

```text
MuJoCo actuator ctrlrange = [-10, 10]
planner torque_limit = 10.0
```

旧的 2Nm 验收数字不能直接用于当前实现。

## 7. 运行命令

正式 benchmark：

```bash
python projects/B_mujoco_mpc_study/simulator/scripts/run_B02_two_link_mpc_tracking_demo.py --config projects/B_mujoco_mpc_study/configs/B02_two_link_mpc.yaml --no-show-viewer
```

轻量 regression：

```bash
python projects/B_mujoco_mpc_study/simulator/scripts/run_B02_two_link_mpc_tracking_demo.py --config projects/B_mujoco_mpc_study/configs/B02_two_link_mpc_regression.yaml
```

短步 smoke check：

```bash
python projects/B_mujoco_mpc_study/simulator/scripts/run_B02_two_link_mpc_tracking_demo.py --no-show-viewer --num-steps 5 --horizon 2 --num-candidates 4 --export-video --save-figures --save-metrics --render-mode hybrid --run-id quick_smoke
```

## 8. 审阅重点

- controller 内部没有混入 marker 绘制逻辑。
- tracking log 每个仿真步恰好对应一行。
- target 与 actual 使用相同仿真时间索引。
- marked video 帧数与 tracking log 长度一致。
- benchmark 与 regression 没有被混用。
- B03 不修改 B02 benchmark / regression 配置。
