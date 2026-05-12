# B01 配置说明

## 目标

B01 不应只依赖命令行参数。默认实验参数应写在配置文件中，命令行只用于临时覆盖。

默认配置文件：

```text
projects/B_mujoco_mpc_study/configs/B01_single_joint_mpc.yaml
```

默认运行方式：

```powershell
D:\anaconda\envs\mujoco_py311\python.exe projects/B_mujoco_mpc_study/simulator/scripts/run_B01_single_joint_mpc_demo.py
```

指定配置文件：

```powershell
D:\anaconda\envs\mujoco_py311\python.exe projects/B_mujoco_mpc_study/simulator/scripts/run_B01_single_joint_mpc_demo.py --config projects/B_mujoco_mpc_study/configs/B01_single_joint_mpc.yaml
```

临时覆盖参数：

```powershell
D:\anaconda\envs\mujoco_py311\python.exe projects/B_mujoco_mpc_study/simulator/scripts/run_B01_single_joint_mpc_demo.py --target-angle 0.8 --num-steps 300 --num-candidates 256
```

## 配置结构

### model

```yaml
model:
  path: simulator/models/B01_single_joint.xml
```

含义：

- `path` 是 MuJoCo XML 模型路径。
- 相对路径以 Project B 根目录为基准。

验证标准：

- 文件必须存在。
- XML 中应包含 `single_hinge` 和 `single_joint_motor`。

### target

```yaml
target:
  angle: 0.5
  profile: smooth
  ramp_duration: 0.5
```

含义：

- `angle` 是最终目标关节角，单位 rad。
- `profile` 是目标输入方式：
  - `step`：从第 0 步开始直接使用最终目标角。
  - `ramp`：在 `ramp_duration` 内线性过渡到最终目标角。
  - `smooth`：在 `ramp_duration` 内用 smoothstep 平滑过渡到最终目标角。
- `ramp_duration` 是 ramp / smooth 从初始角过渡到最终目标角的时间，单位 s。

物理意义：

- 控制器希望单关节跟随 `q_target(t)`，最终到达 `target.angle`。
- 默认使用 `smooth`，避免 0 -> 1.0 rad 阶跃目标带来的早期大误差和力矩饱和。

数学意义：

- step 模式中 horizon cost 的角度 residual 是 `q - q_target`。
- ramp / smooth 模式中 residual 是 `q_k - q_target(t_k)`，目标会随时间变化。

### initial_state

```yaml
initial_state:
  q: 0.0
  dq: 0.0
```

含义：

- `q` 是初始关节角。
- `dq` 是初始关节角速度。

验证标准：

- run log 中的 `initial_q` 和 `initial_dq` 应与配置一致，除非被命令行覆盖。

### simulation

```yaml
simulation:
  dt: 0.01
  num_steps: 200
```

含义：

- `dt` 是 MuJoCo 仿真步长。
- `num_steps` 是闭环控制步数。

物理意义：

- 总仿真时长为 `dt * num_steps`。

### mpc

```yaml
mpc:
  horizon: 20
  num_candidates: 128
  torque_limit: 2.0
```

含义：

- `horizon` 是每次 MPC 向未来 rollout 的步数。
- `num_candidates` 是 predictive sampling 的候选 torque 序列数量。
- `torque_limit` 是力矩限制。

工程判断：

- `horizon` 和 `num_candidates` 越大，通常搜索更充分，但 runtime 更高。
- `torque_limit` 过小可能无法到达目标，过大可能导致控制过激。

### cost

```yaml
cost:
  q_weight: 1.0
  dq_weight: 0.1
  torque_weight: 0.001
  terminal_weight: 5.0
```

含义：

- `q_weight` 惩罚角度误差。
- `dq_weight` 惩罚速度。
- `torque_weight` 惩罚控制力矩。
- `terminal_weight` 惩罚 horizon 末端角度误差。

数学形式：

```text
J = sum w_q (q_k - q_target(t_k))^2
  + sum w_dq dq^2
  + sum w_tau tau^2
  + w_terminal (q_H - q_target(t_H))^2
```

### outputs

```yaml
outputs:
  root: outputs/runs
  run_id:
  export_video: true
  save_figures: true
  save_metrics: true
```

含义：

- `root` 是按任务和时间分组保存结果的根目录。
- `run_id` 是本次运行 ID。为空时，脚本自动使用当前时间 `YYYYMMDD_HHMMSS`。
- `export_video` 控制是否保存 mp4。
- `save_figures` 控制是否保存 tracking / error / torque / cost figures。
- `save_metrics` 控制是否保存 metrics CSV。

Project B 验收要求：

- B01 完成时这三项都应为 `true`。

## 输出文件

B01 每次运行都会创建独立目录：

```text
outputs/runs/B01_single_joint_mpc_demo/<run_id>/
```

其中 `<run_id>` 默认是当前时间，例如：

```text
20260511_153012
```

目录内部结构：

```text
outputs/runs/B01_single_joint_mpc_demo/<run_id>/videos/B01_single_joint_mpc_demo.mp4
outputs/runs/B01_single_joint_mpc_demo/<run_id>/figures/B01_angle_tracking.png
outputs/runs/B01_single_joint_mpc_demo/<run_id>/figures/B01_angle_error.png
outputs/runs/B01_single_joint_mpc_demo/<run_id>/figures/B01_torque.png
outputs/runs/B01_single_joint_mpc_demo/<run_id>/figures/B01_best_cost.png
outputs/runs/B01_single_joint_mpc_demo/<run_id>/metrics/B01_metrics.csv
outputs/runs/B01_single_joint_mpc_demo/<run_id>/logs/B01_run_log.txt
```

### viewer

```yaml
viewer:
  show: false
  real_time: true
```

含义：

- `show` 控制是否打开 MuJoCo 实时窗口。
- `real_time` 控制打开窗口时是否按 `simulation.dt` 节奏播放。

推荐用法：

```powershell
D:\anaconda\envs\mujoco_py311\python.exe projects/B_mujoco_mpc_study/simulator/scripts/run_B01_single_joint_mpc_demo.py --show-viewer --no-export-video --no-save-figures --no-save-metrics
```

说明：

- viewer 只用于实时观察仿真，不改变 MPC 数学逻辑。
- 自动调参、批量验收和无图形界面环境应保持 `show: false`。
- 如果只想快速播放而不按真实时间等待，可以使用 `--no-real-time`。

## 覆盖规则

优先级：

```text
命令行参数 > 配置文件 > 代码默认值
```

例子：

```powershell
...run_B01_single_joint_mpc_demo.py --target-angle 1.0
```

即使配置文件中 `target.angle` 是 `0.5`，本次运行也会使用 `1.0`。

指定 run ID：

```powershell
...run_B01_single_joint_mpc_demo.py --run-id debug_001
```

输出将保存到：

```text
outputs/runs/B01_single_joint_mpc_demo/debug_001/
```
