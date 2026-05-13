# B01 单关节 MPC

B01 的目标是用一个最小单关节 hinge joint 模型学习 MPC 的基本闭环：

```text
当前状态 -> rollout 预测未来 -> horizon cost 评价 -> predictive sampling 选择 torque 序列 -> 只执行第一步 -> 下一轮重新规划
```

这一任务不是为了复杂控制效果，而是把后续 B02/B03 和人形 MPC 中会反复出现的概念压缩到最小系统里。

## 1. 学习对象

B01 使用单自由度转动关节：

```text
hinge joint + torque actuator
```

状态：

```text
x = [q, dq]
```

控制：

```text
u = tau
tau in [-torque_limit, torque_limit]
```

目标：

```text
q -> q_target(t)
```

默认建议使用 smooth target，让目标从初始角平滑过渡到 `target.angle`，避免阶跃目标造成早期大误差和力矩饱和。

## 2. Rollout 与 Cost

rollout 输入：

```text
initial_state = [q0, dq0]
torque_sequence = [tau_0, tau_1, ..., tau_{H-1}]
horizon = H
```

rollout 输出：

```text
states = [x_0, x_1, ..., x_H]
controls = [tau_0, tau_1, ..., tau_{H-1}]
```

基础 horizon cost：

```text
J = sum w_q (q_k - q_target(t_k))^2
  + sum w_dq dq_k^2
  + sum w_tau tau_k^2
  + w_terminal (q_H - q_target(t_H))^2
```

Predictive sampling 的最小逻辑：

```text
for sequence in candidate_sequences:
    trajectory = rollout(initial_state, sequence)
    cost = compute_horizon_cost(trajectory, sequence)
    keep lowest cost sequence

execute best_sequence[0]
```

只执行第一步的原因是 MPC 是 receding horizon control：每个控制周期都重新读取当前状态并重新规划。

## 3. 配置文件

默认配置：

```text
projects/B_mujoco_mpc_study/configs/B01_single_joint_mpc.yaml
```

典型结构：

```yaml
model:
  path: simulator/models/B01_single_joint.xml

target:
  angle: 0.5
  profile: smooth
  ramp_duration: 0.5

initial_state:
  q: 0.0
  dq: 0.0

simulation:
  dt: 0.01
  num_steps: 200

mpc:
  horizon: 20
  num_candidates: 128
  torque_limit: 2.0

cost:
  q_weight: 1.0
  dq_weight: 0.1
  torque_weight: 0.001
  terminal_weight: 5.0

outputs:
  root: outputs/runs
  export_video: true
  save_figures: true
  save_metrics: true

viewer:
  show: false
  real_time: true
```

覆盖规则：

```text
命令行参数 > 配置文件 > 代码默认值
```

## 4. 运行命令

默认运行：

```powershell
D:\anaconda\envs\mujoco_py311\python.exe projects/B_mujoco_mpc_study/simulator/scripts/run_B01_single_joint_mpc_demo.py
```

指定配置：

```powershell
D:\anaconda\envs\mujoco_py311\python.exe projects/B_mujoco_mpc_study/simulator/scripts/run_B01_single_joint_mpc_demo.py --config projects/B_mujoco_mpc_study/configs/B01_single_joint_mpc.yaml
```

临时覆盖：

```powershell
D:\anaconda\envs\mujoco_py311\python.exe projects/B_mujoco_mpc_study/simulator/scripts/run_B01_single_joint_mpc_demo.py --target-angle 0.8 --num-steps 300 --num-candidates 256
```

实时 viewer：

```powershell
D:\anaconda\envs\mujoco_py311\python.exe projects/B_mujoco_mpc_study/simulator/scripts/run_B01_single_joint_mpc_demo.py --show-viewer --no-export-video --no-save-figures --no-save-metrics
```

## 5. 输出文件

输出目录：

```text
outputs/runs/B01_single_joint_mpc_demo/<run_id>/
```

典型输出：

```text
videos/B01_single_joint_mpc_demo.mp4
figures/B01_angle_tracking.png
figures/B01_angle_error.png
figures/B01_torque.png
figures/B01_best_cost.png
metrics/B01_metrics.csv
logs/B01_run_log.txt
```

图像含义：

- `B01_angle_tracking.png`：`q` 与 `q_target(t)`。
- `B01_angle_error.png`：`q - q_target(t)`。
- `B01_torque.png`：执行 torque。
- `B01_best_cost.png`：每个控制步选中的 best horizon cost。

## 6. 验收标准

B01 完成必须包含：

- simulation video
- angle tracking figure
- angle error figure
- torque figure
- best cost figure
- metrics CSV
- run log
- 复现实验命令

核心判断是：能否清楚解释 `q -> q_target(t)` 的闭环 MPC 数据流，而不是追求复杂控制器。
