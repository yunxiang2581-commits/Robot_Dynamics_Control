# Project B Simulator 与视频 Demo 计划

## Simulator 名称

MuJoCo MPC Simulator。

## 目标

后续实现一个 simulation-only 的 MuJoCo MPC 教学仿真器，用最小模型复现 MJPC 的核心思想：task residual、rollout、horizon cost、predictive sampling、receding horizon control。

## Demo 列表

| Demo | 目标 | 视频输出 | 关键指标 |
|---|---|---|---|
| B01_single_joint_mpc_demo | 单关节目标角度跟踪 | `outputs/videos/B01_single_joint_mpc_demo.mp4` | final error、mean tracking error、max torque、runtime per control step |
| B02_two_link_mpc_tracking_demo | 二连杆末端轨迹跟踪 | `outputs/videos/B02_two_link_mpc_tracking_demo.mp4` | final error、mean tracking error、max torque、runtime per control step |
| B03_rollout_predictive_sampling_demo | 多 rollout 选择最低 cost 序列 | `outputs/videos/B03_rollout_predictive_sampling_demo.mp4` | best rollout cost、mean tracking error、max torque、runtime per control step |

## 计划接口

```text
simulator/envs/
simulator/controllers/
simulator/planners/
simulator/run_demo.py
simulator/record_video.py
simulator/metrics.py
```

## 可复现命令规划

```bash
python simulator/run_demo.py --demo B01_single_joint_mpc_demo --export-video
python simulator/run_demo.py --demo B02_two_link_mpc_tracking_demo --export-video
python simulator/run_demo.py --demo B03_rollout_predictive_sampling_demo --export-video
```

当前命令只是后续接口设计，本次不执行。
