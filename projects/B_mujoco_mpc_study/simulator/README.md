# Project B Simulator 规划

## 目标

后续在本目录实现 MuJoCo MPC Simulator，用于 simulation-only 复现 MPC 的最小闭环：状态读取、rollout、horizon cost、控制序列选择、执行第一步控制、视频导出和指标记录。

## 最小可运行命令规划

```bash
python simulator/run_demo.py --demo B01_single_joint_mpc_demo --export-video
python simulator/run_demo.py --demo B02_two_link_mpc_tracking_demo --export-video
python simulator/run_demo.py --demo B03_rollout_predictive_sampling_demo --export-video
python simulator/run_demo.py --demo B04_cart_double_inverted_pendulum_mpc --export-video
python simulator/run_demo.py --demo B05_openloong_model_mpc_setup
python simulator/run_demo.py --demo B06_openloong_standing_balance_mpc --export-video
python simulator/run_demo.py --demo B07_openloong_weight_shift_or_stepping_mpc --export-video
```

当前命令只是后续 TODO skeleton 设计，本次不实现、不执行。

## 输入

- MuJoCo XML 或最小模型定义。
- 初始状态。
- 目标角度、末端轨迹、倒立摆竖直目标、人形 pelvis / torso / foot 参考状态。
- MPC horizon、采样数量、cost 权重、torque limit。

## 输出

- 视频：`outputs/runs/<task_name>/<run_id>/videos/`
- 曲线：`outputs/runs/<task_name>/<run_id>/figures/`
- 日志：`outputs/runs/<task_name>/<run_id>/logs/`
- 指标：`outputs/runs/<task_name>/<run_id>/metrics/`

## 计划文件接口

```text
envs/
controllers/
planners/
run_demo.py
record_video.py
metrics.py
```

## 评价指标

- final error。
- mean tracking error。
- max torque。
- runtime per control step。
