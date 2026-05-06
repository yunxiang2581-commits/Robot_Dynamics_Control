# Project B 仿真-only 复现计划：MuJoCo MPC Simulator

## 最终目标

Project B 最终不是停留在“读过 MJPC 源码”，而是要形成一个基于 MuJoCo 的 MPC 仿真模拟器，并导出可展示的视频 demo。

最终展示形式必须说明：

1. 读懂了 MJPC / MuJoCo MPC 的 task、residual、rollout、planner 核心思想。
2. 把 MPC 抽象成可解释的数学模块。
3. 在 simulation-only 条件下完成最小可运行仿真器。
4. 导出 mp4 视频 demo。
5. 记录 final error、mean tracking error、max torque、runtime per control step 等指标。

## 严格边界

- 不做实物部署。
- 不做 sim2real 实机测试。
- 不接电机 SDK。
- 不接 CAN / EtherCAT / 串口通信。
- 不涉及固件。
- 不做真实机器人安全测试。
- 不做真实传感器标定。
- 不实现硬件接口。

## 后续 TODO skeleton 接口规划

后续只在 `simulator/` 下新增教学型代码骨架：

```text
simulator/
├── envs/                 # TODO: MuJoCo 单关节、二连杆等仿真环境
├── controllers/          # TODO: MPC 控制器、PD baseline
├── planners/             # TODO: predictive sampling / rollout planner
├── run_demo.py           # TODO: 统一 demo 入口
├── record_video.py       # TODO: 离屏渲染与 mp4 导出
└── metrics.py            # TODO: 误差、力矩、运行时间统计
```

本次不实现上述 Python 逻辑，只保留接口规划。

## B01_single_joint_mpc_demo

目标：

- 单关节系统。
- 目标角度跟踪。
- MPC horizon 预测。
- 输出角度误差曲线。
- 导出 mp4 视频。

输入：

- 初始关节角、速度。
- 目标角度或目标角度轨迹。
- horizon、候选控制数量、控制步长。

输出：

- `outputs/videos/B01_single_joint_mpc_demo.mp4`
- `outputs/figures/B01_angle_error.png`
- `outputs/logs/B01_run_log.txt`
- `outputs/metrics/B01_metrics.csv`

评价指标：

- final error。
- mean tracking error。
- max torque。
- runtime per control step。

## B02_two_link_mpc_tracking_demo

目标：

- 二连杆机械臂。
- 末端轨迹跟踪。
- MuJoCo 仿真。
- 控制输入记录。
- 导出 mp4 视频。

输入：

- 二连杆 MuJoCo XML。
- 末端参考轨迹。
- MPC horizon 和 torque limits。

输出：

- `outputs/videos/B02_two_link_mpc_tracking_demo.mp4`
- `outputs/figures/B02_ee_tracking_error.png`
- `outputs/logs/B02_control_log.txt`
- `outputs/metrics/B02_metrics.csv`

评价指标：

- final end-effector error。
- mean tracking error。
- max torque。
- runtime per control step。

## B03_rollout_predictive_sampling_demo

目标：

- 多条未来控制序列 rollout。
- 选择 cost 最低的控制序列。
- receding horizon 执行。
- 导出带轨迹可视化的视频。

输入：

- 当前状态。
- 候选控制序列采样参数。
- cost 权重。

输出：

- `outputs/videos/B03_rollout_predictive_sampling_demo.mp4`
- `outputs/figures/B03_rollout_costs.png`
- `outputs/logs/B03_selected_rollout_log.txt`
- `outputs/metrics/B03_metrics.csv`

评价指标：

- final error。
- mean tracking error。
- best rollout cost。
- max torque。
- runtime per control step。

## 最小可复现命令规划

后续 README 中应给出类似命令：

```bash
python simulator/run_demo.py --demo B01_single_joint_mpc_demo --export-video
python simulator/run_demo.py --demo B02_two_link_mpc_tracking_demo --export-video
python simulator/run_demo.py --demo B03_rollout_predictive_sampling_demo --export-video
```

这些命令当前只是规划，不在本次执行。
