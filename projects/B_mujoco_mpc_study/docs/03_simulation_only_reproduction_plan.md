# Project B 仿真-only 复现计划：OpenLoong-oriented Humanoid MPC Simulator

## 最终目标

Project B 最终不是停留在“读过 MJPC 源码”，也不是只完成单关节或二连杆 demo，而是要形成一个面向 OpenLoong 人形模型的 simulation-only MPC 学习原型，并导出可展示的视频 demo。

本项目采用递进路线：

```text
B01-B03：先用最小模型学清楚 MPC 闭环。
B04：用小车倒立二阶摆学习欠驱动非线性平衡控制。
B05-B07：再切到 OpenLoong 人形模型，形成最终人形 MPC 成果。
```

最终展示形式必须说明：

1. 读懂了 MJPC / MuJoCo MPC 的 task、residual、rollout、planner 核心思想。
2. 把 MPC 抽象成可解释的数学模块。
3. 在 simulation-only 条件下完成最小可运行仿真器。
4. 导出 mp4 视频 demo。
5. 记录 final error、mean tracking error、max torque、runtime per control step 等指标。
6. 为 Project C 的 OpenLoong MPC-WBC-PVT 控制链提供清晰的 MPC target / contact schedule / residual 设计参考。

Project B 的每个任务都必须满足可视化验收：

```text
video + figures + metrics CSV + run log + README 复现实验命令
```

没有可视化输出的任务不算完成；没有 metrics 的视频不算完成；没有 README 复现命令的结果不算完成；没有解释图像含义的结果不算完成。

最终主线关系是：

```text
A: FK / Jacobian / IK / tracking baseline
B: OpenLoong-oriented humanoid MPC prototype
C: OpenLoong MPC-WBC-PVT full control-chain study
D: legged NMPC-WBC-contact-estimation generalization
```

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
├── envs/                 # TODO: MuJoCo 单关节、二连杆、小车倒立二阶摆、OpenLoong 人形仿真环境
├── controllers/          # TODO: MPC 控制器、PD baseline
├── planners/             # TODO: predictive sampling / rollout planner
├── utils/
│   ├── plotting.py        # TODO: 保存 error、torque、cost、runtime 曲线
│   └── visualization.py   # TODO: 绘制轨迹、目标点、rollout candidates、contact schedule
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

- `outputs/runs/B01_single_joint_mpc_demo/<run_id>/videos/B01_single_joint_mpc_demo.mp4`
- `outputs/runs/B01_single_joint_mpc_demo/<run_id>/figures/B01_angle_tracking.png`
- `outputs/runs/B01_single_joint_mpc_demo/<run_id>/figures/B01_angle_error.png`
- `outputs/runs/B01_single_joint_mpc_demo/<run_id>/figures/B01_torque.png`
- `outputs/runs/B01_single_joint_mpc_demo/<run_id>/figures/B01_best_cost.png`
- `outputs/runs/B01_single_joint_mpc_demo/<run_id>/logs/B01_run_log.txt`
- `outputs/runs/B01_single_joint_mpc_demo/<run_id>/metrics/B01_metrics.csv`

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

- `outputs/runs/B02_two_link_mpc_tracking_demo/<run_id>/videos/B02_two_link_mpc_tracking_demo.mp4`
- `outputs/runs/B02_two_link_mpc_tracking_demo/<run_id>/figures/B02_ee_trajectory_xy.png`
- `outputs/runs/B02_two_link_mpc_tracking_demo/<run_id>/figures/B02_ee_tracking_error.png`
- `outputs/runs/B02_two_link_mpc_tracking_demo/<run_id>/figures/B02_joint_torque.png`
- `outputs/runs/B02_two_link_mpc_tracking_demo/<run_id>/logs/B02_control_log.txt`
- `outputs/runs/B02_two_link_mpc_tracking_demo/<run_id>/metrics/B02_metrics.csv`

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

- `outputs/runs/B03_rollout_predictive_sampling_demo/<run_id>/videos/B03_rollout_predictive_sampling_demo.mp4`
- `outputs/runs/B03_rollout_predictive_sampling_demo/<run_id>/figures/B03_rollout_candidates.png`
- `outputs/runs/B03_rollout_predictive_sampling_demo/<run_id>/figures/B03_rollout_costs.png`
- `outputs/runs/B03_rollout_predictive_sampling_demo/<run_id>/figures/B03_selected_rollout.png`
- `outputs/runs/B03_rollout_predictive_sampling_demo/<run_id>/logs/B03_selected_rollout_log.txt`
- `outputs/runs/B03_rollout_predictive_sampling_demo/<run_id>/metrics/B03_metrics.csv`

评价指标：

- final error。
- mean tracking error。
- best rollout cost。
- max torque。
- runtime per control step。

## B04_cart_double_inverted_pendulum_mpc

目标：

- 小车倒立二阶摆系统。
- 学习欠驱动、非线性、不稳定平衡点上的 MPC。
- 使用 predictive sampling + MuJoCo rollout + horizon cost，让两节摆杆接近竖直向上，同时约束小车不要过度偏离中心。
- 作为 B03 与 OpenLoong 站立平衡 MPC 之间的过渡任务。

输入：

- 小车位置 `x` 与速度 `dx`。
- 第一节摆角 `theta1` 与角速度 `dtheta1`。
- 第二节摆角 `theta2` 与角速度 `dtheta2`。
- horizon、候选控制力数量、控制步长。
- 控制力限制、轨道范围、cost 权重。

输出：

- `outputs/runs/B04_cart_double_inverted_pendulum_mpc/<run_id>/videos/B04_cart_double_inverted_pendulum_mpc.mp4`
- `outputs/runs/B04_cart_double_inverted_pendulum_mpc/<run_id>/figures/B04_cart_position.png`
- `outputs/runs/B04_cart_double_inverted_pendulum_mpc/<run_id>/figures/B04_pendulum_angles.png`
- `outputs/runs/B04_cart_double_inverted_pendulum_mpc/<run_id>/figures/B04_angle_errors.png`
- `outputs/runs/B04_cart_double_inverted_pendulum_mpc/<run_id>/figures/B04_control_force.png`
- `outputs/runs/B04_cart_double_inverted_pendulum_mpc/<run_id>/logs/B04_cart_double_pendulum_run_log.txt`
- `outputs/runs/B04_cart_double_inverted_pendulum_mpc/<run_id>/metrics/B04_cart_double_pendulum_metrics.csv`

评价指标：

- final angle error。
- mean angle error。
- cart position max deviation。
- max control force。
- success / failure。
- runtime per control step。

## B05_openloong_model_mpc_setup

目标：

- 切到 OpenLoong 人形模型。
- 读取 MuJoCo 模型、`qpos`、`qvel`、actuator、body、site 和接触候选对象。
- 明确 pelvis、torso、left foot、right foot 等关键对象。
- 输出人形模型摘要，先不实现复杂控制。

输入：

- OpenLoong MuJoCo 模型路径。
- 初始状态。
- 关键 body / site 候选名称。

输出：

- `outputs/runs/B05_openloong_model_mpc_setup/<run_id>/videos/B05_openloong_passive_or_static_preview.mp4`
- `outputs/runs/B05_openloong_model_mpc_setup/<run_id>/figures/B05_body_site_map.png`
- `outputs/runs/B05_openloong_model_mpc_setup/<run_id>/figures/B05_actuator_index_map.png`
- `outputs/runs/B05_openloong_model_mpc_setup/<run_id>/metrics/B05_model_dimensions.csv`
- `outputs/runs/B05_openloong_model_mpc_setup/<run_id>/logs/B05_openloong_model_summary.txt`

评价指标：

- `nq`、`nv`、actuator 数量。
- 关键 body / site 是否找到。
- 初始状态是否能稳定加载并 step。

## B06_openloong_standing_balance_mpc

目标：

- 第一版人形 MPC 成果。
- 在 OpenLoong MuJoCo 模型上做站立平衡 / 姿态保持。
- 重点理解 floating-base humanoid 的 residual、horizon cost 和 receding horizon control。

输入：

- 当前 `qpos`、`qvel`。
- 目标 pelvis 高度、torso 姿态、双脚接触状态。
- horizon、候选控制数量、torque limit、cost 权重。

输出：

- `outputs/runs/B06_openloong_standing_balance_mpc/<run_id>/videos/B06_openloong_standing_balance_mpc.mp4`
- `outputs/runs/B06_openloong_standing_balance_mpc/<run_id>/figures/B06_pelvis_height_error.png`
- `outputs/runs/B06_openloong_standing_balance_mpc/<run_id>/figures/B06_torso_orientation_error.png`
- `outputs/runs/B06_openloong_standing_balance_mpc/<run_id>/figures/B06_joint_torque.png`
- `outputs/runs/B06_openloong_standing_balance_mpc/<run_id>/figures/B06_foot_contact_summary.png`
- `outputs/runs/B06_openloong_standing_balance_mpc/<run_id>/logs/B06_balance_run_log.txt`
- `outputs/runs/B06_openloong_standing_balance_mpc/<run_id>/metrics/B06_balance_metrics.csv`

评价指标：

- pelvis height error。
- torso orientation error。
- foot contact summary。
- mean tracking error。
- max torque。
- runtime per control step。

## B07_openloong_weight_shift_or_stepping_mpc

目标：

- 第二版人形 MPC 成果。
- 在站立平衡基础上加入左右重心转移，或最小小步踏步。
- 引入简化 contact schedule、foot residual 和更明确的 horizon target。

输入：

- 当前人形状态。
- 左右脚接触计划。
- pelvis / CoM / foot target。
- horizon、cost 权重、torque limit。

输出：

- `outputs/runs/B07_openloong_weight_shift_or_stepping_mpc/<run_id>/videos/B07_openloong_weight_shift_or_stepping_mpc.mp4`
- `outputs/runs/B07_openloong_weight_shift_or_stepping_mpc/<run_id>/figures/B07_com_or_pelvis_tracking.png`
- `outputs/runs/B07_openloong_weight_shift_or_stepping_mpc/<run_id>/figures/B07_foot_target_error.png`
- `outputs/runs/B07_openloong_weight_shift_or_stepping_mpc/<run_id>/figures/B07_contact_schedule.png`
- `outputs/runs/B07_openloong_weight_shift_or_stepping_mpc/<run_id>/figures/B07_torque.png`
- `outputs/runs/B07_openloong_weight_shift_or_stepping_mpc/<run_id>/logs/B07_shift_or_step_run_log.txt`
- `outputs/runs/B07_openloong_weight_shift_or_stepping_mpc/<run_id>/metrics/B07_shift_or_step_metrics.csv`

评价指标：

- pelvis / CoM tracking error。
- foot target error。
- contact schedule consistency。
- max torque。
- runtime per control step。

## 最小可复现命令规划

后续 README 中应给出类似命令：

```bash
python simulator/run_demo.py --demo B01_single_joint_mpc_demo --export-video
python simulator/run_demo.py --demo B02_two_link_mpc_tracking_demo --export-video
python simulator/run_demo.py --demo B03_rollout_predictive_sampling_demo --export-video
python simulator/run_demo.py --demo B04_cart_double_inverted_pendulum_mpc --export-video
python simulator/run_demo.py --demo B05_openloong_model_mpc_setup
python simulator/run_demo.py --demo B06_openloong_standing_balance_mpc --export-video
python simulator/run_demo.py --demo B07_openloong_weight_shift_or_stepping_mpc --export-video
```

这些命令当前只是规划，不在本次执行。
