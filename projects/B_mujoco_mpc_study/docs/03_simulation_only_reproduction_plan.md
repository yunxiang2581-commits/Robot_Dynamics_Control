# Project B 仿真-only 复现计划：OpenLoong-oriented Humanoid MPC Simulator

## 最终目标

Project B 最终不是停留在“读过 MJPC 源码”，也不是只完成单关节或二连杆 demo，而是要形成一个面向 OpenLoong 人形模型的 simulation-only MPC 学习原型，并导出可展示的视频 demo。

本项目采用递进路线：

```text
B01-B03：先用最小模型学清楚 MPC 闭环。
B04-B06：再切到 OpenLoong 人形模型，形成最终人形 MPC 成果。
```

最终展示形式必须说明：

1. 读懂了 MJPC / MuJoCo MPC 的 task、residual、rollout、planner 核心思想。
2. 把 MPC 抽象成可解释的数学模块。
3. 在 simulation-only 条件下完成最小可运行仿真器。
4. 导出 mp4 视频 demo。
5. 记录 final error、mean tracking error、max torque、runtime per control step 等指标。
6. 为 Project C 的 OpenLoong MPC-WBC-PVT 控制链提供清晰的 MPC target / contact schedule / residual 设计参考。

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
├── envs/                 # TODO: MuJoCo 单关节、二连杆、OpenLoong 人形仿真环境
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

## B04_openloong_model_mpc_setup

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

- `outputs/logs/B04_openloong_model_summary.txt`
- `outputs/metrics/B04_model_dimensions.csv`
- 必要时导出一段静态或被动仿真视频。

评价指标：

- `nq`、`nv`、actuator 数量。
- 关键 body / site 是否找到。
- 初始状态是否能稳定加载并 step。

## B05_openloong_standing_balance_mpc

目标：

- 第一版人形 MPC 成果。
- 在 OpenLoong MuJoCo 模型上做站立平衡 / 姿态保持。
- 重点理解 floating-base humanoid 的 residual、horizon cost 和 receding horizon control。

输入：

- 当前 `qpos`、`qvel`。
- 目标 pelvis 高度、torso 姿态、双脚接触状态。
- horizon、候选控制数量、torque limit、cost 权重。

输出：

- `outputs/videos/B05_openloong_standing_balance_mpc.mp4`
- `outputs/figures/B05_balance_errors.png`
- `outputs/logs/B05_balance_run_log.txt`
- `outputs/metrics/B05_balance_metrics.csv`

评价指标：

- pelvis height error。
- torso orientation error。
- foot contact summary。
- mean tracking error。
- max torque。
- runtime per control step。

## B06_openloong_weight_shift_or_stepping_mpc

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

- `outputs/videos/B06_openloong_weight_shift_or_stepping_mpc.mp4`
- `outputs/figures/B06_shift_or_step_errors.png`
- `outputs/logs/B06_shift_or_step_run_log.txt`
- `outputs/metrics/B06_shift_or_step_metrics.csv`

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
python simulator/run_demo.py --demo B04_openloong_model_mpc_setup
python simulator/run_demo.py --demo B05_openloong_standing_balance_mpc --export-video
python simulator/run_demo.py --demo B06_openloong_weight_shift_or_stepping_mpc --export-video
```

这些命令当前只是规划，不在本次执行。
