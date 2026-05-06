# Project D 概览：legged_control

## 项目定位

`legged_control` 是四足机器人控制栈，基于 OCS2 和 ros-control，包含 NMPC、WBC、state estimation、sim2real。

重要提醒：项目 README 明确提示该项目已经不再维护，因此原项目当前只作为学习资料，不作为长期工程依赖。Project D 自身的最终交付仍然要形成 simulation-only 的简化四足仿真器或离线算法 demo。

## 已从公开资料确认的信息

- README 明确项目不再维护。
- README 描述它是基于 OCS2 和 ros-control 的 NMPC-WBC legged robot control stack。
- README 框架说明：
  - 目标速度或目标位姿转换为 torso state trajectory。
  - NMPC 计算 optimized system state and input。
  - WBC 根据 NMPC 输出计算 joint torques。
  - torque 作为 feed-forward 发送到底层电机控制器。
  - 低增益 joint-space position / velocity PD 用于减小触地冲击并提升跟踪。
  - 使用 IMU、电机反馈、足端位置和 Kalman filter 估计 base position / velocity。
- OCS2 文档确认它面向 switched systems optimal control，并支持 SLQ、iLQR、SQP、IPM、SLP 等算法。

## 项目具体功能

- 接收目标速度或目标位姿。
- 转换成 torso state trajectory。
- NMPC 计算最优状态和输入。
- WBC 根据 NMPC 输出计算关节力矩。
- 关节力矩作为 feed-forward。
- 叠加低增益 joint-space PD。
- 基于 IMU、电机反馈、足端位置和 Kalman filter 估计 base 状态。
- 支持 Unitree A1 等四足平台示例。

## 待后续源码阅读确认的信息

- 各 ROS package 的准确职责边界。
- NMPC dynamics、cost、constraints 的源码实现位置。
- WBC-QP 的矩阵构造方式。
- state estimation 中 Kalman filter 状态量和观测量定义。
- sim2real 参数与仿真参数差异。

## 适合复现的模块

- NMPC 问题定义笔记。
- WBC 决策变量 `qddot`、contact force、tau 的整理。
- 简化 contact force QP。
- OCS2 optimal control problem 的 dynamics / cost / constraints 组织方式。

## 只适合阅读的模块

- ROS controller manager 流程。
- Unitree hardware interface。
- Gazebo 仿真集成。
- 真机部署。

## 暂不建议执行的模块

- 安装 ROS / OCS2 全套环境。
- 编译 `legged_control`。
- 运行 Gazebo demo。
- 接入真实机器人硬件。

## 仿真模拟器交付要求

- runnable simulator：后续在 `simulator/` 中实现。
- simulation video demo：保存到 `outputs/videos/`。
- video export script：规划为 `simulator/record_video.py`。
- reproducible command：规划为 `python simulator/run_demo.py --demo <demo_name> --export-video`。
- demo metrics：保存到 `outputs/metrics/`。
