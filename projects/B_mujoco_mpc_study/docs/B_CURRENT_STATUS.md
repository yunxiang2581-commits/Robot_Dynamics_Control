# Project B 当前状态

Project B 的主目录是：

```text
projects/B_mujoco_mpc_study/
```

外部源码参考路径是：

```text
external/open_source_repos/mujoco_mpc/
external/open_source_repos/OpenLoong-Dyn-Control/
```

## 1. 项目定位

B 是当前第一优先级项目，也是后续 Project C 的前置学习层。

整体路线采用：

```text
路线 2：先做最小 MPC demo，再升级到 OpenLoong 人形 MPC。
```

目标是先从 MuJoCo MPC / MJPC 学习预测控制思想，再把这些思想迁移到 OpenLoong 人形模型上，形成 simulation-only 的人形 MPC 原型：

- runnable simulator。
- video demo。
- metrics。
- 可复现实验命令。
- OpenLoong-oriented humanoid MPC prototype。

B 的最终成果不再只是单关节或二连杆 demo，而是服务于 C 的人形控制链：

```text
B: state + future target -> rollout -> horizon cost -> MPC target
C: MPC target -> WBC-QP -> PVT / PD -> MuJoCo closed loop
```

## 2. 当前状态

- 项目骨架已完成。
- `docs/`、`notes/`、`simulator/`、`outputs/` 已规划。
- 外部参考仓库已经 clone 到本地并通过 `.gitignore` 隔离。
- `B01_single_joint_mpc_demo` 已完成 smooth target tracking 验收。
- `B02_two_link_mpc_tracking_demo` 已进入 TODO skeleton 阶段。
- 尚未实现 `B04_cart_double_inverted_pendulum_mpc`。
- 尚未实现 OpenLoong 人形模型接入与状态摘要。
- 尚未实现 OpenLoong 人形站立平衡 MPC。
- 已完成 B01 可运行仿真、MP4、figures、metrics CSV 和 run log。
- 已建立 B01-B07 的统一可视化验收输出规范。

### B01 验收记录

B01 当前按 `smooth target tracking` 口径验收通过。该口径表示目标角不是瞬时阶跃，而是从初始角在 `0.5s` 内平滑过渡到 `1.0 rad`，更接近机器人控制中的轨迹跟踪任务。

验收 run：

```text
outputs/runs/B01_single_joint_mpc_demo/20260511_220538/
```

关键配置：

```text
target_angle: 1.0 rad
target_profile: smooth
ramp_duration: 0.5 s
horizon: 10
num_candidates: 128
torque_limit: 1.9 Nm
q_weight: 8.0
dq_weight: 0.1
torque_weight: 0.002
terminal_weight: 1.0
```

验收指标：

```text
final_error: 0.019126844672575194 rad
mean_tracking_error: 0.06666501475904199 rad
max_torque: 1.88158006947223 Nm
runtime_per_control_step: 0.017442599999582552 s
recorded_frames: 300
```

验收产物：

```text
videos/B01_single_joint_mpc_demo.mp4
figures/B01_angle_tracking.png
figures/B01_angle_error.png
figures/B01_torque.png
figures/B01_best_cost.png
metrics/B01_metrics.csv
logs/B01_run_log.txt
```

结论：

```text
B01: ACCEPTED / PASSED for smooth target tracking.
```

说明：

```text
step target 模式保留为压力测试；在 torque_limit=1.9 Nm 下，0 -> 1.0 rad 的瞬时阶跃会导致早期 mean error 和 torque saturation 偏高，不作为 B01 默认验收口径。
```

### B02 启动记录

B02 已开始进入二连杆末端轨迹 tracking 任务。当前只完成学习笔记、配置、模型和教学型 TODO skeleton，不视为完成控制 demo。

已新增入口：

```text
docs/B02_learning_notes.md
configs/B02_two_link_mpc.yaml
simulator/models/B02_two_link.xml
simulator/envs/two_link_env.py
simulator/planners/two_link_predictive_sampling.py
simulator/controllers/two_link_mpc_controller.py
simulator/scripts/run_B02_two_link_mpc_tracking_demo.py
```

B02 的核心学习目标：

```text
q -> p_ee(q) -> p_target(t)
```

也就是从 B01 的角度 residual：

```text
q - q_target(t)
```

扩展到 B02 的 task-space residual：

```text
p_ee(q) - p_target(t)
```

## 3. 下一步

下一步必须进入：

```text
B02_two_link_mpc_tracking_demo TODO skeleton + visualization skeleton
```

推荐大顺序：

1. 以 B01 的输出结构和验收口径为模板，创建 B02 二连杆 tracking 任务。
2. 创建 B02 学习型 TODO skeleton 和 visualization skeleton。
3. 明确二连杆末端轨迹目标、误差定义、torque 限制和 metrics。
4. 完成 B02 后继续 B03 predictive sampling 可视化任务。
5. 插入 B04 小车倒立二阶摆 MPC，学习欠驱动非线性平衡控制。
6. 切到 OpenLoong，完成 B05 人形模型 MPC setup。
7. 完成 B06 OpenLoong standing balance MPC。
8. 再进入 B07 OpenLoong weight shift / stepping MPC。

B01-B03 是基础概念验证；B04 是从简单模型走向人形平衡的欠驱动过渡任务；B05-B07 才是面向最终成果的人形 MPC 主线。

B01 已完成 smooth target tracking 验收；后续 B02-B07 必须沿用同样的 video + figures + metrics + run log 验收规则。


## 4. 近期目标

B 的近期目标不是继续泛泛整理文档，也不是直接跳到复杂人形行走，而是产出：

```text
B02 二连杆 MPC tracking + MP4 视频 + figures + metrics + run log + README 复现命令
```

之后的阶段目标是：

```text
B03 predictive sampling 可视化 demo + MP4 视频 + metrics
B04 小车倒立二阶摆 MPC + MP4 视频 + metrics
B05 OpenLoong 模型状态摘要 + 可运行 MuJoCo 仿真
B06 OpenLoong 站立平衡 MPC + MP4 视频 + metrics
B07 OpenLoong 重心转移或小步踏步 MPC + MP4 视频 + metrics
```

## 5. 边界

- 不直接修改 `external/open_source_repos/mujoco_mpc/`。
- 不编译外部仓库。
- 不运行外部仓库 demo。
- 不做实物部署。
- 不做 sim2real。
- 不接电机 SDK / CAN / EtherCAT / 串口 / 固件。
- 不把 B 的实验代码直接替换 C 的 WBC / PVT 控制链。
- 不接受没有可视化输出、没有 metrics、没有 README 复现命令的任务结果。
