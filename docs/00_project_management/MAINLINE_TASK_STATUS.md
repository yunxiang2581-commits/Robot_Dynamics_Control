# Mainline Task Status

# 主线任务状态

## 一、主线目标

当前主线面向机器人运动控制求职，目标是建立一组 simulation-only 的机器人控制算法项目。

最终展示形式不是“我读过源码”，而是：

- 我能把复杂运动控制项目抽象成可解释的数学模块。
- 我能实现 runnable simulator。
- 我能导出 video demo。
- 我能记录 metrics，例如误差、约束违反、控制输入和运行时间。

本项目不做实物部署、不做 sim2real、不接电机、不写硬件接口、不做真实机器人安全测试。

## 二、主线项目关系

- A 是自研基础能力主线。
- B 从 MuJoCo MPC 学习预测控制。
- C 从 OpenLoong 学习人形 MPC/WBC。
- D 从 legged_control 学习四足 NMPC/WBC/状态估计。
- B/C/D 不是替代 A，而是服务于 A 的能力扩展。

关系可以理解为：

```text
A 基础能力 -> B 预测控制 -> C 人形 MPC/WBC -> D 四足 NMPC/WBC/状态估计
```

## 三、当前主线状态

- A 已经具备基础 QP-IK / task tracking / simulation-only 方向。
- B/C/D 已完成骨架和文档规划。
- 外部仓库已 clone 并通过 `.gitignore` 隔离。
- 尚未产出第一个 B01 视频 demo。

当前主线下一步不是继续泛泛整理文档，而是进入 Project B 的 B01。

## 四、近期主线优先级

1. 提交当前状态文档。
2. 创建 B01 TODO 骨架。
3. 实现 B01 单关节 MPC 仿真。
4. 导出 B01 MP4 视频和 metrics。
5. 再进入 B02 二连杆 MPC。
6. 再进入 C01 简化 WBC-QP。

## 五、当前不做

- 不做实机。
- 不做 sim2real。
- 不做硬件接口。
- 不做大规模删除。
- 不继续无限整理文档。
- 不直接运行复杂外部仓库 demo。

本文记录当前主线任务状态。这里的“主线”指整个仓库从 A_self_baseline 到 B/C/D simulation-only 仿真项目的连续学习与求职展示路线。

## 1. 当前主线定位

当前仓库不是单一脚本项目，而是一个分层的机器人运动控制 simulation-only 项目集：

- Project A：自研基础运动控制 baseline。
- Project B：MuJoCo MPC / MJPC 启发的 MPC 仿真复现。
- Project C：OpenLoong-Dyn-Control 启发的人形 MPC/WBC 仿真复现。
- Project D：legged_control 启发的四足 NMPC/WBC/状态估计仿真复现。

当前主线不是实物部署路线，而是：

```text
基础模型理解 -> FK/Jacobian/IK -> QP-IK/task-space tracking -> MPC/WBC 抽象 -> simulation-only demo -> video + metrics
```

## 2. Project A 当前状态

Project A 是主线基础项目，目录为：

```text
projects/A_self_baseline/
```

当前已有脚本入口包括：

| 脚本 | 作用 |
|---|---|
| `00_reference_and_assets.py` | 资源和参考入口 |
| `01_model_inspect.py` | MuJoCo 模型检查 |
| `02_configuration_site_pose.py` | configuration 与 site pose 查询 |
| `03_site_jacobian_check.py` | site Jacobian 检查 |
| `04_ik_dls_wrapper.py` | DLS differential IK |
| `05_ik_qp_wrapper.py` | task/limit QP-IK |
| `06_target_viewer_wrapper.py` | target mocap tracking |
| `07_actuator_wrapper.py` | MuJoCo actuator tracking |
| `08_collision_avoidance_todo.py` | collision avoidance TODO |
| `09_comparison_report.py` | 对比报告 |
| `10_demo_showcase_video.py` | demo showcase video |
| `11_view_ur5e_x11.py` | UR5e 可视化入口 |
| `12_mujoco_visualization_hub.py` | MuJoCo visualization hub |

Project A 当前价值：

- 提供 MuJoCo / Pinocchio / QP-IK / task-space tracking 基础。
- 为 Project B 的 MPC horizon cost 与参考轨迹提供基础对照。
- 为 Project C/D 的 WBC-QP 和 contact force QP 提供 QP 建模经验。

本轮没有修改 Project A 源码。

## 3. Project B/C/D 当前状态

| Project | 当前状态 | 下一步 |
|---|---|---|
| B | 文档、simulator 骨架、outputs 目录和外部源码阅读地图已建立 | 创建 `B01_single_joint_mpc_demo` TODO 骨架 |
| C | 文档、simulator 骨架、outputs 目录和 OpenLoong 阅读地图已建立 | 等 B01 后进入 `C01_contact_force_allocation_demo` 设计 |
| D | 文档、simulator 骨架、outputs 目录和 legged_control 阅读地图已建立 | 等 B01 后进入 `D01_quadruped_contact_qp_demo` 设计 |

B/C/D 当前都还没有实现控制算法、没有运行仿真、没有导出视频 demo。

## 4. docs 当前状态

本轮已将 `docs/` 从“项目细节堆放处”整理为：

```text
docs/
├── README.md
├── 00_project_management/
├── 06_open_source_project_study/
├── archive/
└── interview/
```

历史 `step*.md` 已迁移到：

```text
docs/archive/project_history/
```

旧主题入口已经迁移：

- A 旧入口 -> `projects/A_self_baseline/docs/legacy_migrated/`
- legged_control 旧入口 -> `projects/D_legged_control_study/docs/legacy_migrated/`
- Unitree RL/MJLab 旧入口 -> `projects/archive/future_studies/C_unitree_rl_mjlab_study/`
- 旧 compare 入口 -> `docs/archive/legacy_compare/`
- 早期准备资料 -> `docs/archive/preparation_history/`
- 旧 B legged_control study -> `projects/archive/legacy_studies/B_legged_control_study/`

## 5. 当前禁止事项

- 不做实物部署。
- 不做 sim2real。
- 不接电机，不写电机 SDK。
- 不写 CAN、EtherCAT、串口、固件或硬件接口。
- 不运行外部开源项目 demo。
- 不编译外部开源项目。
- 不修改 `external/open_source_repos/`。

## 6. 当前推荐下一步

1. 先提交当前 README、项目状态、docs 整理和 B/C/D skeleton 相关变更。
2. 创建 `projects/B_mujoco_mpc_study/docs/B01_single_joint_mpc_demo_design.md`。
3. 创建 B01 的学习型 TODO skeleton，不一次性写完整 MPC。
4. 实现 B01 最小 MuJoCo 单关节 MPC 仿真。
5. 导出 B01 MP4 视频和 metrics。
6. 再进入 B02 二连杆 MPC 或 C01/D01 的简化 QP demo。
