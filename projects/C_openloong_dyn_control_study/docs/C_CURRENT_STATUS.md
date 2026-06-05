# Project C 当前状态

Project C 的主目录是：

```text
projects/C_openloong_dyn_control_study/
```

外部源码参考路径是：

```text
external/open_source_repos/OpenLoong-Dyn-Control/
```

## 1. 项目定位

C 是当前第二优先级项目，也是 Project B 的 OpenLoong 人形 MPC 原型之后的完整控制链学习项目。

目标是从 OpenLoong-Dyn-Control 学习人形机器人 MPC + WBC + PVT + MuJoCo 闭环架构，并抽象为 simulation-only 的简化人形 / 双足 / WBC demo。

当前 BCD 路线为：

```text
B: OpenLoong-oriented humanoid MPC prototype
C: OpenLoong MPC-WBC-PVT full control-chain study
D: legged NMPC-WBC-contact-estimation generalization
```

C 不替代 B 的 MPC 概念验证；C 负责把 B 中形成的 MPC target / contact schedule / residual 思想，接到 WBC-QP、PVT / PD 和 MuJoCo 闭环。

## 2. 当前状态

- 项目骨架已完成。
- `docs/`、`notes/`、`simulator/`、`outputs/` 已规划。
- 外部参考仓库已经 clone 到本地并通过 `.gitignore` 隔离。
- 已完成官方工程复现路径审计和 Docker Ubuntu 22.04 构建复现记录。
- 已完成 `wbc_speed_test` runtime smoke 记录：non-viewer WBC/PVT benchmark 可运行，生成 10000 行 datalog。
- 已完成 `C05A wbc_speed_test` 源码追踪：固定输入 benchmark 链路已经清楚。
- 已完成 `C05B walk_wbc` 主循环源码追踪：MuJoCo -> StateEst -> Pin_KinDyn -> WBC -> PVT -> MuJoCo torque 闭环已经清楚。
- 尚未实现 `C01_contact_force_allocation_demo`。
- 尚未运行 `walk_wbc` GUI。
- 尚未导出视频 demo。
- 尚未生成 Project C 自己的 metrics。

## 3. 下一步

C 的下一步不是直接跑完整 OpenLoong demo，而是：

1. 阅读 OpenLoong 的 MPC-WBC-PVT 数据流。
2. 抽象 command -> gait/contact schedule -> MPC target -> WBC-QP -> MuJoCo control 的链路。
3. 整理 `C01_contact_force_allocation_demo` 的数学模型。
4. 后续再实现简化双足接触力分配 QP。

工程复现路径已经单独整理到：

```text
docs/09_engineering_reproduction_audit.md
docs/10_official_reproduction_runbook.md
```

2026-05-30 到 2026-06-03 阶段结论：

- 外部源码 commit 为 `4dd7a7e4`。
- 当前系统是 Ubuntu 25.10，`g++` 为 15.2.0，不是官方推荐的 Ubuntu 22.04 / g++ 11。
- 已通过 Docker Ubuntu 22.04 + g++ 11 路线完成官方工程构建复现记录。
- 已完成 `wbc_speed_test` non-viewer runtime smoke。
- 当前仍按“外部源码只读”处理，不修改官方仓库。

因此，当前立即下一步不是安装依赖或直接跑 `walk_mpc_wbc`，而是继续做源码理解：

1. `C05C MJ_Interface / StateEst / contact force trace`
2. `C06 walk_wbc GUI/runtime observation`
3. Project C simplified simulator 接口抽象

在 Project B 完成 `B04_openloong_model_mpc_setup` 和 `B05_openloong_standing_balance_mpc` 后，C 可以优先复用这些概念输入：

- desired base state。
- desired pelvis / CoM target。
- desired torso orientation。
- desired foot target。
- optional contact schedule。
- optional desired contact force。

## 4. 边界

- 只做 simulation-only。
- 不做实物部署。
- 不做真实机器人安全测试。
- 不修改 `external/open_source_repos/OpenLoong-Dyn-Control/`。
- 不在未规划 GUI / X11 / OpenGL / 录屏方案前直接运行 viewer demo。
