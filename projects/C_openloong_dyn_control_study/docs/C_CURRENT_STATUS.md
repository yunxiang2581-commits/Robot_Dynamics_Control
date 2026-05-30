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
- 尚未实现 `C01_contact_force_allocation_demo`。
- 尚未运行仿真。
- 尚未导出视频 demo。
- 尚未生成 metrics。

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

2026-05-30 预检结论：

- 外部源码 commit 为 `4dd7a7e4`。
- 当前系统是 Ubuntu 25.10，`g++` 为 15.2.0，不是官方推荐的 Ubuntu 22.04 / g++ 11。
- 当前环境缺少 `cmake`，`gcc-11/g++-11` 也未找到。
- `sudo apt-get update` 需要用户输入密码，Codex 无法代输。
- 外部源码 `git status` 有大量 `M`，抽样看主要是 LF/CRLF 行尾变化；第一阶段仍按“外部源码只读”处理。

因此，当前立即下一步是用户先安装官方构建依赖，然后按 `docs/10_official_reproduction_runbook.md` 从构建命令继续。

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
- 不编译、不运行外部仓库 demo。
