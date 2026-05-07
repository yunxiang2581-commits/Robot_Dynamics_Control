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

C 是当前第二优先级项目。

目标是从 OpenLoong-Dyn-Control 学习人形机器人 MPC + WBC + PVT + MuJoCo 闭环架构，并抽象为 simulation-only 的简化人形 / 双足 / WBC demo。

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

## 4. 边界

- 只做 simulation-only。
- 不做实物部署。
- 不做真实机器人安全测试。
- 不修改 `external/open_source_repos/OpenLoong-Dyn-Control/`。
- 不编译、不运行外部仓库 demo。
