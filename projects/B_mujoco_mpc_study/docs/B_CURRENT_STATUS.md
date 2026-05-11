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
- 尚未实现 `B01_single_joint_mpc_demo`。
- 尚未实现 OpenLoong 人形模型接入与状态摘要。
- 尚未实现 OpenLoong 人形站立平衡 MPC。
- 尚未运行仿真。
- 尚未导出 MP4。
- 尚未生成 metrics。

## 3. 下一步

下一步必须进入：

```text
B01_single_joint_mpc_demo TODO skeleton
```

推荐大顺序：

1. 创建 B01 demo 设计文档。
2. 创建 B01 学习型 TODO skeleton。
3. 实现单关节 MuJoCo 模型。
4. 实现最小 MPC horizon cost。
5. 继续完成 B02 二连杆 tracking 与 B03 predictive sampling。
6. 切到 OpenLoong，完成 B04 人形模型 MPC setup。
7. 完成 B05 OpenLoong standing balance MPC。
8. 再进入 B06 OpenLoong weight shift / stepping MPC。

B01-B03 是概念验证；B04-B06 才是面向最终成果的人形 MPC 主线。

## 4. 近期目标

B 的近期目标不是继续泛泛整理文档，也不是直接跳到复杂人形行走，而是产出：

```text
B01 可运行仿真 + MP4 视频 + metrics
```

之后的阶段目标是：

```text
B04 OpenLoong 模型状态摘要 + 可运行 MuJoCo 仿真
B05 OpenLoong 站立平衡 MPC + MP4 视频 + metrics
B06 OpenLoong 重心转移或小步踏步 MPC + MP4 视频 + metrics
```

## 5. 边界

- 不直接修改 `external/open_source_repos/mujoco_mpc/`。
- 不编译外部仓库。
- 不运行外部仓库 demo。
- 不做实物部署。
- 不做 sim2real。
- 不接电机 SDK / CAN / EtherCAT / 串口 / 固件。
- 不把 B 的实验代码直接替换 C 的 WBC / PVT 控制链。
