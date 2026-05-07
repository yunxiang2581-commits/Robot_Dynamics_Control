# Project B 当前状态

Project B 的主目录是：

```text
projects/B_mujoco_mpc_study/
```

外部源码参考路径是：

```text
external/open_source_repos/mujoco_mpc/
```

## 1. 项目定位

B 是当前第一优先级项目。

目标是从 MuJoCo MPC / MJPC 学习预测控制思想，并抽象成 simulation-only 的可运行仿真器：

- runnable simulator。
- video demo。
- metrics。
- 可复现实验命令。

## 2. 当前状态

- 项目骨架已完成。
- `docs/`、`notes/`、`simulator/`、`outputs/` 已规划。
- 外部参考仓库已经 clone 到本地并通过 `.gitignore` 隔离。
- 尚未实现 `B01_single_joint_mpc_demo`。
- 尚未运行仿真。
- 尚未导出 MP4。
- 尚未生成 metrics。

## 3. 下一步

下一步必须进入：

```text
B01_single_joint_mpc_demo TODO skeleton
```

推荐顺序：

1. 创建 B01 demo 设计文档。
2. 创建 B01 学习型 TODO skeleton。
3. 实现单关节 MuJoCo 模型。
4. 实现最小 MPC horizon cost。
5. 导出 MP4。
6. 输出 final error、mean tracking error、max torque、runtime per control step。

## 4. 近期目标

B 的近期目标不是继续泛泛整理文档，而是产出：

```text
B01 可运行仿真 + MP4 视频 + metrics
```

## 5. 边界

- 不直接修改 `external/open_source_repos/mujoco_mpc/`。
- 不编译外部仓库。
- 不运行外部仓库 demo。
- 不做实物部署。
