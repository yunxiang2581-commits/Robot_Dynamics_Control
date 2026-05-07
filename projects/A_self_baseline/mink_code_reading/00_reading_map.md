# 00 - mink 源码阅读地图

## 本节目标

建立 mink 的整体源码地图，先知道每个模块负责什么，再进入具体文件。

这一步存在的原因是：mink 不是一个单独的 IK 函数，而是一组围绕 MuJoCo configuration、task、limit 和 QP solver 组织起来的抽象。先看地图，可以避免一开始就陷入某个函数细节。

## 推荐源码入口

- `external/mink_upstream/src/mink/__init__.py`
- `external/mink_upstream/src/mink/configuration.py`
- `external/mink_upstream/src/mink/solve_ik.py`
- `external/mink_upstream/src/mink/tasks/`
- `external/mink_upstream/src/mink/limits/`
- `external/mink_upstream/examples/arm_ur5e.py`

## 核心模块关系

```text
examples/arm_ur5e.py
  创建 MuJoCo model / data / viewer target
  创建 mink.Configuration
  创建 tasks 和 limits
  循环调用 solve_ik

Configuration
  持有 model、data、q
  负责更新 MuJoCo data
  提供 frame/site pose 和 Jacobian 查询
  提供 integrate_inplace 更新 q

Task
  描述一个目标
  计算当前误差
  计算任务 Jacobian
  生成 QP objective 的一部分

Limit
  描述一个约束
  根据当前 configuration 生成 QP inequality / bound

solve_ik
  收集所有 task objective
  收集所有 limit constraints
  调用 QP solver
  返回关节速度 dq
```

## 第一轮只需要理解的问题

1. mink 里“状态”由谁管理？
2. mink 里“目标”由谁表达？
3. mink 里“限制”由谁表达？
4. `solve_ik` 的输入和输出分别是什么？
5. UR5e 示例为什么看起来很短，却能完成完整 IK loop？

## 和 A 项目的关系

A 项目当前已经在 `scripts/04_ik_dls_wrapper.py` 和 `scripts/05_ik_qp_wrapper.py` 中做了教学版实现。

可以先建立这个对应关系：

| mink 概念 | A 项目对应位置 |
| --- | --- |
| `Configuration` | `02_configuration_site_pose.py`、`03_site_jacobian_check.py` |
| `FrameTask` | `04_ik_dls_wrapper.py`、`05_ik_qp_wrapper.py` |
| `PostureTask` | `05_ik_qp_wrapper.py` |
| `ConfigurationLimit` / `VelocityLimit` | `05_ik_qp_wrapper.py`、`configs/qp_ik.yaml` |
| `solve_ik` | `04_ik_dls_wrapper.py`、`05_ik_qp_wrapper.py` |

## TODO 学习笔记

- TODO 1: 用自己的话画出 mink 的一次 IK 循环。
- TODO 2: 写出 `Configuration -> Task -> Limit -> solve_ik -> integrate` 的数据流。
- TODO 3: 标记 A 项目里已经实现的部分和尚未实现的部分。

## 详细学习任务与路径

### 任务 0.1: 找到 mink 对外 API

- 阅读源码:
  - `source_annotated/src/mink/__init__.py`
- 目标:
  - 看清楚示例里 `mink.Configuration`、`mink.FrameTask`、`mink.solve_ik` 是从哪里导出的。
- 操作:
  - 把 `__init__.py` 中导出的类和函数分成 4 类: state、task、limit、solver。
- 验收:
  - 能回答 “用户写 `import mink` 后，主要能拿到哪些对象？”

### 任务 0.2: 画出最小 IK 数据流

- 阅读源码:
  - `source_annotated/examples/arm_ur5e.py`
  - `source_annotated/src/mink/solve_ik.py`
- 目标:
  - 先不看细节，只看对象如何流动。
- 操作:
  - 写出一条链:
    `model -> configuration -> tasks/limits -> solve_ik -> vel -> integrate_inplace`
- 验收:
  - 能用 1 分钟口头解释 mink 每一帧做了什么。

### 任务 0.3: 建立第一轮/第二轮阅读边界

- 第一轮必须读:
  - `examples/arm_ur5e.py`
  - `configuration.py`
  - `task.py`
  - `frame_task.py`
  - `posture_task.py`
  - `velocity_limit.py`
  - `configuration_limit.py`
  - `solve_ik.py`
- 第二轮再读:
  - `collision_avoidance_limit.py`
  - `relative_frame_task.py`
  - `com_task.py`
  - `damping_task.py`
  - `kinetic_energy_regularization_task.py`
  - `lie/se3.py`
  - `lie/so3.py`
- 验收:
  - 不会一上来陷入 collision avoidance 或 Lie group 细节。

