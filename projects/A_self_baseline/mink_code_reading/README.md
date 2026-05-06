# mink 源码阅读学习区

本目录用于完整拆解和阅读 `external/mink_upstream` 中的 mink 源码。

这里不是新的实现代码区，也不直接修改 mink 上游源码。它的作用是把 mink 的设计拆成可以逐步学习的笔记：

- 先从 UR5e 示例看完整数据流。
- 再拆 `Configuration` 如何管理 MuJoCo model、data 和 q。
- 再拆 task 如何把目标位姿变成误差和 Jacobian。
- 再拆 limit 如何把关节位置、速度、碰撞等限制变成 QP 约束。
- 最后拆 `solve_ik` 如何组装并求解 QP。

## 推荐阅读顺序

1. `00_reading_map.md`
2. `01_ur5e_example_walkthrough.md`
3. `02_configuration.md`
4. `03_tasks.md`
5. `04_limits.md`
6. `05_solve_ik.md`
7. `06_compare_with_A_self_baseline.md`
8. `07_file_role_and_reading_order.md`

## 对应源码位置

- 完整 mink 上游源码: `external/mink_upstream`
- mink Python 包源码: `external/mink_upstream/src/mink`
- UR5e 示例: `external/mink_upstream/examples/arm_ur5e.py`
- UR5e actuator 示例: `external/mink_upstream/examples/arm_ur5e_actuators.py`
- 已复制的中文注释学习副本: `projects/A_self_baseline/mink_code_reading/source_annotated`

## 学习副本说明

`source_annotated/` 中的 Python 文件来自 `external/mink_upstream` 的局部复制。

这些文件只用于阅读：

- 可以加中文注释。
- 可以做阅读标记。
- 不作为 A 项目的运行源码。
- 不替代 `external/mink_upstream` 的原始源码。

## 学习主线

mink 的核心问题可以先用一句话理解：

给定当前机器人 configuration、一组 task-space 目标和若干 limits，求一个局部最优的关节速度 `dq`，然后把 `dq` 积分到新的 configuration。

它和 A 项目的主线关系是：

```text
MuJoCo model inspect
  -> configuration / site pose
  -> site Jacobian
  -> DLS IK
  -> task + limit + QP-IK
  -> target tracking
  -> actuator tracking
```

## 阅读方法

每一篇笔记建议按这个顺序使用：

1. 先读“本节目标”，明确这段源码解决什么问题。
2. 打开对应源码文件，只看当前小节列出的函数或类。
3. 回答“阅读问题”，不要急着改代码。
4. 在 TODO 区写自己的理解。
5. 最后回到 A 项目，比较自己实现和 mink 抽象有什么差异。

如果只是想快速知道某个源码文件的作用，先看 `07_file_role_and_reading_order.md`。

## 总体学习任务与路径

这套阅读建议分成 7 个小阶段，每个阶段都要留下一个可复盘输出。

### 阶段 0: 建立源码地图

- 阅读文件: `00_reading_map.md`
- 对应源码: `source_annotated/src/mink/__init__.py`、`source_annotated/src/mink/solve_ik.py`
- 学习任务:
  - 找出 mink 对外暴露的核心类和函数。
  - 画出 `Configuration -> Task -> Limit -> solve_ik -> integrate` 数据流。
  - 标记第一轮必须读和第二轮再读的源码。
- 输出:
  - 在 `00_reading_map.md` 的 TODO 区补一份自己的源码地图。

### 阶段 1: 从 UR5e 示例读完整闭环

- 阅读文件: `01_ur5e_example_walkthrough.md`
- 对应源码: `source_annotated/examples/arm_ur5e.py`、`source_annotated/examples/arm_ur5e_actuators.py`
- 学习任务:
  - 找到模型加载、task 定义、limit 定义、target 更新、IK 求解、状态积分。
  - 用伪代码写出主循环。
  - 区分普通 IK 示例和 actuator 示例的区别。
- 输出:
  - 写出 “一帧控制循环” 的 8-12 行伪代码。

### 阶段 2: 拆 Configuration

- 阅读文件: `02_configuration.md`
- 对应源码: `source_annotated/src/mink/configuration.py`
- 学习任务:
  - 理解 `model`、`data`、`q`、`nq`、`nv` 的关系。
  - 跟踪 `update()`、frame pose 查询、frame Jacobian 查询、`integrate_inplace()`。
  - 对照 A02/A03/A04 为什么要分开学。
- 输出:
  - 写出 `Configuration` 负责的 5 个能力。

### 阶段 3: 拆 Tasks

- 阅读文件: `03_tasks.md`
- 对应源码: `source_annotated/src/mink/tasks/task.py`、`frame_task.py`、`posture_task.py`
- 学习任务:
  - 理解 task 如何产生 error、Jacobian、QP objective。
  - 重点读 `FrameTask` 的位姿误差和 `PostureTask` 的姿态保持。
  - 暂时跳过过深的 Lie group 推导，只记录接口和输入输出。
- 输出:
  - 写出 FrameTask 和 PostureTask 的输入、输出、矩阵 shape。

### 阶段 4: 拆 Limits

- 阅读文件: `04_limits.md`
- 对应源码: `source_annotated/src/mink/limits/limit.py`、`velocity_limit.py`、`configuration_limit.py`
- 学习任务:
  - 理解 limit 如何变成 `G dq <= h`。
  - 重点读速度限制和位置限制。
  - 碰撞限制只做第一轮结构了解。
- 输出:
  - 写出 velocity limit 和 configuration limit 的区别。

### 阶段 5: 拆 solve_ik

- 阅读文件: `05_solve_ik.md`
- 对应源码: `source_annotated/src/mink/solve_ik.py`
- 学习任务:
  - 跟踪 task objective 如何累加。
  - 跟踪 limit inequalities 如何合并。
  - 理解为什么返回 velocity，而不是直接返回 q。
- 输出:
  - 写出 mink QP-IK 的最小数学形式。

### 阶段 6: 对照 A 项目

- 阅读文件: `06_compare_with_A_self_baseline.md`
- 对应 A 项目文件:
  - `projects/A_self_baseline/scripts/02_configuration_site_pose.py`
  - `projects/A_self_baseline/scripts/03_site_jacobian_check.py`
  - `projects/A_self_baseline/scripts/04_dls_differential_ik.py`
  - `projects/A_self_baseline/scripts/05_task_limit_qp_ik.py`
- 学习任务:
  - 标出 A 项目已经手写实现的 mink 概念。
  - 标出暂时不应该照搬的 mink 抽象。
  - 形成下一轮 A05/A06 学习改进清单。
- 输出:
  - 补全 `06_compare_with_A_self_baseline.md` 的对照表。

## 每天建议节奏

- 第 1 天: 读 `README.md`、`00_reading_map.md`、`01_ur5e_example_walkthrough.md`。
- 第 2 天: 读 `02_configuration.md` 和 `configuration.py`。
- 第 3 天: 读 `03_tasks.md`、`task.py`、`frame_task.py`、`posture_task.py`。
- 第 4 天: 读 `04_limits.md`、`velocity_limit.py`、`configuration_limit.py`。
- 第 5 天: 读 `05_solve_ik.md` 和 `solve_ik.py`。
- 第 6 天: 读 `06_compare_with_A_self_baseline.md`，补对照表。
- 第 7 天: 回头重读 `arm_ur5e.py`，确认整个闭环能从头讲出来。
