# 01 - UR5e 示例入口拆解

## 本节目标

阅读 `external/mink_upstream/examples/arm_ur5e.py`，从一个完整示例理解 mink 的使用方式。

这一步的重点不是马上读懂 mink 内部实现，而是先看清楚用户层代码如何组织：

- 加载 MuJoCo 模型。
- 创建 configuration。
- 定义末端任务。
- 定义限制条件。
- 在仿真循环中调用 `solve_ik`。
- 把求得的 `dq` 积分回 configuration。

## 对应源码

- `external/mink_upstream/examples/arm_ur5e.py`
- `external/mink_upstream/examples/arm_ur5e_actuators.py`

## 建议阅读顺序

1. 先看 import，确认示例从 mink 中用了哪些类和函数。
2. 找到 MuJoCo model 的加载路径。
3. 找到 `Configuration` 创建位置。
4. 找到 task 列表，重点看末端 frame/site 的任务。
5. 找到 limit 列表，重点看 joint position / velocity 相关限制。
6. 找到主循环中调用 `mink.solve_ik` 的位置。
7. 找到 configuration 被更新的位置。

## 需要特别关注的概念

### Configuration

示例中不会手动到处传 `model`、`data`、`q`，而是把它们收进 `Configuration`。

阅读时注意：

- 当前 q 从哪里来？
- 每次循环 q 是否变化？
- 更新 q 后是否刷新了 MuJoCo data？

### Task

UR5e 示例的核心目标是让末端跟踪一个目标 pose。

阅读时注意：

- 任务目标是谁？
- 当前末端 pose 从哪里查？
- task error 是 position、orientation，还是二者都有？

### Limit

limit 让 IK 不只是“尽快追目标”，还要满足物理和数值边界。

阅读时注意：

- 速度限制和位置限制分别在哪里定义？
- limit 是在 example 里手动判断，还是交给 `solve_ik` 组装？

### solve_ik

示例中真正求解 `dq` 的入口通常很短。

阅读时注意：

- `solve_ik` 输入了哪些对象？
- 返回值是什么？
- 返回值如何作用到 simulation / configuration？

## 阅读问题

1. UR5e 示例里的 target 是如何定义的？
2. 示例中哪些代码属于 MuJoCo，哪些代码属于 mink？
3. 如果没有 `Configuration`，示例代码会多出哪些重复逻辑？
4. `solve_ik` 为什么返回的是速度而不是直接返回目标 q？
5. actuator 示例和普通 IK 示例的主要区别是什么？

## TODO 学习笔记

- TODO 1: 记录 `arm_ur5e.py` 的主循环伪代码。
- TODO 2: 列出示例中所有 task 和 limit。
- TODO 3: 写出 `dq` 从求解到应用的完整路径。
- TODO 4: 和 A 项目的 `06_target_viewer_wrapper.py`、`07_actuator_wrapper.py` 做对应。

## 详细学习任务与路径

### 任务 1.1: 读 import 和资源路径

- 阅读源码:
  - `source_annotated/examples/arm_ur5e.py`
- 目标:
  - 确认这个示例依赖哪些库，模型 XML 从哪里来。
- 操作:
  - 记录 `_XML` 指向哪个 `scene.xml`。
  - 标记哪些 import 属于 MuJoCo，哪些属于 mink。
- 验收:
  - 能说清楚 “模型输入是什么”。

### 任务 1.2: 找到 task 和 limit 定义

- 阅读源码:
  - `source_annotated/examples/arm_ur5e.py`
- 目标:
  - 看懂示例中 “目标” 和 “约束” 如何声明。
- 操作:
  - 列出 `tasks = [...]` 中的每个 task。
  - 列出 `limits = [...]` 中的每个 limit。
  - 记录每个对象传入的关键参数。
- 验收:
  - 能区分 `FrameTask`、`PostureTask`、`ConfigurationLimit`、`VelocityLimit` 分别管什么。

### 任务 1.3: 拆一帧主循环

- 阅读源码:
  - `source_annotated/examples/arm_ur5e.py`
- 目标:
  - 理解实时循环中每一步的顺序。
- 操作:
  - 写出主循环伪代码:
    1. 从 mocap target 读目标 pose。
    2. 设置 end-effector task target。
    3. 调用 `solve_ik` 得到 `vel`。
    4. 调用 `integrate_inplace` 更新 configuration。
    5. 刷新 MuJoCo 可视化。
- 验收:
  - 能解释为什么 target 每帧更新，configuration 也每帧更新。

### 任务 1.4: 对比 actuator 示例

- 阅读源码:
  - `source_annotated/examples/arm_ur5e_actuators.py`
- 目标:
  - 理解 “求 IK” 和 “驱动 actuator” 是两个层次。
- 操作:
  - 找出 actuator 示例中 control input 设置在哪里。
  - 和普通 `arm_ur5e.py` 对比末尾执行部分。
- 验收:
  - 能说清楚 A06 target tracking 和 A07 actuator tracking 的区别。

