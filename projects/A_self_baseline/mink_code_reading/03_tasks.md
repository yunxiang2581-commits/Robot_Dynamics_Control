# 03 - Task 源码拆解

## 本节目标

阅读 `external/mink_upstream/src/mink/tasks/`。

Task 是 mink 中“目标”的表达方式。它把一个控制目标转换成 QP objective 需要的误差和 Jacobian。

## 对应源码

- `external/mink_upstream/src/mink/tasks/task.py`
- `external/mink_upstream/src/mink/tasks/frame_task.py`
- `external/mink_upstream/src/mink/tasks/posture_task.py`
- 可辅助阅读: `external/mink_upstream/tests/test_frame_task.py`
- 可辅助阅读: `external/mink_upstream/tests/test_posture_task.py`

## 为什么需要 Task

IK 的核心不是“移动关节”，而是让任务空间目标变好。例如：

- 末端位置接近目标点。
- 末端姿态接近目标姿态。
- 整体姿态接近参考关节角。
- 某些自由度保持稳定。

Task 的作用是把这些目标统一写成：

```text
minimize || J(q) * dq - desired_task_velocity ||^2
```

或者等价的 QP objective。

## 推荐阅读顺序

1. 先读 base `Task`，看所有 task 需要提供哪些接口。
2. 再读 `FrameTask`，看 position / orientation error 如何组成 6D 残差。
3. 再读 `PostureTask`，看关节姿态参考如何写成 identity Jacobian 形式。
4. 最后看测试文件，理解 expected shape 和异常情况。

## FrameTask 重点

阅读时关注：

- target frame 是什么名字。
- target transform 如何保存。
- 当前 transform 如何从 `Configuration` 查询。
- position error 和 orientation error 如何组合。
- task Jacobian 如何从 `Configuration` 查询。
- cost / gain / damping 等参数如何影响 QP。

## PostureTask 重点

阅读时关注：

- reference q 从哪里设置。
- 当前 q 和 reference q 的误差如何计算。
- 为什么 posture task 的 Jacobian 接近 identity。
- 为什么 posture task 常作为 regularization，而不是主任务。

## 关键阅读问题

1. `FrameTask` 的误差是在 world frame、body frame，还是 local frame 表达？
2. orientation error 使用什么 Lie group / SE3 逻辑？
3. task cost 是如何作用到 position 和 orientation 的？
4. 多个 task 同时存在时，它们如何被合并？
5. posture task 会不会和 frame task 冲突？冲突时由什么决定优先级？

## 和 A 项目的关系

A 项目的 `05_ik_qp_wrapper.py` 已经有教学版 task 结构：

- position-only 或 pose-aware FrameTask。
- PostureTask。
- task weight。
- 通过加权堆叠构造 QP objective。

mink 的 task 抽象更完整，适合阅读它如何把不同任务统一成 solver 可以吃的矩阵形式。

## TODO 学习笔记

- TODO 1: 画出 `FrameTask.compute_error` 的输入和输出。
- TODO 2: 记录 `FrameTask` 中 orientation error 的数学表达。
- TODO 3: 写出 `PostureTask` 对应的误差向量和 Jacobian。
- TODO 4: 对比 A05 里 task 构造方式和 mink 的差异。

## 详细学习任务与路径

### 任务 3.1: 先读 Task 基类

- 阅读源码:
  - `source_annotated/src/mink/tasks/task.py`
- 学习目标:
  - 知道所有 task 必须提供哪些接口。
- 操作:
  - 找出 `compute_error`、`compute_jacobian`、`compute_qp_objective`。
  - 记录 `Objective` 里包含哪些矩阵/向量。
- 验收:
  - 能说清楚 task 如何最终进入 QP。

### 任务 3.2: 拆 FrameTask 目标设置

- 阅读源码:
  - `source_annotated/src/mink/tasks/frame_task.py`
- 重点函数:
  - `__init__`
  - `set_target`
  - `set_target_from_configuration`
- 学习目标:
  - 理解末端目标 pose 如何保存。
- 操作:
  - 记录 `frame_name`、`frame_type`、`transform_target_to_world` 的作用。
  - 对照 UR5e 示例里的 `"attachment_site"`。
- 验收:
  - 能说清楚 target pose 和 current pose 分别从哪里来。

### 任务 3.3: 拆 FrameTask error 和 Jacobian

- 阅读源码:
  - `source_annotated/src/mink/tasks/frame_task.py`
  - 需要时查 `source_annotated/src/mink/lie/se3.py`
- 重点函数:
  - `compute_error`
  - `compute_jacobian`
- 学习目标:
  - 理解 6D task residual 如何建立。
- 操作:
  - 记录 error 的 shape。
  - 记录 Jacobian 的 shape。
  - 暂时把 Lie group 公式当作 “位姿差转 6D 向量” 理解。
- 验收:
  - 能解释 FrameTask 为什么同时需要 error 和 Jacobian。

### 任务 3.4: 拆 PostureTask

- 阅读源码:
  - `source_annotated/src/mink/tasks/posture_task.py`
- 重点函数:
  - `set_target`
  - `compute_error`
  - `compute_jacobian`
- 学习目标:
  - 理解姿态保持 task 如何作为 regularization。
- 操作:
  - 记录 posture error 的 shape。
  - 记录 Jacobian 为什么接近 identity。
  - 观察 freejoint dof 如何处理。
- 验收:
  - 能解释 PostureTask 为什么通常 cost 很小。

### 任务 3.5: 第二轮扩展 task

- 阅读源码:
  - `relative_frame_task.py`
  - `damping_task.py`
  - `dof_freezing_task.py`
  - `com_task.py`
- 学习目标:
  - 只理解用途，不深入推导。
- 验收:
  - 能给每个 task 写一句 “它解决什么控制目标”。

