# 04 - Limit 源码拆解

## 本节目标

阅读 `external/mink_upstream/src/mink/limits/`。

Limit 是 mink 中“约束”的表达方式。它让 IK 求解不只是最小化 task error，还要满足关节位置、速度、碰撞等限制。

## 对应源码

- `external/mink_upstream/src/mink/limits/limit.py`
- `external/mink_upstream/src/mink/limits/configuration_limit.py`
- `external/mink_upstream/src/mink/limits/velocity_limit.py`
- `external/mink_upstream/src/mink/limits/collision_avoidance_limit.py`
- 可辅助阅读: `external/mink_upstream/tests/test_configuration_limit.py`
- 可辅助阅读: `external/mink_upstream/tests/test_velocity_limit.py`
- 可辅助阅读: `external/mink_upstream/tests/test_collision_avoidance_limit.py`

## 为什么需要 Limit

无约束 IK 可能会得到很大的 `dq`，或者把关节推到物理边界外。QP-IK 的重要升级是：

```text
在尽量完成 task 的同时，满足约束。
```

常见约束包括：

- 关节速度上限。
- 积分后的关节位置不超过上下限。
- 与障碍物保持距离。
- 冻结某些自由度。

## 推荐阅读顺序

1. 先读 base `Limit`，看 limit 要向 solver 提供什么。
2. 读 `VelocityLimit`，这是最容易理解的 box bound。
3. 读 `ConfigurationLimit`，理解为什么约束的是积分后的 q。
4. 最后粗读 `CollisionAvoidanceLimit`，先理解输入输出，不追求一次读完全部细节。

## VelocityLimit 重点

阅读时关注：

- 速度上限如何配置。
- 对每个 dof 生成的 lower / upper bound 是什么。
- bound 是否和 `dt` 有关。

## ConfigurationLimit 重点

阅读时关注：

- MuJoCo model 里的 joint range 如何读取。
- 当前 q 离上下限还有多少 margin。
- 如何把位置限制转成对 `dq` 的限制。
- 为什么有些 joint 不能简单按 qpos 下标处理。

## CollisionAvoidanceLimit 重点

第一轮只需要知道：

- 它需要哪些 geometry pair。
- 它如何根据距离生成 inequality。
- 它为什么比 position / velocity limit 复杂得多。

## 关键阅读问题

1. limit 最终给 QP solver 的是什么矩阵或向量？
2. velocity limit 和 configuration limit 有什么区别？
3. position limit 为什么需要 `dt`？
4. 如果某个 joint 没有 range，mink 如何处理？
5. collision avoidance 是硬约束还是软约束？

## 和 A 项目的关系

A 项目当前 A05 的重点是最小 box-constrained QP-IK：

- 速度限制。
- 关节位置限制。
- 暂不完整实现 collision avoidance。

这和 mink 的学习顺序一致：先理解关节边界，再进入碰撞约束。

## TODO 学习笔记

- TODO 1: 写出 velocity limit 如何变成 `dq_min <= dq <= dq_max`。
- TODO 2: 写出 configuration limit 如何根据当前位置和 joint range 推出 `dq` bound。
- TODO 3: 标记 collision avoidance 需要哪些 MuJoCo geometry 信息。
- TODO 4: 对比 A05 的 limit 实现和 mink 的完整实现差异。

## 详细学习任务与路径

### 任务 4.1: 先读 Limit 基类

- 阅读源码:
  - `source_annotated/src/mink/limits/limit.py`
- 学习目标:
  - 理解 limit 统一输出什么。
- 操作:
  - 找出 `Constraint` 数据结构。
  - 找出 `compute_qp_inequalities` 接口。
- 验收:
  - 能说清楚为什么 limit 最终要变成 `G dq <= h`。

### 任务 4.2: 拆 VelocityLimit

- 阅读源码:
  - `source_annotated/src/mink/limits/velocity_limit.py`
- 学习目标:
  - 理解速度上限如何变成当前一步的 `delta_q` 上下界。
- 操作:
  - 记录 joint name 如何变成 dof index。
  - 记录 `dt` 如何进入限制。
  - 写出不等式:
    `-v_max * dt <= delta_q <= v_max * dt`
- 验收:
  - 能和 A05 `velocity_limit` 配置对应起来。

### 任务 4.3: 拆 ConfigurationLimit

- 阅读源码:
  - `source_annotated/src/mink/limits/configuration_limit.py`
- 学习目标:
  - 理解位置限制不是限制当前 q，而是限制下一步积分后的 q。
- 操作:
  - 找出 MuJoCo joint range 的读取位置。
  - 找出 lower/upper 如何转成 `delta_q` bound。
  - 注意 free joint 如何跳过。
- 验收:
  - 能解释 position limit 和 velocity limit 的数学差异。

### 任务 4.4: 粗读 CollisionAvoidanceLimit

- 阅读源码:
  - `source_annotated/src/mink/limits/collision_avoidance_limit.py`
- 学习目标:
  - 第一轮只理解它需要哪些输入。
- 操作:
  - 找出 geometry pair、distance、normal、Jacobian 相关变量。
  - 不要求一次读懂完整不等式。
- 验收:
  - 能说出为什么 collision avoidance 比 joint limit 难。

### 任务 4.5: 对照 A05

- 阅读 A 项目:
  - `projects/A_self_baseline/scripts/05_ik_qp_wrapper.py`
  - `projects/A_self_baseline/configs/qp_ik.yaml`
- 学习目标:
  - 明确 A05 只做最小 box-constrained QP-IK。
- 验收:
  - 写出 A05 当前实现了哪些 limit，暂缓了哪些 limit。

