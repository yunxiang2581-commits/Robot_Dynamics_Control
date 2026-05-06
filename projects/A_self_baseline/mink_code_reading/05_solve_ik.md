# 05 - solve_ik 源码拆解

## 本节目标

阅读 `external/mink_upstream/src/mink/solve_ik.py`。

`solve_ik` 是 mink 的核心求解入口。它把 configuration、tasks、limits、dt 和 solver 参数组合起来，构造 QP 并返回关节速度 `dq`。

## 对应源码

- `external/mink_upstream/src/mink/solve_ik.py`
- 可辅助阅读: `external/mink_upstream/tests/test_solve_ik.py`

## 为什么需要 solve_ik

前面的模块分别回答了：

- `Configuration`: 当前机器人在哪里？
- `Task`: 想让机器人做什么？
- `Limit`: 机器人不能违反什么边界？

`solve_ik` 要回答的是：

```text
在当前状态下，下一小步应该给哪个关节速度 dq？
```

## 推荐阅读顺序

1. 看 `solve_ik` 的函数签名，列出所有输入。
2. 找到 task objective 的构造过程。
3. 找到 limit constraints 的构造过程。
4. 找到 QP solver 被调用的位置。
5. 找到返回值如何从 solver 结果转换成 `dq`。
6. 看异常处理：不可行、求解失败、维度不匹配时如何处理。

## QP-IK 的数学形态

第一轮可以用这个简化形式理解：

```text
minimize    0.5 * dq.T @ H @ dq + c.T @ dq
subject to  G @ dq <= h
            lb <= dq <= ub
```

其中：

- task 主要贡献 objective，也就是 `H` 和 `c`。
- limit 主要贡献 constraint，也就是 inequality 或 bound。
- solver 返回当前最优的 `dq`。

## 需要特别关注的点

### 多 task 如何合并

多个 task 会共同影响 objective。阅读时注意：

- 每个 task 是否独立生成 objective。
- task weight / cost 如何进入矩阵。
- 多 task 冲突时是否有严格优先级。

### limit 如何进入 QP

阅读时注意：

- limit 是否全部是 inequality。
- box bound 和一般 inequality 是否分开处理。
- `dt` 在 limit 中如何使用。

### 返回 velocity 而不是 q

mink 的 differential IK 返回的是速度 `dq`，然后由 configuration 或外部 loop 积分。

这样做的好处是：

- 适合实时控制循环。
- 可以自然加入速度限制。
- 可以每一小步重新线性化 task 和 limit。

## 关键阅读问题

1. `solve_ik` 是否直接修改 configuration？
2. QP 的变量维度是 `nq` 还是 `nv`？
3. task objective 如何转换成 QP 的 `H` 和 `c`？
4. limit constraint 如何转换成 QP 的约束？
5. solver 失败时调用者应该如何处理？

## 和 A 项目的关系

A 项目的 `05_task_limit_qp_ik.py` 已经实现了最小 QP-IK 教学版。

阅读 mink 的 `solve_ik.py` 时，可以重点比较：

- A05 是否把 position task 和 posture task 合并成同一个 objective。
- A05 是否使用 box bound 表达 velocity / position limit。
- A05 是否处理了更一般的 inequality。
- A05 的 solver 失败策略是否足够清楚。

## TODO 学习笔记

- TODO 1: 写出 mink `solve_ik` 的输入列表和输出。
- TODO 2: 写出 QP 变量维度为什么是 `nv`。
- TODO 3: 记录 task objective 如何累加。
- TODO 4: 记录 limit constraints 如何合并。
- TODO 5: 对比 A05 的 QP 构造和 mink 的 QP 构造。

## 详细学习任务与路径

### 任务 5.1: 读函数签名

- 阅读源码:
  - `source_annotated/src/mink/solve_ik.py`
- 重点函数:
  - `build_ik`
  - `solve_ik`
- 学习目标:
  - 明确 IK 求解入口需要哪些输入。
- 操作:
  - 列出 `configuration`、`tasks`、`dt`、`solver`、`damping`、`limits`、`constraints` 的含义。
- 验收:
  - 能写出 `solve_ik` 的输入输出表。

### 任务 5.2: 跟踪 task objective

- 阅读源码:
  - `_compute_qp_objective`
  - `Task.compute_qp_objective`
- 学习目标:
  - 理解多个 task 如何合成一个 QP objective。
- 操作:
  - 记录初始 `H` 和 `c` 是什么。
  - 记录每个 task 的 `H_task`、`c_task` 如何累加。
- 验收:
  - 能写出 “task 主要贡献 H 和 c”。

### 任务 5.3: 跟踪 limit inequalities

- 阅读源码:
  - `_compute_qp_inequalities`
  - `VelocityLimit.compute_qp_inequalities`
  - `ConfigurationLimit.compute_qp_inequalities`
- 学习目标:
  - 理解多个 limit 如何合成约束矩阵。
- 操作:
  - 记录 `G_list`、`h_list` 如何堆叠。
  - 记录没有 active limit 时返回什么。
- 验收:
  - 能写出 “limit 主要贡献 G 和 h”。

### 任务 5.4: 理解 equality constraints

- 阅读源码:
  - `_compute_qp_equalities`
- 学习目标:
  - 区分普通 task 和必须精确满足的 constraint。
- 操作:
  - 记录 `A`、`b` 如何从 task Jacobian 和 error 得到。
- 验收:
  - 能说明 equality constraint 和 weighted task 的区别。

### 任务 5.5: 跟踪 solver 返回值

- 阅读源码:
  - `solve_ik`
- 学习目标:
  - 理解 `delta_q` 和 velocity 的关系。
- 操作:
  - 找出 `qpsolvers.solve_problem`。
  - 记录 `delta_q = result.x`。
  - 记录 `v = delta_q / dt`。
- 验收:
  - 能解释为什么 `configuration.integrate_inplace(vel, dt)` 能和 `solve_ik` 接上。
