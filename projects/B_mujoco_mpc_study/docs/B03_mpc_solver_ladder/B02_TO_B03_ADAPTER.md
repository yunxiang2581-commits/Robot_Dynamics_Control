# B02 to B03 Adapter

这份文档专门解释 `simulator/adapters/b02_to_b03_adapter.py` 的职责，以及后续怎么把新的 planner 接到 B03 统一接口里。

## 1. 这份脚本做什么

它做的是“翻译层”：

- B02 负责提供真实 MuJoCo 状态、末端目标、rollout 和 task-space cost。
- B03 负责提供统一的 `MPCProblem -> solve() -> MPCSolution` 接口。
- adapter 负责把 B02 的事实层包装成 B03 solver 可以直接消费的输入。

它不改 B02 的动力学，也不改 B03 的 solver 协议。

## 2. 真实调用顺序

典型路径是：

```text
B02 model/data
-> B02TrackingSnapshot
-> MPCProblem
-> solver.solve(problem)
-> candidate_controls
-> problem.rollout_cost_fn(candidate_controls, problem)
-> B02 env.rollout() + B02 cost
-> MPCSolution
```

如果是 sampling 类 solver，B03 只负责采样和排序；真正的 rollout 和 cost 仍然由 B02 env 完成。

## 3. 逐个函数

`extract_b02_tracking_snapshot()`
: 从 B02 的 `model/data` 读取当前状态，拼成 `state=[q1,q2,dq1,dq2]`，并记录当前末端位置和误差。

`build_b03_problem_from_b02_snapshot()`
: 把快照、未来目标轨迹和配置打包成 B03 的 `MPCProblem`。

`rollout_cost_candidates()`
: 对一批候选控制序列逐条调用 B02 `env.rollout()`，再用末端位置误差、速度正则、力矩正则和 terminal error 计算 cost。

`make_b02_rollout_cost_fn()`
: 返回一个闭包，把 B02 的 env 和配置藏起来，暴露成 B03 统一回调 `rollout_cost_fn(candidate_controls, problem)`。

`B02BaselineSolver`
: 把旧 B02 controller 包装成 B03 的 `BaseMPCSolver` 实现，方便统一比较和日志。

`wrap_b02_controller_as_solver()`
: 一个工厂函数，外部直接写 `solver = wrap_b02_controller_as_solver(controller, env)` 即可。

## 4. 新 planner 怎么接

### 路线 A: 直接做成 B03-native solver

推荐优先走这条。

你需要做的只有：

1. 继承 `BaseMPCSolver`。
2. 实现 `solve(problem, previous_solution=None)`。
3. 返回标准 `MPCSolution`。

如果是 sampling 类 planner，就直接复用：

- `problem.current_state`
- `problem.target_horizon`
- `problem.solver_config`
- `problem.rollout_cost_fn`

这样就不需要碰 adapter。

### 路线 B: 先包装旧接口

如果新的 planner 还在用旧式接口，比如：

```text
compute_control(env, current_state, target_sequence)
```

那就仿照 `B02BaselineSolver` 写一个 wrapper：

1. 把 `MPCProblem.current_state` 转成旧接口需要的状态格式。
2. 把 `MPCProblem.target_horizon` 转成旧接口需要的目标序列。
3. 调旧 planner 的主函数。
4. 把输出包装成 `MPCSolution`。

## 5. 接入检查清单

接新 planner 前，先确认这几项：

- 输入是 `MPCProblem`，不是散落的自定义参数。
- 输出是 `MPCSolution`，不是单独一个控制向量。
- 如果要做 sampling，已经接好 `rollout_cost_fn`。
- 如果要做旧接口包装，先保留原 planner 的行为，再做最小封装。
- 不要让外层 runner 直接依赖 planner 内部细节。

## 6. 你可以先记住的最短公式

```text
B02 提供事实层
-> adapter 变成 MPCProblem
-> B03 planner 产生 candidate_controls
-> rollout_cost_fn 复用 B02 rollout 和 cost
-> B03 返回 MPCSolution
```
