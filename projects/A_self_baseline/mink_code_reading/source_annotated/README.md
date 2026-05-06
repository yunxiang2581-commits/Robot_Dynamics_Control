# mink 源码学习副本

本目录是 `external/mink_upstream` 的局部源码学习副本，只用于阅读和加中文注释。

请注意：

- 这里不是可安装的正式 mink 包。
- 这里的中文注释服务于学习，不保证和上游保持同步。
- 如果要运行 mink 示例，优先运行 `external/mink_upstream` 中的原始代码。
- 如果要做 A 项目实现，优先修改 `projects/A_self_baseline/scripts/` 或 `src/robot_baseline/`，不要把这里当成业务代码。

## 复制范围

- `examples/arm_ur5e.py`
- `examples/arm_ur5e_actuators.py`
- `src/mink/configuration.py`
- `src/mink/solve_ik.py`
- `src/mink/tasks/*.py`
- `src/mink/limits/*.py`
- `src/mink/constants.py`
- `src/mink/exceptions.py`
- `src/mink/utils.py`
- `src/mink/__init__.py`
- `src/mink/lie/`
- `src/mink/contrib/`
- `src/mink/py.typed`

## 阅读顺序

建议先读：

1. `examples/arm_ur5e.py`
2. `src/mink/configuration.py`
3. `src/mink/tasks/frame_task.py`
4. `src/mink/tasks/posture_task.py`
5. `src/mink/limits/velocity_limit.py`
6. `src/mink/limits/configuration_limit.py`
7. `src/mink/solve_ik.py`

复杂模块如 `collision_avoidance_limit.py` 可以放到第二轮阅读。
`src/mink/lie/` 是理解 pose error、SE3/SO3 差分和 Jacobian 修正时会用到的数学工具层。
