# Local Mink Reference Note

## 本目录用途

本目录只保存 mink 的最小参考材料，用于 A 项目对标 UR5e 示例。

这里的 copied files are reference assets/examples, not our implementation.

## 文件说明

- `examples/arm_ur5e.py`：用于对照 differential IK、task、limit 和 `solve_ik`。
- `examples/arm_ur5e_actuators.py`：用于对照 MuJoCo actuator tracking。
- `README.md`：上游项目说明。
- `LICENSE`：上游许可证。
- `pyproject.toml`：上游依赖和项目元信息参考。

## 边界

- 不直接调用 mink 替代本项目实现。
- 不把 `src/mink/` 复制进来。
- 不把完整 mink 仓库复制进 A 项目。
- A 项目标准实现仍然在 `projects/A_self_baseline/scripts/` 和 `projects/A_self_baseline/src/robot_baseline/`。

如果未来需要深入 task/limit 源码，应单独审计：

- `external/mink_upstream/src/mink/tasks`
- `external/mink_upstream/src/mink/limits`
- `external/mink_upstream/src/mink/solve_ik.py`
