# 07 - mink 文件作用与阅读顺序总索引

## 本文件目标

这个文件用于标注 `source_annotated/` 中所有 mink 学习副本文件的作用和简单阅读顺序。

阅读原则：

- 第一轮只读主链路，目标是跑通 `example -> Configuration -> Task -> Limit -> solve_ik`。
- 第二轮读数学工具和扩展 task / limit。
- 第三轮读交互、contrib、C 扩展和类型辅助文件。

## 总体阅读顺序

### 第一轮: 必读主链路

1. `source_annotated/examples/arm_ur5e.py`
2. `source_annotated/src/mink/__init__.py`
3. `source_annotated/src/mink/configuration.py`
4. `source_annotated/src/mink/tasks/task.py`
5. `source_annotated/src/mink/tasks/frame_task.py`
6. `source_annotated/src/mink/tasks/posture_task.py`
7. `source_annotated/src/mink/limits/limit.py`
8. `source_annotated/src/mink/limits/velocity_limit.py`
9. `source_annotated/src/mink/limits/configuration_limit.py`
10. `source_annotated/src/mink/solve_ik.py`

第一轮读完后，应能解释：

- 当前状态由谁管理。
- 末端目标由谁表达。
- 关节限制由谁表达。
- QP 问题在哪里组装。
- 求出的 velocity 如何积分回 configuration。

### 第二轮: 数学和扩展能力

1. `source_annotated/src/mink/lie/base.py`
2. `source_annotated/src/mink/lie/so3.py`
3. `source_annotated/src/mink/lie/se3.py`
4. `source_annotated/src/mink/lie/utils.py`
5. `source_annotated/src/mink/tasks/relative_frame_task.py`
6. `source_annotated/src/mink/tasks/damping_task.py`
7. `source_annotated/src/mink/tasks/dof_freezing_task.py`
8. `source_annotated/src/mink/tasks/equality_constraint_task.py`
9. `source_annotated/src/mink/tasks/com_task.py`
10. `source_annotated/src/mink/tasks/kinetic_energy_regularization_task.py`
11. `source_annotated/src/mink/limits/collision_avoidance_limit.py`

第二轮读完后，应能解释：

- 位姿误差为什么需要 SO3 / SE3。
- 除 FrameTask 和 PostureTask 之外，还有哪些 task 类型。
- collision avoidance 为什么比 joint limit 更复杂。

### 第三轮: 示例、工具和附加功能

1. `source_annotated/examples/arm_ur5e_actuators.py`
2. `source_annotated/src/mink/utils.py`
3. `source_annotated/src/mink/constants.py`
4. `source_annotated/src/mink/exceptions.py`
5. `source_annotated/src/mink/contrib/keyboard_teleop/teleop_mocap.py`
6. `source_annotated/src/mink/contrib/keyboard_teleop/keycodes.py`
7. `source_annotated/src/mink/contrib/keyboard_teleop/KEYBOARD.md`
8. `source_annotated/src/mink/lie/_lie_ops_c.c`
9. `source_annotated/src/mink/lie/_lie_ops_c.pyi`
10. `source_annotated/src/mink/py.typed`

第三轮读完后，应能解释：

- actuator 示例和普通 IK 示例的区别。
- mocap target / keyboard teleop 如何服务交互式目标。
- C 扩展和类型文件只是加速/类型辅助，不是第一轮理解主线。

## 文件作用清单

| 文件 | 优先级 | 作用 | 阅读重点 |
| --- | --- | --- | --- |
| `source_annotated/README.md` | 第一轮 | 学习副本说明 | 明确这里不是运行源码，而是带注释阅读副本 |
| `source_annotated/examples/arm_ur5e.py` | 第一轮 | UR5e 普通 IK 示例 | 主循环、task/limit 定义、`solve_ik`、`integrate_inplace` |
| `source_annotated/examples/arm_ur5e_actuators.py` | 第三轮 | UR5e actuator tracking 示例 | IK 结果如何接到 MuJoCo actuator 控制 |
| `source_annotated/src/mink/__init__.py` | 第一轮 | mink 对外 API 导出 | 示例中 `mink.Xxx` 从哪里来 |
| `source_annotated/src/mink/configuration.py` | 第一轮 | 状态中心 | `model`、`data`、`q`、pose 查询、Jacobian 查询、积分 |
| `source_annotated/src/mink/solve_ik.py` | 第一轮 | QP-IK 构造与求解入口 | task objective、limit constraints、QP solver、velocity 返回 |
| `source_annotated/src/mink/tasks/task.py` | 第一轮 | Task 基类 | error、Jacobian、QP objective 的统一接口 |
| `source_annotated/src/mink/tasks/frame_task.py` | 第一轮 | 末端位姿任务 | 6D pose error、frame Jacobian、QP objective |
| `source_annotated/src/mink/tasks/posture_task.py` | 第一轮 | 姿态保持任务 | 参考 q、posture error、identity Jacobian |
| `source_annotated/src/mink/tasks/__init__.py` | 第一轮 | task 导出入口 | mink 支持哪些 task 类型 |
| `source_annotated/src/mink/limits/limit.py` | 第一轮 | Limit 基类 | `G dq <= h` 的统一接口 |
| `source_annotated/src/mink/limits/velocity_limit.py` | 第一轮 | 速度限制 | `v_max * dt` 如何变成 `delta_q` bound |
| `source_annotated/src/mink/limits/configuration_limit.py` | 第一轮 | 关节位置限制 | joint range 如何变成当前步 `delta_q` 约束 |
| `source_annotated/src/mink/limits/__init__.py` | 第一轮 | limit 导出入口 | mink 支持哪些 limit 类型 |
| `source_annotated/src/mink/lie/base.py` | 第二轮 | Lie group 抽象基类 | exp/log、plus/minus 的接口意义 |
| `source_annotated/src/mink/lie/so3.py` | 第二轮 | 3D 旋转 SO3 | orientation error、quaternion、log map |
| `source_annotated/src/mink/lie/se3.py` | 第二轮 | 3D 位姿 SE3 | pose error、transform compose/inverse |
| `source_annotated/src/mink/lie/utils.py` | 第二轮 | Lie 计算工具 | SO3/SE3 内部辅助函数 |
| `source_annotated/src/mink/lie/__init__.py` | 第二轮 | Lie 模块导出 | SO3、SE3 如何导出 |
| `source_annotated/src/mink/tasks/relative_frame_task.py` | 第二轮 | 相对位姿任务 | 控制两个 frame 之间的相对 pose |
| `source_annotated/src/mink/tasks/damping_task.py` | 第二轮 | 阻尼任务 | 抑制过大速度，作为正则项 |
| `source_annotated/src/mink/tasks/dof_freezing_task.py` | 第二轮 | 自由度冻结任务 | 指定 dof 速度趋近 0 |
| `source_annotated/src/mink/tasks/equality_constraint_task.py` | 第二轮 | 等式约束任务 | task 作为精确约束而非软目标 |
| `source_annotated/src/mink/tasks/com_task.py` | 第二轮 | 质心任务 | COM position error 和 COM Jacobian |
| `source_annotated/src/mink/tasks/kinetic_energy_regularization_task.py` | 第二轮 | 动能正则任务 | 使用质量矩阵做速度正则 |
| `source_annotated/src/mink/limits/collision_avoidance_limit.py` | 第二轮 | 碰撞避免限制 | geom pair、距离、法向、约束生成 |
| `source_annotated/src/mink/utils.py` | 第三轮 | 工具函数 | frame 查找、mocap target、MuJoCo helper |
| `source_annotated/src/mink/constants.py` | 第三轮 | 常量 | 内部复用常量 |
| `source_annotated/src/mink/exceptions.py` | 第三轮 | 异常类型 | limit 越界、target 未设置、solver 失败 |
| `source_annotated/src/mink/contrib/__init__.py` | 第三轮 | contrib 导出 | 附加功能入口 |
| `source_annotated/src/mink/contrib/keyboard_teleop/__init__.py` | 第三轮 | keyboard teleop 导出 | 键盘遥操作入口 |
| `source_annotated/src/mink/contrib/keyboard_teleop/teleop_mocap.py` | 第三轮 | 键盘控制 mocap target | 交互式移动目标位姿 |
| `source_annotated/src/mink/contrib/keyboard_teleop/keycodes.py` | 第三轮 | 键盘码定义 | teleop 按键映射 |
| `source_annotated/src/mink/contrib/keyboard_teleop/KEYBOARD.md` | 第三轮 | 键盘说明 | teleop 操作说明 |
| `source_annotated/src/mink/lie/_lie_ops_c.c` | 第三轮 | Lie 运算 C 加速实现 | 不必第一轮读，知道它服务性能即可 |
| `source_annotated/src/mink/lie/_lie_ops_c.pyi` | 第三轮 | C 扩展类型声明 | 给 Python 类型检查使用 |
| `source_annotated/src/mink/py.typed` | 第三轮 | 类型标记文件 | 告诉类型检查器 mink 提供类型信息 |

## 建议打卡顺序

### 打卡 A: 能讲清楚示例

- 读: `arm_ur5e.py`
- 产出: 写出主循环伪代码。
- 标准: 能说清楚每一帧为什么先设 target，再 solve，再 integrate。

### 打卡 B: 能讲清楚状态

- 读: `configuration.py`
- 产出: 写出 `Configuration` 的 5 个职责。
- 标准: 能解释 `nq`、`nv`、`q`、`velocity` 的关系。

### 打卡 C: 能讲清楚任务

- 读: `task.py`、`frame_task.py`、`posture_task.py`
- 产出: 写出 FrameTask / PostureTask 的 error 和 Jacobian shape。
- 标准: 能解释 task 为什么最后会变成 QP objective。

### 打卡 D: 能讲清楚限制

- 读: `limit.py`、`velocity_limit.py`、`configuration_limit.py`
- 产出: 写出两个 limit 的不等式形式。
- 标准: 能解释 velocity limit 和 position limit 的区别。

### 打卡 E: 能讲清楚 QP-IK

- 读: `solve_ik.py`
- 产出: 写出最小 QP 形式。
- 标准: 能解释 `H`、`c`、`G`、`h`、`delta_q`、`v` 分别来自哪里。

### 打卡 F: 能对照 A 项目

- 读: `06_compare_with_A_self_baseline.md`
- 对照:
  - `scripts/02_configuration_site_pose.py`
  - `scripts/03_site_jacobian_check.py`
  - `scripts/04_dls_differential_ik.py`
  - `scripts/05_task_limit_qp_ik.py`
- 产出: 补全对照表。
- 标准: 能说清楚 A 项目当前是在复现 mink 的哪些核心概念。

