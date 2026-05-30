from __future__ import annotations


TOPICS = {
    "ilqr": """iLQR 学习笔记

学习目标：
理解 iterative Linear Quadratic Regulator 如何用局部线性化和二次近似，迭代改进非线性系统的控制序列。

直觉：
iLQR 每轮先沿当前控制序列 rollout，再在这条轨迹附近近似成 LQR 问题，反向求反馈增益，正向线搜索更新控制。

输入：
- 动力学函数 x_next = f(x, u)
- 初始状态 x0
- 参考轨迹或目标状态
- running cost 与 terminal cost
- 初始控制序列

输出：
- 优化后的控制序列
- 状态轨迹
- cost history
- feedback gains

在本项目中的作用：
B03 用它作为 MuJoCo / two-link 控制 demo 的核心优化控制学习对象。""",
    "mpc": """MPC 学习笔记

学习目标：
理解 Model Predictive Control 如何在滚动时域内反复规划，只执行第一步控制。

直觉：
MPC 不是一次性规划完整未来，而是每个控制周期重新看当前状态，再解一个有限时域优化问题。

输入：
- 当前状态
- 动力学模型
- 参考轨迹
- 代价函数与约束

输出：
- 当前时刻要执行的控制
- 预测轨迹
- 优化诊断信息

在本项目中的作用：
B 项目用 MPC 串起 iLQR、MPPI、CEM 等求解器比较。""",
    "mppi": """MPPI 学习笔记

学习目标：
理解 Model Predictive Path Integral 如何用采样和加权平均更新控制序列。

直觉：
MPPI 随机扰动很多条控制序列，rollout 后根据代价给低成本轨迹更高权重。

输入：
- 当前状态
- 初始控制均值
- 噪声尺度
- rollout 数量
- 代价函数

输出：
- 更新后的控制序列
- 当前执行控制
- 采样代价统计

在本项目中的作用：
可作为 B03 中与 iLQR 对比的 sampling MPC baseline。""",
}


def explain_topic(topic: str) -> str:
    key = topic.lower()
    return TOPICS.get(key, f"{topic} 暂无内置解释。建议按：学习目标、直觉、输入、输出、项目作用 五段补充。")
