"""
@Author  : Administrator  # 创建者（自动获取系统用户名）
@Time    : 2026/4/1 21:55  # 创建时间（自动生成）
@File    : pd_controller.py  # 文件名
@Description :  # 单摆控制器
     该文件实现了 PD 控制器。控制器的作用是根据当前位置和速度与目标位置
     和目标速度的误差计算出控制输入（力矩），用于驱动机械系统。

    控制公式：
    tau = Kp * (qd - q) + Kd * (dqd - dq)
    其中：
    - tau: 控制输入（力矩）
    - qd: 目标位置
    - q: 当前位置信息
    - dqd: 目标速度
    - dq: 当前速度

    输入输出：
    - 输入：
        - kp: 比例增益
        - kd: 微分增益
        - qd: 目标位置
        - dqd: 目标速度（默认为 0）
        - q: 当前位置
        - dq: 当前速度
    - 输出：
        - tau: 计算得到的控制力矩

    适用范围：
    适用于简单的力控或位置控制任务。特别是针对具有简单动力学的系统，如单关节机械臂。
    可以扩展为更复杂的控制任务，但需要根据具体情况调整控制器参数。
"""
class PDController:
    def __init__(self, kp: float, kd: float, qd: float, dqd: float = 0.0):
        self.kp = kp  # 比例增益
        # 这行代码将传入的参数 kp 保存到对象的属性 self.kp 中
        self.kd = kd  # 微分增益
        self.qd = qd  # 目标位置
        self.dqd = dqd  # 目标速度，默认是 0

    def compute(self, q: float, dq: float) -> float:
        """
        计算控制输入（力矩或力）：
        tau = kp * (qd - q) + kd * (dqd - dq)
        """
        tau = self.kp * (self.qd - q) + self.kd * (self.dqd - dq)
        return float(tau)

    def update_target(self, qd: float, dqd: float = 0.0):
        """
        更新目标位置和目标速度
        """
        self.qd = qd
        self.dqd = dqd

