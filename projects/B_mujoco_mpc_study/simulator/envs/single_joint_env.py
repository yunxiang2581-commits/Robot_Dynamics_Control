"""B01 single-joint MuJoCo environment skeleton.

本文件只定义教学型接口，不实现完整 MuJoCo 仿真逻辑。
"""

from __future__ import annotations

from pathlib import Path    
from typing import Any

import mujoco

SCRIPT_PATH = Path(__file__).resolve()
SIMULATOR_ROOT = SCRIPT_PATH.parents[1]
DEFAULT_SINGLE_JOINT_XML = SIMULATOR_ROOT / "models" / "B01_single_joint.xml"

class SingleJointEnv:
    """单关节 hinge joint 环境骨架。

    这个类负责把 MuJoCo 单关节模型封装成 MPC 可以调用的接口。
    当前只保留教学型 TODO，不实现真实 step / rollout / render。
    """

    def __init__(self, model_path: str | None = None, dt: float = 0.01) -> None:
        """初始化单关节 MuJoCo 环境。

        TODO:
        - 要实现什么：加载或创建一个单关节 hinge joint MuJoCo 模型，并保存 model / data / dt。
        - 为什么要实现：MPC 的 rollout 必须依赖同一个物理模型预测未来状态。
        - 输入是什么：`model_path` 是可选 MuJoCo XML 路径，`dt` 是仿真步长。
        - 输出是什么：初始化后的环境对象，内部应持有模型、仿真数据和时间步长。
        - 物理意义：定义关节惯量、阻尼、重力、力矩 actuator 等物理属性。
        - 数学意义：建立状态转移函数 `x_{k+1} = f(x_k, tau_k)` 的数值实现。
        - 验证标准：模型能 reset，`get_state()` 能返回 `[q, dq]`，step 后状态发生合理变化。
        """
        if dt <= 0:
            raise ValueError("dt must be positive.")

        self.model_path = Path(model_path) if model_path is not None else DEFAULT_SINGLE_JOINT_XML
        if not self.model_path.exists():
            raise FileNotFoundError(f"找不到 MuJoCo XML 文件: {self.model_path}")

        self.dt = dt
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.model.opt.timestep = dt
        self.data = mujoco.MjData(self.model)
        self.renderer: Any | None = None

        self._camera = mujoco.MjvCamera()
        camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "fixed")
        if camera_id >= 0:
            self._camera.fixedcamid = camera_id
            self._camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
        else:
            self._camera.lookat[:] = [0.0, 0.0, 0.0]
            self._camera.distance = 2.5
            self._camera.azimuth = 90
            self._camera.elevation = -30

        if self.model.nq < 1 or self.model.nv < 1:
            raise ValueError("单关节环境至少需要 1 个 qpos 和 1 个 qvel。")
        if self.model.nu < 1:
            raise ValueError("单关节环境至少需要 1 个 actuator，用于输入 torque。")

        self.joint_id = 0
        self.qpos_id = int(self.model.jnt_qposadr[self.joint_id])
        self.qvel_id = int(self.model.jnt_dofadr[self.joint_id])
        self.actuator_id = 0

        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def reset(self, q: float = 0.0, dq: float = 0.0) -> tuple[float, float]:
        """重置环境状态。

        TODO:
        - 要实现什么：把关节角度和角速度设置为给定初始值。
        - 为什么要实现：每次实验和每条 rollout 都需要从明确状态开始。
        - 输入是什么：`q` 是初始角度，`dq` 是初始角速度。
        - 输出是什么：重置后的状态 `(q, dq)`。
        - 物理意义：把单关节放到指定位置和速度。
        - 数学意义：设置 MPC 初始状态 `x_0 = [q, dq]`。
        - 验证标准：reset 后 `get_state()` 返回值应与输入一致。
        """
        mujoco.mj_resetData(self.model, self.data)

        self.data.qpos[self.qpos_id] = q
        self.data.qvel[self.qvel_id] = dq
        if self.model.nu > 0:
            self.data.ctrl[:] = 0.0

        mujoco.mj_forward(self.model, self.data)

        return self.get_state()
        
    def get_state(self) -> tuple[float, float]:
        """读取当前状态。

        TODO:
        - 要实现什么：从 MuJoCo data 中读取当前关节角度 `q` 和角速度 `dq`。
        - 为什么要实现：MPC 每个控制周期都要基于当前状态重新规划。
        - 输入是什么：无显式输入，读取环境内部仿真状态。
        - 输出是什么：状态 `(q, dq)`。
        - 物理意义：测量关节当前运动状态。
        - 数学意义：得到当前 `x_k = [q_k, dq_k]`。
        - 验证标准：返回长度为 2，并且 step 前后数值变化合理。
        """
        if self.model is None or self.data is None:
            raise RuntimeError("MuJoCo model/data 尚未初始化，请先检查 __init__。")
        q = float(self.data.qpos[self.qpos_id])
        dq = float(self.data.qvel[self.qvel_id])

        return q, dq
        

    def set_state(self, q: float, dq: float) -> None:
        """设置当前状态。

        TODO:
        - 要实现什么：把 MuJoCo data 的 qpos / qvel 设置成给定状态。
        - 为什么要实现：rollout 需要临时复制并恢复状态，不能污染真实控制环境。
        - 输入是什么：`q` 是目标角度，`dq` 是目标角速度。
        - 输出是什么：无返回值，但环境内部状态应被更新。
        - 物理意义：把仿真器放到某个假设状态。
        - 数学意义：设置 rollout 起点 `x_start`。
        - 验证标准：调用后 `get_state()` 应返回相同状态。
        """
        if self.model is None or self.data is None:
            raise RuntimeError("MuJoCo model/data 尚未初始化，请先检查 __init__。")

        # 设置单关节的位置和速度。
        self.data.qpos[self.qpos_id] = q
        self.data.qvel[self.qvel_id] = dq

        # 清空当前控制输入，避免旧 torque 影响下一步仿真。
        if self.model.nu > 0:
            self.data.ctrl[:] = 0.0

        # 修改 qpos/qvel 后，重新计算 MuJoCo 内部派生量。
        mujoco.mj_forward(self.model, self.data)

    def step(self, tau: float) -> tuple[float, float]:
        """执行一步仿真。

        TODO:
        - 要实现什么：把力矩 `tau` 写入 actuator，然后推进 MuJoCo 一个仿真步。
        - 为什么要实现：控制器最终只执行 best torque sequence 的第一个 torque。
        - 输入是什么：`tau` 是当前控制步施加的关节力矩。
        - 输出是什么：执行后的新状态 `(q_next, dq_next)`。
        - 物理意义：力矩改变角加速度，从而改变角速度和角度。
        - 数学意义：计算 `x_{k+1} = f(x_k, tau_k)`。
        - 验证标准：正负 torque 应导致角速度变化方向不同，且不应产生 NaN。
        """
        # 1. 把当前控制力矩写入第一个 actuator。
        # 对 B01 来说，tau 就是单关节 motor 的控制输入。
        ctrl_min, ctrl_max = self.model.actuator_ctrlrange[self.actuator_id]
        tau = max(float(ctrl_min), min(float(ctrl_max), float(tau)))
        
        self.data.ctrl[self.actuator_id] = tau

        # 2. 推进 MuJoCo 一个仿真步。
        # 数学上对应 x_{k+1} = f(x_k, tau_k)。
        mujoco.mj_step(self.model, self.data)

        # 3. 返回推进后的状态。
        return self.get_state()

    def rollout(self, initial_state: tuple[float, float], torque_sequence: list[float]) -> list[tuple[float, float]]:
        """根据 torque 序列预测未来状态。

        TODO:
        - 要实现什么：从 `initial_state` 出发，依次执行 `torque_sequence`，记录状态序列。
        - 为什么要实现：predictive sampling 需要用 rollout 比较不同候选控制序列。
        - 输入是什么：初始状态 `(q, dq)` 和候选力矩序列 `[tau_0, ..., tau_{H-1}]`。
        - 输出是什么：状态序列 `[x_0, x_1, ..., x_H]`。
        - 物理意义：预测未来关节会怎样运动。
        - 数学意义：重复应用 `x_{k+1} = f(x_k, tau_k)`。
        - 验证标准：输出长度应为 `len(torque_sequence) + 1`，并且不能改变真实环境状态。
        """
        """根据 torque 序列预测未来状态。"""

        # 1. 保存真实环境当前状态。
        # rollout 是“假设预测”，不能改变真实闭环控制正在使用的状态。
        real_state = self.get_state()

        # 2. 把仿真器临时放到 rollout 的初始状态。
        q0, dq0 = initial_state
        self.set_state(q0, dq0)

        # 3. 记录 x_0。
        states: list[tuple[float, float]] = [self.get_state()]

        # 4. 依次执行 torque_sequence 中的每个 tau，记录状态。
        for tau in torque_sequence:
            next_state = self.step(tau)
            states.append(next_state)

        # 5. 恢复真实环境状态。
        self.set_state(*real_state)

        return states


    def render_frame(self) -> Any:
        """渲染一帧画面。

        TODO:
        - 要实现什么：从 MuJoCo 离屏渲染器获取当前画面帧。
        - 为什么要实现：B01 必须输出 simulation video，而不是只有日志。
        - 输入是什么：无显式输入，使用当前仿真状态和相机配置。
        - 输出是什么：一帧图像数据，例如 RGB array。
        - 物理意义：让单关节运动过程可观看。
        - 数学意义：不改变 MPC 数学，只把状态轨迹转为可视化证据。
        - 验证标准：返回图像尺寸固定，连续帧能组成 mp4。
        """
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=480, width=640)

        # 根据当前 data 更新渲染场景。
        self.renderer.update_scene(self.data, self._camera)

        # 输出当前画面的 RGB array。
        frame = self.renderer.render()

        return frame
