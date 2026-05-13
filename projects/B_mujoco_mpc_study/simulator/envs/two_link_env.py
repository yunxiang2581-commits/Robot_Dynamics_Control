"""B02 two-link MuJoCo environment skeleton.

本文件用于 B02 二连杆 MPC tracking 学习任务。
当前阶段只建立清晰接口和中文 TODO，后续再逐步补全真实仿真逻辑。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import mujoco
import numpy as np


SCRIPT_PATH = Path(__file__).resolve()
SIMULATOR_ROOT = SCRIPT_PATH.parents[1]
DEFAULT_TWO_LINK_XML = SIMULATOR_ROOT / "models" / "B02_two_link.xml"


class TwoLinkEnv:
    """二连杆 MuJoCo 环境骨架。

    这个类负责把二连杆 MuJoCo 模型封装成 MPC 可以调用的接口。
    第一版重点是学习：状态读取、末端 site 查询、rollout 状态恢复和可视化输出。
    """

    def __init__(
        self,
        model_path: str | Path | None = None,
        dt: float = 0.01,
        end_effector_site: str = "ee_site",
    ) -> None:
        """初始化二连杆环境。

        TODO:
        - 要实现什么：加载 `B02_two_link.xml`，保存 model、data、dt 和末端 site id。
        - 为什么要实现：MPC rollout 必须依赖同一个二连杆动力学模型预测未来。
        - 输入是什么：MuJoCo XML 路径、仿真步长 dt、末端 site 名称。
        - 输出是什么：初始化后的环境对象。
        - 物理意义：定义二连杆长度、惯量、关节、actuator 和末端点。
        - 数学意义：建立状态转移函数 `x_{k+1}=f(x_k,u_k)`。
        - 验证标准：XML 存在，模型至少有 2 个 qpos、2 个 qvel、2 个 actuator。
        """
        if dt <= 0:
            raise ValueError("dt must be positive.")

        self.model_path = Path(model_path) if model_path is not None else DEFAULT_TWO_LINK_XML
        if not self.model_path.exists():
            raise FileNotFoundError(f"找不到 MuJoCo XML 文件: {self.model_path}")

        self.dt = dt
        self.end_effector_site = end_effector_site

        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.model.opt.timestep = dt
        self.data = mujoco.MjData(self.model)
        self.renderer: Any | None = None
        self.last_applied_torque: tuple[float, float] = (0.0, 0.0)

        if self.model.nq < 2 or self.model.nv < 2:
            raise ValueError("二连杆环境至少需要 2 个 qpos 和 2 个 qvel。")
        if self.model.nu < 2:
            raise ValueError("二连杆环境至少需要 2 个 actuator，用于输入 [tau1, tau2]。")

        self.joint_ids = self._find_joint_ids(["shoulder", "elbow"])
        self.qpos_ids = [int(self.model.jnt_qposadr[joint_id]) for joint_id in self.joint_ids]
        self.qvel_ids = [int(self.model.jnt_dofadr[joint_id]) for joint_id in self.joint_ids]
        self.actuator_ids = [0, 1]
        self.end_effector_site_id = self._find_site_id(end_effector_site)

        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def _find_joint_ids(self, joint_names: list[str]) -> list[int]:
        """按名称查找关节 id。

        B02 当前明确关注 shoulder/elbow 两个关节。这里仍然先通过 MuJoCo
        名称表查找，而不是直接假设第 0、1 个 joint，方便后续阅读 XML 时对齐。
        """
        joint_ids: list[int] = []
        available_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(self.model.njnt)
        ]

        for joint_name in joint_names:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id < 0:
                raise ValueError(f"找不到关节 {joint_name!r}，可用关节: {available_names}")
            joint_ids.append(int(joint_id))

        return joint_ids

    def _find_site_id(self, site_name: str) -> int:
        """按名称查找末端 site id。"""
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if site_id < 0:
            available_names = [
                mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SITE, i)
                for i in range(self.model.nsite)
            ]
            raise ValueError(f"找不到末端 site {site_name!r}，可用 site: {available_names}")

        return int(site_id)

    def reset(self, q: list[float] | tuple[float, float], dq: list[float] | tuple[float, float]) -> tuple[float, float, float, float]:
        """重置二连杆状态。

        TODO:
        - 要实现什么：设置 `q1,q2,dq1,dq2` 并调用 MuJoCo forward。
        - 为什么要实现：每次实验和每条 rollout 都必须从明确状态开始。
        - 输入是什么：`q=[q1,q2]` 和 `dq=[dq1,dq2]`。
        - 输出是什么：重置后的状态 `(q1,q2,dq1,dq2)`。
        - 物理意义：把二连杆放到指定初始姿态和速度。
        - 数学意义：设置 MPC 初始状态 `x0`。
        - 验证标准：reset 后 `get_state()` 返回值和输入一致。
        """
        if len(q) != 2 or len(dq) != 2:
            raise ValueError("reset 需要 q=[q1, q2] 和 dq=[dq1, dq2]。")

        mujoco.mj_resetData(self.model, self.data)

        self.data.qpos[self.qpos_ids[0]] = float(q[0])
        self.data.qpos[self.qpos_ids[1]] = float(q[1])
        self.data.qvel[self.qvel_ids[0]] = float(dq[0])
        self.data.qvel[self.qvel_ids[1]] = float(dq[1])
        self.data.ctrl[:] = 0.0
        self.last_applied_torque = (0.0, 0.0)

        mujoco.mj_forward(self.model, self.data)

        return self.get_state()

    def get_state(self) -> tuple[float, float, float, float]:
        """读取当前二连杆状态。

        TODO:
        - 要实现什么：从 MuJoCo data 读取 `q1,q2,dq1,dq2`。
        - 为什么要实现：MPC 每个控制周期都要基于当前状态重新规划。
        - 输入是什么：无显式输入，读取环境内部状态。
        - 输出是什么：状态元组 `(q1,q2,dq1,dq2)`。
        - 物理意义：测量两个关节的位置和速度。
        - 数学意义：得到当前 `x_k`。
        - 验证标准：输出长度为 4，且无 NaN。
        """
        if self.model is None or self.data is None:
            raise RuntimeError("MuJoCo model/data 尚未初始化，请先检查 __init__。")

        q1 = float(self.data.qpos[self.qpos_ids[0]])
        q2 = float(self.data.qpos[self.qpos_ids[1]])
        dq1 = float(self.data.qvel[self.qvel_ids[0]])
        dq2 = float(self.data.qvel[self.qvel_ids[1]])

        return q1, q2, dq1, dq2

    def get_end_effector_position(self) -> tuple[float, float]:
        """读取末端 site 的二维位置。

        TODO:
        - 要实现什么：从 MuJoCo site xipos 读取 `ee_site` 的 x/y 坐标。
        - 为什么要实现：B02 的 residual 是末端位置误差，不是关节角误差。
        - 输入是什么：环境内部 model/data 和末端 site id。
        - 输出是什么：末端位置 `(x_ee, y_ee)`。
        - 物理意义：得到机械臂末端在平面中的实际位置。
        - 数学意义：计算 task-space 输出 `p_ee(q)`。
        - 验证标准：静止状态下该位置应和手写 FK 结果接近。
        """
        if self.model is None or self.data is None:
            raise RuntimeError("MuJoCo model/data 尚未初始化，请先检查 __init__。")

        mujoco.mj_forward(self.model, self.data)
        site_position = self.data.site_xpos[self.end_effector_site_id]

        return float(site_position[0]), float(site_position[1])

    def set_state(self, state: tuple[float, float, float, float]) -> None:
        """设置当前仿真状态。

        TODO:
        - 要实现什么：把 MuJoCo qpos/qvel 设置为给定 `state`。
        - 为什么要实现：rollout 需要临时复制状态，不能污染真实闭环状态。
        - 输入是什么：`state=(q1,q2,dq1,dq2)`。
        - 输出是什么：无返回，但内部仿真状态被更新。
        - 物理意义：把机械臂放到某个假设状态。
        - 数学意义：设置 rollout 起点。
        - 验证标准：调用后 `get_state()` 应返回相同状态。
        """
        if self.model is None or self.data is None:
            raise RuntimeError("MuJoCo model/data 尚未初始化，请先检查 __init__。")
        if len(state) != 4:
            raise ValueError("set_state 需要 state=(q1, q2, dq1, dq2)。")

        q1, q2, dq1, dq2 = state

        # 设置两个关节的位置和速度，对应状态 x=[q1,q2,dq1,dq2]。
        self.data.qpos[self.qpos_ids[0]] = float(q1)
        self.data.qpos[self.qpos_ids[1]] = float(q2)
        self.data.qvel[self.qvel_ids[0]] = float(dq1)
        self.data.qvel[self.qvel_ids[1]] = float(dq2)

        # 清空旧控制输入，避免上一条 rollout 的 torque 影响下一次预测。
        self.data.ctrl[:] = 0.0
        self.last_applied_torque = (0.0, 0.0)

        # 修改 qpos/qvel 后，让 MuJoCo 重新计算 site、body 等派生量。
        mujoco.mj_forward(self.model, self.data)

    def get_last_applied_torque(self) -> tuple[float, float]:
        """返回上一仿真步真正施加给 actuator 的 torque。

        这个接口用来区分：
        - controller/planner 给出的原始 torque 命令；
        - MuJoCo 按 actuator `ctrlrange` 裁剪后的实际执行 torque。
        """
        return self.last_applied_torque

    def step(self, torque: tuple[float, float] | list[float]) -> tuple[float, float, float, float]:
        """执行一步二连杆仿真。

        TODO:
        - 要实现什么：写入 `[tau1,tau2]`，推进 MuJoCo 一个 step。
        - 为什么要实现：receding horizon control 最终只执行最优序列的第一步。
        - 输入是什么：两个关节力矩。
        - 输出是什么：下一状态 `(q1,q2,dq1,dq2)`。
        - 物理意义：两个 actuator 同时驱动二连杆运动。
        - 数学意义：计算 `x_{k+1}=f(x_k,u_k)`。
        - 验证标准：输出有限，力矩被限制在 actuator ctrlrange 内。
        """
        if len(torque) != 2:
            raise ValueError("step 需要 torque=(tau1, tau2)。")

        applied_torque: list[float] = []
        for i, actuator_id in enumerate(self.actuator_ids):
            ctrl_min, ctrl_max = self.model.actuator_ctrlrange[actuator_id]
            clipped_torque = max(float(ctrl_min), min(float(ctrl_max), float(torque[i])))
            self.data.ctrl[actuator_id] = clipped_torque
            applied_torque.append(clipped_torque)

        self.last_applied_torque = (float(applied_torque[0]), float(applied_torque[1]))

        mujoco.mj_step(self.model, self.data)

        return self.get_state()

    def rollout(self, initial_state: tuple[float, float, float, float], torque_sequence: list[tuple[float, float]]) -> list[tuple[float, float, float, float]]:
        """根据 torque 序列预测未来状态。

        TODO:
        - 要实现什么：从 initial_state 出发，依次执行 torque_sequence 并记录状态。
        - 为什么要实现：predictive sampling 要比较多条未来控制方案。
        - 输入是什么：初始状态和长度为 horizon 的 torque 序列。
        - 输出是什么：长度为 `horizon+1` 的状态序列。
        - 物理意义：预测二连杆未来运动。
        - 数学意义：重复应用状态转移函数。
        - 验证标准：rollout 不应改变真实闭环环境状态。
        """
        real_state = self.get_state()

        self.set_state(initial_state)
        states: list[tuple[float, float, float, float]] = [self.get_state()]

        for torque in torque_sequence:
            states.append(self.step(torque))

        self.set_state(real_state)

        return states

    def render_frame(self) -> Any:
        """渲染当前画面。

        TODO:
        - 要实现什么：使用 MuJoCo Renderer 获取 RGB frame。
        - 为什么要实现：B02 验收必须有 simulation video。
        - 输入是什么：当前 MuJoCo model/data。
        - 输出是什么：RGB 图像数组。
        - 物理意义：让二连杆末端跟踪过程可观看。
        - 数学意义：不改变控制数学，只把状态轨迹变成可视证据。
        - 验证标准：连续帧可组成 mp4，图像非空。
        """
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=480, width=640)

        self.renderer.update_scene(self.data)
        frame = self.renderer.render()

        return frame

    def render_frame_with_scene_geoms(self, scene_geoms: list[dict[str, Any]]) -> Any:
        """渲染带 scene marker 的当前帧。

        TODO:
        - 要实现什么：在 MuJoCo 离屏场景里追加临时 marker 几何体，再输出 RGB 帧。
        - 为什么需要：第二版 scene marker 和第三版 hybrid render 都需要真实的 3D 场景 marker。
        - 输入是什么：scene_geoms，列表中每个元素描述一个临时几何体。
        - 输出是什么：带 scene marker 的 RGB 图像。
        - 验证标准：至少能渲染 sphere/line 两类临时 geom，且返回 HxWx3 图像。
        """
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=480, width=640)

        mujoco.mj_forward(self.model, self.data)
        self.renderer.update_scene(self.data)
        scene = self.renderer.scene

        for geom_spec in scene_geoms:
            if scene.ngeom >= scene.maxgeom:
                break

            geom = scene.geoms[scene.ngeom]
            geom_type = geom_spec["geom_type"]
            size = np.asarray(geom_spec["size"], dtype=np.float64)
            pos = np.asarray(geom_spec["pos"], dtype=np.float64)
            rgba = np.asarray(geom_spec["rgba"], dtype=np.float32)
            mat = np.eye(3, dtype=np.float64).reshape(-1)

            if geom_type == "sphere":
                mujoco.mjv_initGeom(
                    geom,
                    mujoco.mjtGeom.mjGEOM_SPHERE,
                    size,
                    pos,
                    mat,
                    rgba,
                )
            elif geom_type == "line":
                from_pos = np.asarray(geom_spec["from_pos"], dtype=np.float64)
                to_pos = np.asarray(geom_spec["to_pos"], dtype=np.float64)
                mujoco.mjv_initGeom(
                    geom,
                    mujoco.mjtGeom.mjGEOM_LINE,
                    size,
                    from_pos,
                    mat,
                    rgba,
                )
                mujoco.mjv_connector(
                    geom,
                    mujoco.mjtGeom.mjGEOM_LINE,
                    float(size[0]),
                    from_pos,
                    to_pos,
                )
                geom.rgba[:] = rgba
            else:
                raise ValueError(f"暂不支持的 scene geom_type: {geom_type}")

            scene.ngeom += 1

        return self.renderer.render()
