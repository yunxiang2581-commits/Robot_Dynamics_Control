# 学习副本说明:
# - 原始文件来自 external/mink_upstream/examples/arm_ur5e_actuators.py。
# - 本文件只用于阅读 mink 如何把 IK 结果接到 MuJoCo actuator tracking。
# - 阅读重点: 和 arm_ur5e.py 相比，本例多了一层 actuator / data.ctrl 执行。
#
# 推荐学习路径:
# 1. 先读完 arm_ur5e.py，理解普通 differential IK 闭环。
# 2. 再读本文件，重点找“哪里仍然是 IK，哪里开始进入 actuator 控制”。
# 3. 对比普通示例:
#    - arm_ur5e.py: solve_ik -> integrate_inplace -> viewer 显示 configuration。
#    - arm_ur5e_actuators.py: solve_ik -> integrate_inplace -> data.ctrl -> mj_step。
# 4. 第一轮只理解执行链路，不急着深挖 TeleopMocap 的键盘交互。

from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
from loop_rate_limiters import RateLimiter

import mink
from mink.contrib import TeleopMocap

_HERE = Path(__file__).parent
_XML = _HERE / "universal_robots_ur5e" / "scene.xml"


if __name__ == "__main__":
    # 学习步骤 1: 加载 MuJoCo model，并显式创建 data。
    #
    # 和 arm_ur5e.py 的区别:
    # - 普通示例让 Configuration 内部创建 data。
    # - actuator 示例显式创建 data，因为后面要用 data.ctrl 和 mujoco.mj_step。
    model = mujoco.MjModel.from_xml_path(_XML.as_posix())
    data = mujoco.MjData(model)

    ## =================== ##
    ## Setup IK.
    ## =================== ##

    # 学习步骤 2: 创建 Configuration。
    #
    # Configuration 仍然负责 IK 侧的状态、pose、Jacobian 和积分。
    configuration = mink.Configuration(model)

    # 学习步骤 3: 定义 IK tasks。
    #
    # 这部分和 arm_ur5e.py 基本一样:
    # - FrameTask 负责末端 attachment_site 跟踪目标位姿。
    # - PostureTask 负责弱姿态保持。
    tasks = [
        end_effector_task := mink.FrameTask(
            frame_name="attachment_site",
            frame_type="site",
            position_cost=1.0,
            orientation_cost=1.0,
            lm_damping=1e-6,
        ),
        posture_task := mink.PostureTask(model, cost=1e-3),
    ]

    # 学习步骤 4: 定义 collision pairs。
    #
    # 这里和普通示例略有不同:
    # - 普通示例直接写 body/geom 名称。
    # - 这里先通过 get_body_geom_ids 找 wrist_3_link 下面的 geoms。
    #
    # 目的:
    # - 更精确地告诉碰撞约束要检查哪些 geom。
    wrist_3_geoms = mink.get_body_geom_ids(model, model.body("wrist_3_link").id)
    collision_pairs = [
        (wrist_3_geoms, ["floor", "wall"]),
    ]

    # 学习步骤 5: 定义 IK limits。
    #
    # 仍然包括:
    # - ConfigurationLimit: 关节位置范围。
    # - CollisionAvoidanceLimit: 碰撞避免。
    # - VelocityLimit: 速度上限。
    limits = [
        mink.ConfigurationLimit(model=configuration.model),
        mink.CollisionAvoidanceLimit(
            model=configuration.model,
            geom_pairs=collision_pairs,
        ),
    ]

    max_velocities = {
        "shoulder_pan": np.pi,
        "shoulder_lift": np.pi,
        "elbow": np.pi,
        "wrist_1": np.pi,
        "wrist_2": np.pi,
        "wrist_3": np.pi,
    }
    velocity_limit = mink.VelocityLimit(model, max_velocities)
    limits.append(velocity_limit)

    ## =================== ##

    # 学习步骤 6: 获取 mocap target id。
    #
    # mid 在这个示例中也不是主线变量，看到这里知道 target 是 mocap body 即可。
    mid = model.body("target").mocapid[0]

    # 学习步骤 7: IK 求解参数。
    #
    # solver:
    # - QP solver 后端。
    #
    # pos_threshold / ori_threshold:
    # - 每一帧内部会迭代 IK，直到误差足够小或达到 max_iters。
    #
    # max_iters:
    # - 每个 viewer 帧最多做多少次 differential IK 小步。
    solver = "daqp"
    pos_threshold = 1e-4
    ori_threshold = 1e-4
    max_iters = 20

    # 学习步骤 8: 键盘遥操作 target。
    #
    # TeleopMocap 会根据键盘输入移动 mocap target。
    # 第一轮可以只知道它改变目标位姿，细节放到 contrib/keyboard_teleop 再读。
    key_callback = TeleopMocap(data)

    with mujoco.viewer.launch_passive(
        model=model,
        data=data,
        show_left_ui=False,
        show_right_ui=False,
        key_callback=key_callback,
    ) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        # 学习步骤 9: 初始化 MuJoCo data 和 Configuration。
        #
        # mj_resetDataKeyframe:
        # - 把 MuJoCo data 设置到 home keyframe。
        #
        # configuration.update(data.qpos):
        # - 把 MuJoCo 当前 qpos 同步到 mink Configuration。
        #
        # mj_forward:
        # - 让 MuJoCo 根据当前 qpos 计算完整状态。
        mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
        configuration.update(data.qpos)
        mujoco.mj_forward(model, data)

        # 把当前 home 姿态设置为姿态保持目标。
        posture_task.set_target_from_configuration(configuration)

        # 学习步骤 10: 初始化 mocap target 到末端位置。
        #
        # 这样开始时 target 和 attachment_site 重合，避免一开始就有大误差。
        mink.move_mocap_to_frame(model, data, "target", "attachment_site", "site")

        # 学习步骤 11: 固定控制频率。
        rate = RateLimiter(frequency=200.0, warn=False)

        while viewer.is_running():
            # 学习步骤 12: 从 mocap target 读取当前目标位姿，并设置给 FrameTask。
            #
            # 目标可以被键盘移动，所以每帧都要重新读。
            T_wt = mink.SE3.from_mocap_name(model, data, "target")
            end_effector_task.set_target(T_wt)

            # 学习步骤 13: 处理键盘自动移动。
            #
            # 如果用户按键或启用连续移动，这一步会改变 mocap target。
            # 它影响下一轮从 mocap 读取到的 T_wt。
            key_callback.auto_key_move()

            # 学习步骤 14: 每一帧内部做多次 IK 小步。
            #
            # 和普通示例相比，本例多了 for i in range(max_iters):
            # - 目标是让 configuration 更快接近 target。
            # - 每次迭代都 solve_ik，然后 integrate_inplace。
            #
            # 注意:
            # - 这里仍然是在更新 mink 的 configuration。
            # - 真正驱动 MuJoCo actuator 是后面的 data.ctrl 和 mj_step。
            for i in range(max_iters):
                vel = mink.solve_ik(
                    configuration, tasks, rate.dt, solver, limits=limits
                )
                configuration.integrate_inplace(vel, rate.dt)

                # 学习步骤 15: 检查 task error 是否已经足够小。
                #
                # err[:3]:
                # - 位置误差。
                #
                # err[3:]:
                # - 姿态误差。
                #
                # 如果位置和姿态都达标，就提前结束本帧内部迭代。
                err = end_effector_task.compute_error(configuration)
                pos_achieved = np.linalg.norm(err[:3]) <= pos_threshold
                ori_achieved = np.linalg.norm(err[3:]) <= ori_threshold
                if pos_achieved and ori_achieved:
                    break

            # 学习步骤 16: 把 IK 得到的 configuration.q 作为 actuator 控制目标。
            #
            # 这是本例和 arm_ur5e.py 最大的区别:
            # - 普通示例只是更新 configuration 并显示。
            # - actuator 示例把目标 q 写入 data.ctrl。
            #
            # 直观理解:
            # - solve_ik 得到一个“希望机械臂到达的 q”。
            # - MuJoCo actuator 根据 data.ctrl 驱动真实仿真状态去跟踪它。
            data.ctrl = configuration.q

            # 学习步骤 17: 推进 MuJoCo 物理仿真一步。
            #
            # mj_step 会根据 actuator、动力学、约束等更新 data。
            # 这一步比普通 example 更接近真实控制。
            mujoco.mj_step(model, data)

            # 学习步骤 18: viewer 同步和循环限频。
            viewer.sync()
            rate.sleep()
