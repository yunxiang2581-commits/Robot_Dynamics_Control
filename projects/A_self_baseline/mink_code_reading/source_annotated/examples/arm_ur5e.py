# 学习副本说明:
# - 原始文件来自 external/mink_upstream/examples/arm_ur5e.py。
# - 本文件只用于阅读 mink 的 UR5e differential IK 数据流。
# - 阅读重点: 按函数调用顺序理解 model -> Configuration -> tasks/limits -> solve_ik -> integrate。
#
# 推荐学习路径:
# 1. 先看 MuJoCo 模型如何加载: MjModel.from_xml_path。
# 2. 再看 mink 如何封装当前机器人状态: Configuration(model)。
# 3. 再看任务如何定义: FrameTask + PostureTask。
# 4. 再看约束如何定义: ConfigurationLimit + CollisionAvoidanceLimit + VelocityLimit。
# 5. 再看初始化: keyframe、posture target、mocap target。
# 6. 最后看 viewer loop: 读取目标、设置 task、solve_ik、integrate。
# 7. 第一轮只需要理解数据流，不急着深挖 SE3 和 collision avoidance 的数学细节。

from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
from loop_rate_limiters import RateLimiter

import mink

_HERE = Path(__file__).parent
_XML = _HERE / "universal_robots_ur5e" / "scene.xml"


if __name__ == "__main__":
    # 学习步骤 1: 加载 MuJoCo 模型。
    #
    # 输入:
    # - scene.xml，里面描述 UR5e 的 body、joint、site、geom、keyframe、mocap target 等。
    #
    # 输出:
    # - model: 只读的模型结构，包含 nq、nv、joint range、site 名称等。
    #
    # 这一行对应 A 项目的 A01 model inspect。
    model = mujoco.MjModel.from_xml_path(_XML.as_posix())

    # 学习步骤 2: 创建 mink.Configuration。
    #
    # Configuration 是 mink 的状态中心:
    # - 内部持有 model 和 data。
    # - 保存当前 q。
    # - 负责 forward kinematics。
    # - 提供 frame/site pose 查询。
    # - 提供 frame/site Jacobian 查询。
    # - 提供 integrate_inplace，把 velocity 积分回 q。
    #
    # 这一行对应 A 项目的 A02/A03/A04 的组合能力。
    configuration = mink.Configuration(model)

    # 学习步骤 3: 定义 tasks，也就是“希望机器人做到什么”。
    #
    # FrameTask:
    # - 控制 attachment_site 这个 site 的位姿。
    # - position_cost 控制位置误差权重。
    # - orientation_cost 控制姿态误差权重。
    # - lm_damping 是 task 层面的阻尼，帮助数值稳定。
    #
    # PostureTask:
    # - 让 q 尽量保持接近参考姿态。
    # - cost 很小，说明它是弱正则，不应该压过末端跟踪任务。
    #
    # 阅读建议:
    # - 第一轮只看 task 的输入和作用。
    # - 第二轮再跳到 frame_task.py / posture_task.py 看 error 和 Jacobian。
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

    # 学习步骤 4: 定义 limits，也就是“求解 IK 时不能违反什么”。
    #
    # collision_pairs 表示需要避免碰撞的几何体组合:
    # - wrist_3_link 不要和 floor / wall 太近。
    #
    # 阅读建议:
    # - 第一轮知道 CollisionAvoidanceLimit 是高级约束即可。
    # - 不需要马上读懂距离、法向、接触 Jacobian 的推导。
    collision_pairs = [
        (["wrist_3_link"], ["floor", "wall"]),
    ]

    # ConfigurationLimit:
    # - 限制积分后的 q 不超过关节位置范围。
    #
    # CollisionAvoidanceLimit:
    # - 把几何体距离转换成 QP 不等式约束。
    limits = [
        mink.ConfigurationLimit(model=model),
        mink.CollisionAvoidanceLimit(model=model, geom_pairs=collision_pairs),
    ]

    # VelocityLimit:
    # - 限制每个关节的最大速度。
    # - 在 QP 里会变成 delta_q 的上下界。
    #
    # 这里所有 UR5e 关节都给了 pi rad/s 的速度上限。
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

    # 学习步骤 5: 准备 MuJoCo viewer 和控制循环需要的对象。
    #
    # target 是模型里的 mocap body，用来表示用户拖动或程序设置的目标位姿。
    # mid 是 mocap id。这个示例里后面没有直接使用 mid，但它说明 target 是 mocap 对象。
    mid = model.body("target").mocapid[0]

    # 从 configuration 里取 model/data，保证 viewer 和 IK 用的是同一套状态对象。
    model = configuration.model
    data = configuration.data

    # QP solver 后端。mink.solve_ik 最终会调用 qpsolvers。
    solver = "daqp"

    with mujoco.viewer.launch_passive(
        model=model, data=data, show_left_ui=False, show_right_ui=False
    ) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        # 学习步骤 6: 初始化机器人姿态。
        #
        # update_from_keyframe("home"):
        # - 从 MuJoCo XML 里的 home keyframe 读取 q。
        # - 更新 configuration 内部 data。
        configuration.update_from_keyframe("home")

        # 把当前 home 姿态设为 PostureTask 的参考目标。
        # 后续 IK 会优先追末端，同时尽量不要偏离这个姿态太多。
        posture_task.set_target(configuration.q)

        # 学习步骤 7: 初始化 mocap target。
        #
        # 作用:
        # - 把名为 target 的 mocap body 移动到当前 attachment_site 的位置。
        # - 这样仿真刚开始时，目标和末端重合，不会突然产生很大误差。
        mink.move_mocap_to_frame(model, data, "target", "attachment_site", "site")

        # 学习步骤 8: 固定控制循环频率。
        #
        # RateLimiter(frequency=200.0):
        # - 目标频率是 200 Hz。
        # - rate.dt 约等于 0.005 s。
        # - rate.sleep() 会在循环末尾控制节奏。
        rate = RateLimiter(frequency=200.0, warn=False)

        while viewer.is_running():
            # 学习步骤 9: 从 mocap target 读取当前目标位姿。
            #
            # T_wt 可以理解为:
            # - w: world
            # - t: target
            # - target 在 world 中的 SE3 位姿。
            #
            # 第一轮只需要知道: 这一步把 MuJoCo 里的目标转换成 mink.SE3。
            T_wt = mink.SE3.from_mocap_name(model, data, "target")

            # 学习步骤 10: 更新末端任务目标。
            #
            # end_effector_task 会在 solve_ik 里比较:
            # - 当前 attachment_site 位姿
            # - 目标 T_wt
            #
            # 然后生成 task error 和 task Jacobian。
            end_effector_task.set_target(T_wt)

            # 学习步骤 11: 求解 differential IK。
            #
            # 输入:
            # - configuration: 当前机器人状态。
            # - tasks: 末端位姿任务 + 姿态保持任务。
            # - rate.dt: 当前控制周期。
            # - solver: QP solver 名称。
            # - limits: 位置、碰撞、速度限制。
            #
            # 输出:
            # - vel: tangent space velocity，shape 是 (nv,)。
            #
            # 重要理解:
            # - solve_ik 不直接返回目标 q。
            # - solve_ik 返回当前这一小步应该走的速度。
            vel = mink.solve_ik(configuration, tasks, rate.dt, solver, limits=limits)

            # 学习步骤 12: 把速度积分回 configuration。
            #
            # integrate_inplace 做两件事:
            # - 用 MuJoCo 的积分函数从 q 和 vel 得到下一步 q。
            # - 调用 update，让 data 中的 FK 结果刷新。
            #
            # 这一步完成后，下一帧 solve_ik 会基于新的 configuration 继续求解。
            configuration.integrate_inplace(vel, rate.dt)

            # 更新 camera light，让 viewer 显示更正常。
            mujoco.mj_camlight(model, data)

            # 下面两行是碰撞避免可视化相关。
            # 第一轮阅读可以先跳过；它们服务 CollisionAvoidanceLimit 的传感器显示。
            mujoco.mj_fwdPosition(model, data)
            mujoco.mj_sensorPos(model, data)

            # 学习步骤 13: viewer 同步和循环限频。
            #
            # viewer.sync(): 把当前 data 显示到窗口。
            # rate.sleep(): 维持 200 Hz 左右的循环。
            viewer.sync()
            rate.sleep()
