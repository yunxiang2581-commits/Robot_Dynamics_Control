"""
@Author  : Administrator  # 创建者（自动获取系统用户名）
@Time    : 2026/4/1 21:55  # 创建时间（自动生成）
@File    : exp_pd_joint_control.py  # 文件名
@Description :  # 实验脚本
    这个实验脚本实现了一个单关节机械臂的 PD 控制实验。
    目标是通过 PD 控制器控制机械臂从初始位置收敛到目标位置（0 弧度）。

    使用的模型：
    - 单关节机械臂模型（single_joint.xml）。该模型在 `envs/` 文件夹中。

    控制器：
    - 使用了一个经典的 PD 控制器，控制公式为：
        tau = Kp * (qd - q) + Kd * (dqd - dq)
        其中：
        - tau: 控制输入（力矩）
        - qd: 目标位置
        - q: 当前位置
        - dqd: 目标速度
        - dq: 当前速度

    输出文件：
    - `pd_joint_log.csv`：记录时间、位置（q）、速度（dq）、控制输入（tau）和目标位置（qd）的实验数据。
    - `pd_joint_position.png`：记录位置响应（目标位置与实际位置随时间变化的曲线图）。
    - `pd_joint_torque.png`：记录控制力矩随时间变化的曲线图。
    - `run_info.txt`：实验元信息文件，记录本次实验的参数和运行设置。
"""

import os
import sys
import time
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer

# 让 Python 能找到 controllers 目录
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(PROJECT_ROOT)

from controllers.pd_controller import PDController


def main():
    # ===== 0. 实验配置 =====
    experiment_name = "exp_pd_joint_control"
    run_time = datetime.now().strftime("%Y%m%d_%H%M%S")

    # 可视化开关
    enable_viewer = True       # True: 打开 MuJoCo 实时窗口
    realtime_playback = True   # True: 尽量按真实时间播放

    # ===== 1. 为本次实验创建独立文件夹 =====
    run_dir = os.path.join(PROJECT_ROOT, "outputs", experiment_name, run_time)
    data_dir = os.path.join(run_dir, "data")
    plots_dir = os.path.join(run_dir, "plots")
    meta_dir = os.path.join(run_dir, "meta")

    os.makedirs(data_dir, exist_ok=True)
    os.makedirs(plots_dir, exist_ok=True)
    os.makedirs(meta_dir, exist_ok=True)

    # ===== 2. 加载模型 =====
    model_path = os.path.join(PROJECT_ROOT, "envs", "single_joint.xml")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # ===== 3. 设置初始状态 =====
    initial_q = 0.8
    initial_dq = 0.0
    data.qpos[0] = initial_q
    data.qvel[0] = initial_dq
    mujoco.mj_forward(model, data)

    # ===== 4. 定义控制器 =====
    kp = 80
    kd = 16.0
    qd = 0.0
    dqd = 0.0
    controller = PDController(kp=kp, kd=kd, qd=qd, dqd=dqd)

    # ===== 5. 仿真参数 =====
    sim_time = 5.0
    dt = model.opt.timestep
    steps = int(sim_time / dt)

    # ===== 6. 日志记录 =====
    time_log = []
    q_log = []
    dq_log = []
    tau_log = []
    qd_log = []

    # ===== 7. 主仿真逻辑 =====
    def run_loop(viewer=None):
        wall_start = time.perf_counter()

        for step in range(steps):
            # 当前状态
            q = float(data.qpos[0])
            dq = float(data.qvel[0])

            # 计算控制输入
            tau = controller.compute(q, dq)

            # 记录“步进前”的数据，保证各列长度一致
            time_log.append(float(data.time))
            q_log.append(q)
            dq_log.append(dq)
            tau_log.append(tau)
            qd_log.append(qd)

            # 施加控制
            data.ctrl[0] = tau

            # 仿真一步
            mujoco.mj_step(model, data)

            # 可视化同步
            if viewer is not None:
                viewer.sync()

                # 真实时间播放：让仿真速度接近现实时间
                if realtime_playback:
                    sim_elapsed = (step + 1) * dt
                    wall_elapsed = time.perf_counter() - wall_start
                    sleep_time = sim_elapsed - wall_elapsed
                    if sleep_time > 0:
                        time.sleep(sleep_time)

                # 如果窗口被手动关掉，就提前结束
                if not viewer.is_running():
                    break

    # ===== 8. 运行仿真（有/无 viewer） =====
    if enable_viewer:
        # passive viewer 是非阻塞模式：你自己写循环、自己 mj_step、自己 sync
        with mujoco.viewer.launch_passive(model, data) as viewer:
            run_loop(viewer)
    else:
        run_loop()

    # ===== 9. 转为数组 =====
    time_log = np.array(time_log)
    q_log = np.array(q_log)
    dq_log = np.array(dq_log)
    tau_log = np.array(tau_log)
    qd_log = np.array(qd_log)

    # ===== 10. 保存 csv =====
    save_data = np.column_stack([time_log, q_log, dq_log, tau_log, qd_log])
    csv_path = os.path.join(data_dir, "pd_joint_log.csv")
    np.savetxt(
        csv_path,
        save_data,
        delimiter=",",
        header="time,q,dq,tau,qd",
        comments=""
    )

    # ===== 11. 保存本次实验元信息 =====
    info_path = os.path.join(meta_dir, "run_info.txt")
    with open(info_path, "w", encoding="utf-8") as f:
        f.write(f"experiment_name: {experiment_name}\n")
        f.write(f"run_time: {run_time}\n")
        f.write(f"model_path: {model_path}\n")
        f.write(f"sim_time: {sim_time}\n")
        f.write(f"dt: {dt}\n")
        f.write(f"steps: {steps}\n")
        f.write(f"kp: {kp}\n")
        f.write(f"kd: {kd}\n")
        f.write(f"qd: {qd}\n")
        f.write(f"dqd: {dqd}\n")
        f.write(f"initial_q: {initial_q}\n")
        f.write(f"initial_dq: {initial_dq}\n")
        if len(q_log) > 0:
            f.write(f"final_q: {q_log[-1]:.6f}\n")
            f.write(f"final_dq: {dq_log[-1]:.6f}\n")
        f.write(f"enable_viewer: {enable_viewer}\n")
        f.write(f"realtime_playback: {realtime_playback}\n")

    # ===== 12. 画图并保存 =====
    position_fig_path = os.path.join(plots_dir, "pd_joint_position.png")
    torque_fig_path = os.path.join(plots_dir, "pd_joint_torque.png")

    # 图1：角度响应
    plt.figure(figsize=(8, 5))
    plt.plot(time_log, q_log, label="q")
    plt.plot(time_log, qd_log, "--", label="qd")
    plt.xlabel("Time [s]")
    plt.ylabel("Joint Angle [rad]")
    plt.title("PD Joint Position Control")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(position_fig_path, dpi=200)
    plt.show()

    # 图2：控制力矩
    plt.figure(figsize=(8, 5))
    plt.plot(time_log, tau_log, label="tau")
    plt.xlabel("Time [s]")
    plt.ylabel("Torque [Nm]")
    plt.title("Control Torque")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(torque_fig_path, dpi=200)
    plt.show()

    # ===== 13. 终端输出 =====
    print("Experiment finished.")
    if len(q_log) > 0:
        print(f"Final q = {q_log[-1]:.4f} rad")
        print(f"Final dq = {dq_log[-1]:.4f} rad/s")
    print(f"Run directory: {run_dir}")
    print(f"CSV saved to: {csv_path}")
    print(f"Plots saved to: {plots_dir}")
    print(f"Meta saved to: {info_path}")


if __name__ == "__main__":
    main()