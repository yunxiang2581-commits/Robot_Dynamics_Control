from __future__ import annotations

import argparse
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path


def force_x11_for_viewer() -> None:
    """Force MuJoCo viewer to use X11/XWayland before importing mujoco."""
    needs_reexec = (
        os.environ.get("WAYLAND_DISPLAY")
        and os.environ.get("_MUJOCO_FORCE_X11_DONE") != "1"
    )

    os.environ["XDG_SESSION_TYPE"] = "x11"
    os.environ["GDK_BACKEND"] = "x11"
    os.environ["QT_QPA_PLATFORM"] = "xcb"
    os.environ.pop("WAYLAND_DISPLAY", None)

    if needs_reexec:
        os.environ["_MUJOCO_FORCE_X11_DONE"] = "1"
        os.execvpe(sys.executable, [sys.executable, *sys.argv], os.environ)


force_x11_for_viewer()

import mujoco
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MJCF_CANDIDATES = [
    PROJECT_ROOT / "third_party/unitree_ros/robots/h1_description/mjcf/scene_with_hand_bright.xml",
    PROJECT_ROOT / "unitree_ros/robots/h1_description/mjcf/scene_with_hand_bright.xml",
    PROJECT_ROOT / "third_party/unitree_ros/robots/h1_description/mjcf/h1_with_hand.xml",
    PROJECT_ROOT / "unitree_ros/robots/h1_description/mjcf/h1_with_hand.xml",
]


def resolve_mjcf() -> Path:
    for path in MJCF_CANDIDATES:
        if path.exists():
            return path
    raise FileNotFoundError(
        "MJCF not found. Checked:\n" + "\n".join(str(p) for p in MJCF_CANDIDATES)
    )


def run_simulation(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    duration: float,
    amp_deg: float,
    freq_hz: float,
    use_viewer: bool,
    kp: float,
    kd: float,
    lock_base: bool,
):
    joint_name = "left_knee_joint"
    foot_body_name = "left_ankle_link"
    pelvis_body_name = "pelvis"

    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        raise RuntimeError(f"Joint not found: {joint_name}")

    foot_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, foot_body_name)
    if foot_body_id < 0:
        raise RuntimeError(f"Body not found: {foot_body_name}")

    pelvis_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, pelvis_body_name)
    if pelvis_body_id < 0:
        raise RuntimeError(f"Body not found: {pelvis_body_name}")

    # actuator 与 joint 同名时，优先走 torque 控制。
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name)

    qpos_adr = model.jnt_qposadr[joint_id]
    dof_adr = model.jnt_dofadr[joint_id]
    amp_rad = math.radians(amp_deg)

    # 初始状态：优先使用 keyframe 0（若存在）。
    if model.nkey > 0:
        data.qpos[:] = model.key_qpos[0]
        data.qvel[:] = 0.0
        if model.nu > 0:
            data.ctrl[:] = 0.0
        mujoco.mj_forward(model, data)

    # 固定基观测模式：锁定 freejoint 的 qpos/qvel，避免自然下坠影响实验观察。
    root_qpos = None
    if lock_base and model.nq >= 7 and model.nv >= 6:
        root_qpos = data.qpos[:7].copy()

    def enforce_base_lock() -> None:
        if root_qpos is None:
            return
        data.qpos[:7] = root_qpos
        data.qvel[:6] = 0.0

    enforce_base_lock()
    mujoco.mj_forward(model, data)

    q0 = float(data.qpos[qpos_adr])
    p0 = data.xpos[foot_body_id].copy()
    pelvis0 = data.xpos[pelvis_body_id].copy()
    pr0 = p0 - pelvis0

    logs = []

    def step_once(sim_t: float) -> None:
        enforce_base_lock()

        q = float(data.qpos[qpos_adr])
        dq = float(data.qvel[dof_adr])
        q_des = q0 + amp_rad * math.sin(2.0 * math.pi * freq_hz * sim_t)

        if act_id >= 0:
            tau = kp * (q_des - q) + kd * (0.0 - dq)
            data.ctrl[act_id] = tau
        else:
            # 无 actuator 时直接写 qpos（教学/调试 fallback）
            data.qpos[qpos_adr] = q_des
            mujoco.mj_forward(model, data)

        mujoco.mj_step(model, data)
        enforce_base_lock()
        mujoco.mj_forward(model, data)

        p = data.xpos[foot_body_id].copy()
        pelvis_p = data.xpos[pelvis_body_id].copy()
        pr = p - pelvis_p
        dp = p - p0

        logs.append(
            (
                float(data.time),
                q,
                dq,
                q_des,
                float(p[0]),
                float(p[1]),
                float(p[2]),
                float(pelvis_p[0]),
                float(pelvis_p[1]),
                float(pelvis_p[2]),
                float(pr[0]),
                float(pr[1]),
                float(pr[2]),
                float(dp[0]),
                float(dp[1]),
                float(dp[2]),
            )
        )

    if use_viewer:
        import mujoco.viewer as mj_viewer

        wall_t0 = time.perf_counter()
        with mj_viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and data.time < duration:
                sim_t = float(data.time)
                step_once(sim_t)
                viewer.sync()

                # 近似实时播放
                wall_elapsed = time.perf_counter() - wall_t0
                sim_elapsed = float(data.time)
                sleep_s = sim_elapsed - wall_elapsed
                if sleep_s > 0:
                    time.sleep(sleep_s)
    else:
        while data.time < duration:
            sim_t = float(data.time)
            step_once(sim_t)

    return {
        "joint_name": joint_name,
        "joint_id": int(joint_id),
        "qpos_adr": int(qpos_adr),
        "dof_adr": int(dof_adr),
        "actuator_id": int(act_id),
        "foot_body_name": foot_body_name,
        "foot_body_id": int(foot_body_id),
        "pelvis_body_name": pelvis_body_name,
        "pelvis_body_id": int(pelvis_body_id),
        "q0": q0,
        "amp_deg": amp_deg,
        "amp_rad": amp_rad,
        "freq_hz": freq_hz,
        "duration": duration,
        "lock_base": lock_base,
        "p0": p0,
        "pelvis0": pelvis0,
        "pr0": pr0,
        "logs": logs,
    }


def save_plots(logs_arr: np.ndarray, out_dir: Path) -> list[Path]:
    """保存变化图表：关节角/速度跟踪、左脚相对 pelvis 位移、左脚世界位移。"""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plot_dir = out_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)

    t = logs_arr[:, 0]
    q = logs_arr[:, 1]
    dq = logs_arr[:, 2]
    q_des = logs_arr[:, 3]
    dq_des = np.gradient(q_des, t, edge_order=1)

    # world positions
    px, py, pz = logs_arr[:, 4], logs_arr[:, 5], logs_arr[:, 6]
    # relative-to-pelvis positions
    prx, pry, prz = logs_arr[:, 10], logs_arr[:, 11], logs_arr[:, 12]

    dprx = prx - prx[0]
    dpry = pry - pry[0]
    dprz = prz - prz[0]

    paths: list[Path] = []

    # 图1：关节角跟踪
    fig1 = plt.figure(figsize=(9, 5))
    ax1 = fig1.add_subplot(111)
    ax1.plot(t, q, label="q (left_knee)")
    ax1.plot(t, q_des, "--", label="q_des")
    ax1.set_xlabel("time [s]")
    ax1.set_ylabel("joint angle [rad]")
    ax1.set_title("Left Knee Angle Tracking")
    ax1.grid(True)
    ax1.legend()
    p1 = plot_dir / "left_knee_tracking.png"
    fig1.tight_layout()
    fig1.savefig(p1, dpi=180)
    plt.close(fig1)
    paths.append(p1)

    # 图2：关节速度跟踪（你要的速度曲线）
    fig2 = plt.figure(figsize=(9, 5))
    ax2 = fig2.add_subplot(111)
    ax2.plot(t, dq, label="dq (left_knee)")
    ax2.plot(t, dq_des, "--", label="dq_des")
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("joint velocity [rad/s]")
    ax2.set_title("Left Knee Velocity Tracking")
    ax2.grid(True)
    ax2.legend()
    p2 = plot_dir / "left_knee_velocity.png"
    fig2.tight_layout()
    fig2.savefig(p2, dpi=180)
    plt.close(fig2)
    paths.append(p2)

    # 图3：左脚相对 pelvis 位移变化
    fig3 = plt.figure(figsize=(9, 5))
    ax3 = fig3.add_subplot(111)
    ax3.plot(t, dprx, label="dpr_x")
    ax3.plot(t, dpry, label="dpr_y")
    ax3.plot(t, dprz, label="dpr_z")
    ax3.set_xlabel("time [s]")
    ax3.set_ylabel("relative displacement [m]")
    ax3.set_title("Left Foot Relative Displacement (w.r.t Pelvis)")
    ax3.grid(True)
    ax3.legend()
    p3 = plot_dir / "left_foot_relative_displacement.png"
    fig3.tight_layout()
    fig3.savefig(p3, dpi=180)
    plt.close(fig3)
    paths.append(p3)

    # 图4：左脚世界坐标轨迹（3轴）
    fig4 = plt.figure(figsize=(9, 5))
    ax4 = fig4.add_subplot(111)
    ax4.plot(t, px, label="p_x")
    ax4.plot(t, py, label="p_y")
    ax4.plot(t, pz, label="p_z")
    ax4.set_xlabel("time [s]")
    ax4.set_ylabel("position in world [m]")
    ax4.set_title("Left Foot World Position")
    ax4.grid(True)
    ax4.legend()
    p4 = plot_dir / "left_foot_world_position.png"
    fig4.tight_layout()
    fig4.savefig(p4, dpi=180)
    plt.close(fig4)
    paths.append(p4)

    return paths


def main() -> None:
    parser = argparse.ArgumentParser(description="MuJoCo realtime visualization for left-knee perturbation")
    parser.add_argument("--duration", type=float, default=8.0, help="Simulation duration in seconds")
    parser.add_argument("--amp-deg", type=float, default=10.0, help="Left knee sinusoidal amplitude in degree")
    parser.add_argument("--freq-hz", type=float, default=0.5, help="Sinusoidal frequency in Hz")
    parser.add_argument("--kp", type=float, default=80.0, help="PD proportional gain")
    parser.add_argument("--kd", type=float, default=6.0, help="PD derivative gain")
    parser.add_argument("--headless", action="store_true", help="Run without viewer window")
    parser.add_argument(
        "--free-base",
        action="store_true",
        help="Enable floating-base dynamics (robot may fall). Default is locked-base observation mode.",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Skip plotting module (default: plots enabled).",
    )
    args = parser.parse_args()

    mjcf_path = resolve_mjcf()
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)

    lock_base = not args.free_base

    result = run_simulation(
        model=model,
        data=data,
        duration=args.duration,
        amp_deg=args.amp_deg,
        freq_hz=args.freq_hz,
        use_viewer=not args.headless,
        kp=args.kp,
        kd=args.kd,
        lock_base=lock_base,
    )

    logs = result["logs"]
    if not logs:
        raise RuntimeError("No simulation logs collected.")

    logs_arr = np.asarray(logs, dtype=float)

    p0 = result["p0"]
    pr0 = result["pr0"]

    pend = logs_arr[-1, 4:7]
    pelvis_end = logs_arr[-1, 7:10]
    pr_end = logs_arr[-1, 10:13]

    dp_global = pend - p0
    dp_rel_pelvis = pr_end - pr0

    lines = []
    lines.append("MuJoCo H1 Left-Knee Perturbation Visualization")
    lines.append("=" * 56)
    lines.append(f"MJCF: {mjcf_path}")
    lines.append(f"nq={model.nq}, nv={model.nv}, nu={model.nu}")
    lines.append("")
    lines.append("[Mode]")
    lines.append(f"lock_base={result['lock_base']}  (use --free-base to disable)")
    lines.append("")
    lines.append("[Perturbation]")
    lines.append(f"joint={result['joint_name']} (id={result['joint_id']}, qpos_adr={result['qpos_adr']})")
    lines.append(f"actuator_id={result['actuator_id']}")
    lines.append(f"q0={result['q0']:.6f} rad")
    lines.append(f"amp={result['amp_deg']:.4f} deg ({result['amp_rad']:.6f} rad)")
    lines.append(f"freq={result['freq_hz']:.4f} Hz, duration={result['duration']:.4f} s")
    lines.append("")
    lines.append("[Left Foot Body]")
    lines.append(f"body={result['foot_body_name']} (id={result['foot_body_id']})")
    lines.append(f"start_pos_world=[{p0[0]: .6f}, {p0[1]: .6f}, {p0[2]: .6f}]")
    lines.append(f"end_pos_world  =[{pend[0]: .6f}, {pend[1]: .6f}, {pend[2]: .6f}]")
    lines.append(f"delta_world    =[{dp_global[0]: .6f}, {dp_global[1]: .6f}, {dp_global[2]: .6f}]")
    lines.append("")
    lines.append("[Left Foot Relative To Pelvis]")
    lines.append(f"start_rel=[{pr0[0]: .6f}, {pr0[1]: .6f}, {pr0[2]: .6f}]")
    lines.append(f"end_rel  =[{pr_end[0]: .6f}, {pr_end[1]: .6f}, {pr_end[2]: .6f}]")
    lines.append(
        f"delta_rel=[{dp_rel_pelvis[0]: .6f}, {dp_rel_pelvis[1]: .6f}, {dp_rel_pelvis[2]: .6f}]"
    )
    lines.append("")
    lines.append(f"pelvis_end_world=[{pelvis_end[0]: .6f}, {pelvis_end[1]: .6f}, {pelvis_end[2]: .6f}]")
    lines.append(f"samples={len(logs)}")

    report = "\n".join(lines)
    print(report)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = PROJECT_ROOT / "outputs" / "mujoco_left_knee_perturb" / ts
    out_dir.mkdir(parents=True, exist_ok=True)

    report_path = out_dir / "report.txt"
    report_path.write_text(report, encoding="utf-8")

    csv_path = out_dir / "timeseries.csv"
    header = "time,q,dq,q_des,px,py,pz,pelx,pely,pelz,prx,pry,prz,dpx,dpy,dpz"
    np.savetxt(csv_path, logs_arr, delimiter=",", header=header, comments="")

    print(f"\nSaved report: {report_path}")
    print(f"Saved csv   : {csv_path}")

    if not args.no_plot:
        try:
            plot_paths = save_plots(logs_arr, out_dir)
            for p in plot_paths:
                print(f"Saved plot  : {p}")
        except Exception as exc:
            print(f"Plot module skipped due to error: {exc}")


if __name__ == "__main__":
    main()
