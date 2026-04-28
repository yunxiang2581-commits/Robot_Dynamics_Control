from __future__ import annotations

from pathlib import Path

import numpy as np
import pinocchio as pin


def resolve_urdf(project_root: Path) -> Path:
    candidates = [
        project_root / "third_party/unitree_ros/robots/h1_description/urdf/h1_with_hand.urdf",
        project_root / "unitree_ros/robots/h1_description/urdf/h1_with_hand.urdf",
    ]
    for path in candidates:
        if path.exists():
            return path
    raise FileNotFoundError(
        "URDF not found. Checked:\n" + "\n".join(str(p) for p in candidates)
    )


def pick_left_foot_frame(model: pin.Model) -> str:
    candidates = [
        "left_ankle_link",
        "left_ankle_joint",
        "left_foot",
        "left_sole",
    ]
    frame_names = {frame.name for frame in model.frames}
    for name in candidates:
        if name in frame_names:
            return name
    raise RuntimeError(
        "Left foot frame not found. Tried: " + ", ".join(candidates)
    )


def pick_left_knee_joint(model: pin.Model) -> str:
    candidates = [
        "left_knee_joint",
        "left_knee",
    ]
    joint_names = set(model.names)
    for name in candidates:
        if name in joint_names:
            return name
    raise RuntimeError(
        "Left knee joint not found. Tried: " + ", ".join(candidates)
    )


def save_compare_plot(
    out_path: Path,
    v_pred: np.ndarray,
    v_fd: np.ndarray,
    err: np.ndarray,
) -> None:
    """绘制 Jacobian 预测速度与有限差分速度对比图。"""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels = ["x", "y", "z"]
    x = np.arange(3)
    width = 0.35

    fig = plt.figure(figsize=(8, 5))
    ax = fig.add_subplot(111)

    ax.bar(x - width / 2, v_pred, width=width, label="Jv (pred)")
    ax.bar(x + width / 2, v_fd, width=width, label="finite-diff")

    for i in range(3):
        ax.text(x[i], max(v_pred[i], v_fd[i]), f"e={err[i]:.2e}", ha="center", va="bottom", fontsize=8)

    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_xlabel("axis")
    ax.set_ylabel("linear velocity [m/s]")
    ax.set_title("Left Foot Linear Velocity: Jv vs Finite Difference")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()

    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def main() -> None:
    project_root = Path(__file__).resolve().parents[1]

    # 1-3) 读取 URDF 并创建浮动基模型
    urdf_path = resolve_urdf(project_root)
    model = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    data = model.createData()

    # 4) neutral configuration
    q = pin.neutral(model)

    # 5) 选择左脚 frame
    target_frame = pick_left_foot_frame(model)
    frame_id = model.getFrameId(target_frame)

    # 6) 选择左膝关节
    knee_joint_name = pick_left_knee_joint(model)
    knee_joint_id = model.getJointId(knee_joint_name)
    if knee_joint_id <= 0:
        raise RuntimeError(f"Invalid joint id for {knee_joint_name}: {knee_joint_id}")

    # 7) 构造广义速度 v（只给左膝速度）
    knee_vel_value = 0.2  # rad/s
    v = np.zeros(model.nv)
    idx_v = model.idx_vs[knee_joint_id]
    nvs = model.nvs[knee_joint_id]
    if nvs <= 0:
        raise RuntimeError(f"Joint {knee_joint_name} has invalid nvs={nvs}")
    v[idx_v] = knee_vel_value

    # 8) Jacobian 管线
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    pin.computeJointJacobians(model, data, q)
    j_frame = pin.getFrameJacobian(model, data, frame_id, pin.LOCAL_WORLD_ALIGNED)
    j_frame = np.asarray(j_frame)

    # 9) Jacobian 预测线速度
    v_pred = j_frame[:3, :] @ v

    # 10) 有限差分验证
    dt = 1e-4

    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    p_now = data.oMf[frame_id].translation.copy()

    q_next = pin.integrate(model, q, v * dt)
    pin.forwardKinematics(model, data, q_next)
    pin.updateFramePlacements(model, data)
    p_next = data.oMf[frame_id].translation.copy()

    v_fd = (p_next - p_now) / dt

    err = v_pred - v_fd
    err_norm = float(np.linalg.norm(err))

    # 11) 保存输出
    out_dir = project_root / "outputs" / "jacobian"
    out_dir.mkdir(parents=True, exist_ok=True)

    txt_path = out_dir / "h1_left_foot_jacobian_check.txt"
    plot_path = out_dir / "h1_left_foot_jacobian_check.png"

    lines = []
    lines.append("H1 Left Foot Jacobian Check")
    lines.append("=" * 44)
    lines.append(f"URDF: {urdf_path}")
    lines.append(f"target frame: {target_frame}")
    lines.append(f"selected knee joint: {knee_joint_name}")
    lines.append(f"nq={model.nq}, nv={model.nv}, njoints={model.njoints}, nframes={len(model.frames)}")
    lines.append(f"nonzero knee velocity: {knee_vel_value:.6f} rad/s")
    lines.append(f"dt: {dt:.1e}")
    lines.append("")
    lines.append("predicted linear velocity from Jv:")
    lines.append(f"[{v_pred[0]: .9f}, {v_pred[1]: .9f}, {v_pred[2]: .9f}]")
    lines.append("")
    lines.append("finite-difference linear velocity:")
    lines.append(f"[{v_fd[0]: .9f}, {v_fd[1]: .9f}, {v_fd[2]: .9f}]")
    lines.append("")
    lines.append("velocity error (Jv - FD):")
    lines.append(f"[{err[0]: .9e}, {err[1]: .9e}, {err[2]: .9e}]")
    lines.append(f"error norm: {err_norm:.9e}")

    report = "\n".join(lines)
    print(report)

    txt_path.write_text(report, encoding="utf-8")
    save_compare_plot(plot_path, v_pred, v_fd, err)

    print(f"\nSaved text: {txt_path}")
    print(f"Saved plot: {plot_path}")


if __name__ == "__main__":
    main()
