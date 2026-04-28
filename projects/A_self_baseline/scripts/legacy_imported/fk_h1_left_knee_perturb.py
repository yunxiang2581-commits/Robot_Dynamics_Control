from __future__ import annotations

from datetime import datetime
from pathlib import Path
import argparse

import numpy as np
import pinocchio as pin


PROJECT_ROOT = Path(__file__).resolve().parents[1]
URDF_CANDIDATES = [
    PROJECT_ROOT / "third_party/unitree_ros/robots/h1_description/urdf/h1_with_hand.urdf",
    PROJECT_ROOT / "unitree_ros/robots/h1_description/urdf/h1_with_hand.urdf",
]


def resolve_urdf() -> Path:
    for path in URDF_CANDIDATES:
        if path.exists():
            return path
    raise FileNotFoundError(
        "URDF not found. Checked:\n" + "\n".join(str(p) for p in URDF_CANDIDATES)
    )


def pick_frame(model: pin.Model, keywords: list[str]) -> str | None:
    frame_names = [frame.name for frame in model.frames]
    for kw in keywords:
        for name in frame_names:
            if kw.lower() in name.lower():
                return name
    return None


def run_fk(model: pin.Model, data: pin.Data, q: np.ndarray) -> None:
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)


def save_jacobian_compare_plot(out_dir: Path, dp_real: np.ndarray, dp_pred: np.ndarray) -> Path:
    """保存实测位移 vs Jacobian 线性预测位移对比图。"""
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise RuntimeError(
            "matplotlib is required for plotting. Install with: pip install matplotlib"
        ) from exc

    axes = ["x", "y", "z"]
    x = np.arange(len(axes))
    width = 0.35

    fig = plt.figure(figsize=(8, 5))
    ax = fig.add_subplot(111)

    ax.bar(x - width / 2, dp_real, width, label="actual Δp")
    ax.bar(x + width / 2, dp_pred, width, label="jacobian-pred Δp")

    ax.set_xticks(x)
    ax.set_xticklabels(axes)
    ax.set_xlabel("axis")
    ax.set_ylabel("displacement [m]")
    ax.set_title("Left Foot Δp: FK Actual vs Jacobian Prediction")
    ax.grid(True, axis="y", alpha=0.35)
    ax.legend()

    plot_dir = out_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    plot_path = plot_dir / "left_foot_delta_compare.png"

    fig.tight_layout()
    fig.savefig(plot_path, dpi=180)
    plt.close(fig)

    return plot_path


def main() -> None:
    parser = argparse.ArgumentParser(description="Left-knee perturbation FK experiment for H1")
    parser.add_argument(
        "--delta-deg",
        type=float,
        default=10.0,
        help="Perturbation on left_knee_joint in degree (default: 10.0)",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Skip comparison plot generation.",
    )
    args = parser.parse_args()

    # 1) 读取 URDF
    urdf_path = resolve_urdf()

    # 2) 创建模型和 data
    model = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    data = model.createData()

    # 找关键 joint/frame
    left_knee_joint_name = "left_knee_joint"
    left_knee_jid = model.getJointId(left_knee_joint_name)
    if left_knee_jid <= 0:
        raise RuntimeError(f"Joint not found: {left_knee_joint_name}")

    left_foot_frame = pick_frame(model, ["left_ankle_link", "left_foot", "left_ankle"])
    if left_foot_frame is None:
        raise RuntimeError("Cannot find left foot frame")

    # 3) 取 neutral configuration
    q_base = pin.neutral(model)

    # 4) 做 FK（基线）
    run_fk(model, data, q_base)
    left_foot_fid = model.getFrameId(left_foot_frame)
    p_base = data.oMf[left_foot_fid].translation.copy()

    # 对左膝做单关节扰动
    delta_rad = np.deg2rad(args.delta_deg)
    q_perturbed = q_base.copy()
    idx_q = model.idx_qs[left_knee_jid]
    nq_joint = model.nqs[left_knee_jid]
    if nq_joint != 1:
        raise RuntimeError(
            f"Expected 1-DoF for {left_knee_joint_name}, but got nqs={nq_joint}"
        )

    q_perturbed[idx_q] += delta_rad

    # 限幅到关节位置限制
    lower = model.lowerPositionLimit[idx_q]
    upper = model.upperPositionLimit[idx_q]
    q_perturbed[idx_q] = np.clip(q_perturbed[idx_q], lower, upper)

    # 4) 做 FK（扰动后）
    run_fk(model, data, q_perturbed)
    p_perturbed = data.oMf[left_foot_fid].translation.copy()

    # 5) 找 pelvis / 左脚 / 右脚 frame（用于记录）
    pelvis_frame = pick_frame(model, ["pelvis", "base", "torso"])
    right_foot_frame = pick_frame(model, ["right_ankle_link", "right_foot", "right_ankle"])

    # 6) 输出位姿变化（实测）
    dp_real = p_perturbed - p_base
    dist = float(np.linalg.norm(dp_real))

    # Jacobian 线性预测：Δp_pred ≈ J_linear[:,k] * Δq_k
    pin.forwardKinematics(model, data, q_base)
    pin.updateFramePlacements(model, data)
    pin.computeJointJacobians(model, data, q_base)
    j_frame = pin.getFrameJacobian(model, data, left_foot_fid, pin.LOCAL_WORLD_ALIGNED)
    j_frame = np.asarray(j_frame)

    idx_v = model.idx_vs[left_knee_jid]
    nv_joint = model.nvs[left_knee_jid]
    if nv_joint != 1:
        raise RuntimeError(
            f"Expected 1-DoF velocity for {left_knee_joint_name}, but got nvs={nv_joint}"
        )

    delta_q_eff = float(q_perturbed[idx_q] - q_base[idx_q])
    j_col_linear = j_frame[:3, idx_v]
    dp_pred = j_col_linear * delta_q_eff

    pred_err = dp_real - dp_pred
    pred_err_norm = float(np.linalg.norm(pred_err))

    lines = []
    lines.append("H1 Left-Knee Perturbation FK Experiment")
    lines.append("=" * 50)
    lines.append(f"URDF: {urdf_path}")
    lines.append(
        f"model: nq={model.nq}, nv={model.nv}, njoints={model.njoints}, nframes={len(model.frames)}"
    )
    lines.append("")
    lines.append("[Frames]")
    lines.append(f"pelvis: {pelvis_frame}")
    lines.append(f"left_foot: {left_foot_frame}")
    lines.append(f"right_foot: {right_foot_frame}")
    lines.append("")
    lines.append("[Perturbation]")
    lines.append(f"joint: {left_knee_joint_name} (jid={left_knee_jid}, q_index={idx_q}, v_index={idx_v})")
    lines.append(f"delta_cmd: {args.delta_deg:.4f} deg ({delta_rad:.6f} rad)")
    lines.append(f"delta_eff: {delta_q_eff:.6f} rad")
    lines.append(f"joint_limit: [{lower:.6f}, {upper:.6f}] rad")
    lines.append(f"q_base[{idx_q}]={q_base[idx_q]:.6f} rad")
    lines.append(f"q_perturbed[{idx_q}]={q_perturbed[idx_q]:.6f} rad")
    lines.append("")
    lines.append("[Left Foot Position in World]")
    lines.append(
        "base      : "
        f"[{p_base[0]: .6f}, {p_base[1]: .6f}, {p_base[2]: .6f}]"
    )
    lines.append(
        "perturbed : "
        f"[{p_perturbed[0]: .6f}, {p_perturbed[1]: .6f}, {p_perturbed[2]: .6f}]"
    )
    lines.append(
        "delta_real: "
        f"[{dp_real[0]: .6f}, {dp_real[1]: .6f}, {dp_real[2]: .6f}]"
    )
    lines.append(f"delta_real_norm: {dist:.6f} m")
    lines.append("")
    lines.append("[Jacobian Linear Prediction]")
    lines.append(
        "J_linear_col(knee): "
        f"[{j_col_linear[0]: .6f}, {j_col_linear[1]: .6f}, {j_col_linear[2]: .6f}]"
    )
    lines.append(
        "delta_pred       : "
        f"[{dp_pred[0]: .6f}, {dp_pred[1]: .6f}, {dp_pred[2]: .6f}]"
    )
    lines.append(
        "pred_error       : "
        f"[{pred_err[0]: .6f}, {pred_err[1]: .6f}, {pred_err[2]: .6f}]"
    )
    lines.append(f"pred_error_norm: {pred_err_norm:.6f} m")

    report = "\n".join(lines)
    print(report)

    # 7) 保存结果
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = PROJECT_ROOT / "outputs" / "fk" / timestamp
    out_dir.mkdir(parents=True, exist_ok=True)

    out_file = out_dir / "h1_left_knee_perturb_report.txt"
    out_file.write_text(report, encoding="utf-8")
    print(f"\nSaved report: {out_file}")

    if not args.no_plot:
        plot_path = save_jacobian_compare_plot(out_dir, dp_real, dp_pred)
        print(f"Saved plot  : {plot_path}")


if __name__ == "__main__":
    main()
