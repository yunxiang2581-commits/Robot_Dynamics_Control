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
        "Left foot frame not found. Tried candidates: "
        + ", ".join(candidates)
    )


def main() -> None:
    project_root = Path(__file__).resolve().parents[1]

    # 1) Resolve URDF path
    urdf_path = resolve_urdf(project_root)

    # 2) Build floating-base model and data
    model = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    data = model.createData()

    # 3) Neutral configuration
    q = pin.neutral(model)

    # 4) Select target frame
    target_frame = pick_left_foot_frame(model)
    frame_id = model.getFrameId(target_frame)

    # 5) Compute Jacobian pipeline
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    pin.computeJointJacobians(model, data, q)
    j_frame = pin.getFrameJacobian(model, data, frame_id, pin.LOCAL_WORLD_ALIGNED)

    j_frame = np.asarray(j_frame)
    j_linear = j_frame[:3, :]
    j_angular = j_frame[3:, :]

    # 6) Build report
    lines = []
    lines.append("H1 Left Foot Jacobian Analysis")
    lines.append("=" * 48)
    lines.append(f"URDF: {urdf_path}")
    lines.append(f"target frame: {target_frame}")
    lines.append(
        f"nq={model.nq}, nv={model.nv}, njoints={model.njoints}, nframes={len(model.frames)}"
    )
    lines.append(f"Jacobian shape: {j_frame.shape}")
    lines.append("")
    lines.append("[Full Jacobian]")
    lines.append(np.array2string(j_frame, precision=6, suppress_small=False))
    lines.append("")
    lines.append("[Linear Velocity Jacobian (first 3 rows)]")
    lines.append(np.array2string(j_linear, precision=6, suppress_small=False))
    lines.append("")
    lines.append("[Angular Velocity Jacobian (last 3 rows)]")
    lines.append(np.array2string(j_angular, precision=6, suppress_small=False))

    report = "\n".join(lines)
    print(report)

    # 7) Save output
    out_dir = project_root / "outputs" / "jacobian"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_file = out_dir / "h1_left_foot_jacobian.txt"
    out_file.write_text(report, encoding="utf-8")
    print(f"\nSaved to: {out_file}")


if __name__ == "__main__":
    main()
