from pathlib import Path
import pinocchio as pin

# FK script for Unitree H1 (with hand).
# 功能：加载 URDF（浮动基），在 neutral 位形下计算并导出关键 frame 位姿。

# 以脚本位置为基准定位项目根，避免依赖当前 shell 的工作目录。
PROJECT_ROOT = Path(__file__).resolve().parents[1]

# 兼容两种目录布局：
# 1) third_party 下的 unitree_ros
# 2) 当前项目根目录下的 unitree_ros
URDF_CANDIDATES = [
    PROJECT_ROOT / "third_party/unitree_ros/robots/h1_description/urdf/h1_with_hand.urdf",
    PROJECT_ROOT / "unitree_ros/robots/h1_description/urdf/h1_with_hand.urdf",
]

# FK 输出目录（固定到 outputs/fk）。
OUT_DIR = PROJECT_ROOT / "outputs" / "fk"
OUT_DIR.mkdir(parents=True, exist_ok=True)


def pick_frame(model, keywords):
    # 简单策略：按关键词顺序，返回第一个命中的 frame 名。
    frame_names = [f.name for f in model.frames]
    for kw in keywords:
        for name in frame_names:
            if kw.lower() in name.lower():
                return name
    return None


def fmt_pose(name, oMf):
    # 将 Pinocchio 的 SE3 位姿格式化成可读文本。
    p = oMf.translation
    R = oMf.rotation
    return (
        f"{name}\n"
        f"  position: [{p[0]: .6f}, {p[1]: .6f}, {p[2]: .6f}]\n"
        f"  rotation:\n{R}\n"
    )


def resolve_urdf() -> Path:
    # 顺序尝试候选 URDF，找不到则抛明确错误。
    for p in URDF_CANDIDATES:
        if p.exists():
            return p
    raise FileNotFoundError(
        "URDF not found. Checked:\n" + "\n".join(str(p) for p in URDF_CANDIDATES)
    )


def main():
    # 1. 读取 URDF
    urdf_path = resolve_urdf()

    # 2. 创建模型和 data
    model = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    data = model.createData()

    # 3. 取 neutral configuration
    q = pin.neutral(model)

    # 4. 做 FK
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    # 5. 找 pelvis / 左脚 / 右脚 frame
    targets = {
        "pelvis": ["pelvis", "base", "torso"],
        "left_foot": ["left_ankle_link", "left_foot", "l_ankle", "l_foot", "left_ankle"],
        "right_foot": ["right_ankle_link", "right_foot", "r_ankle", "r_foot", "right_ankle"],
    }
    chosen = {k: pick_frame(model, v) for k, v in targets.items()}

    # 6. 输出位姿
    lines = []
    lines.append(f"URDF: {urdf_path}")
    lines.append(
        f"nq={model.nq}, nv={model.nv}, njoints={model.njoints}, nframes={len(model.frames)}"
    )
    lines.append("")
    lines.append("=== chosen frames ===")
    for k, v in chosen.items():
        lines.append(f"{k}: {v}")
    lines.append("")

    for tag, frame_name in chosen.items():
        if frame_name is None:
            lines.append(f"{tag}: NOT FOUND\n")
            continue
        fid = model.getFrameId(frame_name)
        lines.append(fmt_pose(frame_name, data.oMf[fid]))

    report = "\n".join(lines)
    print(report)

    # 7. 保存结果
    out_file = OUT_DIR / "h1_fk_report.txt"
    out_file.write_text(report, encoding="utf-8")
    print(f"\nSaved to: {out_file}")


if __name__ == "__main__":
    main()
