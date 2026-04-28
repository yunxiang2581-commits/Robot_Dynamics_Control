from pathlib import Path
from datetime import datetime
import argparse
import pinocchio as pin


def find_candidate_urdf(project_root: Path) -> Path | None:
    candidates = [
        project_root / "unitree_ros/robots/h1_description/urdf",
        project_root / "unitree_ros/robots/h1_2_description/urdf",
        project_root / "unitree_ros/robots/r1_description/urdf",
        project_root / "unitree_ros/robots/g1_description/urdf",
    ]

    for folder in candidates:
        if folder.exists():
            urdfs = sorted(folder.glob("*.urdf"))
            if urdfs:
                return urdfs[0]
    return None


def print_model_summary(model: pin.Model, tag: str) -> None:
    print(f"\n{'=' * 20} {tag} {'=' * 20}")
    print(f"model.name: {model.name}")
    print(f"nq: {model.nq}")
    print(f"nv: {model.nv}")
    print(f"njoints: {model.njoints}")
    print(f"nframes: {len(model.frames)}")

    print("\n[Joint Names]")
    for i, name in enumerate(model.names):
        print(f"{i:>3}: {name}")

    print("\n[First 40 Frames]")
    for i, frame in enumerate(model.frames[:40]):
        print(f"{i:>3}: {frame.name} (parent joint id: {frame.parent})")

    print("\n[First 20 Inertias]")
    for i, inertia in enumerate(model.inertias[:20]):
        print(
            f"{i:>3}: mass={inertia.mass:.6f}, "
            f"lever={inertia.lever.T}"
        )


def collect_interesting_frames(model: pin.Model):
    keywords = ["foot", "ankle", "hand", "wrist", "pelvis", "torso", "imu"]
    matched = []
    for i, frame in enumerate(model.frames):
        lname = frame.name.lower()
        if any(k in lname for k in keywords):
            matched.append((i, frame.name, frame.parent))
    return matched


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--urdf",
        type=str,
        default=None,
        help="Path to URDF file. If omitted, script will auto-search common Unitree humanoid folders.",
    )
    args = parser.parse_args()

    project_root = Path(__file__).resolve().parents[1]

    if args.urdf is not None:
        urdf_path = Path(args.urdf).resolve()
    else:
        found = find_candidate_urdf(project_root)
        if found is None:
            raise FileNotFoundError(
                "No URDF found automatically. Please pass --urdf path/to/model.urdf"
            )
        urdf_path = found.resolve()

    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    print(f"Using URDF: {urdf_path}")

    # 固定基模型
    model_fixed = pin.buildModelFromUrdf(str(urdf_path))
    print_model_summary(model_fixed, "FIXED BASE")

    # 浮动基模型
    model_ff = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    print_model_summary(model_ff, "FLOATING BASE")

    print("\n[Interesting Frames in Floating Base Model]")
    for item in collect_interesting_frames(model_ff):
        print(f"id={item[0]:>3}, name={item[1]}, parent={item[2]}")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_dir = project_root / "outputs" / "task1_inspect_humanoid_model" / timestamp
    report_dir.mkdir(parents=True, exist_ok=True)
    report_path = report_dir / "task1_model_summary.txt"

    with report_path.open("w", encoding="utf-8") as f:
        f.write(f"URDF: {urdf_path}\n")
        f.write(f"FIXED BASE nq={model_fixed.nq}, nv={model_fixed.nv}, njoints={model_fixed.njoints}\n")
        f.write(f"FLOATING BASE nq={model_ff.nq}, nv={model_ff.nv}, njoints={model_ff.njoints}\n\n")
        f.write("[Joint Names]\n")
        for i, name in enumerate(model_ff.names):
            f.write(f"{i:>3}: {name}\n")

        f.write("\n[Interesting Frames]\n")
        for item in collect_interesting_frames(model_ff):
            f.write(f"id={item[0]:>3}, name={item[1]}, parent={item[2]}\n")

    print(f"\nSummary written to: {report_path}")


if __name__ == "__main__":
    main()
