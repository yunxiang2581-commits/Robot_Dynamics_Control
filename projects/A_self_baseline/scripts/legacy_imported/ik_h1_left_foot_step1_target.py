from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pinocchio as pin


# =============================
# 路径常量
# =============================
# TODO(Path-Const-1): 观察 __file__ 与 Path.cwd() 的区别
# - 你可以把这一行临时改成 Path.cwd()，比较两者在不同启动目录下的行为差异。
# - 学习目标：理解“脚本所在目录”与“命令执行目录”的区别。
PROJECT_ROOT = Path(__file__).resolve().parents[1]

# TODO(Path-Const-2): 修改输出子目录命名
# - 例如改成 PROJECT_ROOT / "outputs" / "ik" / "step1"。
# - 学习目标：掌握 pathlib 的路径拼接与分层管理。
OUT_DIR = PROJECT_ROOT / "outputs" / "ik"

# URDF 候选路径（按优先级顺序尝试）
# TODO(Path-In-1): 你可以在这里追加自己的 URDF 路径候选。
# - 建议规则：把“最常用路径”放在前面。
# - 学习目标：理解 fallback 路径策略。
URDF_CANDIDATES = [
    PROJECT_ROOT / "third_party/unitree_ros/robots/h1_description/urdf/h1_with_hand.urdf",
    PROJECT_ROOT / "unitree_ros/robots/h1_description/urdf/h1_with_hand.urdf",
]

# 左脚 frame 候选名（按优先级顺序尝试）
# TODO(Frame-1): 如果你的 URDF 命名不同，在这里添加新候选。
# - 学习目标：理解“命名不一致时的兼容查找”。
LEFT_FOOT_FRAME_CANDIDATES = [
    "left_ankle_link",
    "left_ankle_joint",
    "left_foot",
    "left_sole",
]

# 输出文件：任务 4.1 的目标点记录
# TODO(Path-Out-1): 你可以把文件名改成带日期或任务编号的形式。
# - 学习目标：形成可追溯的输出习惯。
OUT_FILE = OUT_DIR / "h1_left_foot_step1_target.txt"


def resolve_urdf() -> Path:
    """
    从候选列表中返回第一个存在的 URDF 路径。

    TODO(Path-In-2): 你可以把这个函数扩展为支持 override 参数。
    - 例如：resolve_urdf(override: str | None = None)
    - 规则建议：有 override 时优先检查 override；否则走候选列表。
    """
    for urdf_path in URDF_CANDIDATES:
        if urdf_path.exists():
            return urdf_path

    # TODO(Path-In-3): 练习把报错信息改成更“教学友好”的格式，
    # 例如分行列出每个路径并标注是否存在。
    raise FileNotFoundError(
        "URDF not found. Checked:\n" + "\n".join(str(p) for p in URDF_CANDIDATES)
    )


def pick_frame(model: pin.Model, candidates: list[str]) -> str:
    """
    从候选 frame 名中选出模型中真实存在的第一个名字。

    TODO(Frame-2): 你可以打印 model.frames 的前 N 个名字，
    帮助理解为什么某些候选匹配不到。
    """
    frame_names = {frame.name for frame in model.frames}
    for name in candidates:
        if name in frame_names:
            return name

    # TODO(Frame-3): 练习增强异常信息，附带建议：
    # “先运行模型检查脚本，确认真实 frame 名后再补候选列表”。
    raise RuntimeError("Target frame not found. Tried: " + ", ".join(candidates))


def main() -> None:
    """
    任务 4.1：左脚 IK 目标定义（骨架版）

    说明：
    - 本脚本故意保留关键逻辑为 TODO，便于你逐步手写学习。
    - 请按 TODO 0 -> TODO 9 顺序依次补齐。
    """

    # TODO 0: 处理路径输入参数（URDF 输入路径 + 输出文件路径）
    # 这个块要做什么：支持命令行覆盖默认路径，练习“输入路径”和“输出路径”管理。
    # 为什么需要这一步：真实项目中路径经常变化，不能写死在代码里。
    # 期望 API：
    #   - argparse.ArgumentParser()
    #   - parser.add_argument("--urdf", type=str, default=None)
    #   - parser.add_argument("--out", type=str, default=None)
    #   - args = parser.parse_args()
    # 输入：命令行参数（可选）
    # 输出：args.urdf, args.out
    # 练习提示：
    #   - 如果 args.urdf 不为空，优先使用 Path(args.urdf).resolve()
    #   - 如果 args.out 不为空，优先使用 Path(args.out).resolve()
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=str, default=None)
    parser.add_argument("--out", type=str, default=None)
    args = parser.parse_args()

    # TODO 1: 解析 URDF 路径
    # 这个块要做什么：确定后续建模要使用哪个 URDF 文件。
    # 为什么需要这一步：Pinocchio 建模必须知道 URDF 文件路径。
    # 期望 API：resolve_urdf() + Path.exists()
    # 输入：args.urdf（可选）
    # 输出：urdf_path（Path）
    # 示例：
    #   - 无参数时：urdf_path = resolve_urdf()
    #   - 有参数时：urdf_path = Path(args.urdf).resolve()
    if args.urdf is None:
        urdf_path = resolve_urdf()
    else:
        urdf_path = Path(args.urdf).resolve()
        if not urdf_path.exists():
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")

    # TODO 2: 构建浮动基模型和 data
    # 这个块要做什么：构建浮动基模型（free-flyer）并创建 data 缓冲区。
    # 为什么需要这一步：后续 FK / frame 位姿读取都依赖 model 和 data。
    # 期望 API：
    #   - pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    #   - model.createData()
    # 输入：urdf_path（Path）
    # 输出：model（pin.Model）, data（pin.Data）
    model = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    data = model.createData()

    # TODO 3: 取 neutral configuration
    # 这个块要做什么：获取机器人中立位形 q。
    # 为什么需要这一步：FK 需要一个具体的关节配置作为输入。
    # 期望 API：pin.neutral(model)
    # 输入：model
    # 输出：q（通常是 np.ndarray）
    q = pin.neutral(model)


    # TODO 4: 做 FK 和 updateFramePlacements
    # 这个块要做什么：在 q 下更新全模型 frame 位姿。
    # 为什么需要这一步：只有更新后 data.oMf[frame_id] 才是当前有效位姿。
    # 期望 API：
    #   - pin.forwardKinematics(model, data, q)
    #   - pin.updateFramePlacements(model, data)
    # 输入：model, data, q
    # 输出：data 中 frame 位姿被更新
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    # TODO 5: 选择左脚 frame
    # 这个块要做什么：从候选名中选出左脚 frame，并拿到 frame_id。
    # 为什么需要这一步：不同 URDF 命名不统一，要先做兼容查找。
    # 期望 API：
    #   - left_foot_frame = pick_frame(model, LEFT_FOOT_FRAME_CANDIDATES)
    #   - frame_id = model.getFrameId(left_foot_frame)
    # 输入：model, LEFT_FOOT_FRAME_CANDIDATES
    # 输出：left_foot_frame（str）, frame_id（int）
    left_foot_frame = pick_frame(model, LEFT_FOOT_FRAME_CANDIDATES)
    frame_id = model.getFrameId(left_foot_frame)

    # TODO 6: 读取当前左脚位置 p0
    # 这个块要做什么：读取当前左脚 frame 的平移向量。
    # 为什么需要这一步：IK 目标通常由当前点 p0 加一个位移得到。
    # 期望 API：data.oMf[frame_id].translation
    # 输入：data, frame_id
    # 输出：p0（shape=(3,)）
    p0 = data.oMf[frame_id].translation

    # TODO 7: 定义目标位置 p_des = p0 + [0.02, 0, 0]
    # 这个块要做什么：把左脚目标沿 x 方向前移 2cm。
    # 为什么需要这一步：这是 IK 下一步要逼近的目标点。
    # 期望 API：np.array([0.02, 0.0, 0.0])
    # 输入：p0
    # 输出：p_des
    # 示例：p_des = p0 + np.array([0.02, 0.0, 0.0])
    p_des = p0 + np.array([0.02, 0.0, 0.0])

    # TODO 8: 组织输出文本内容（report 字符串）
    # 这个块要做什么：把关键中间结果整理为可读文本。
    # 为什么需要这一步：先把内容组织清楚，再写文件更容易调试。
    # 期望 API：f-string / "\n".join(...)
    # 输入：urdf_path, left_foot_frame, p0, p_des
    # 输出：report（str）

    report = f"URDF Path: {urdf_path}\nLeft Foot Frame: {left_foot_frame}\nCurrent Position: {p0}\nDesired Position: {p_des}"

    # TODO 9: 处理输出路径并写文件
    # 这个块要做什么：根据默认路径或 --out 参数，最终把 report 写到磁盘。
    # 为什么需要这一步：这是“路径输出管理”的核心练习。
    # 期望 API：
    #   - out_file.parent.mkdir(parents=True, exist_ok=True)
    #   - out_file.write_text(report, encoding="utf-8")
    # 输入：args.out（可选）, report（str）
    # 输出：输出文件（默认 OUT_FILE）
    # 练习提示：
    #   - 无参数时：out_file = OUT_FILE
    #   - 有参数时：out_file = Path(args.out).resolve()

    if args.out is None:
        out_file = OUT_FILE
    else:
        out_file = Path(args.out).resolve()
    out_file.parent.mkdir(parents=True, exist_ok=True)
    out_file.write_text(report, encoding="utf-8")
    

if __name__ == "__main__":
    main()
