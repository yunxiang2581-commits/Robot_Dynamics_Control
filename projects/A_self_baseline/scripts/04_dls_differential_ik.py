"""A04 DLS differential IK TODO learning skeleton.

Pipeline 步骤：
- A04 DLS differential IK。

对标 mink 的概念：
- `mink.solve_ik` 的最小无约束教学版。
- 本脚本只规划如何从 task-space error 和 A03 已验证的 Jacobian 求关节速度 `dq`。

与 A03 的关系：
- A03 已验证 target site 的速度映射 `site velocity = J(q) dq`。
- A04 未来会复用 A03 的 target site、初始 q、site Jacobian 验证结果和误差解释。
- 如果 A03 的 Jacobian 方向、site id 或维度错误，A04 的 IK 会朝错误方向更新。

本脚本输入：
- `projects/A_self_baseline/configs/ik.yaml`。
- `projects/A_self_baseline/outputs/cache/A01_model_summary.json`。
- `projects/A_self_baseline/outputs/cache/A02_site_pose.json`。
- `projects/A_self_baseline/outputs/cache/A03_jacobian_check.json`。
- `shared/robot_assets/models/mink_universal_robots_ur5e/scene.xml`。
- target site，例如 `attachment_site`。
- target pose。
- damping。
- gain。
- max_iter。

本脚本未来输出：
- `outputs/trajectories/A04_dls_ik_q_traj.npy`。
- `outputs/logs/A04_dls_ik_error.csv`。
- `outputs/figures/A04_dls_ik_error.png`。
- `outputs/reports/A04_dls_ik_report.md`。

pose-aware 规划补充：
- A04 保留 position mode 作为第一版最小学习目标。
- 后续 pose_6d mode 会把 position error 和 orientation error 组合成同一个 task error。
- position mode:
  e_task = w_pos * e_pos
  J_task = w_pos * J_pos
- pose_6d mode:
  e_task = [w_pos * e_pos; w_rot * e_rot]
  J_task = [w_pos * J_pos; w_rot * J_rot]
- orientation error 未来使用 SO(3) log map:
  R_err = R_target R_current^T
  e_rot = log(R_err)
- 本次只补充接口、TODO 和文档说明，不强制完整实现 rotation log map。

当前状态：
- TODO learning skeleton。
- 不调用 mink 替代自己的实现。
- 不做 QP / actuator tracking / MuJoCo 控制 / collision avoidance / video recording。
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path
import sys
import json
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt



A_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_IK_CONFIG = A_ROOT / "configs" / "ik.yaml"
DEFAULT_ROBOT_CONFIG = A_ROOT / "configs" / "robot.yaml"
DEFAULT_A01_SUMMARY = A_ROOT / "outputs" / "cache" / "A01_model_summary.json"
DEFAULT_A02_POSE = A_ROOT / "outputs" / "cache" / "A02_site_pose.json"
DEFAULT_A03_JACOBIAN = A_ROOT / "outputs" / "cache" / "A03_jacobian_check.json"
DEFAULT_OUTPUT_DIR = A_ROOT / "outputs"
DEFAULT_SITE_NAME = "attachment_site"
SRC_ROOT = A_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.append(str(SRC_ROOT))

from robot_baseline import model_loader  # noqa: E402


PRINCIPLE_NOTES = [
    "differential IK 是在当前 q 附近求一个小 dq，而不是一次直接求完整 q。",
    "IK 可以用速度级迭代来做，因为小位移下有 Delta x 约等于 J(q) Delta q。",
    "DLS 比普通伪逆更稳定，因为 damping 能抑制奇异附近过大的 dq。",
    "阻尼 lambda / damping 的作用是让更新更温和，避免速度爆炸。",
    "gain 控制每一步朝目标误差靠近的强度，过大会震荡，过小会很慢。",
    "A04 必须建立在 A03 Jacobian 验证之后，否则 IK 会沿错误方向更新。",
    "A04 只做无约束 IK；A05 才加入 task、limit 和 QP-IK。",
    "A04 生成 q_traj，后续 A06 target tracking 和 A07 actuator tracking 会复用。",
    "A04 第一版保留 position mode；pose_6d mode 后续会加入 orientation error 和 J_rot。",
    "position 和 rotation 单位不同，pose-aware DLS 必须用 position_weight / orientation_weight 平衡尺度。",
]


TODO_TITLES = [
    "TODO 1: 读取 A01 / A02 / A03 前置产物",
    "TODO 2: 读取 robot.yaml / ik.yaml 并解析 scene.xml",
    "TODO 3: 加载 MuJoCo model 和 data",
    "TODO 4: 恢复初始 q",
    "TODO 5: 定义 target pose",
    "TODO 6: 计算当前 site pose 和误差 e",
    "TODO 7: 计算 site position Jacobian J_pos",
    "TODO 8: DLS 求解 dq",
    "TODO 9: 积分更新 q",
    "TODO 10: 迭代终止条件",
    "TODO 10A: IK 主循环结构",
    "TODO 11: 记录 q trajectory 和 error log",
    "TODO 12: 规划误差图输出",
    "TODO 13: 规划 Markdown report 输出",
    "TODO 14: 说明 A04 不进入 A05/A07",
    "TODO 15: 定义 target orientation",
    "TODO 16: 计算 orientation error",
    "TODO 17: 组合 task error",
    "TODO 18: 组合 task Jacobian",
    "TODO 19: 记录 position / orientation error",
]


def parse_args() -> argparse.Namespace:
    """解析 A04 TODO skeleton 的 CLI 参数。"""
    parser = argparse.ArgumentParser(description="A04 TODO skeleton: DLS differential IK.")
    parser.add_argument("--ik-config", default=str(DEFAULT_IK_CONFIG), help="未来 ik.yaml 路径。")
    parser.add_argument("--robot-config", default=str(DEFAULT_ROBOT_CONFIG), help="robot.yaml 路径。")
    parser.add_argument("--a01-summary", default=str(DEFAULT_A01_SUMMARY), help="A01 model summary。")
    parser.add_argument("--a02-pose", default=str(DEFAULT_A02_POSE), help="A02 site pose cache。")
    parser.add_argument("--a03-jacobian", default=str(DEFAULT_A03_JACOBIAN), help="A03 Jacobian check cache。")
    parser.add_argument("--mjcf", default=None, help="可选：覆盖 robot.yaml 中的 scene.xml 路径。")
    parser.add_argument("--site", default=DEFAULT_SITE_NAME, help="目标 site，默认 attachment_site。")
    parser.add_argument(
        "--task-mode",
        choices=("position", "pose_6d"),
        default="position",
        help="A04 任务模式：position 只用 J_pos；pose_6d 未来组合 J_pos/J_rot。",
    )
    parser.add_argument("--target-pose", default=None, help="TODO：未来目标 pose 输入。")
    parser.add_argument("--target-offset", default="0.03,0.00,0.00", help="TODO：第一版 position target offset。")
    parser.add_argument(
        "--target-position-offset",
        default="0.05,0.00,0.00",
        help="TODO：pose-aware 规划中的目标位置偏移，格式 dx,dy,dz。",
    )
    parser.add_argument(
        "--target-orientation-mode",
        choices=("keep_current", "fixed_rpy", "fixed_quat"),
        default="keep_current",
        help="TODO：目标姿态来源；第一版 keep_current，后续支持 fixed_rpy / fixed_quat。",
    )
    parser.add_argument("--position-weight", type=float, default=1.0, help="TODO：position error 权重。")
    parser.add_argument("--orientation-weight", type=float, default=0.2, help="TODO：orientation error 权重。")
    parser.add_argument("--damping", type=float, default=1e-3, help="DLS damping 占位参数。")
    parser.add_argument("--gain", type=float, default=1.0, help="误差缩放占位参数。")
    parser.add_argument("--dt", type=float, default=1e-2, help="未来 q 积分步长。")
    parser.add_argument("--max-iter", type=int, default=50, help="未来最大迭代次数。")
    parser.add_argument("--tolerance", type=float, default=1e-4, help="未来收敛阈值。")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR), help="未来输出根目录。")
    parser.add_argument("--log-level", default="INFO", help="日志级别，例如 INFO / DEBUG。")
    return parser.parse_args()


def log_principles() -> None:
    """打印 A04 的原理说明。"""
    logging.info("A04 原理说明:")
    for note in PRINCIPLE_NOTES:
        logging.info("- %s", note)


def log_todo_titles() -> None:
    """打印 A04 TODO 简要列表。"""
    logging.info("A04 TODO 任务清单:")
    for title in TODO_TITLES:
        logging.info("- %s", title)


def main() -> None:
    """A04 TODO learning skeleton 主流程。

    当前只打印 A04 的未来输入、未来输出、原理说明和 TODO 清单。
    核心 DLS differential IK 逻辑继续保留为 NotImplementedError。
    """
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(levelname)s: %(message)s",
    )

    output_root = Path(args.output_dir).expanduser()
    if not output_root.is_absolute():
        output_root = A_ROOT / output_root

    A04_traj = output_root / "trajectories" / "A04_dls_ik_q_traj.npy"
    A04_log = output_root / "logs" / "A04_dls_ik_error.csv"
    A04_figure = output_root / "figures" / "A04_dls_ik_error.png"
    A04_report = output_root / "reports" / "A04_dls_ik_report.md"

    logging.info("A04 当前状态: TODO learning skeleton")
    logging.info("本步骤不实现 DLS IK，只规划从误差和 Jacobian 求 dq 的学习结构。")
    logging.info("未来输入:")
    logging.info("- ik config: %s", args.ik_config)
    logging.info("- robot config: %s", args.robot_config)
    logging.info("- A01 summary: %s", args.a01_summary)
    logging.info("- A02 pose: %s", args.a02_pose)
    logging.info("- A03 Jacobian: %s", args.a03_jacobian)
    logging.info("- mjcf override: %s", args.mjcf)
    logging.info("- target site: %s", args.site)
    logging.info("- task mode: %s", args.task_mode)
    logging.info("- target pose: %s", args.target_pose)
    logging.info("- target offset: %s", args.target_offset)
    logging.info("- target position offset: %s", args.target_position_offset)
    logging.info("- target orientation mode: %s", args.target_orientation_mode)
    logging.info("- position/orientation weight: %s / %s", args.position_weight, args.orientation_weight)
    logging.info("- damping/gain/dt/max_iter/tolerance: %s / %s / %s / %s / %s",
                 args.damping, args.gain, args.dt, args.max_iter, args.tolerance)
    logging.info("未来输出:")
    logging.info("- trajectory: %s", A04_traj)
    logging.info("- error log: %s", A04_log)
    logging.info("- error figure: %s", A04_figure)
    logging.info("- report: %s", A04_report)

    log_principles()
    log_todo_titles()

    # =============================
    # TODO 1: 读取 A01 / A02 / A03 前置产物
    # =============================
    # 要做什么：
    # - 未来读取 A01_model_summary.json、A02_site_pose.json、A03_jacobian_check.json。
    # - 确认 nq/nv/nu、target site、q_source、A03 Jacobian 检查误差。
    #
    # 为什么需要：
    # - A04 依赖 A03 已验证的 Jacobian；如果模型、q 或 site 不一致，IK 结果不可解释。
    #
    # 对标 mink：
    # - 对标 solve_ik 依赖当前 Configuration 和任务 frame 的一致性。
    #
    # 推荐 API：
    # - json.loads。
    # - Path.read_text。
    #
    # 输入：
    # - A01 summary。
    # - A02 pose cache。
    # - A03 Jacobian check cache。
    #
    # 输出：
    # - model_summary。
    # - pose_summary。
    # - jacobian_summary。
    #
    # 验证：
    # - nq=6, nv=6, nu=6。
    # - site_name == attachment_site。
    # - A03 的 J_pos shape 为 (3, 6)。
    a01_summary_path = Path(args.a01_summary).expanduser()
    a02_pose_path = Path(args.a02_pose).expanduser()
    a03_jacobian_path = Path(args.a03_jacobian).expanduser()

    if not a01_summary_path.is_absolute():
        a01_summary_path = A_ROOT / a01_summary_path
    if not a02_pose_path.is_absolute():
        a02_pose_path = A_ROOT / a02_pose_path
    if not a03_jacobian_path.is_absolute():
        a03_jacobian_path = A_ROOT / a03_jacobian_path

    if not a01_summary_path.exists():
        raise FileNotFoundError(f"找不到 A01 模型摘要: {a01_summary_path}")
    if not a02_pose_path.exists():
        raise FileNotFoundError(f"找不到 A02 site pose: {a02_pose_path}")
    if not a03_jacobian_path.exists():
        raise FileNotFoundError(f"找不到 A03 Jacobian 检查结果: {a03_jacobian_path}")

    model_summary = json.loads(a01_summary_path.read_text(encoding="utf-8"))
    pose_summary = json.loads(a02_pose_path.read_text(encoding="utf-8"))
    jacobian_summary = json.loads(a03_jacobian_path.read_text(encoding="utf-8"))

    if args.site not in model_summary["site_names"]:
        raise ValueError(f"目标 site 不在 A01 site_names 中: {args.site}")

    if pose_summary["site_name"] != args.site:
        raise ValueError(f"A02 site_name={pose_summary['site_name']} 与当前 site={args.site} 不一致")

    if jacobian_summary["site_name"] != args.site:
        raise ValueError(f"A03 site_name={jacobian_summary['site_name']} 与当前 site={args.site} 不一致")

    if jacobian_summary["J_pos_shape"] != [3, model_summary["nv"]]:
        raise ValueError(
            f"A03 J_pos_shape={jacobian_summary['J_pos_shape']}，但期望 [3, {model_summary['nv']}]"
        )

    logging.info("已读取 A01 模型摘要: %s", a01_summary_path)
    logging.info("已读取 A02 site pose: %s", a02_pose_path)
    logging.info("已读取 A03 Jacobian 检查结果: %s", a03_jacobian_path)
    logging.info("检查通过: A02/A03 与 A04 使用同一个 site: %s", args.site)

    # =============================
    # TODO 2: 读取 robot.yaml / ik.yaml 并解析 scene.xml
    # =============================
    # 要做什么：
    # - 未来读取 robot.yaml 获取 MJCF 路径。
    # - 未来读取 ik.yaml 获取 target offset、damping、gain、dt、max_iter、tolerance。
    #
    # 为什么需要：
    # - IK 参数必须配置化，便于复现实验和调参。
    #
    # 对标 mink：
    # - 对标 mink examples 中模型路径和 solver 参数集中定义的方式。
    #
    # 推荐 API：
    # - model_loader.load_yaml_config。
    # - model_loader.resolve_path。
    #
    # 输入：
    # - robot.yaml。
    # - ik.yaml。
    #
    # 输出：
    # - mjcf_path。
    # - ik_config。
    #
    # 验证：
    # - scene.xml exists=True。
    # - damping > 0。
    # - gain > 0。
    # - max_iter >= 1。

    robot_config = model_loader.load_yaml_config(args.robot_config)

    ik_config_path = Path(args.ik_config).expanduser()
    if not ik_config_path.is_absolute():
        ik_config_path = A_ROOT / ik_config_path

    if ik_config_path.exists():
        ik_config = model_loader.load_yaml_config(ik_config_path)
    else:
        logging.warning("未找到 ik.yaml，使用命令行默认参数: %s", ik_config_path)
        ik_config = {}

    mjcf_path = model_loader.resolve_path(
        args.mjcf or robot_config["mjcf_path"],
        A_ROOT,
    )

    solver_config = ik_config.get("solver", {})

    damping = float(solver_config.get("damping", args.damping))
    gain = float(solver_config.get("gain", args.gain))
    dt = float(solver_config.get("dt", solver_config.get("step_size", args.dt)))
    max_iter = int(solver_config.get("max_iter", args.max_iter))
    tolerance = float(solver_config.get("tolerance", args.tolerance))

    if not mjcf_path.exists():
        raise FileNotFoundError(f"找不到 MuJoCo scene.xml: {mjcf_path}")
    if damping <= 0:
        raise ValueError(f"damping 必须 > 0，当前是 {damping}")
    if gain <= 0:
        raise ValueError(f"gain 必须 > 0，当前是 {gain}")
    if dt <= 0:
        raise ValueError(f"dt 必须 > 0，当前是 {dt}")
    if max_iter < 1:
        raise ValueError(f"max_iter 必须 >= 1，当前是 {max_iter}")

    logging.info("已读取 robot.yaml: %s", args.robot_config)
    logging.info("已读取 ik.yaml: %s", ik_config_path)
    logging.info("MuJoCo scene.xml: %s", mjcf_path)
    logging.info("当前 IK 参数: damping=%s, gain=%s, dt=%s, max_iter=%s, tolerance=%s",
                damping, gain, dt, max_iter, tolerance)

    # =============================
    # TODO 3: 加载 MuJoCo model 和 data
    # =============================
    # 要做什么：
    # - 未来加载 MuJoCo model。
    # - 创建 mujoco.MjData(model)。
    #
    # 为什么需要：
    # - A04 每次迭代都要 forward q、读 site pose、计算 site Jacobian。
    #
    # 对标 mink：
    # - 对标 Configuration 持有 model/data 并负责更新状态。
    #
    # 推荐 API：
    # - model_loader.load_mujoco_model。
    # - mujoco.MjData。
    #
    # 输入：
    # - mjcf_path。
    #
    # 输出：
    # - model。
    # - data。
    #
    # 验证：
    # - model.nq == 6。
    # - model.nv == 6。
    # - data.qpos shape 与 nq 对齐。
    model = model_loader.load_mujoco_model(mjcf_path)
    data = mujoco.MjData(model)

    if model.nq != model_summary["nq"]:
        raise ValueError(f"model.nq={model.nq} 与 A01 nq={model_summary['nq']} 不匹配")

    if model.nv != model_summary["nv"]:
        raise ValueError(f"model.nv={model.nv} 与 A01 nv={model_summary['nv']} 不匹配")

    if model.nu != model_summary["nu"]:
        raise ValueError(f"model.nu={model.nu} 与 A01 nu={model_summary['nu']} 不匹配")

    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, args.site)
    if site_id < 0:
        raise ValueError(f"找不到目标 site: {args.site}")

    logging.info("已加载 MuJoCo model: nq=%s, nv=%s, nu=%s", model.nq, model.nv, model.nu)
    logging.info("已创建 MuJoCo data: qpos shape=%s, qvel shape=%s", data.qpos.shape, data.qvel.shape)
    logging.info("检查通过: 目标 site=%s, site_id=%s", args.site, site_id)

    # =============================
    # TODO 4: 恢复初始 q
    # =============================
    # 要做什么：
    # - 未来从 A02_site_pose.json 恢复 q。
    # - 或从 keyframe home 恢复初始配置。
    #
    # 为什么需要：
    # - A04 的 IK 起点必须和 A02/A03 验证过的 q 一致。
    #
    # 对标 mink：
    # - 对标 Configuration 当前 q。
    #
    # 推荐 API：
    # - numpy.asarray。
    # - model.key_qpos。
    # - mujoco.mj_forward。
    #
    # 输入：
    # - q_source。
    # - q。
    #
    # 输出：
    # - q0。
    #
    # 验证：
    # - q0.shape == (model.nq,)。
    # - q0 不含 NaN。
    q = np.asarray(pose_summary["q"], dtype=np.float64)

    if q.shape != (model.nq,):
        raise ValueError(f"q.shape={q.shape}，但期望 ({model.nq},)")

    if not np.all(np.isfinite(q)):
        raise ValueError(f"q 中存在 NaN 或 inf: {q}")

    q0 = q.copy()

    data.qpos[:] = q
    mujoco.mj_forward(model, data)

    logging.info("已从 A02 恢复初始 q")
    logging.info("A02 的 q 来源: %s", pose_summary.get("q_source"))
    logging.info("q0 shape: %s", q0.shape)
    logging.info("q0: %s", q0)

    # =============================
    # TODO 5: 定义 target pose
    # =============================
    # 要做什么：
    # - 第一版定义 position target：target = current + small offset。
    # - 暂不做 orientation target。
    #
    # 为什么需要：
    # - IK 需要明确目标；small offset 能避免一开始就测试不可达或大步线性化失效。
    #
    # 对标 mink：
    # - 对标 FrameTask / target transform 的目标定义。
    #
    # 推荐 API：
    # - numpy.asarray。
    # - data.site_xpos。
    #
    # 输入：
    # - current site position。
    # - target offset。
    #
    # 输出：
    # - x_target。
    #
    # 验证：
    # - x_target.shape == (3,)。
    # - target offset norm 不宜过大。

    offset_values = args.target_offset.split(",")
    if len(offset_values) != 3:
        raise ValueError(f"--target-offset 必须是 3 个数，当前是: {args.target_offset}")

    target_offset = np.array([float(v) for v in offset_values], dtype=np.float64)

    if target_offset.shape != (3,):
        raise ValueError(f"target_offset.shape={target_offset.shape}，但期望 (3,)")

    if not np.all(np.isfinite(target_offset)):
        raise ValueError(f"target_offset 中存在 NaN 或 inf: {target_offset}")

    x_current_initial = data.site_xpos[site_id].copy()
    x_target = x_current_initial + target_offset

    if x_target.shape != (3,):
        raise ValueError(f"x_target.shape={x_target.shape}，但期望 (3,)")

    logging.info("初始 site 位置: %s", x_current_initial)
    logging.info("目标位置偏移: %s", target_offset)
    logging.info("目标 site 位置: %s", x_target)

    # =============================
    # TODO 15: 定义 target orientation
    # =============================
    # 要做什么：
    # - 在 pose_6d mode 中规划目标姿态 R_target。
    # - keep_current: R_target = R_current。
    # - fixed_rpy: 未来从 roll/pitch/yaw 得到 R_target。
    # - fixed_quat: 未来从 quaternion 得到 R_target。
    #
    # 为什么需要：
    # - 机械臂末端任务通常不仅要到达某个点，还要以正确姿态到达。
    # - 抓取、插入、对接和工具操作都需要 orientation 约束。
    #
    # 对标 mink：
    # - 对标 FrameTask 的 target transform，其中包含 position 和 orientation。
    #
    # 推荐 API：
    # - scipy.spatial.transform.Rotation。
    # - 或后续在 src/robot_baseline 中自写 SO(3) helper。
    #
    # 输入：
    # - 当前 site rotation。
    # - target_orientation_mode。
    # - 未来 fixed_rpy / fixed_quat 参数。
    #
    # 输出：
    # - R_target。
    #
    # 验证：
    # - R_target.shape == (3, 3)。
    # - R_target.T @ R_target 约等于 I。
    R_current_initial = data.site_xmat[site_id].reshape(3, 3).copy()

    if args.target_orientation_mode == "keep_current":
        R_target = R_current_initial.copy()
        logging.info("目标姿态模式: keep_current，R_target = R_current")
    else:
        raise NotImplementedError(f"目标姿态模式 {args.target_orientation_mode} 尚未实现")
    
    if R_target.shape != (3, 3):
        raise ValueError(f"R_target.shape={R_target.shape}，但期望 (3, 3)") 
    
    ort_error = np.linalg.norm(R_target.T @ R_target - np.eye(3))
    if ort_error > 1e-6:
        raise ValueError(f"R_target 不是有效的旋转矩阵，R^T R 与 I 的差异为 {ort_error}")
    
    logging.info("初始 site 姿态 (rotation matrix):\n%s", R_current_initial)
    logging.info("目标 site 姿态 (rotation matrix):\n%s", R_target)

    # =============================
    # TODO 10A: IK 主循环结构
    # =============================
    # 要做什么：
    # - 后续把 TODO 6/10/7/17/18/8/9/11 收进同一个 IK 迭代循环。
    # - 推荐结构如下：
    #   for iter_index in range(max_iter):
    #       # TODO 6: forward 当前 q，计算当前 site pose、e_pos、e_rot
    #       # TODO 10: 先做收敛检查，误差已经足够小时不再求 dq
    #       # TODO 7: 计算 J_pos / J_rot
    #       # TODO 17: 根据 task_mode 组合 e_task
    #       # TODO 18: 根据 task_mode 组合 J_task
    #       # TODO 8: 用 DLS 从 J_task 和 e_task 求 dq
    #       # TODO 9: 用 dq 和 dt 积分得到 q_next
    #       # TODO 11/19: 记录 q、pose、position/orientation/task error 和 dq_norm
    #
    # 为什么需要：
    # - A04 不是一组散落代码块，而是“误差 -> Jacobian -> DLS -> integrate -> log”的迭代过程。
    # - 把循环结构先写清楚，Step 12B 实现 position mode 时更容易检查每一步放在哪里。
    #
    # 对标 mink：
    # - 对标 mink example 中 viewer loop / solver loop 每一帧调用 solve_ik 并更新 configuration。
    #
    # 推荐 API：
    # - range(max_iter)。
    # - mujoco.mj_forward。
    # - mujoco.mj_jacSite。
    # - mujoco.mj_integratePos。
    # - list / csv.DictWriter / numpy.save。
    #
    # 输入：
    # - q0、max_iter、tolerance、target pose、damping、gain、dt。
    #
    # 输出：
    # - q_traj。
    # - error_log。
    # - stop_reason。
    #
    # 验证：
    # - 每轮日志都有 iter_index。
    # - 收敛检查发生在 DLS 求解前。
    # - q_traj 长度和 error_log 行数对齐。
    # =============================
    q_traj = [q0.copy()]
    error_log = []

    converged = False
    stop_reason = "max_iter_reached"
    for iter_index in range(max_iter):
    # TODO 6: 计算当前 site pose 和误差 e
    # =============================
    # 要做什么：
    # - 每轮 forward q。
    # - 读取 current site position。
    # - 计算 e = x_target - x_current。
    #
    # 为什么需要：
    # - DLS 的目标是让 J dq 接近 gain * e。
    #
    # 对标 mink：
    # - 对标 task error 计算。
    #
    # 推荐 API：
    # - mujoco.mj_forward。
    # - data.site_xpos。
    # - numpy.linalg.norm。
    #
    # 输入：
    # - model。
    # - data。
    # - q。
    # - x_target。
    #
    # 输出：
    # - x_current。
    # - e。
    # - error_norm。
    #
    # 验证：
    # - e.shape == (3,)。
    # - error_norm 是有限数。
        data.qpos[:] = q
        mujoco.mj_forward(model, data)

        x_current = data.site_xpos[site_id].copy()
        e_pos = x_target - x_current
        error_norm = float(np.linalg.norm(e_pos))

        if e_pos.shape != (3,):
            raise ValueError(f"e_pos.shape={e_pos.shape}，但期望 (3,)")

        if not np.isfinite(error_norm):
            raise ValueError(f"error_norm 不是有限数: {error_norm}")

        logging.info("当前 site 位置: %s", x_current)
        logging.info("当前位置误差 e_pos: %s", e_pos)
        logging.info("当前位置误差范数: %s", error_norm)

    # =============================
    # TODO 16: 计算 orientation error
    # =============================
    # 要做什么：
    # - 在 pose_6d mode 中计算旋转误差：
    #   R_err = R_target R_current^T
    #   e_rot = log(R_err)
    # - e_rot 是三维 rotation vector。
    #
    # 为什么需要：
    # - orientation 不能简单用欧拉角直接相减；SO(3) log map 更适合描述小旋转误差。
    #
    # 对标 mink：
    # - 对标 FrameTask 的 orientation error 和 task-space residual。
    #
    # 推荐 API：
    # - scipy.spatial.transform.Rotation.from_matrix(...).as_rotvec()。
    # - 或自写 SO(3) log map helper。
    #
    # 输入：
    # - R_current。
    # - R_target。
    #
    # 输出：
    # - e_rot，方向是旋转轴，模长是旋转角，单位 rad。
    #
    # 验证：
    # - keep_current 时 e_rot norm 应接近 0。
        R_current = data.site_xmat[site_id].reshape(3, 3).copy()

        rotation_delta = R_target @ R_current.T
        e_rot = Rotation.from_matrix(rotation_delta).as_rotvec()
        e_rot_norm = np.linalg.norm(e_rot)

        if e_rot.shape != (3,):
            raise ValueError(f"e_rot.shape={e_rot.shape}，但期望 (3,)")
        
        if not np.isfinite(e_rot_norm):
            raise ValueError(f"e_rot_norm 不是有限数: {e_rot_norm}")
        
        logging.info("当前 site 姿态 (rotation matrix):\n%s", R_current)
        logging.info("旋转误差 e_rot (rotation vector): %s", e_rot)
        logging.info("旋转误差范数 (rotation angle in rad): %s", e_rot_norm)    
    # =============================
    # TODO 7: 计算 site position Jacobian J_pos
    # =============================
    # 要做什么：
    # - 未来计算 attachment_site 的 translational Jacobian。
    # - 第一版只用 J_pos，不使用 J_rot。
    #
    # 为什么需要：
    # - position IK 的线性关系是 Delta x 约等于 J_pos Delta q。
    #
    # 对标 mink：
    # - 对标 FrameTask 的 task Jacobian。
    #
    # 推荐 API：
    # - mujoco.mj_jacSite。
    # - numpy.zeros。
    #
    # 输入：
    # - model。
    # - data。
    # - site_id。
    #
    # 输出：
    # - J_pos。
    #
    # 验证：
    # - J_pos.shape == (3, model.nv)。
    # - J_pos 不含 NaN。
        jacp = np.zeros((3, model.nv), dtype=np.float64)
        jacr = np.zeros((3, model.nv), dtype=np.float64)
        mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
        J_pos = jacp.reshape(3, model.nv)
        if J_pos.shape != (3, model.nv):
            raise ValueError(f"J_pos.shape={J_pos.shape}，但期望 (3, {model.nv})")
        if not np.all(np.isfinite(J_pos)):
            raise ValueError(f"J_pos 中存在 NaN 或 inf")
        
        logging.info("site position Jacobian J_pos shape: %s", J_pos.shape)
        logging.info("site position Jacobian J_pos:\n%s", J_pos)

    # =============================
    # TODO 17: 组合 task error
    # =============================
    # 要做什么：
    # - position mode:
    #   e_task = w_pos * e_pos
    # - pose_6d mode:
    #   e_task = concat(w_pos * e_pos, w_rot * e_rot)
    #
    # 为什么需要：
    # - position 的单位是 m，rotation 的单位是 rad，直接拼接会造成数值尺度不清楚。
    #
    # 对标 mink：
    # - 对标 Task residual 的加权组合。
    #
    # 推荐 API：
    # - numpy.concatenate。
    # - numpy.asarray。
    #
    # 输入：
    # - e_pos。
    # - e_rot。
    # - position_weight。
    # - orientation_weight。
    # - task_mode。
    #
    # 输出：
    # - e_task。
    #
    # 验证：
    # - position mode 维度为 3。
    # - pose_6d mode 维度为 6。
        if args.task_mode == "position":
            e_task = args.position_weight * e_pos
            logging.info("任务模式: position，e_task = position_weight * e_pos")

        elif args.task_mode == "pose_6d":
            e_task = np.concatenate([
                args.position_weight * e_pos,
                args.orientation_weight * e_rot,
            ])
            logging.info("任务模式: pose_6d，e_task = concat(position_weight * e_pos, orientation_weight * e_rot)")
        else:
            raise ValueError(f"未知的 task_mode: {args.task_mode}")
        
        if args.task_mode == "position" and e_task.shape != (3,):
            raise ValueError(f"position mode 下 e_task.shape={e_task.shape}，但期望 (3,)")
        if args.task_mode == "pose_6d" and e_task.shape != (6,):
            raise ValueError(f"pose_6d mode 下 e_task.shape={e_task.shape}，但期望 (6,)")
        
        if not np.all(np.isfinite(e_task)):
            raise ValueError(f"e_task 中存在 NaN 或 inf: {e_task}")
        
        task_error_norm = np.linalg.norm(e_task)
        logging.info("组合后的 task error e_task: %s", e_task)
        logging.info("组合后的 task error 范数: %s", task_error_norm)   
    

    # =============================
    # TODO 18: 组合 task Jacobian
    # =============================
    # 要做什么：
    # - position mode:
    #   J_task = w_pos * J_pos
    # - pose_6d mode:
    #   J_task = stack([w_pos * J_pos, w_rot * J_rot])
    #
    # 为什么需要：
    # - DLS 求解器只关心 J_task 和 e_task；任务模式切换应体现在这两个对象上。
    #
    # 对标 mink：
    # - 对标 FrameTask 生成的 task Jacobian。
    #
    # 推荐 API：
    # - numpy.vstack。
    # - mujoco.mj_jacSite 同时返回 J_pos / J_rot。
    #
    # 输入：
    # - J_pos。
    # - J_rot。
    # - position_weight。
    # - orientation_weight。
    # - task_mode。
    #
    # 输出：
    # - J_task。
    #
    # 验证：
    # - J_task.shape == (3, model.nv) 或 (6, model.nv)。
        if args.task_mode == "position":
            J_task = float(args.position_weight) * J_pos

        elif args.task_mode == "pose_6d":
            J_task = np.vstack(
                [
                    float(args.position_weight) * J_pos,
                    float(args.orientation_weight) * jacr,
                ]
            )

        else:
            raise ValueError(f"不支持的 task_mode: {args.task_mode}")

        expected_rows = 3 if args.task_mode == "position" else 6
        if J_task.shape != (expected_rows, model.nv):
            raise ValueError(f"J_task.shape={J_task.shape}，但期望 ({expected_rows}, {model.nv})")
        if not np.all(np.isfinite(J_task)):
            raise ValueError(f"J_task 中存在 NaN 或 inf")
        if J_task.shape[0] != e_task.shape[0]:
            raise ValueError(f"J_task 的行数 {J_task.shape[0]} 与 e_task 的维度 {e_task.shape[0]} 不匹配")
        
        logging.info("组合后的 task Jacobian J_task shape: %s", J_task.shape)
        logging.info("组合后的 task Jacobian J_task:\n%s", J_task)  

    # =============================
    # TODO 8: DLS 求解 dq
    # =============================
    # 要做什么：
    # - 未来用 DLS 从 J_pos 和 e 求 dq。
    # - 必须保留核心数学结构：
    #   e = target - current
    #   dq = J.T @ solve(J @ J.T + λI, gain * e)
    #
    # 为什么需要：
    # - DLS 是 A04 的核心，用阻尼项提升奇异附近稳定性。
    #
    # 对标 mink：
    # - 对标 `mink.solve_ik` 的最小无约束教学版。
    #
    # 推荐 API：
    # - numpy.eye。
    # - numpy.linalg.solve。
    # - numpy.isfinite。
    #
    # 输入：
    # - J_pos。
    # - e。
    # - damping / lambda。
    # - gain。
    #
    # 输出：
    # - dq。
    #
    # 验证：
    # - dq.shape == (model.nv,)。
    # - dq 不含 NaN。
    # - dq norm 不异常大。
        J = J_task
        e = e_task
        task_dim = J.shape[0]
        damping_matrix = (damping ** 2) * np.eye(task_dim)

        lhs = J @ J.T + damping_matrix
        rhs = gain * e

        y = np.linalg.solve(lhs, rhs)
        dq = J.T @ y
        dq_norm = np.linalg.norm(dq)

        if dq.shape != (model.nv,):
            raise ValueError(f"dq.shape={dq.shape}，但期望 ({model.nv},)")
        if not np.all(np.isfinite(dq)):
            raise ValueError(f"dq 中存在 NaN 或 inf: {dq}")
        if dq_norm > 10.0:
            logging.warning(f"dq 的范数 {dq_norm} 超过 10.0，可能需要调整 gain 或 damping")

        logging.info("求解得到 dq: %s", dq)
        logging.info("dq 的范数: %s", dq_norm)
    # =============================
    # TODO 9: 积分更新 q
    # =============================
    # 要做什么：
    # - 未来用 q_next = integrate(q, dq, dt) 更新 configuration。
    #
    # 为什么需要：
    # - differential IK 是速度级小步迭代，求出的 dq 需要积分回 q。
    #
    # 对标 mink：
    # - 对标 Configuration.integrate_inplace 或类似的 q 更新。
    #
    # 推荐 API：
    # - mujoco.mj_integratePos。
    # - 或固定基简单模型下 q + dq * dt。
    #
    # 输入：
    # - q。
    # - dq。
    # - dt。
    #
    # 输出：
    # - q_next。
    #
    # 验证：
    # - q_next.shape == (model.nq,)。
    # - q_next 不含 NaN。
        q_next = q.copy()
        mujoco.mj_integratePos(model, q_next, dq, dt)

        if q_next.shape != (model.nq,):
            raise ValueError(f"q_next.shape={q_next.shape}，但期望 ({model.nq},)")
        if not np.all(np.isfinite(q_next)):
            raise ValueError(f"q_next 中存在 NaN 或 inf: {q_next}")
        
        logging.info("已积分更新 q 到 q_next：dt = %s", dt)
        logging.info("更新后的 q_next: %s", q_next)


    # =============================
    # TODO 10: 迭代终止条件
    # =============================
    # 要做什么：
    # - 未来根据 error_norm、max_iter、NaN 检查和 dq_norm 判断是否停止。
    #
    # 为什么需要：
    # - IK 可能不收敛，必须有清楚的停止边界。
    #
    # 对标 mink：
    # - 对标 solver loop 的收敛判断和安全退出。
    #
    # 推荐 API：
    # - numpy.linalg.norm。
    # - numpy.isfinite。
    #
    # 输入：
    # - error_norm。
    # - dq_norm。
    # - iter_index。
    # - tolerance。
    # - max_iter。
    #
    # 输出：
    # - stop_reason。
    # - converged。
    #
    # 验证：
    # - 达到 tolerance 能停止。
    # - 超过 max_iter 能停止。
    # - NaN 能触发错误。
        converged = False
        stop_reason = "max_iter_reached"

        if not np.isfinite(error_norm):
            stop_reason = "error_norm_nan"
            raise ValueError(f"error_norm 不是有限数: {error_norm}")
        
        if not np.isfinite(dq_norm):
            stop_reason = "dq_norm_nan"
            raise ValueError(f"dq_norm 不是有限数: {dq_norm}")
        
        if error_norm < tolerance:
            stop_reason = "tolerance_reached"
            converged = True
            stop_reason = "tolerance_reached"
            logging.info("迭代 %s: error_norm %s 已小于 tolerance %s，认为收敛", 0, error_norm, tolerance)
            break

        if dq_norm < 1e-12:
            stop_reason = "dq_norm_too_small"
            logging.warning("迭代 %s: dq_norm %s 太小，可能停滞", 0, dq_norm)
            break
    # =============================
    # TODO 11: 记录 q trajectory 和 error log & # TODO 19: 记录 position / orientation error
    # =============================
    # 要做什么：
    # - 未来每轮保存 q、error_norm、dq_norm、site position。
    #
    # 为什么需要：
    # - q_traj 是 A05/A06/A07 的后续输入；error log 用于判断是否收敛。
    #
    # 对标 mink：
    # - 对标 example 中的 solver debug / trajectory artifact。
    #
    # 推荐 API：
    # - list。
    # - numpy.save。
    # - csv.DictWriter。
    #
    # 输入：
    # - 每轮 q。
    # - 每轮 error_norm。
    # - 每轮 dq_norm。
    #
    # 输出：
    # - outputs/trajectories/A04_dls_ik_q_traj.npy。
    # - outputs/logs/A04_dls_ik_error.csv。
    #
    # 验证：
    # - q_traj 长度与 error log 行数一致。
    # - CSV 列名清楚。
     # =============================
    
    # =============================
    # 要做什么：
    # - 未来 error log 字段应包括 iteration、position_error_norm、orientation_error_norm、
    #   task_error_norm、dq_norm、task_mode、damping、gain。
    #
    # 为什么需要：
    # - pose-aware IK 需要区分 position 是否下降、orientation 是否下降以及加权 task error 是否下降。
    #
    # 对标 mink：
    # - 对标 solver debug log 中对每个 task residual 和 solver 更新量的记录。
    #
    # 推荐 API：
    # - csv.DictWriter。
    # - numpy.linalg.norm。
    #
    # 输入：
    # - e_pos。
    # - e_rot。
    # - e_task。
    # - dq。
    # - task_mode。
    #
    # 输出：
    # - A04_dls_ik_error.csv 的扩展字段。
    #
    # 验证：
    # - position mode 下 orientation_error_norm 可以为空或记录为 0。
    # - pose_6d mode 下 position_error_norm 和 orientation_error_norm 都可复盘。
        q_traj.append(q_next.copy())
        error_log.append({
            "iteration": iter_index,
            "error_norm": error_norm,
            "task_error_norm": task_error_norm,
            "dq_norm": dq_norm,
            "position_error_norm": np.linalg.norm(e_pos),
            "orientation_error_norm": np.linalg.norm(e_rot),
            "site_x": x_current[0],
            "site_y": x_current[1],
            "site_z": x_current[2],
            "damping": damping,
            "gain": gain,
            "dt": dt,
        })
   

    # =============================
    # TODO 12: 规划误差图输出
    # =============================
    # 要做什么：
    # - 未来绘制 error_norm vs iteration。
    #
    # 为什么需要：
    # - 图能直观看出误差是否下降、是否震荡、是否停滞。
    #
    # 对标 mink：
    # - 对标 IK example 的收敛诊断。
    #
    # 推荐 API：
    # - matplotlib。
    #
    # 输入：
    # - iteration list。
    # - error_norm list。
    #
    # 输出：
    # - outputs/figures/A04_dls_ik_error.png。
    #
    # 验证：
    # - 图可打开。
    # - 曲线能说明 final error 是否小于 initial error。figure_dir = output_root / "figures"
    figure_dir = output_root / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)
    figure_path = figure_dir / "A04_dls_ik_error.png"

    iterations = [row["iteration"] for row in error_log]
    position_errors = [row["position_error_norm"] for row in error_log]
    orientation_errors = [row["orientation_error_norm"] for row in error_log]
    task_errors = [row["task_error_norm"] for row in error_log]

    plt.figure(figsize=(8, 5))
    plt.plot(iterations, position_errors, marker="o", label="position error")
    plt.plot(iterations, orientation_errors, marker="^", label="orientation error")
    plt.plot(iterations, task_errors, marker="s", label="task error")
    plt.xlabel("iteration")
    plt.ylabel("error norm")
    plt.title("A04 DLS IK error")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(figure_path, dpi=150)
    plt.close()

    logging.info("已写出 A04 误差图: %s", figure_path)

    # =============================
    # TODO 13: 规划 Markdown report 输出
    # =============================
    # 要做什么：
    # - 未来输出 A04_dls_ik_report.md。
    # - 报告记录输入、目标、DLS 参数、收敛状态、误差指标和边界说明。
    #
    # 为什么需要：
    # - A04 是求职展示中从 Jacobian 到 IK 的关键连接，需要可解释报告。
    #
    # 对标 mink：
    # - 对标 example-level result summary。
    #
    # 推荐 API：
    # - Markdown 字符串。
    # - Path.write_text。
    #
    # 输入：
    # - q_traj summary。
    # - error log summary。
    # - DLS 参数。
    #
    # 输出：
    # - outputs/reports/A04_dls_ik_report.md。
    #
    # 验证：
    # - 报告说明 DLS 公式、是否收敛、为什么不做 QP。
    A04_report.parent.mkdir(parents=True, exist_ok=True)

    initial_error = error_log[0]["position_error_norm"] if error_log else None
    final_error = error_log[-1]["position_error_norm"] if error_log else None

    report_lines = [
        "# A04 DLS 差分逆运动学报告",
        "",
        "## 目标",
        "",
        "- 任务: 让 attachment_site 从初始位置移动到目标位置。",
        "- 模式: position-only DLS IK。",
        "- 边界: 不做 QP、不做 actuator tracking、不做 MuJoCo 控制、不做 collision avoidance。",
        "",
        "## 输入",
        "",
        f"- 目标 site: {args.site}",
        f"- 任务模式: {args.task_mode}",
        f"- 目标位置偏移: {args.target_offset}",
        f"- 目标位置: {x_target.tolist()}",
        "",
        "## 求解器参数",
        "",
        f"- damping: {damping}",
        f"- gain: {gain}",
        f"- dt: {dt}",
        f"- max_iter: {max_iter}",
        f"- tolerance: {tolerance}",
        "",
        "## 结果",
        "",
        f"- 是否收敛: {converged}",
        f"- 停止原因: {stop_reason}",
        f"- 初始位置误差范数: {initial_error}",
        f"- 最终位置误差范数: {final_error}",
        f"- q 轨迹长度: {len(q_traj)}",
        "",
        "## 输出文件",
        "",
        f"- q 轨迹: {A04_traj}",
        f"- 误差日志: {A04_log}",
        f"- 误差图: {A04_figure}",
        "",
        "## DLS 公式",
        "",
        "```text",
        "e = x_target - x_current",
        "dq = J.T @ solve(J @ J.T + damping^2 I, gain * e)",
        "q_next = q + dq * dt",
        "```",
        "",
        "## 说明",
        "",
        "- A04 使用 A03 已验证过的 site Jacobian。",
        "- A04 第一版只做位置 IK，不控制姿态。",
        "- DLS 中的 damping 用于降低奇异附近 dq 过大的风险。",
        "- A05 才会进入 QP-IK、约束和关节限制。",
        "",
    ]

    A04_report.write_text("\n".join(report_lines), encoding="utf-8")
    logging.info("已写出 A04 中文 Markdown 报告: %s", A04_report)


    


if __name__ == "__main__":
    main()
