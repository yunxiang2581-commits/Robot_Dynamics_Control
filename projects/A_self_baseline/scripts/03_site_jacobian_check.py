"""A03 site Jacobian check minimal implementation.

Pipeline 步骤：
- A03 site Jacobian check。

对标 mink 的概念：
- differential IK 背后的 velocity mapping。
- A03 要学习 `site velocity = J(q) dq`，也就是用 Jacobian 把关节速度映射到
  任务空间 site 速度。

与 A02 的关系：
- A02 已确认 q source 为 `keyframe:home`。
- A02 已确认 target site 为 `attachment_site`。
- A02 已输出 `outputs/cache/A02_site_pose.json`，记录 q、site pose 和 body pose。
- A03 读取 A02 的 q 和 target site，在同一个 configuration 下验证 Jacobian。

本脚本输入：
- `projects/A_self_baseline/configs/robot.yaml`。
- `projects/A_self_baseline/outputs/cache/A01_model_summary.json`。
- `projects/A_self_baseline/outputs/cache/A02_site_pose.json`。
- `shared/robot_assets/models/mink_universal_robots_ur5e/scene.xml`。
- target site，例如 `attachment_site`。
- q。
- dq。
- dt。

本脚本输出：
- `outputs/reports/A03_jacobian_check_report.md`。
- `outputs/cache/A03_jacobian_check.json`。
- `outputs/figures/A03_jacobian_fd_error.png`。

当前状态：
- 最小可运行 Jacobian / finite difference check。
- 不调用 mink 替代自己的实现。
- 不做 IK / QP / WBC / MuJoCo 控制 / collision avoidance / video recording。
"""

from __future__ import annotations

import argparse
import os
import logging
from pathlib import Path
import json
import numpy as np
import mujoco
import sys
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

A_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = A_ROOT / "configs" / "robot.yaml"
DEFAULT_A01_SUMMARY = A_ROOT / "outputs" / "cache" / "A01_model_summary.json"
DEFAULT_A02_POSE = A_ROOT / "outputs" / "cache" / "A02_site_pose.json"
DEFAULT_OUTPUT_DIR = A_ROOT / "outputs"
DEFAULT_SITE_NAME = "attachment_site"
DEFAULT_DQ_SOURCE = "unit:shoulder_pan"
DEFAULT_DT = 1e-6
DEFAULT_SWEEP_STEPS = "1,2,5,10,20,50,100,200,500,1000"
SRC_ROOT = A_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from robot_baseline import model_loader  # noqa: E402

PRINCIPLE_NOTES = [
    "Jacobian 是在当前 q 附近，把关节速度 dq 映射到任务空间速度的线性近似。",
    "对于 site，可以写成 site velocity = J(q) dq；这里 J 依赖当前 configuration q。",
    "A03 必须在 A04 DLS IK 之前，因为 DLS IK 需要可信的 J(q) 才能从误差求 dq。",
    "linear Jacobian 描述 site 位置速度，angular Jacobian 描述 site 姿态角速度。",
    "MuJoCo 的 site Jacobian 是针对某个 site id，在当前 data 状态下计算的任务空间导数。",
    "finite difference 用 q 和 q + dq * dt 的位姿差验证 J(q) dq，能检查坐标系和索引错误。",
    "A03 验证 velocity mapping；A04 才用误差和 Jacobian 求 IK 更新；A05 才把任务和限制写成 QP。",
]


TODO_TITLES = [
    "TODO 1: 读取 A01 / A02 前置产物",
    "TODO 2: 读取 robot.yaml 和解析 scene.xml",
    "TODO 3: 加载 MuJoCo model 和 data",
    "TODO 4: 恢复 A02 使用的 q",
    "TODO 5: 选择 dq 测试向量",
    "TODO 6: 计算 site Jacobian",
    "TODO 7: 计算 Jacobian 预测速度",
    "TODO 8: 有限差分验证 position velocity",
    "TODO 9: 规划 angular velocity 验证",
    "TODO 10: 组织 A03 summary",
    "TODO 11: 规划 JSON 输出",
    "TODO 12: 规划 Markdown report 输出",
    "TODO 13: 规划误差图输出",
    "TODO 14: 说明 A03 不进入 A04/A05",
]


def parse_args() -> argparse.Namespace:
    """解析 A03 site Jacobian check 的 CLI 参数。"""
    parser = argparse.ArgumentParser(description="A03: site Jacobian finite difference check.")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="robot.yaml 配置路径。")
    parser.add_argument(
        "--a01-summary",
        default=str(DEFAULT_A01_SUMMARY),
        help="A01_model_summary.json 路径。",
    )
    parser.add_argument(
        "--a02-pose",
        default=str(DEFAULT_A02_POSE),
        help="A02_site_pose.json 路径。",
    )
    parser.add_argument(
        "--mjcf",
        default=None,
        help="可选：覆盖 robot.yaml 中的 scene.xml 路径。",
    )
    parser.add_argument(
        "--site",
        default=DEFAULT_SITE_NAME,
        help="目标 site 名称，默认使用 A02 已确认的 attachment_site。",
    )
    parser.add_argument(
        "--dq-source",
        default=DEFAULT_DQ_SOURCE,
        help="dq 来源，例如 unit:shoulder_pan / unit-index:0 / file:path。",
    )
    parser.add_argument("--dt", type=float, default=DEFAULT_DT, help="有限差分步长。")
    parser.add_argument(
        "--fd-steps",
        type=int,
        default=1000,
        help="有限差分循环步数；总时间为 dt * fd_steps，用于观察更大位姿变化下的误差。",
    )
    parser.add_argument(
        "--sweep-steps",
        default=DEFAULT_SWEEP_STEPS,
        help="逗号分隔的 fd_steps sweep 列表，用来画误差随总位移变化的曲线。",
    )
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR), help="输出根目录。")
    parser.add_argument("--log-level", default="INFO", help="日志级别，例如 INFO / DEBUG。")
    return parser.parse_args()


def log_principles() -> None:
    """打印 A03 的原理说明。"""
    logging.info("A03 原理说明:")
    for note in PRINCIPLE_NOTES:
        logging.info("- %s", note)


def log_todo_titles() -> None:
    """打印 TODO 1-14 简要列表。"""
    logging.info("A03 TODO 任务清单:")
    for title in TODO_TITLES:
        logging.info("- %s", title)


def parse_positive_int_list(text: str) -> list[int]:
    """解析逗号分隔的正整数列表，并去重排序。"""
    values: list[int] = []
    for raw_item in text.split(","):
        item = raw_item.strip()
        if not item:
            continue
        value = int(item)
        if value < 1:
            raise ValueError("--sweep-steps 中的每个值都必须 >= 1")
        values.append(value)
    if not values:
        raise ValueError("--sweep-steps 至少需要一个正整数")
    return sorted(set(values))


def rotation_vector_from_delta(rotation_delta: np.ndarray) -> np.ndarray:
    """把两个旋转矩阵之间的相对旋转转成最小 rotation vector。"""
    trace_value = np.trace(rotation_delta)
    cos_angle = (trace_value - 1.0) / 2.0
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    angle = np.arccos(cos_angle)

    if abs(angle) < 1e-12:
        return np.zeros(3)

    rotation_axis = np.array(
        [
            rotation_delta[2, 1] - rotation_delta[1, 2],
            rotation_delta[0, 2] - rotation_delta[2, 0],
            rotation_delta[1, 0] - rotation_delta[0, 1],
        ]
    ) / (2.0 * np.sin(angle))
    return rotation_axis * angle


def evaluate_loop_finite_difference(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    q0: np.ndarray,
    dq: np.ndarray,
    dt: float,
    fd_steps: int,
    site_id: int,
    initial_linear_velocity: np.ndarray,
    initial_angular_velocity: np.ndarray,
    record_trace: bool = False,
) -> dict:
    """循环积分 q，比较真实位移平均速度和初始 J(q0)dq。

    这里的误差是“大位移线性化误差”：它使用初始 q0 的 Jacobian 预测整个
    时间段的平均 site velocity。fd_steps 越大，总位移越大，误差通常会增长。
    """
    if fd_steps < 1:
        raise ValueError("fd_steps must be >= 1")

    data.qpos[:] = q0
    mujoco.mj_forward(model, data)
    site_pos_bef = data.site_xpos[site_id].copy()
    site_rot_bef = data.site_xmat[site_id].reshape(3, 3).copy()

    q_next = q0.copy()
    trace_rows = []

    for step in range(1, fd_steps + 1):
        mujoco.mj_integratePos(model, q_next, dq, dt)
        data.qpos[:] = q_next
        mujoco.mj_forward(model, data)

        if record_trace:
            J_pos_step = np.zeros((3, model.nv))
            J_rot_step = np.zeros((3, model.nv))
            mujoco.mj_jacSite(model, data, J_pos_step, J_rot_step, site_id)
            trace_rows.append(
                {
                    "step": step,
                    "time": float(step * dt),
                    "q": q_next.tolist(),
                    "site_position": data.site_xpos[site_id].tolist(),
                    "site_rotation_matrix": data.site_xmat[site_id].reshape(3, 3).tolist(),
                    "J_pos": J_pos_step.tolist(),
                    "J_rot": J_rot_step.tolist(),
                }
            )

    site_pos_aft = data.site_xpos[site_id].copy()
    site_rot_aft = data.site_xmat[site_id].reshape(3, 3).copy()
    fd_total_time = dt * fd_steps

    site_displacement = site_pos_aft - site_pos_bef
    v_pos_fd = site_displacement / fd_total_time
    rotation_vector = rotation_vector_from_delta(site_rot_aft @ site_rot_bef.T)
    v_rot_fd = rotation_vector / fd_total_time
    v_pos_err = v_pos_fd - initial_linear_velocity
    v_rot_err = v_rot_fd - initial_angular_velocity

    return {
        "fd_steps": int(fd_steps),
        "fd_total_time": float(fd_total_time),
        "q_final": q_next.tolist(),
        "finite_difference_site_displacement": site_displacement.tolist(),
        "final_site_displacement_norm": float(np.linalg.norm(site_displacement)),
        "finite_difference_linear_velocity": v_pos_fd.tolist(),
        "finite_difference_angular_velocity": v_rot_fd.tolist(),
        "linear_velocity_error": v_pos_err.tolist(),
        "angular_velocity_error": v_rot_err.tolist(),
        "linear_velocity_error_norm": float(np.linalg.norm(v_pos_err)),
        "angular_velocity_error_norm": float(np.linalg.norm(v_rot_err)),
        "trace": trace_rows,
    }


def main() -> None:
    """A03 site Jacobian check 主流程。

    当前实现最小可运行的 MuJoCo site Jacobian、有限差分验证、JSON/report/figure 输出。
    本脚本仍不进入 IK / QP / 控制。
    """
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(levelname)s: %(message)s",
    )

    output_root = Path(args.output_dir).expanduser()
    if not output_root.is_absolute():
        output_root = A_ROOT / output_root

    future_cache = output_root / "cache" / "A03_jacobian_check.json"
    future_report = output_root / "reports" / "A03_jacobian_check_report.md"
    future_figure = output_root / "figures" / "A03_jacobian_fd_error.png"

    logging.info("A03 当前状态: minimal site Jacobian finite difference check")
    logging.info("本步骤只验证 velocity mapping，不进入 IK / QP / 控制。")
    logging.info("当前输入:")
    logging.info("- config: %s", args.config)
    logging.info("- A01 summary: %s", args.a01_summary)
    logging.info("- A02 pose: %s", args.a02_pose)
    logging.info("- mjcf override: %s", args.mjcf)
    logging.info("- target site: %s", args.site)
    logging.info("- dq source: %s", args.dq_source)
    logging.info("- dt: %s", args.dt)
    logging.info("- finite difference steps: %s", args.fd_steps)
    logging.info("- sweep steps: %s", args.sweep_steps)
    logging.info("输出:")
    logging.info("- cache: %s", future_cache)
    logging.info("- report: %s", future_report)
    logging.info("- figure: %s", future_figure)

    log_principles()
    log_todo_titles()

    # =============================
    # TODO 1: 读取 A01 / A02 前置产物
    # =============================
    # 要做什么：
    # - 未来读取 `outputs/cache/A01_model_summary.json`。
    # - 未来读取 `outputs/cache/A02_site_pose.json`。
    # - 确认 nq/nv/nu。
    # - 确认 target site = attachment_site。
    # - 确认 q source = keyframe:home。
    #
    # 为什么这一步存在：
    # - A03 必须和 A01/A02 使用同一个模型、同一个 q、同一个 target site。
    # - 否则 Jacobian 验证结果不能作为 A04 IK 的可信输入。
    #
    # 对标 mink 的哪个概念：
    # - 对标 differential IK 中 task Jacobian 必须绑定当前 configuration 和当前 task frame。
    #
    # 推荐 API：
    # - json.loads
    # - Path.read_text
    #
    # 输入是什么：
    # - A01 summary。
    # - A02 pose cache。
    #
    # 输出是什么：
    # - model_summary: dict。
    # - pose_summary: dict。
    #
    # 如何验证：
    # - nq=6, nv=6。
    # - target site 存在。
    # - A02 pose cache 中 site_name 为 attachment_site。
    # model_summary = json.loads(Path(args.a01_summary).read_text(encoding="utf-8"))
    # pose_summary = json.loads(Path(args.a02_pose).read_text(encoding="utf-8"))
    a01_summary_path = Path(args.a01_summary).expanduser()
    a02_pose_path = Path(args.a02_pose).expanduser()
    if not a01_summary_path.is_absolute():
        a01_summary_path = A_ROOT / a01_summary_path
    if not a02_pose_path.is_absolute():
        a02_pose_path = A_ROOT / a02_pose_path

    if not a01_summary_path.exists():
        logging.warning("A01 summary not found at %s. TODO 1 未来读取 A01 产物。", a01_summary_path)
        raise NotImplementedError("TODO 1: load A01 summary for model info.")
    if not a02_pose_path.exists():
        logging.warning("A02 pose cache not found at %s. TODO 1 未来读取 A02 产物。", a02_pose_path)
        raise NotImplementedError("TODO 1: load A02 site pose cache for q and site info.")
    
    model_summary = json.loads(a01_summary_path.read_text(encoding="utf-8"))
    pose_summary = json.loads(a02_pose_path.read_text(encoding="utf-8"))

    if args.site not in model_summary["site_names"]:
        logging.warning("Target site '%s' not found in A01 model summary. TODO 1 未来确认 target site。", args.site)
        raise ValueError(f"TODO 1: confirm target site '{args.site}' exists in model summary.")
    if pose_summary.get("site_name") != args.site:
        logging.warning("A02 site pose cache site_name '%s' does not match target site '%s'. TODO 1 未来确认 A02 使用同一 target site。", pose_summary.get("site_name"), args.site)
        raise ValueError(f"TODO 1: confirm A02 site pose cache uses the same target site '{args.site}'.")
    logging.info("A01 summary and A02 site pose cache loaded successfully. Target site: %s.", args.site)

    

   
    # =============================
    # TODO 2: 读取 robot.yaml 和解析 scene.xml
    # =============================
    # 要做什么：
    # - 未来读取 configs/robot.yaml。
    # - 解析 mjcf_path。
    # - 与 A01/A02 使用同一个模型。
    #
    # 为什么这一步存在：
    # - Jacobian 与 model 结构强相关；换了 scene.xml，关节维度和 site id 都可能不同。
    #
    # 对标 mink 的哪个概念：
    # - 对标 mink 示例中 Configuration 和 task 都基于同一个 MuJoCo model。
    #
    # 推荐 API：
    # - model_loader.load_yaml_config
    # - model_loader.resolve_path
    #
    # 输入是什么：
    # - robot.yaml。
    #
    # 输出是什么：
    # - mjcf_path: Path。
    #
    # 如何验证：
    # - scene.xml exists=True。
    # config = model_loader.load_yaml_config(args.config)
    # mjcf_path = model_loader.resolve_path(args.mjcf or config["mjcf_path"], A_ROOT)

    config = model_loader.load_yaml_config(args.config)
    mjcf_path = model_loader.resolve_path(args.mjcf or config["mjcf_path"], A_ROOT)
    # =============================
    # TODO 3: 加载 MuJoCo model 和 data
    # =============================
    # 要做什么：
    # - 未来加载 model。
    # - 创建 mujoco.MjData(model)。
    # - 不在本 TODO 阶段真正实现。
    #
    # 为什么这一步存在：
    # - `mj_jacSite` 需要当前 model 和经过 forward 刷新的 data。
    #
    # 对标 mink 的哪个概念：
    # - 对标 Configuration 持有 model/data 的状态对象。
    #
    # 推荐 API：
    # - model_loader.load_mujoco_model
    # - mujoco.MjData
    #
    # 输入是什么：
    # - mjcf_path。
    #
    # 输出是什么：
    # - model。
    # - data。
    #
    # 如何验证：
    # - model.nq 和 A01 summary 对齐。
    model = model_loader.load_mujoco_model(mjcf_path)
    data = mujoco.MjData(model)

    if model.nq != model_summary["nq"]:
        raise ValueError(f"model.nq={model.nq} 与 A01 nq={model_summary['nq']} 不匹配")
    
    if model.nv != model_summary["nv"]:
        raise ValueError(f"model.nv={model.nv} 与 A01 nv={model_summary['nv']} 不匹配")
    
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, args.site)
    if site_id < 0:
        raise ValueError(f"Site '{args.site}' not found in MuJoCo model.")

    logging.info("载入 MuJoCo model: nq=%s, nv=%s, nu=%s", model.nq, model.nv, model.nu)
    logging.info("创建 MuJoCo data: qpos shape=%s, qvel shape=%s", data.qpos.shape, data.qvel.shape)
    logging.info("所选 site: %s, site_id=%s", args.site, site_id)
        # =============================
    # TODO 4: 恢复 A02 使用的 q
    # =============================
    # 要做什么：
    # - 未来从 A02_site_pose.json 或 keyframe home 恢复 q。
    # - q shape 必须等于 model.nq。
    #
    # 为什么这一步存在：
    # - Jacobian 是在当前 q 处的局部线性化，不同 q 会得到不同 J(q)。
    #
    # 对标 mink 的哪个概念：
    # - 对标 Configuration 当前 q；solve_ik 每一步都基于当前 configuration 的 Jacobian。
    #
    # 推荐 API：
    # - model.key_qpos
    # - numpy.asarray
    #
    # 输入是什么：
    # - q source。
    #
    # 输出是什么：
    # - q。
    #
    # 如何验证：
    # - q.shape == (model.nq,)。
    q = np.asarray(pose_summary["q"], dtype=float)
    if q.shape != data.qpos.shape:
        raise ValueError(f"q 的形状 {q.shape} 与 {data.qpos.shape}不匹配")
    data.qpos[:] = q
    mujoco.mj_forward(model,data)
    logging.info("q 是从 A02 恢复的.")
    logging.info("A02 的 q 来源: %s", pose_summary["q_source"])
    logging.info("q 维度: %s", q.shape)
    logging.info("q具体数值: %s", q)
    # =============================
    # TODO 5: 选择 dq 测试向量
    # =============================
    # 要做什么：
    # - A03 需要测试一个关节速度扰动 dq。
    # - 第一版可以规划 unit vector 或指定某个关节速度。
    # - dq shape 必须等于 model.nv。
    #
    # 为什么这一步存在：
    # - 没有 dq，就无法比较 `J(q) dq` 和有限差分速度。
    #
    # 对标 mink 的哪个概念：
    # - 对标 differential IK 中 solver 求出的关节速度增量。
    #
    # 推荐 API：
    # - numpy.zeros
    # - joint index lookup
    #
    # 输入是什么：
    # - --joint-name 或 --dq-source。
    #
    # 输出是什么：
    # - dq。
    #
    # 如何验证：
    # - dq.shape == (model.nv,)。
    # - norm > 0。
    dq = np.zeros(model.nv)
    if args.dq_source == "unit:shoulder_pan":
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "shoulder_pan")
        if joint_id < 0:
            raise ValueError("Joint 'shoulder_pan' 在 MuJoCo model中未发现.")

        dof_adr = model.jnt_dofadr[joint_id]
        dq[dof_adr] = 1.0

    else:
        raise ValueError(f"不支持的dq格式: {args.dq_source}")
    # =============================
    # TODO 6: 计算 site Jacobian
    # =============================
    # 要做什么：
    # - 未来用 MuJoCo 计算 attachment_site 的 Jacobian。
    # - 分别理解 translational Jacobian 和 rotational Jacobian。
    #
    # 为什么这一步存在：
    # - A03 的核心目标是得到可信的 J(q)，供 A04 DLS IK 使用。
    #
    # 对标 mink 的哪个概念：
    # - 对标 task Jacobian：把关节速度映射到任务空间线速度和角速度。
    #
    # 推荐 API：
    # - mujoco.mj_forward
    # - mujoco.mj_jacSite
    #
    # 输入是什么：
    # - model。
    # - data。
    # - site_id。
    #
    # 输出是什么：
    # - J_pos。
    # - J_rot。
    # - J_6d。
    #
    # 如何验证：
    # - J_pos shape = (3, nv)。
    # - J_rot shape = (3, nv)。
    # J_pos = np.zeros((3, model.nv))
    # J_rot = np.zeros((3, model.nv))
    # mujoco.mj_jacSite(model, data, J_pos, J_rot, site_id)
    data.qpos[:] = q
    mujoco.mj_forward(model,data)

    J_pos = np.zeros((3,model.nv))
    J_rot = np.zeros((3,model.nv))

    mujoco.mj_jacSite(model,data,J_pos,J_rot,site_id)
    logging.info("当前计算的是哪个 site: %s", args.site)
    logging.info("J_pos:\n%s", J_pos)
    logging.info("J_rot:\n%s", J_rot)


    # =============================
    # TODO 7: 计算 Jacobian 预测速度
    # =============================
    # 要做什么：
    # - 未来计算 `v_pos = J_pos @ dq`。
    # - 未来计算 `v_rot = J_rot @ dq`。
    #
    # 为什么这一步存在：
    # - 这是 differential IK 的核心速度映射：site velocity = J(q) dq。
    #
    # 对标 mink 的哪个概念：
    # - 对标 solve_ik 内部使用 task Jacobian 把 dq 映射到 task velocity。
    #
    # 推荐 API：
    # - numpy.matmul
    # - `@` 矩阵乘法。
    #
    # 输入是什么：
    # - J_pos。
    # - J_rot。
    # - dq。
    #
    # 输出是什么：
    # - predicted site velocity。
    #
    # 如何验证：
    # - 维度正确。
    # - 无 NaN。
    v_pos = J_pos @ dq
    v_rot = J_rot @ dq

    logging.info("线速度: %s", v_pos)
    logging.info("角速度: %s", v_rot)


    # =============================
    # TODO 8: 有限差分验证 position velocity
    # =============================
    # 要做什么：
    # - 未来用 q 和 q + dq * dt 的 site position 差分验证 J_pos @ dq。
    #
    # 为什么这一步存在：
    # - 有限差分能检查 Jacobian 的关节索引、坐标系和实现调用是否正确。
    #
    # 对标 mink 的哪个概念：
    # - 对标 IK solver 前的 task Jacobian 数值 sanity check。
    #
    # 推荐 API：
    # - mujoco.mj_integratePos 或安全 q 更新。
    # - mujoco.mj_forward。
    # - data.site_xpos。
    # - numpy.linalg.norm。
    #
    # 输入是什么：
    # - q。
    # - dq。
    # - dt。
    # - site position。
    #
    # 输出是什么：
    # - finite_difference_velocity。
    #
    # 如何验证：
    # - finite_difference_velocity 与 J_pos @ dq 误差较小。
    # q_next = ...
    # finite_difference_velocity = (site_position_next - site_position) / dt
    loop_fd_result = evaluate_loop_finite_difference(
        model=model,
        data=data,
        q0=q,
        dq=dq,
        dt=args.dt,
        fd_steps=args.fd_steps,
        site_id=site_id,
        initial_linear_velocity=v_pos,
        initial_angular_velocity=v_rot,
        record_trace=True,
    )
    fd_total_time = loop_fd_result["fd_total_time"]
    q_next = np.asarray(loop_fd_result["q_final"], dtype=float)
    v_pos_fd = np.asarray(loop_fd_result["finite_difference_linear_velocity"], dtype=float)
    v_pos_err = np.asarray(loop_fd_result["linear_velocity_error"], dtype=float)

    # =============================
    # TODO 9: 规划 angular velocity 验证
    # =============================
    # 要做什么：
    # - rotation finite difference 比 position 更复杂。
    #
    #
    # 为什么这一步存在：
    # - 姿态差分涉及 SO(3) / quaternion 误差，直接相减 rotation matrix 容易误导。
    #
    # 对标 mink 的哪个概念：
    # - 对标 task orientation error 和 angular Jacobian 的后续验证。
    #
    # 推荐 API：
    # - rotation matrix / quaternion 差分。
    # - MuJoCo orientation helper。
    #
    # 输入是什么：
    # - site rotation before/after。
    #
    # 输出是什么：
    # - angular velocity check。
    #
    v_rot_fd = np.asarray(loop_fd_result["finite_difference_angular_velocity"], dtype=float)
    v_rot_err = np.asarray(loop_fd_result["angular_velocity_error"], dtype=float)

    sweep_steps = parse_positive_int_list(args.sweep_steps)
    if args.fd_steps not in sweep_steps:
        sweep_steps.append(args.fd_steps)
        sweep_steps = sorted(set(sweep_steps))

    multi_step_records = []
    for sweep_step in sweep_steps:
        sweep_result = evaluate_loop_finite_difference(
            model=model,
            data=data,
            q0=q,
            dq=dq,
            dt=args.dt,
            fd_steps=sweep_step,
            site_id=site_id,
            initial_linear_velocity=v_pos,
            initial_angular_velocity=v_rot,
            record_trace=False,
        )
        multi_step_records.append(
            {
                "fd_steps": sweep_result["fd_steps"],
                "fd_total_time": sweep_result["fd_total_time"],
                "final_site_displacement_norm": sweep_result["final_site_displacement_norm"],
                "linear_velocity_error_norm": sweep_result["linear_velocity_error_norm"],
                "angular_velocity_error_norm": sweep_result["angular_velocity_error_norm"],
            }
        )

    trace_path = output_root / "cache" / "A03_multi_step_trace.json"
    trace_path.parent.mkdir(parents=True, exist_ok=True)
    trace_summary = {
        "q_source": pose_summary.get("q_source"),
        "site_name": args.site,
        "dq_source": args.dq_source,
        "dt": float(args.dt),
        "fd_steps": int(args.fd_steps),
        "note": "逐步日志用于和 A04 迭代 IK 对照；大位移误差不等价于 Jacobian API 错误。",
        "steps": loop_fd_result["trace"],
    }
    trace_path.write_text(json.dumps(trace_summary, indent=2, ensure_ascii=False), encoding="utf-8")
    logging.info("Wrote A03 multi-step trace: %s", trace_path)

    multi_step_figure_path = output_root / "figures" / "A03_multi_step_linearization_error.png"
    multi_step_figure_path.parent.mkdir(parents=True, exist_ok=True)
    displacement_norms = [row["final_site_displacement_norm"] for row in multi_step_records]
    linear_error_norms = [row["linear_velocity_error_norm"] for row in multi_step_records]
    angular_error_norms = [row["angular_velocity_error_norm"] for row in multi_step_records]

    plt.figure(figsize=(8, 4))
    plt.plot(displacement_norms, linear_error_norms, marker="o", label="linear velocity error")
    plt.plot(displacement_norms, angular_error_norms, marker="s", label="angular velocity error")
    plt.xlabel("Final site displacement norm")
    plt.ylabel("Velocity error norm")
    plt.title("A03 Error vs Total Displacement")
    plt.legend()
    plt.tight_layout()
    plt.savefig(multi_step_figure_path)
    plt.close()
    logging.info("Wrote A03 multi-step error figure: %s", multi_step_figure_path)

    # =============================
    # TODO 10: 组织 A03 summary
    # =============================
    # 要做什么：
    # - 汇总 q、dq、dt、site name、J shape、predicted velocity、
    #   finite difference velocity、error norm。
    #
    # 为什么这一步存在：
    # - A03 summary 是 A04 DLS IK 使用 Jacobian 前的可复盘证据。
    #
    # 对标 mink 的哪个概念：
    # - 对标 solver 调试中记录 task Jacobian、task velocity 和误差指标。
    #
    # 推荐 API：
    # - dict
    # - float conversion
    #
    # 输入是什么：
    # - Jacobian check 结果。
    #
    # 输出是什么：
    # - JSON-serializable summary。
    #
    # 如何验证：
    # - summary 可 json.dumps。
    # summary = {...}
    jacobian_summary = {
        "q_source": pose_summary.get("q_source"),
        "site_name": args.site,
        "dq_source": args.dq_source,
        "dt": float(args.dt),
        "fd_steps": int(args.fd_steps),
        "fd_total_time": float(fd_total_time),
        "q": q.tolist(),
        "q_final": q_next.tolist(),
        "dq": dq.tolist(),
        "J_pos_shape": list(J_pos.shape),
        "J_rot_shape": list(J_rot.shape),
        "v_pos": v_pos.tolist(),
        "v_rot": v_rot.tolist(),
        "predicted_linear_velocity": v_pos.tolist(),
        "predicted_angular_velocity": v_rot.tolist(),
        "J_pos": J_pos.tolist(),
        "J_rot": J_rot.tolist(),
        "v_pos_fd": v_pos_fd.tolist(),
        "finite_difference_linear_velocity": v_pos_fd.tolist(),
        "finite_difference_site_displacement": loop_fd_result[
            "finite_difference_site_displacement"
        ],
        "v_pos_err": v_pos_err.tolist(),
        "v_rot_fd": v_rot_fd.tolist(),
        "finite_difference_angular_velocity": v_rot_fd.tolist(),
        "v_rot_err": v_rot_err.tolist(),
        "linear_velocity_error_norm": float(np.linalg.norm(v_pos_err)),
        "angular_velocity_error_norm": float(np.linalg.norm(v_rot_err)),
        "angular_check_status": "minimal_rotation_matrix_log_check",
        "error_interpretation": {
            "small_step_velocity_mapping_error": (
                "用于检查 mj_jacSite API、site id、dq 维度和速度映射方向是否正确。"
            ),
            "large_displacement_linearization_error": (
                "使用初始 J(q0)dq 预测较大总位移下的平均速度；误差随总位移增大是正常现象。"
            ),
        },
        "multi_step_trace_path": str(trace_path),
        "multi_step_figure_path": str(multi_step_figure_path),
        "multi_step_check": {
            "sweep_steps": sweep_steps,
            "records": multi_step_records,
        },
    
        "model_nq": int(model.nq),
        "model_nv": int(model.nv),
        "model_nu": int(model.nu),
        "source_mjcf": str(mjcf_path),
    }
    # =============================
    # TODO 11: 规划 JSON 输出
    # =============================
    # 要做什么：
    # - 未来输出 `outputs/cache/A03_jacobian_check.json`。
    # - 字段至少包括 q_source、site_name、dq_source、dt、J_pos_shape、J_rot_shape、
    #   predicted_linear_velocity、finite_difference_linear_velocity、
    #   linear_velocity_error_norm、angular_check_status。
    #
    # 为什么这一步存在：
    # - JSON cache 让 A04 能复用 Jacobian 验证结论，而不是重新猜数值。
    #
    # 对标 mink 的哪个概念：
    # - 对标 task Jacobian / solver debug artifact。
    #
    # 推荐 API：
    # - json.dump
    # - Path.write_text
    #
    # 输入是什么：
    # - summary。
    #
    # 输出是什么：
    # - outputs/cache/A03_jacobian_check.json。
    #
    # 如何验证：
    # - JSON 可读取，字段完整。
    # Path(...).write_text(...)
    json_path = output_root / "cache" / "A03_jacobian_check.json"
    json_path.parent.mkdir(parents=True, exist_ok=True)

    json_text = json.dumps(jacobian_summary, indent=2, ensure_ascii=False)
    json_path.write_text(json_text, encoding="utf-8")

    logging.info("Wrote A03 JSON summary: %s", json_path)

    # =============================
    # TODO 12: 规划 Markdown report 输出
    # =============================
    # 要做什么：
    # - 未来输出 `outputs/reports/A03_jacobian_check_report.md`。
    # - 内容至少包括 A03 目标、输入 q/dq/site、J_pos/J_rot shape、
    #   Jacobian predicted velocity、finite difference velocity、error norm、
    #   angular check 暂缓原因、与 A04 DLS IK 的关系。
    #
    # 为什么这一步存在：
    # - Markdown report 让人能复盘 Jacobian 是否可信，再进入 IK。
    #
    # 对标 mink 的哪个概念：
    # - 对标 differential IK 示例中对 task Jacobian 和速度映射的解释。
    #
    # 推荐 API：
    # - Markdown 字符串。
    # - Path.write_text。
    #
    # 输入是什么：
    # - summary。
    #
    # 输出是什么：
    # - outputs/reports/A03_jacobian_check_report.md。
    #
    # 如何验证：
    # - 报告可读，能说明 site velocity = J(q) dq。
    # report_path.write_text(...)
    report_path = output_root / "reports" / "A03_jacobian_check_report.md"
    report_path.parent.mkdir(parents=True, exist_ok=True)

    report_lines = [
        "# A03 Site Jacobian Check Report",
        "",
        "## Input",
        f"- q source: {jacobian_summary['q_source']}",
        f"- site name: {jacobian_summary['site_name']}",
        f"- dq source: {jacobian_summary['dq_source']}",
        f"- dt: {jacobian_summary['dt']}",
        f"- finite difference steps: {jacobian_summary['fd_steps']}",
        f"- finite difference total time: {jacobian_summary['fd_total_time']}",
        f"- source MJCF: {jacobian_summary['source_mjcf']}",
        "",
        "## Model Dimensions",
        f"- nq: {jacobian_summary['model_nq']}",
        f"- nv: {jacobian_summary['model_nv']}",
        f"- nu: {jacobian_summary['model_nu']}",
        "",
        "## Velocity Mapping",
        "- Formula: site_velocity = J(q) dq",
        f"- J_pos shape: {jacobian_summary['J_pos_shape']}",
        f"- J_rot shape: {jacobian_summary['J_rot_shape']}",
        f"- linear velocity from Jacobian: {jacobian_summary['v_pos']}",
        f"- linear velocity from finite difference: {jacobian_summary['v_pos_fd']}",
        f"- finite difference site displacement: {jacobian_summary['finite_difference_site_displacement']}",
        f"- linear velocity error: {jacobian_summary['v_pos_err']}",
        f"- linear velocity error norm: {jacobian_summary['linear_velocity_error_norm']}",
        
        "",
        "## Angular Velocity",
        f"- angular velocity from Jacobian: {jacobian_summary['v_rot']}",
        f"- angular velocity from finite difference: {jacobian_summary['v_rot_fd']}",
        f"- angular velocity error: {jacobian_summary['v_rot_err']}",
        f"- angular velocity error norm: {jacobian_summary['angular_velocity_error_norm']}",
        f"- angular check status: {jacobian_summary['angular_check_status']}",
        "",
        "## Error Interpretation",
        "- Small-step velocity mapping error checks mj_jacSite API, site id, dq dimension, and velocity mapping direction.",
        "- Large-displacement linearization error uses the initial J(q0)dq over a longer total displacement.",
        "- A larger error at larger displacement is expected and does not by itself mean the Jacobian API is wrong.",
        "",
        "## Multi-Step Linearization Sweep",
        f"- trace JSON: {jacobian_summary['multi_step_trace_path']}",
        f"- error figure: {jacobian_summary['multi_step_figure_path']}",
        "",
        "| fd_steps | displacement_norm | linear_error_norm | angular_error_norm |",
        "| --- | --- | --- | --- |",
        *[
            (
                f"| {record['fd_steps']} | {record['final_site_displacement_norm']} | "
                f"{record['linear_velocity_error_norm']} | {record['angular_velocity_error_norm']} |"
            )
            for record in multi_step_records
        ],
        "",
        "## Boundary",
        "- A03 checks site Jacobian velocity mapping.",
        "- A03 does not solve IK.",
        "- A03 does not build QP.",
        "- A03 does not run controller or simulation loop.",
        "",
    ]

    report_text = "\n".join(report_lines)
    report_path.write_text(report_text, encoding="utf-8")

    logging.info("Wrote A03 Markdown report: %s", report_path)

    # =============================
    # TODO 13: 规划误差图输出
    # =============================
    # 要做什么：
    # - 未来输出 `outputs/figures/A03_jacobian_fd_error.png`。
    # - 第一版可以规划不同 dt 下的 error norm。
    #
    # 为什么这一步存在：
    # - 有限差分误差随 dt 变化的趋势能帮助判断数值验证是否合理。
    #
    # 对标 mink 的哪个概念：
    # - 对标 solver 数值稳定性和 Jacobian sanity check。
    #
    # 推荐 API：
    # - matplotlib。
    #
    # 输入是什么：
    # - dt list。
    # - error norm list。
    #
    # 输出是什么：
    # - 误差曲线图。
    #
    # 如何验证：
    # - 图能显示 finite difference 收敛趋势。
    # plt.savefig(...)

    # =============================
    figure_path = output_root / "figures" / "A03_jacobian_fd_error.png"
    figure_path.parent.mkdir(parents=True, exist_ok=True)

    labels = ["vx", "vy", "vz", "wx", "wy", "wz"]
    errors = np.concatenate([v_pos_err, v_rot_err])

    plt.figure(figsize=(8, 4))
    plt.bar(labels, errors)
    plt.axhline(0.0, color="black", linewidth=1.0)
    plt.title("A03 Jacobian Finite Difference Error")
    plt.xlabel("Velocity component")
    plt.ylabel("FD velocity - Jacobian velocity")
    plt.tight_layout()
    plt.savefig(figure_path)
    plt.close()

    logging.info("Wrote A03 error figure: %s", figure_path)

    # =============================
    # 多步差分增强说明
    # =============================
    # 当前已实现：
    # - 不同 fd_steps 的 sweep，输出误差随总位移变化的曲线。
    # - 当前 fd_steps 内每一步的 q_k、site pose、J(q_k) trace。
    # - report / JSON 中拆分“小步速度映射误差”和“大位移线性化误差”的解释口径。
    #
    # 后续仍可增强：
    # - 增加多组 dt sweep。
    # - 对比“固定初始 J(q0)”和“每步刷新 J(q_k)”两种累计预测。
    # - 把 trace CSV 化，方便用 pandas 或 spreadsheet 检查。


if __name__ == "__main__":
    main()
