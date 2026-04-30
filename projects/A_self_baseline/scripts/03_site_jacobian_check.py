"""A03 pipeline 第三步: site Jacobian check 学习型 TODO 骨架。

所属 pipeline 步骤:
- A03 site Jacobian check。

对标 mink 的概念:
- differential IK 背后的速度映射 `site velocity = J(q) dq`。
- A04/A05 需要用这个 Jacobian 把任务空间误差转为关节速度。

本脚本输入:
- A01/A02 确认的 MJCF 模型和目标 site。
- 当前配置 `q`、扰动 `dq`、有限差分步长 `dt`。

本脚本输出:
- `outputs/reports/A03_jacobian_check_report.md`。
- `outputs/figures/A03_jacobian_fd_error.png`。
- `outputs/cache/A03_jacobian_check.json`。

当前状态:
- TODO learning skeleton。
- 不实现完整 Jacobian 或有限差分验证。
- 不调用 mink 替代自己的实现。

Legacy references:
- `scripts/legacy_imported/jacobian_h1.py`
- `scripts/legacy_imported/jacobian_h1_check.py`
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path


A_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A03 TODO skeleton: site Jacobian check.")
    parser.add_argument("--config", default=str(A_ROOT / "configs" / "robot.yaml"))
    parser.add_argument("--mjcf", default=None, help="可选：覆盖 robot.yaml 中的 scene.xml 路径。")
    parser.add_argument("--site", default="attachment_site", help="A01/A02 确认后的目标 site。")
    parser.add_argument("--dq-source", default="unit-test-vector", help="TODO：未来 dq 扰动来源。")
    parser.add_argument("--dt", type=float, default=1e-6)
    parser.add_argument("--output-dir", default=str(A_ROOT / "outputs"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    logging.info("A03 当前状态: TODO learning skeleton")
    logging.info("config: %s", args.config)
    logging.info("target site: %s", args.site)
    logging.info("dt placeholder: %s", args.dt)
    logging.info("future report: %s", Path(args.output_dir) / "reports" / "A03_jacobian_check_report.md")
    logging.info("future cache: %s", Path(args.output_dir) / "cache" / "A03_jacobian_check.json")

    # =============================
    # TODO 1: 确认 q、dq 和目标 site
    # =============================
    # - 前置产物: A01 的模型信息和 A02 确认的目标 site。
    # - 要实现什么: 明确当前配置 q、速度扰动 dq、目标 site 和 dt。
    # - 为什么需要: Jacobian 验证必须知道线性化点 q 和测试速度 dq。
    # - 推荐 API: A02 cache、numpy.ndarray、MuJoCo site id lookup。
    # - 输入是什么: scene.xml、q、dq、site name、dt。
    # - 输出是什么: 已确认的 site_id、q/dq shape 和检查参数。
    # - 如何验证: q shape 对齐 model.nq，dq shape 对齐 model.nv。

    # =============================
    # TODO 2: site velocity = J(q) dq
    # =============================
    # - 要实现什么: 后续用 mj_jacSite 得到 site Jacobian，并计算 J(q) dq。
    # - 为什么需要: A04/A05 differential IK 依赖这个速度映射。
    # - 推荐 API: mujoco.mj_forward、mujoco.mj_jacSite。
    # - 输入是什么: model、data、site_id、q、dq。
    # - 输出是什么: J、site_velocity_from_jacobian。
    # - 如何验证: J 的列数等于 nv，速度维度与选择的位置/姿态误差一致。

    # =============================
    # TODO 3: 有限差分检查
    # =============================
    # - 要实现什么: 后续比较 q 与 q+dq*dt 的 site pose 差分速度。
    # - 为什么需要: 验证 Jacobian 方向和索引是否正确，避免后续 IK 调错坐标系。
    # - 推荐 API: mujoco.mj_forward、data.site_xpos、numpy.linalg.norm。
    # - 输入是什么: q、dq、dt、当前和扰动后的 site pose。
    # - 输出是什么: finite_difference_velocity、误差范数、报告和误差图。
    # - 如何验证: 误差范数应随 dt 合理变化，方向应与 dq 造成的 site 位移一致。
    raise NotImplementedError("TODO: A03 remains a site Jacobian check learning skeleton.")


if __name__ == "__main__":
    main()
