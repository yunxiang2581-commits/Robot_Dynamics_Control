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


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check frame Jacobian with finite differences.")
    parser.add_argument("--urdf", default="../../shared/robot_assets/models/your_robot.urdf")
    parser.add_argument("--frame", default="left_foot")
    parser.add_argument("--dt", type=float, default=1e-6)
    parser.add_argument("--output-dir", default=str(PROJECT_ROOT / "outputs" / "03_jacobian_fd_check"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    logging.info("Jacobian check output directory placeholder: %s", args.output_dir)

    # Pipeline TODO(中文):
    # - 前置产物: A01 的模型信息和 A02 确认的目标 site。
    # - 本步产物: site Jacobian、有限差分速度、误差范数和验证报告。
    # - 后续消费: A04 使用通过验证的 J 做 DLS-IK, A05 使用同一任务 Jacobian 构造 QP。
    # - 输出路径: 后续应通过 pipeline_io 生成 A03 report/cache/figure 路径。
    # TODO(中文):
    # 1. 要实现什么: 计算 MuJoCo site Jacobian, 再用 q 与 q+dq 的 site pose 差做有限差分验证。
    # 2. 为什么这一步存在: A04/A05 的 differential IK 依赖 J(q) 把关节速度映射到 site 速度。
    # 3. 对标 mink 的哪个概念: mink solve_ik 背后的任务空间速度映射。
    # 4. 推荐 API: mujoco.mj_jacSite, mujoco.mj_forward。
    # 5. 输入是什么: MJCF、目标 site、q、dq、dt。
    # 6. 输出是什么: site Jacobian、有限差分速度、误差范数、报告和误差图。
    # 7. 如何验证: 误差范数应随 dt 合理变化, 方向应与 dq 造成的 site 位移一致。
    # 8. legacy 参考: scripts/legacy_imported/jacobian_h1.py 和 jacobian_h1_check.py。
    raise NotImplementedError("TODO: implement Jacobian finite-difference check.")


if __name__ == "__main__":
    main()
