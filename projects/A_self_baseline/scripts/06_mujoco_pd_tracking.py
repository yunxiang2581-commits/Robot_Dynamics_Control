"""MuJoCo joint PD trajectory tracking learning entry.

This is a TODO learning entry. References:
root experiments/exp_pd_joint_control.py
scripts/legacy_imported/mujoco_h1_left_knee_perturb.py
scripts/legacy_imported/mujoco_h1_sim_learning.py
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a MuJoCo PD tracking placeholder.")
    parser.add_argument("--model-xml", default="../../shared/robot_assets/models/your_robot.xml")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--output-dir", default=str(PROJECT_ROOT / "outputs" / "06_mujoco_pd_tracking"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    logging.info("MuJoCo PD output directory placeholder: %s", args.output_dir)

    # TODO(中文):
    # 1. 要实现什么: 加载 MuJoCo XML, 生成关节期望轨迹, 用 PD 计算 torque 并记录跟踪误差。
    # 2. 求职重要性: PD 轨迹跟踪是控制实验的最小闭环, 能连接仿真、日志和稳定性调试。
    # 3. 推荐 API: mujoco.MjModel.from_xml_path, mujoco.MjData, mujoco.mj_step, numpy。
    # 4. 输入: MuJoCo XML、目标关节、q_des/dq_des 轨迹、kp/kd、仿真时长。
    # 5. 输出: CSV 日志、误差曲线、关节位置/速度/力矩图。
    # 6. 如何验证: 误差有界, torque 无异常尖峰, 仿真无 NaN, 图像与日志一致。
    # 7. legacy 参考: root PD 实验和 scripts/legacy_imported/mujoco_*.py。
    raise NotImplementedError("TODO: implement MuJoCo PD tracking learning step.")


if __name__ == "__main__":
    main()
