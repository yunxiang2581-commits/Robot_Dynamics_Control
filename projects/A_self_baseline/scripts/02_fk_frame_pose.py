"""A02 pipeline 第二步: configuration / site pose 学习型 TODO 骨架。

所属 pipeline 步骤:
- A02 configuration / site pose。

对标 mink 的概念:
- `mink.Configuration`。
- 从关节配置 `q` 更新 MuJoCo `data`, 再查询 body/site pose。

本脚本输入:
- A01 确认的 MuJoCo MJCF 模型。
- 末端 site/body 名称, 例如 `attachment_site` 或 `tool0`。
- 初始或配置文件给定的关节配置 `q`。

本脚本输出:
- `outputs/reports/A02_site_pose_report.md`。
- `outputs/cache/A02_site_pose.json`。

当前状态:
- TODO learning skeleton。
- 不实现完整 FK/site pose 逻辑。
- 不调用 mink 替代自己的实现。

Legacy reference:
- `scripts/legacy_imported/fk_h1.py` 仅作为历史学习参考。
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compute target frame pose using FK.")
    parser.add_argument("--urdf", default="../../shared/robot_assets/models/your_robot.urdf")
    parser.add_argument("--frame", default="left_foot")
    parser.add_argument("--output-dir", default=str(PROJECT_ROOT / "outputs" / "02_fk_frame_pose"))
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))
    logging.info("FK output directory placeholder: %s", args.output_dir)

    # Pipeline TODO(中文):
    # - 前置产物: A01 的模型摘要和候选 site/body 清单。
    # - 本步产物: 指定 site/body 的位置、姿态和 pose 摘要。
    # - 后续消费: A03 使用同一个 site 做 Jacobian 验证; A04/A05 使用该 site 定义 IK 目标。
    # - 输出路径: 后续应通过 pipeline_io 生成 A02 report/cache 路径。
    # TODO(中文):
    # 1. 要实现什么: 加载 MuJoCo model, 创建 data, 写入 q, 调用 forward 后读取 site/body pose。
    # 2. 为什么这一步存在: mink-style IK 需要先知道当前末端 site 在世界系的位置和姿态。
    # 3. 对标 mink 的哪个概念: mink.Configuration 中的 q -> data -> frame/site pose 查询。
    # 4. 推荐 API: mujoco.MjData, mujoco.mj_forward, data.site_xpos, data.site_xmat, data.xpos, data.xmat。
    # 5. 输入是什么: MJCF 路径、目标 site/body 名称、关节配置 q。
    # 6. 输出是什么: 目标 site/body 的位置、旋转矩阵和 JSON/Markdown 摘要。
    # 7. 如何验证: 改变一个关节角后, site pose 应发生连续且方向合理的变化。
    # 8. legacy 参考: scripts/legacy_imported/fk_h1.py 仅用于理解 FK 学习方式。
    raise NotImplementedError("TODO: implement FK frame pose learning step.")


if __name__ == "__main__":
    main()
