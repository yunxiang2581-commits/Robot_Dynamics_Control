# Robot Script Builder v1

## 用途
本模板用于后续新建机器人学习脚本时的统一规范：
- 先搭骨架，再逐步补全。
- 关键逻辑保留为 TODO，便于学习与手写练习。
- 路径输入/输出必须可配置、可追溯。

## 核心原则
1. 可学习优先：关键步骤使用 TODO，不一次性写满。
2. 可运行优先：最小可运行输出（至少打印脚本状态）。
3. 可维护优先：路径统一用 pathlib，主逻辑统一在 main()。
4. 最小修改：只改当前任务相关文件，不动无关内容。

## 新脚本标准结构
1. imports
2. 路径常量（PROJECT_ROOT, OUT_DIR, OUT_FILE）
3. 候选列表（URDF / frame / joint）
4. 工具函数（resolve_urdf, pick_frame 等）
5. main()
6. if __name__ == "__main__": main()

## TODO 编写规则（必须）
每个 TODO 都写清楚：
- 这个块要做什么
- 为什么需要这一步
- 期望 API（Pinocchio / MuJoCo / NumPy）
- 输入是什么
- 输出是什么

## 路径输入/输出 TODO（必须）
至少包含以下 TODO：
1. 命令行路径输入：`--urdf`、`--out`
2. 默认路径回退：无参数时走候选路径
3. 路径存在性检查：不存在时抛清晰错误
4. 输出目录创建：`out_file.parent.mkdir(parents=True, exist_ok=True)`
5. 报告写盘：`out_file.write_text(report, encoding="utf-8")`

## 建议的 main() TODO 顺序
0. 解析命令行参数（路径输入）
1. 解析 URDF 路径
2. 构建模型和 data
3. 取 neutral / 初值
4. FK / Jacobian / Dynamics 计算
5. 目标 frame / joint 选择
6. 组织结果（文本或图）
7. 输出写盘（路径输出）

## 输出约定
- 文本输出优先放到：`outputs/<task_name>/<timestamp>/`
- 报告至少包含：
  - 使用的 URDF 路径
  - 目标 frame/joint
  - 模型维度（nq, nv, njoints, nframes）
  - 关键结果与简短解释

## 复用方式
当需要新建脚本时，直接说明：
- 任务名
- 输入（URDF/frame/joint）
- 输出（txt/png）
- 希望保留哪些 TODO

然后按本模板自动生成学习型骨架。
