from __future__ import annotations


def build_guards_report() -> str:
    return """RoboControl Atlas 安全边界

禁止动作：
- git add
- git commit
- git push
- 长时间 MuJoCo 仿真
- 外部 repo 训练脚本
- 未知安装脚本
- 下载网络资源

禁止修改路径：
- external/open_source_repos
- shared/robot_assets
- projects/A-D 主项目源码，除非后续任务明确要求

工具边界：
- 只做静态扫描、文本报告、提示词生成、验证计划和 demo assets 缺失检查。
- 不接入外部 LLM API。
- 不自动安装或运行 Understand Anything。"""
