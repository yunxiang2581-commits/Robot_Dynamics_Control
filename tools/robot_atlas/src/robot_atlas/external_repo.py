from __future__ import annotations

from pathlib import Path

from .models import ExternalRepoReport
from .scanner import is_entrypoint_file, is_test_file

CONFIG_NAMES = {"config.yaml", "config.yml", "config.json", "pyproject.toml", "setup.cfg", "CMakeLists.txt"}
DEPENDENCY_NAMES = {"requirements.txt", "environment.yml", "package.json", "pyproject.toml", "setup.py", "CMakeLists.txt"}
ROBOT_KEYWORDS = ("robot", "mujoco", "pinocchio", "controller", "planner", "dynamics", "trajectory", "env", "sim")


def analyze_external_repo(repo_path: Path) -> ExternalRepoReport:
    path = repo_path.resolve()
    files = [file for file in path.rglob("*") if file.is_file()] if path.exists() else []
    py_files = [file for file in files if file.suffix == ".py"]
    cpp_files = [file for file in files if file.suffix in {".cpp", ".hpp", ".h", ".cc"}]
    ts_files = [file for file in files if file.suffix in {".ts", ".js"}]
    if py_files and not cpp_files:
        language = "Python"
    elif cpp_files and not py_files:
        language = "C++"
    elif ts_files and not py_files and not cpp_files:
        language = "TypeScript"
    elif py_files or cpp_files or ts_files:
        language = "Mixed"
    else:
        language = "Unknown"
    readmes = [file for file in files if file.name.lower().startswith("readme")]
    entrypoints = [file for file in py_files if is_entrypoint_file(file)]
    config_files = [file for file in files if file.name in CONFIG_NAMES or file.suffix in {".yaml", ".yml", ".toml", ".json"}][:50]
    dependency_files = [file for file in files if file.name in DEPENDENCY_NAMES]
    test_files = [file for file in py_files if is_test_file(file) or "test" in file.parts]
    dirs = [directory for directory in path.rglob("*") if directory.is_dir()]
    robot_dirs = [directory for directory in dirs if any(key in directory.name.lower() for key in ROBOT_KEYWORDS)]
    risks = []
    if dependency_files:
        risks.append("依赖安装可能影响本地环境，建议先只读审查。")
    if not entrypoints:
        risks.append("未发现明确入口，需要人工确认最小复现命令。")
    if any("train" in file.name.lower() for file in files):
        risks.append("存在训练脚本，避免直接运行长时间训练。")
    candidates = []
    for directory in robot_dirs[:20]:
        name = directory.as_posix().lower()
        if "controller" in name or "control" in name:
            candidates.append(f"{directory} -> simulator/controllers/")
        elif "planner" in name or "mpc" in name or "ilqr" in name:
            candidates.append(f"{directory} -> simulator/planners/")
        elif "env" in name or "mujoco" in name or "sim" in name:
            candidates.append(f"{directory} -> simulator/envs/")
        else:
            candidates.append(f"{directory} -> 需人工判断")
    return ExternalRepoReport(
        path=path,
        name=path.name,
        main_language_guess=language,
        readme_files=readmes,
        entrypoints=entrypoints,
        config_files=config_files,
        dependency_files=dependency_files,
        test_files=test_files,
        robot_related_dirs=robot_dirs,
        reproduce_risks=risks,
        porting_candidates=candidates,
    )


def build_read_repo_report(repo_path: Path) -> str:
    if not repo_path.exists():
        return f"路径不存在：{repo_path}"
    return analyze_external_repo(repo_path).as_text()


def build_entrypoints_report(repo_path: Path) -> str:
    if not repo_path.exists():
        return f"路径不存在：{repo_path}"
    report = analyze_external_repo(repo_path)
    entries = "\n".join(f"- {path}" for path in report.entrypoints) or "- 未发现"
    return f"""入口判断结果

Repo：{report.path}

入口候选：
{entries}

建议最小复现入口：
- 优先阅读 README 中的 demo / eval 命令。
- 若 README 不清楚，优先尝试 demo.py、run_*.py、examples/、scripts/ 中的短脚本。

安全边界：
- 默认只读，不运行未知安装脚本，不运行长时间训练。"""


def build_reproduce_plan(repo_path: Path) -> str:
    if not repo_path.exists():
        return f"路径不存在：{repo_path}"
    report = analyze_external_repo(repo_path)
    return f"""复现路线

1. 只读审查 README、依赖文件、配置文件和入口脚本。
2. 判断是否存在最小 demo / eval / smoke run。
3. 建立隔离环境，不直接污染当前项目环境。
4. 先不运行训练脚本，只确认短入口和参数。
5. 记录复现风险与缺失信息。

入口候选：
{chr(10).join(f"- {path}" for path in report.entrypoints) if report.entrypoints else "- 未发现"}

风险：
{chr(10).join(f"- {risk}" for risk in report.reproduce_risks) if report.reproduce_risks else "- 暂无明显风险，但仍需人工确认"}"""


def build_port_to_b_report(repo_path: Path) -> str:
    if not repo_path.exists():
        return f"路径不存在：{repo_path}"
    report = analyze_external_repo(repo_path)
    return f"""迁移到 B_mujoco_mpc_study 的建议

可迁移候选：
{chr(10).join(f"- {item}" for item in report.porting_candidates) if report.porting_candidates else "- 未发现明确候选"}

建议目标位置：
- controller/control -> projects/B_mujoco_mpc_study/simulator/controllers/
- planner/mpc/ilqr/ilqg/mppi/cem -> projects/B_mujoco_mpc_study/simulator/planners/
- env/environment/mujoco -> projects/B_mujoco_mpc_study/simulator/envs/
- trajectory/rollout/utils -> projects/B_mujoco_mpc_study/simulator/utils/
- config/yaml/json -> projects/B_mujoco_mpc_study/configs/

边界：
- 不直接复制外部源码。
- 先写接口适配说明，再做最小教学实现。"""


def generate_codex_reproduce_prompt(repo_path: Path) -> str:
    return f"""# Codex 外部 repo 复现审读提示词

外部 repo 路径：
{repo_path}

只读边界：
- 不要修改外部 repo 源码。
- 不要执行 git add。
- 不要执行 git commit。
- 不要执行 git push。
- 不要运行长时间训练。
- 不要直接运行未知安装脚本。

审读目标：
1. 找 README、依赖、配置和入口。
2. 判断最小复现路线。
3. 判断哪些思想适合迁移到 B_mujoco_mpc_study。
4. 输出风险和缺失信息。

最终汇报：
必须说明是否修改外部源码，答案应为没有。"""
