from __future__ import annotations

import json
from collections import Counter
from pathlib import Path
from typing import Any

from .external_repo import analyze_external_repo
from .models import UnderstandGraphSummary

FILE_SUFFIXES = (".py", ".cpp", ".hpp", ".h", ".ts", ".js", ".yaml", ".yml", ".json", ".toml", ".md")
ENTRY_KEYS = ("main", "run", "demo", "train", "eval", "evaluate", "launch", "script", "example")
ALGO_KEYS = (
    "controller",
    "control",
    "mpc",
    "ilqr",
    "ilqg",
    "mppi",
    "cem",
    "planner",
    "planning",
    "trajectory",
    "rollout",
    "cost",
    "loss",
    "reward",
    "dynamics",
    "env",
    "environment",
    "mujoco",
    "pinocchio",
    "qp",
    "wbc",
    "ik",
    "fk",
    "jacobian",
)
ROBOT_KEYS = ("robot", "mujoco", "pinocchio", "urdf", "mjcf", "controller", "planner", "dynamics", "trajectory", "torque", "joint", "task-space", "end-effector")


def find_understand_graph(repo_path: Path) -> Path | None:
    graph = repo_path / ".understand-anything" / "knowledge-graph.json"
    return graph if graph.exists() else None


def load_understand_graph(graph_path: Path) -> dict[str, Any]:
    try:
        return json.loads(graph_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}


def extract_graph_lists(graph: dict[str, Any]) -> tuple[list[Any], list[Any]]:
    def get(container: Any, key: str) -> list[Any]:
        if isinstance(container, dict) and isinstance(container.get(key), list):
            return container[key]
        return []

    nodes = get(graph, "nodes") or get(graph, "files")
    edges = get(graph, "edges") or get(graph, "relationships") or get(graph, "links")
    for key in ("graph", "knowledge_graph"):
        inner = graph.get(key) if isinstance(graph, dict) else None
        nodes = nodes or get(inner, "nodes") or get(inner, "files")
        edges = edges or get(inner, "edges") or get(inner, "relationships") or get(inner, "links")
    return nodes, edges


def node_to_text(node: Any) -> str:
    if isinstance(node, str):
        return node
    if isinstance(node, dict):
        fields = ["id", "name", "label", "type", "kind", "path", "file", "symbol", "summary", "description"]
        return " ".join(str(node.get(field, "")) for field in fields if node.get(field))
    return str(node)


def edge_to_text(edge: Any) -> str:
    if isinstance(edge, str):
        return edge
    if isinstance(edge, dict):
        fields = ["source", "target", "from", "to", "type", "kind", "label", "relationship"]
        return " ".join(str(edge.get(field, "")) for field in fields if edge.get(field))
    return str(edge)


def classify_node_text(text: str) -> dict[str, bool]:
    lower = text.lower()
    return {
        "is_file": any(suffix in lower for suffix in FILE_SUFFIXES),
        "is_function": any(key in lower for key in ("function", "def", "method", "callable", "func:")),
        "is_class": "class" in lower,
        "is_entrypoint": any(key in lower for key in ENTRY_KEYS),
        "is_algorithm": any(key in lower for key in ALGO_KEYS),
        "is_robot_related": any(key in lower for key in ROBOT_KEYS),
    }


def summarize_understand_graph(repo_path: Path) -> UnderstandGraphSummary:
    graph_path = find_understand_graph(repo_path)
    if graph_path is None:
        return UnderstandGraphSummary(
            repo_path=repo_path,
            graph_path=repo_path / ".understand-anything" / "knowledge-graph.json",
            graph_exists=False,
            graph_loaded=False,
            warnings=["未发现 Understand Anything 图谱，本工具不会自动安装或自动运行 Understand Anything。"],
        )
    graph = load_understand_graph(graph_path)
    if not graph:
        return UnderstandGraphSummary(
            repo_path=repo_path,
            graph_path=graph_path,
            graph_exists=True,
            graph_loaded=False,
            warnings=["发现图谱，但解析失败。可能是 JSON 格式变化或文件损坏。"],
        )
    nodes, edges = extract_graph_lists(graph)
    texts = [node_to_text(node) for node in nodes]
    file_nodes: list[str] = []
    function_nodes: list[str] = []
    class_nodes: list[str] = []
    entrypoints: list[str] = []
    algorithms: list[str] = []
    for text in texts:
        flags = classify_node_text(text)
        if flags["is_file"]:
            file_nodes.append(text)
        if flags["is_function"]:
            function_nodes.append(text)
        if flags["is_class"]:
            class_nodes.append(text)
        if flags["is_entrypoint"]:
            entrypoints.append(text)
        if flags["is_algorithm"] or flags["is_robot_related"]:
            algorithms.append(text)
    endpoints = []
    for edge in edges:
        if isinstance(edge, dict):
            endpoints.extend(str(edge.get(key, "")) for key in ("source", "target", "from", "to") if edge.get(key))
    hotspots = [item for item, _count in Counter(endpoints).most_common(20)]
    return UnderstandGraphSummary(
        repo_path=repo_path,
        graph_path=graph_path,
        graph_exists=True,
        graph_loaded=True,
        top_level_keys=list(graph.keys()),
        node_count=len(nodes),
        edge_count=len(edges),
        file_nodes=file_nodes[:30],
        function_nodes=function_nodes[:30],
        class_nodes=class_nodes[:30],
        entrypoint_candidates=entrypoints[:30],
        algorithm_candidates=algorithms[:30],
        dependency_hotspots=hotspots,
    )


def _list(items: list[str]) -> str:
    return "\n".join(f"- {item}" for item in items[:30]) if items else "- 未发现"


def build_import_understand_report(repo_path: Path) -> str:
    summary = summarize_understand_graph(repo_path)
    if not summary.graph_exists:
        return f"""Understand Anything 图谱导入检查

Repo：{repo_path}
Graph：{summary.graph_path}
状态：未发现

未发现 Understand Anything 图谱：
{summary.graph_path}

请先在目标 repo 中运行 Understand Anything，例如 /understand。
本工具不会自动安装或自动运行 Understand Anything。"""
    if not summary.graph_loaded:
        return "发现图谱，但解析失败。\n可能是 JSON 格式变化或文件损坏。"
    return f"""Understand Anything 图谱导入检查

Repo：{summary.repo_path}
Graph：{summary.graph_path}
状态：已读取

图谱摘要：
- 节点数：{summary.node_count}
- 边数：{summary.edge_count}
- 顶层字段：{', '.join(summary.top_level_keys)}

候选节点：
{_list(summary.file_nodes + summary.function_nodes + summary.class_nodes + summary.algorithm_candidates)}

警告：
{_list(summary.warnings)}

安全边界：
- 只读读取 knowledge-graph.json，不修改外部 repo。"""


def build_understand_summary_report(repo_path: Path) -> str:
    summary = summarize_understand_graph(repo_path)
    if not summary.graph_loaded:
        return build_import_understand_report(repo_path)
    return f"""项目图谱摘要

核心文件候选：
{_list(summary.file_nodes)}

核心类 / 函数候选：
{_list(summary.class_nodes + summary.function_nodes)}

依赖热点：
{_list(summary.dependency_hotspots)}

建议优先阅读顺序：
1. README / docs
2. entrypoint candidates
3. controller / planner / dynamics 相关节点
4. config 与测试

与静态扫描结果的互补关系：
- 图谱提供符号和关系线索。
- 静态扫描提供文件、依赖和入口的保守判断。

风险与限制：
- 只做启发式解析，不保证 schema 永远稳定。"""


def build_understand_entrypoints_report(repo_path: Path) -> str:
    summary = summarize_understand_graph(repo_path)
    static = analyze_external_repo(repo_path) if repo_path.exists() else None
    if not summary.graph_loaded:
        return build_import_understand_report(repo_path)
    static_entries = [str(path) for path in static.entrypoints] if static else []
    combined = summary.entrypoint_candidates + static_entries
    return f"""入口判断结果

README / 文档入口：
{_list([str(path) for path in static.readme_files] if static else [])}

demo 入口：
{_list([item for item in combined if "demo" in item.lower()])}

train 入口：
{_list([item for item in combined if "train" in item.lower()])}

eval 入口：
{_list([item for item in combined if "eval" in item.lower()])}

main / run 入口：
{_list([item for item in combined if "main" in item.lower() or "run" in item.lower()])}

高连接度入口候选：
{_list(summary.dependency_hotspots)}

建议最小复现入口：
- 优先选 demo / eval / run 类短入口，不直接运行 train。

建议阅读顺序：
1. README
2. demo / run 入口
3. config
4. controller / planner 实现"""


def build_understand_port_to_b_report(repo_path: Path) -> str:
    summary = summarize_understand_graph(repo_path)
    if not summary.graph_loaded:
        return build_import_understand_report(repo_path)
    candidates = summary.algorithm_candidates + summary.file_nodes
    return f"""可迁移模块总览

controller 候选：
{_list([item for item in candidates if "control" in item.lower()])}

planner 候选：
{_list([item for item in candidates if any(key in item.lower() for key in ("planner", "mpc", "ilqr", "ilqg", "mppi", "cem"))])}

cost function 候选：
{_list([item for item in candidates if "cost" in item.lower() or "loss" in item.lower()])}

trajectory generator 候选：
{_list([item for item in candidates if "trajectory" in item.lower() or "rollout" in item.lower()])}

environment / simulator 候选：
{_list([item for item in candidates if "env" in item.lower() or "mujoco" in item.lower() or "sim" in item.lower()])}

visualization 候选：
{_list([item for item in candidates if "plot" in item.lower() or "visual" in item.lower()])}

config system 候选：
{_list([item for item in candidates if any(key in item.lower() for key in ("config", ".yaml", ".json", ".toml"))])}

logging / benchmark 候选：
{_list([item for item in candidates if "log" in item.lower() or "benchmark" in item.lower()])}

建议迁移目标位置：
- controller/control -> projects/B_mujoco_mpc_study/simulator/controllers/
- planner/mpc/ilqr/ilqg/mppi/cem -> projects/B_mujoco_mpc_study/simulator/planners/
- env/environment/mujoco -> projects/B_mujoco_mpc_study/simulator/envs/
- trajectory/rollout/utils -> projects/B_mujoco_mpc_study/simulator/utils/
- demo/run/script -> projects/B_mujoco_mpc_study/simulator/scripts/
- config/yaml/json -> projects/B_mujoco_mpc_study/configs/

不建议直接迁移的内容：
- 硬件接口、ROS launch、大规模训练框架、闭源依赖封装。

需要适配的接口：
- 状态向量、控制向量、动力学步进、cost 函数、日志格式。

风险与边界：
- 不直接复制外部源码。
- 不修改外部 repo。

下一步 Codex 任务：
- 基于候选模块写只读审读报告，再设计 B_mujoco_mpc_study 的最小教学实现。"""


def generate_codex_understand_prompt(repo_path: Path) -> str:
    graph_path = repo_path / ".understand-anything" / "knowledge-graph.json"
    return f"""# Codex 审读提示词：Understand Anything 图谱增强

任务背景：
使用 RoboControl Atlas 读取外部 repo 已有的 Understand Anything 图谱，辅助做只读审读和 port-to-B 分析。

外部 repo 路径：
{repo_path}

Understand Anything 图谱路径：
{graph_path}

只读边界：
- 不要修改外部 repo 源码。
- 不要执行 git add。
- 不要执行 git commit。
- 不要执行 git push。
- 不要运行长时间训练。
- 不要直接运行未知安装脚本。
- 不要下载大数据集。
- 不要把外部 repo 代码直接复制进 B 项目。
- 不要声称已经完成复现，除非实际运行并验证。

审读目标：
1. 是否发现 Understand Anything 图谱。
2. 入口文件识别。
3. 核心算法模块识别。
4. 最小复现路线。
5. port-to-B：迁移到 B_mujoco_mpc_study 的候选模块。

输出报告要求：
1. 是否发现 Understand Anything 图谱
2. 图谱路径
3. 项目主入口候选
4. 核心算法模块候选
5. 最小复现路线
6. 迁移到 B_mujoco_mpc_study 的候选模块
7. 不建议迁移的内容
8. 风险与缺失信息
9. 是否修改外部源码，必须说明没有
10. 是否执行 git add/commit/push，必须说明没有"""
