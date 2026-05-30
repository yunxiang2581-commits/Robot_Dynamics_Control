import json
from pathlib import Path

from robot_atlas.understand_adapter import (
    build_import_understand_report,
    build_understand_entrypoints_report,
    build_understand_port_to_b_report,
    generate_codex_understand_prompt,
    summarize_understand_graph,
)


def test_missing_graph_does_not_crash(tmp_path: Path) -> None:
    (tmp_path / "README.md").write_text("demo", encoding="utf-8")
    (tmp_path / "demo.py").write_text("print('demo')\n", encoding="utf-8")

    summary = summarize_understand_graph(tmp_path)
    report = build_import_understand_report(tmp_path)

    assert summary.graph_exists is False
    assert summary.graph_loaded is False
    assert summary.warnings
    assert "未发现" in report
    assert "knowledge-graph.json" in report
    assert "不会自动安装" in report


def test_reads_simple_knowledge_graph(tmp_path: Path) -> None:
    graph_dir = tmp_path / ".understand-anything"
    graph_dir.mkdir()
    (graph_dir / "knowledge-graph.json").write_text(
        json.dumps(
            {
                "nodes": [
                    {"id": "file:demo.py", "type": "file", "path": "demo.py"},
                    {"id": "func:main", "type": "function", "name": "main"},
                    {"id": "class:MPCController", "type": "class", "name": "MPCController"},
                    {"id": "file:controllers/mpc.py", "type": "file", "path": "controllers/mpc.py"},
                ],
                "edges": [
                    {"source": "file:demo.py", "target": "func:main", "type": "defines"},
                    {"source": "func:main", "target": "class:MPCController", "type": "uses"},
                ],
            }
        ),
        encoding="utf-8",
    )

    summary = summarize_understand_graph(tmp_path)
    text = build_import_understand_report(tmp_path)

    assert summary.graph_exists is True
    assert summary.graph_loaded is True
    assert summary.node_count == 4
    assert summary.edge_count == 2
    assert "demo.py" in text
    assert "MPCController" in text
    assert "controllers/mpc.py" in text


def test_understand_entrypoints_report(tmp_path: Path) -> None:
    graph_dir = tmp_path / ".understand-anything"
    graph_dir.mkdir()
    (graph_dir / "knowledge-graph.json").write_text(
        json.dumps({"nodes": [{"label": "demo.py"}, {"label": "main"}], "edges": []}),
        encoding="utf-8",
    )

    report = build_understand_entrypoints_report(tmp_path)

    assert "demo" in report
    assert "main" in report
    assert "建议最小复现入口" in report


def test_understand_port_to_b_report(tmp_path: Path) -> None:
    graph_dir = tmp_path / ".understand-anything"
    graph_dir.mkdir()
    (graph_dir / "knowledge-graph.json").write_text(
        json.dumps({"nodes": [{"label": "controllers/mpc.py"}, {"label": "planner_ilqr"}], "edges": []}),
        encoding="utf-8",
    )

    report = build_understand_port_to_b_report(tmp_path)

    assert "B_mujoco_mpc_study" in report
    assert "controllers" in report
    assert "planners" in report
    assert "需要适配" in report
    assert "风险" in report


def test_codex_understand_prompt_contains_boundaries(tmp_path: Path) -> None:
    prompt = generate_codex_understand_prompt(tmp_path)

    assert "只读" in prompt
    assert "不要修改外部 repo 源码" in prompt
    assert "不要执行 git add" in prompt
    assert "不要执行 git commit" in prompt
    assert "不要执行 git push" in prompt
    assert "Understand Anything" in prompt
    assert "knowledge-graph.json" in prompt
    assert "port-to-B" in prompt


def test_unknown_json_schema_is_supported(tmp_path: Path) -> None:
    graph_dir = tmp_path / ".understand-anything"
    graph_dir.mkdir()
    (graph_dir / "knowledge-graph.json").write_text(
        json.dumps(
            {
                "knowledge_graph": {
                    "nodes": [
                        {"label": "scripts/run_demo.py", "kind": "file"},
                        {"label": "planner_ilqr", "kind": "function"},
                    ],
                    "edges": [],
                }
            }
        ),
        encoding="utf-8",
    )

    summary = summarize_understand_graph(tmp_path)

    assert summary.node_count == 2
    assert summary.graph_loaded is True
