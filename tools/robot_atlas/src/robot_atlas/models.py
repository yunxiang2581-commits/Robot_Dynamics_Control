from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class ProjectLine:
    id: str
    name: str
    path: Path
    goal: str
    exists: bool
    python_files: list[Path] = field(default_factory=list)
    test_files: list[Path] = field(default_factory=list)
    markdown_files: list[Path] = field(default_factory=list)
    todo_count: int = 0
    not_implemented_count: int = 0
    entrypoint_files: list[Path] = field(default_factory=list)

    def summary(self) -> str:
        state = "存在" if self.exists else "缺失"
        return (
            f"{self.id} - {self.name} ({state})\n"
            f"  目标：{self.goal}\n"
            f"  Python：{len(self.python_files)}，测试：{len(self.test_files)}，Markdown：{len(self.markdown_files)}\n"
            f"  TODO：{self.todo_count}，NotImplementedError：{self.not_implemented_count}"
        )


@dataclass
class Stage:
    id: str
    project_line: str
    title: str
    status: str
    docs: list[Path] = field(default_factory=list)
    related_files: list[Path] = field(default_factory=list)
    next_step: str = ""


@dataclass
class ScanResult:
    repo_root: Path
    project_lines: dict[str, ProjectLine]
    docs_count: int
    stage_docs: list[Path]
    total_todos: int
    total_not_implemented: int
    warnings: list[str] = field(default_factory=list)

    def as_text(self) -> str:
        lines = [
            "RoboControl Atlas Scan",
            "",
            "仓库根目录：",
            str(self.repo_root),
            "",
            "项目线：",
        ]
        lines.extend(line.summary() for line in self.project_lines.values())
        lines.extend(
            [
                "",
                "统计：",
                f"- docs/00_project_management 文档数量：{self.docs_count}",
                f"- TODO 总数：{self.total_todos}",
                f"- NotImplementedError 总数：{self.total_not_implemented}",
                "",
                "警告：",
            ]
        )
        lines.extend(f"- {warning}" for warning in self.warnings) if self.warnings else lines.append("- 无")
        return "\n".join(lines)


@dataclass
class DemoAssetReport:
    stage: str
    required: list[str]
    existing: list[Path]
    missing: list[str]
    conclusion: str


@dataclass
class ExternalRepoReport:
    path: Path
    name: str
    main_language_guess: str
    readme_files: list[Path] = field(default_factory=list)
    entrypoints: list[Path] = field(default_factory=list)
    config_files: list[Path] = field(default_factory=list)
    dependency_files: list[Path] = field(default_factory=list)
    test_files: list[Path] = field(default_factory=list)
    robot_related_dirs: list[Path] = field(default_factory=list)
    reproduce_risks: list[str] = field(default_factory=list)
    porting_candidates: list[str] = field(default_factory=list)

    def as_text(self) -> str:
        return "\n".join(
            [
                "外部 Repo 审读报告",
                f"路径：{self.path}",
                f"名称：{self.name}",
                f"语言初判：{self.main_language_guess}",
                _paths("README / 文档", self.readme_files),
                _paths("入口候选", self.entrypoints),
                _paths("配置文件", self.config_files),
                _paths("依赖文件", self.dependency_files),
                _paths("测试文件", self.test_files),
                _paths("机器人相关目录", self.robot_related_dirs),
                _items("复现风险", self.reproduce_risks),
                _items("迁移候选", self.porting_candidates),
                "",
                "安全边界：默认只读，不修改外部 repo 源码。",
            ]
        )


@dataclass
class UnderstandGraphSummary:
    repo_path: Path
    graph_path: Path | None
    graph_exists: bool
    graph_loaded: bool
    top_level_keys: list[str] = field(default_factory=list)
    node_count: int = 0
    edge_count: int = 0
    file_nodes: list[str] = field(default_factory=list)
    function_nodes: list[str] = field(default_factory=list)
    class_nodes: list[str] = field(default_factory=list)
    entrypoint_candidates: list[str] = field(default_factory=list)
    algorithm_candidates: list[str] = field(default_factory=list)
    dependency_hotspots: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


def _paths(title: str, paths: list[Path]) -> str:
    values = [str(path) for path in paths[:20]]
    return _items(title, values)


def _items(title: str, items: list[str]) -> str:
    if not items:
        return f"{title}：\n- 未发现"
    return f"{title}：\n" + "\n".join(f"- {item}" for item in items[:20])
