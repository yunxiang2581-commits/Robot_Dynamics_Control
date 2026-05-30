from __future__ import annotations

from pathlib import Path

from .models import ProjectLine, ScanResult

IGNORE_NAMES = {".git", "__pycache__", ".pytest_cache", ".venv", "external/open_source_repos"}
PROJECT_LINES = {
    "A": ("A_self_baseline", "projects/A_self_baseline", "自研 baseline 学习与 demo"),
    "B": ("B_mujoco_mpc_study", "projects/B_mujoco_mpc_study", "MuJoCo MPC / iLQR / iLQG demo"),
    "C": ("C_openloong_dyn_control_study", "projects/C_openloong_dyn_control_study", "OpenLoong-Dyn-Control study"),
    "D": ("D_legged_control_study", "projects/D_legged_control_study", "legged-control study"),
}


def default_repo_root() -> Path:
    cwd = Path.cwd()
    if (cwd / "projects").exists() and (cwd / "docs" / "00_project_management").exists():
        return cwd
    return Path("/home/ubuntu/Robot_Dynamics_Control")


def is_ignored(path: Path, ignore_names: set[str] | None = None) -> bool:
    ignore_names = ignore_names or IGNORE_NAMES
    parts = set(path.parts)
    if any(name in parts for name in ignore_names if "/" not in name):
        return True
    normalized = path.as_posix()
    return any(name in normalized for name in ignore_names if "/" in name)


def collect_files(root: Path, suffixes: tuple[str, ...], ignore_names: set[str] | None = None) -> list[Path]:
    if not root.exists():
        return []
    files: list[Path] = []
    for path in root.rglob("*"):
        if is_ignored(path, ignore_names):
            continue
        if path.is_file() and path.suffix in suffixes:
            files.append(path)
    return sorted(files)


def count_text_markers(files: list[Path], markers: list[str]) -> int:
    count = 0
    for path in files:
        try:
            text = path.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        count += sum(text.count(marker) for marker in markers)
    return count


def is_test_file(path: Path) -> bool:
    name = path.name
    return name.startswith("test_") or name.endswith("_test.py")


def is_entrypoint_file(path: Path) -> bool:
    if path.suffix != ".py":
        return False
    name = path.name
    stem = path.stem
    direct = name in {"main.py", "run.py", "demo.py", "train.py", "eval.py", "test.py"}
    prefixed = stem.startswith(("run_", "demo_", "train_", "eval_"))
    in_entry_dir = any(part in {"scripts", "examples"} for part in path.parts)
    return direct or prefixed or in_entry_dir


def scan_project_line(repo_root: Path, project_id: str, relative_path: str, goal: str) -> ProjectLine:
    path = repo_root / relative_path
    python_files = collect_files(path, (".py",), IGNORE_NAMES)
    markdown_files = collect_files(path, (".md",), IGNORE_NAMES)
    text_files = python_files + markdown_files
    return ProjectLine(
        id=project_id,
        name=Path(relative_path).name,
        path=path,
        goal=goal,
        exists=path.exists(),
        python_files=python_files,
        test_files=[file for file in python_files if is_test_file(file)],
        markdown_files=markdown_files,
        todo_count=count_text_markers(text_files, ["TODO", "todo", "待实现"]),
        not_implemented_count=count_text_markers(python_files, ["NotImplementedError"]),
        entrypoint_files=[file for file in python_files if is_entrypoint_file(file)],
    )


def scan_repo(repo_root: Path | None = None) -> ScanResult:
    root = repo_root or default_repo_root()
    project_lines = {
        project_id: scan_project_line(root, project_id, relative_path, goal)
        for project_id, (_name, relative_path, goal) in PROJECT_LINES.items()
    }
    docs_root = root / "docs" / "00_project_management"
    stage_docs = collect_files(docs_root, (".md",), IGNORE_NAMES)
    warnings = [f"项目线 {pid} 目录不存在：{line.path}" for pid, line in project_lines.items() if not line.exists]
    return ScanResult(
        repo_root=root,
        project_lines=project_lines,
        docs_count=len(stage_docs),
        stage_docs=stage_docs,
        total_todos=sum(line.todo_count for line in project_lines.values()),
        total_not_implemented=sum(line.not_implemented_count for line in project_lines.values()),
        warnings=warnings,
    )
