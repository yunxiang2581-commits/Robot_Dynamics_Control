from __future__ import annotations

import argparse
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Collect the current Project E reproduction status.")
    parser.add_argument("--project-root", required=True, help="Path to the Project E root directory.")
    parser.add_argument("--output-dir", required=True, help="Directory to write the markdown report into.")
    return parser.parse_args()


def list_children(path: Path, kind: str) -> list[str]:
    if not path.exists():
        return []
    if kind == "dir":
        return sorted(entry.name for entry in path.iterdir() if entry.is_dir())
    return sorted(entry.name for entry in path.iterdir() if entry.is_file())


def find_public_artifacts(project_root: Path) -> list[str]:
    patterns = ["*.pt", "*.pth", "*.onnx", "*.ckpt", "*.bag", "*.mcap", "*.zip", "*.tar", "*.sif"]
    matches: set[str] = set()
    for pattern in patterns:
        for path in project_root.rglob(pattern):
            matches.add(str(path.relative_to(project_root)))
    return sorted(matches)


def build_report(project_root: Path) -> str:
    expected_dirs = [
        "docs",
        "scripts",
        "configs",
        "notes",
        "outputs/audits",
        "outputs/logs",
        "outputs/manifests",
        "outputs/reproduction",
        "external/open_source_repos",
    ]
    lines = [
        "# Project E Reproduction Status",
        "",
        f"- project root: `{project_root}`",
        "",
        "## Directory Checks",
    ]
    for rel in expected_dirs:
        target = project_root / rel
        lines.append(f"- `{rel}`: `{'present' if target.exists() else 'missing'}`")

    repo_root = project_root / "external" / "open_source_repos"
    lines.extend(
        [
            "",
            "## External Repositories",
            "```text",
            "\n".join(list_children(repo_root, "dir")) or "(none)",
            "```",
            "",
            "## Output Directories",
            "### outputs/audits",
            "```text",
            "\n".join(list_children(project_root / "outputs" / "audits", "dir")) or "(none)",
            "```",
            "### outputs/logs",
            "```text",
            "\n".join(list_children(project_root / "outputs" / "logs", "file")) or "(none)",
            "```",
            "### outputs/reproduction",
            "```text",
            "\n".join(list_children(project_root / "outputs" / "reproduction", "dir")) or "(none)",
            "```",
            "",
            "## Public Bundles / Model Artifacts",
            "```text",
            "\n".join(find_public_artifacts(project_root)) or "(none found)",
            "```",
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> int:
    args = parse_args()
    project_root = Path(args.project_root).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    report_path = output_dir / "reproduction_status.md"
    report_path.write_text(build_report(project_root), encoding="utf-8")
    print(report_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
