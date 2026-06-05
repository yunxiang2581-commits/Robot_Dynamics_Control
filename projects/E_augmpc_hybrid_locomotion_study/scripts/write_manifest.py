from __future__ import annotations

import argparse
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Write a Project E manifest.json and manifest.md.")
    parser.add_argument("--project-root", required=True, help="Path to the Project E root directory.")
    parser.add_argument("--output-dir", required=True, help="Directory to write manifest files into.")
    return parser.parse_args()


def run_command(cmd: list[str], cwd: Path) -> str:
    try:
        result = subprocess.run(cmd, cwd=cwd, check=False, capture_output=True, text=True)
    except OSError as exc:
        return str(exc)
    return ((result.stdout or "") + (result.stderr or "")).strip()


def collect_key_files(project_root: Path) -> list[str]:
    matches: list[str] = []
    for pattern in ["README.md", "E_CURRENT_STATUS.md", "docs/*.md", "scripts/*.py"]:
        for path in sorted(project_root.glob(pattern)):
            matches.append(str(path.relative_to(project_root)))
    return matches


def collect_outputs_tree(project_root: Path) -> list[str]:
    outputs_root = project_root / "outputs"
    if not outputs_root.exists():
        return []
    entries: list[str] = []
    base_depth = len(outputs_root.parts)
    for path in sorted(outputs_root.rglob("*")):
        depth = len(path.parts) - base_depth
        if depth > 3:
            continue
        entries.append(str(path.relative_to(project_root)))
    return entries


def build_manifest(project_root: Path) -> dict:
    repo_root = project_root.parents[1]
    return {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "project_root": str(project_root),
        "git_status_short_branch": run_command(["git", "status", "--short", "--branch"], cwd=repo_root),
        "key_files": collect_key_files(project_root),
        "outputs_tree": collect_outputs_tree(project_root),
    }


def manifest_markdown(manifest: dict) -> str:
    lines = [
        "# Project E Manifest",
        "",
        f"- generated_at_utc: `{manifest['generated_at_utc']}`",
        f"- project_root: `{manifest['project_root']}`",
        "",
        "## git status --short --branch",
        "```text",
        manifest["git_status_short_branch"] or "(empty)",
        "```",
        "",
        "## Key Files",
        "```text",
        "\n".join(manifest["key_files"]) or "(none)",
        "```",
        "",
        "## Outputs Tree",
        "```text",
        "\n".join(manifest["outputs_tree"]) or "(none)",
        "```",
    ]
    return "\n".join(lines) + "\n"


def main() -> int:
    args = parse_args()
    project_root = Path(args.project_root).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest = build_manifest(project_root)
    json_path = output_dir / "manifest.json"
    md_path = output_dir / "manifest.md"
    json_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")
    md_path.write_text(manifest_markdown(manifest), encoding="utf-8")
    print(json_path)
    print(md_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
