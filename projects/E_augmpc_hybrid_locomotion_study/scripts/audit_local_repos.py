from __future__ import annotations

import argparse
import subprocess
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Audit a local repository without running its code.")
    parser.add_argument("--repo-root", required=True, help="Path to the repository root to inspect.")
    parser.add_argument("--output-dir", required=True, help="Directory to write the markdown report into.")
    return parser.parse_args()


def run_command(cmd: list[str], cwd: Path | None = None) -> tuple[bool, str]:
    try:
        result = subprocess.run(cmd, cwd=cwd, check=False, capture_output=True, text=True)
    except OSError as exc:
        return False, str(exc)
    output = ((result.stdout or "") + (result.stderr or "")).strip()
    return result.returncode == 0, output


def list_tree(repo_root: Path, max_depth: int) -> list[str]:
    base_depth = len(repo_root.parts)
    entries: list[str] = []
    for path in sorted(repo_root.rglob("*")):
        depth = len(path.parts) - base_depth
        if depth > max_depth:
            continue
        entries.append(str(path.relative_to(repo_root)))
    return entries


def detect_special_files(repo_root: Path) -> dict[str, bool]:
    patterns = {
        "README": ["README.md", "README.rst", "README"],
        "LICENSE": ["LICENSE", "LICENSE.md", "COPYING"],
        "pyproject": ["pyproject.toml"],
        "requirements": ["requirements.txt", "requirements-dev.txt"],
        "Dockerfile": ["Dockerfile", "docker/Dockerfile"],
        "Singularity": ["Singularity", "Singularity.def"],
        "Apptainer": ["Apptainer.def", "apptainer.def"],
    }
    result: dict[str, bool] = {}
    for key, names in patterns.items():
        result[key] = any((repo_root / name).exists() for name in names)
    return result


def build_report(repo_root: Path) -> str:
    if not repo_root.exists():
        return "\n".join(
            [
                "# Local Repo Audit",
                "",
                f"- repo root: `{repo_root}`",
                "- status: missing",
            ]
        ) + "\n"

    git_ok, head = run_command(["git", "rev-parse", "HEAD"], cwd=repo_root)
    _, branch = run_command(["git", "branch", "--show-current"], cwd=repo_root)
    _, remote = run_command(["git", "remote", "-v"], cwd=repo_root)
    files = detect_special_files(repo_root)
    top_tree = list_tree(repo_root, max_depth=1)
    second_tree = list_tree(repo_root, max_depth=2)

    lines = [
        "# Local Repo Audit",
        "",
        f"- repo root: `{repo_root}`",
        f"- git head: `{head if git_ok else 'not a git repository or unavailable'}`",
        f"- git branch: `{branch or 'not available'}`",
        "",
        "## Remote",
        "```text",
        remote or "not available",
        "```",
        "",
        "## Special Files",
    ]
    for key, present in files.items():
        lines.append(f"- {key}: `{'present' if present else 'missing'}`")

    lines.extend(["", "## Top-Level Tree", "```text"])
    lines.extend(top_tree or ["(empty)"])
    lines.extend(["```", "", "## Second-Level Tree", "```text"])
    lines.extend(second_tree or ["(empty)"])
    lines.extend(["```"])
    return "\n".join(lines) + "\n"


def main() -> int:
    args = parse_args()
    repo_root = Path(args.repo_root).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    report_path = output_dir / "local_repo_audit.md"
    report_path.write_text(build_report(repo_root), encoding="utf-8")
    print(report_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
