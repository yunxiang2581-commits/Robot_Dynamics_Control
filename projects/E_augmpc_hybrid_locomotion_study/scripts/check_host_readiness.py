from __future__ import annotations

import argparse
import os
import platform
import shutil
import subprocess
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check local host readiness for Project E reproduction.")
    parser.add_argument("--output-dir", required=True, help="Directory to write the markdown report into.")
    return parser.parse_args()


def run_command(cmd: list[str]) -> tuple[bool, str]:
    try:
        result = subprocess.run(cmd, check=False, capture_output=True, text=True)
    except OSError as exc:
        return False, str(exc)

    output = (result.stdout or "") + (result.stderr or "")
    return result.returncode == 0, output.strip()


def read_os_release() -> str:
    path = Path("/etc/os-release")
    if not path.exists():
        return "not found"
    return path.read_text(encoding="utf-8").strip()


def tool_summary(tool_name: str) -> tuple[str, str]:
    path = shutil.which(tool_name)
    if path is None:
        return "missing", "not found in PATH"
    ok, output = run_command([tool_name, "--version"])
    preview = output.splitlines()[0] if output else "no version output"
    return path, preview if ok or preview else "failed to query version"


def preview_command(tool_name: str, args: list[str], max_lines: int = 8) -> str:
    path = shutil.which(tool_name)
    if path is None:
        return "not found in PATH"
    ok, output = run_command([tool_name, *args])
    lines = output.splitlines()[:max_lines]
    prefix = "\n".join(lines).strip()
    if not prefix:
        return "no output"
    if not ok:
        return f"command returned non-zero exit code\n{prefix}"
    return prefix


def build_report() -> str:
    docker_path, docker_version = tool_summary("docker")
    apptainer_path, apptainer_version = tool_summary("apptainer")
    singularity_path, singularity_version = tool_summary("singularity")
    nvcc_path, nvcc_version = tool_summary("nvcc")
    nvidia_preview = preview_command("nvidia-smi", [], max_lines=12)

    lines = [
        "# Project E Host Readiness Report",
        "",
        "## OS",
        f"- kernel: `{platform.uname()}`",
        "",
        "```text",
        read_os_release(),
        "```",
        "",
        "## Python",
        f"- executable: `{shutil.which('python3') or 'not found'}`",
        f"- version: `{platform.python_version()}`",
        "",
        "## Container Runtimes",
        f"- docker: `{docker_path}`",
        f"- docker version: `{docker_version}`",
        f"- apptainer: `{apptainer_path}`",
        f"- apptainer version: `{apptainer_version}`",
        f"- singularity: `{singularity_path}`",
        f"- singularity version: `{singularity_version}`",
        "",
        "## GPU Tooling",
        f"- nvcc: `{nvcc_path}`",
        f"- nvcc version: `{nvcc_version}`",
        "",
        "### nvidia-smi Preview",
        "```text",
        nvidia_preview,
        "```",
        "",
        "## Display And Device Environment",
        f"- DISPLAY: `{os.environ.get('DISPLAY', 'unset')}`",
        f"- NVIDIA_VISIBLE_DEVICES: `{os.environ.get('NVIDIA_VISIBLE_DEVICES', 'unset')}`",
        f"- CUDA_VISIBLE_DEVICES: `{os.environ.get('CUDA_VISIBLE_DEVICES', 'unset')}`",
        "",
        "## Summary",
        "- This report is read-only and does not install or start anything.",
        "- Use it to judge whether container-first reproduction is plausible on the current host.",
    ]
    return "\n".join(lines) + "\n"


def main() -> int:
    args = parse_args()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    report_path = output_dir / "host_readiness_report.md"
    report_path.write_text(build_report(), encoding="utf-8")
    print(report_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
