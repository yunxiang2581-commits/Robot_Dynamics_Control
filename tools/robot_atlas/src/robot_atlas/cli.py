from __future__ import annotations

import argparse
from pathlib import Path

from .codex_prompt import generate_codex_prompt
from .demo_assets import build_demo_assets_report
from .explain import explain_topic
from .external_repo import (
    build_entrypoints_report,
    build_port_to_b_report,
    build_read_repo_report,
    build_reproduce_plan,
    generate_codex_reproduce_prompt,
)
from .guards import build_guards_report
from .next_step import build_next_step
from .router import route_request
from .scanner import scan_repo
from .status import build_status
from .understand_adapter import (
    build_import_understand_report,
    build_understand_entrypoints_report,
    build_understand_port_to_b_report,
    build_understand_summary_report,
    generate_codex_understand_prompt,
)
from .validate import build_validation_plan


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="robot-atlas", description="RoboControl Atlas local project understanding tool.")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("scan")
    status = sub.add_parser("status")
    status.add_argument("project_id")
    next_cmd = sub.add_parser("next")
    next_cmd.add_argument("stage_id")
    explain = sub.add_parser("explain")
    explain.add_argument("topic")
    codex = sub.add_parser("generate-codex")
    codex.add_argument("task_id")
    validate = sub.add_parser("validate")
    validate.add_argument("stage")
    demo = sub.add_parser("demo-assets")
    demo.add_argument("stage")
    sub.add_parser("guards")
    route = sub.add_parser("route")
    route.add_argument("user_request")
    for name in [
        "read-repo",
        "entrypoints",
        "reproduce-plan",
        "port-to-B",
        "generate-codex-reproduce",
        "import-understand",
        "understand-summary",
        "understand-entrypoints",
        "understand-port-to-B",
        "generate-codex-understand",
    ]:
        cmd = sub.add_parser(name)
        cmd.add_argument("repo_path")
    return parser


def _path(value: str) -> Path:
    return Path(value)


def dispatch(args: argparse.Namespace) -> str:
    command = args.command
    if command == "scan":
        return scan_repo().as_text()
    if command == "status":
        return build_status(args.project_id, scan_repo())
    if command == "next":
        return build_next_step(args.stage_id)
    if command == "explain":
        return explain_topic(args.topic)
    if command == "generate-codex":
        return generate_codex_prompt(args.task_id)
    if command == "validate":
        return build_validation_plan(args.stage)
    if command == "demo-assets":
        return build_demo_assets_report(args.stage)
    if command == "guards":
        return build_guards_report()
    if command == "route":
        return route_request(args.user_request).as_text()

    repo_path = _path(args.repo_path)
    if command in {
        "read-repo",
        "entrypoints",
        "reproduce-plan",
        "port-to-B",
        "generate-codex-reproduce",
        "import-understand",
        "understand-summary",
        "understand-entrypoints",
        "understand-port-to-B",
        "generate-codex-understand",
    } and not repo_path.exists():
        return f"路径不存在：{repo_path}"

    if command == "read-repo":
        return build_read_repo_report(repo_path)
    if command == "entrypoints":
        return build_entrypoints_report(repo_path)
    if command == "reproduce-plan":
        return build_reproduce_plan(repo_path)
    if command == "port-to-B":
        return build_port_to_b_report(repo_path)
    if command == "generate-codex-reproduce":
        return generate_codex_reproduce_prompt(repo_path)
    if command == "import-understand":
        return build_import_understand_report(repo_path)
    if command == "understand-summary":
        return build_understand_summary_report(repo_path)
    if command == "understand-entrypoints":
        return build_understand_entrypoints_report(repo_path)
    if command == "understand-port-to-B":
        return build_understand_port_to_b_report(repo_path)
    if command == "generate-codex-understand":
        return generate_codex_understand_prompt(repo_path)
    return f"未知命令：{command}"


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    print(dispatch(args))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
