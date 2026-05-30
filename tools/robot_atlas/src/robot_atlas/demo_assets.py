from __future__ import annotations

from pathlib import Path

from .models import DemoAssetReport

REQUIRED_DEMO_ASSETS = {
    "B03": [
        "tracking_error.png",
        "cost_history.png",
        "torque_profile.png",
        "state_tracking_smoke.mp4",
        "smoke_summary.json",
        "stepB03R4C1B_state_tracking_smoke_run.md",
    ]
}


def check_demo_assets(stage: str, repo_root: Path | None = None) -> DemoAssetReport:
    root = repo_root or Path.cwd()
    required = REQUIRED_DEMO_ASSETS.get(stage.upper(), [])
    existing: list[Path] = []
    missing: list[str] = []
    for name in required:
        matches = list(root.rglob(name)) if root.exists() else []
        if matches:
            existing.extend(matches)
        else:
            missing.append(name)
    conclusion = "demo 产物完整" if not missing else "demo 产物缺失，需要补齐证据链"
    return DemoAssetReport(stage=stage.upper(), required=required, existing=existing, missing=missing, conclusion=conclusion)


def build_demo_assets_report(stage: str, repo_root: Path | None = None) -> str:
    report = check_demo_assets(stage, repo_root)
    existing = "\n".join(f"- {path}" for path in report.existing) or "- 无"
    missing = "\n".join(f"- {item}" for item in report.missing) or "- 无"
    return f"""Demo Assets 检查：{report.stage}

必需产物：
{chr(10).join(f"- {item}" for item in report.required) if report.required else "- 暂无内置要求"}

已存在：
{existing}

缺失：
{missing}

结论：
{report.conclusion}"""
