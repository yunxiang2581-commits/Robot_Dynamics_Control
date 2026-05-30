from __future__ import annotations

import re
from dataclasses import dataclass


@dataclass
class RouteResult:
    request_type: str
    commands: list[str]
    reason: str
    notes: str

    def as_text(self) -> str:
        command_text = "\n".join(self.commands)
        return "\n".join(
            [
                "识别到的需求类型：",
                self.request_type,
                "",
                "推荐命令：",
                command_text,
                "",
                "理由：",
                self.reason,
                "",
                "注意事项：",
                self.notes,
            ]
        )


ALGORITHM_ALIASES = [
    ("iLQG-lite", ["ilqg-lite", "ilqg lite", "iLQG-lite"]),
    ("iLQR", ["ilqr", "iLQR"]),
    ("MPC", ["mpc", "MPC"]),
    ("MPPI", ["mppi", "MPPI"]),
    ("CEM", ["cem", "CEM"]),
    ("QP", ["qp", "QP"]),
    ("task-space tracking", ["task-space", "task space", "任务空间"]),
    ("two-link dynamics", ["two-link", "two link", "双连杆"]),
]


def _lower_text(request: str) -> str:
    return request.lower()


def _has_any(text: str, keywords: list[str]) -> bool:
    return any(keyword.lower() in text for keyword in keywords)


def _extract_task_id(request: str) -> str | None:
    match = re.search(r"\b([A-D]\d{2}(?:-[A-Z]\d*[A-Z]?)?(?:-\d+[A-Z]?)?)\b", request, re.IGNORECASE)
    if not match:
        return None
    return match.group(1).upper()


def _extract_repo_path(request: str) -> str | None:
    candidates = re.findall(r"([A-Za-z]:\\[^\s]+|(?:external|\.|/)[^\s]+)", request)
    for candidate in candidates:
        if "repo" in candidate.lower() or "open_source_repos" in candidate.lower() or "\\" in candidate or "/" in candidate:
            return candidate.rstrip("。,.，")
    return None


def _route_external_commands(repo_path: str | None) -> tuple[list[str], str]:
    path = repo_path or "<repo_path>"
    notes = "外部 repo 默认只读；不要自动 clone、安装依赖或运行训练脚本。"
    if repo_path is None:
        notes += " 需要提供本地 repo 路径。"
    return (
        [
            f"robot-atlas read-repo {path}",
            f"robot-atlas entrypoints {path}",
            f"robot-atlas reproduce-plan {path}",
        ],
        notes,
    )


def route_request(request: str) -> RouteResult:
    text = _lower_text(request)
    repo_path = _extract_repo_path(request)

    if _has_any(text, ["禁止", "安全边界", "不能改", "不要改", "guards", "git commit", "git push"]):
        return RouteResult(
            request_type="安全边界 / 禁止事项",
            commands=["robot-atlas guards"],
            reason="用户询问工具或项目的禁止事项，应优先确认安全边界。",
            notes="不要执行 git add / git commit / git push；不要修改 external/open_source_repos 或 shared/robot_assets。",
        )

    task_id = _extract_task_id(request)
    if task_id and _has_any(text, ["codex", "提示词", "prompt", "命令", "自动实现"]):
        return RouteResult(
            request_type="Codex 提示词生成",
            commands=[f"robot-atlas generate-codex {task_id}"],
            reason=f"用户明确要求为任务 {task_id} 生成 Codex 提示词。",
            notes="只生成当前任务提示词；不要自动扩展到后续控制任务。",
        )

    if _has_any(text, ["understand anything", "knowledge-graph", "knowledge graph", "图谱", "/understand"]):
        path = repo_path or "<repo_path>"
        notes = "只读取已有 .understand-anything/knowledge-graph.json，不自动运行 /understand。"
        if repo_path is None:
            notes += " 需要提供本地 repo 路径。"
        return RouteResult(
            request_type="Understand Anything 图谱审读",
            commands=[
                f"robot-atlas import-understand {path}",
                f"robot-atlas understand-summary {path}",
            ],
            reason="用户要求基于 Understand Anything 图谱理解项目。",
            notes=notes,
        )

    if _has_any(text, ["demo", "产物", "视频", "图", "日志", "求职展示"]):
        return RouteResult(
            request_type="demo 产物检查",
            commands=["robot-atlas demo-assets B03"],
            reason="用户要求检查 B03 demo 证据链是否齐全。",
            notes="缺失产物只应列入 missing list，不应静默忽略。",
        )

    if _has_any(text, ["验证", "验收", "测试", "怎么验", "跑哪些"]):
        return RouteResult(
            request_type="验证计划",
            commands=["robot-atlas validate B03"],
            reason="用户询问 B03 或当前阶段如何验证。",
            notes="验证计划不等于长时间仿真；优先 smoke run、单元测试和文本产物检查。",
        )

    if _has_any(text, ["下一步", "后面怎么", "推进", "进入哪个任务"]):
        return RouteResult(
            request_type="B03 下一步任务规划",
            commands=["robot-atlas next B03"],
            reason="用户询问当前学习路径下一步，默认主线是 B03。",
            notes="不要跳到 task-space tracking、WBC 或完整控制架构。",
        )

    if _has_any(text, ["状态", "进度", "做到哪", "当前项目", "b 项目", "b_mujoco"]):
        command = "robot-atlas status B" if _has_any(text, ["b", "b 项目", "b_mujoco"]) else "robot-atlas scan"
        return RouteResult(
            request_type="内部项目状态",
            commands=[command],
            reason="用户询问 Robot_Dynamics_Control 内部项目状态。",
            notes="如果没有指定项目线，先 scan；当前默认主线优先 B。",
        )

    if _has_any(text, ["迁移到 b", "接入我的 b", "port-to-b", "哪些模块值得学", "有什么帮助"]):
        path = repo_path or "<repo_path>"
        notes = "外部代码默认只读；只生成迁移建议，不直接复制源码。"
        if repo_path is None:
            notes += " 需要提供本地 repo 路径。"
        return RouteResult(
            request_type="外部 repo 迁移到 B 项目",
            commands=[f"robot-atlas port-to-B {path}"],
            reason="用户询问外部项目哪些模块适合迁移到 B_mujoco_mpc_study。",
            notes=notes,
        )

    if _has_any(text, ["外部", "repo", "复现", "入口", "train", "eval", "demo 在哪里", "怎么跑"]):
        commands, notes = _route_external_commands(repo_path)
        return RouteResult(
            request_type="外部 repo 审读 / 复现路线",
            commands=commands,
            reason="用户询问外部开源项目结构、入口或复现路线。",
            notes=notes,
        )

    for topic, aliases in ALGORITHM_ALIASES:
        if _has_any(text, aliases):
            return RouteResult(
                request_type="算法解释",
                commands=[f"robot-atlas explain {topic}"],
                reason=f"用户提到 {topic}，适合输出学习笔记式解释。",
                notes="解释应包含任务目标、输入输出、核心公式、项目位置、验证标准和禁止事项。",
            )

    return RouteResult(
        request_type="未匹配到明确场景",
        commands=["robot-atlas scan"],
        reason="未识别到更具体的路由规则，先扫描当前仓库状态。",
        notes="如果需求涉及外部 repo，请提供本地 repo 路径。",
    )
