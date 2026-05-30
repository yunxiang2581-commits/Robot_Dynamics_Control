"""A pipeline input/output path helpers.

本模块只提供 TODO 教学骨架, 用来统一 A01-A07 的输出路径约定。
当前不实现完整 JSON/Markdown 写入逻辑, 后续按 pipeline 契约逐步补全。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any


def get_a_project_root() -> Path:
    """Return the A project root path.

    TODO(中文):
    - 要解决什么路径问题: 标准脚本可能从任意 shell 工作目录启动, 不能依赖当前目录推断 A 项目根目录。
    - 为什么 pipeline 需要统一输出管理: A01-A07 都要把报告、缓存、轨迹放到同一套 `outputs/` 规范下。
    - 输入是什么: 无显式输入, 后续应基于本文件位置或调用方传入路径定位。
    - 输出是什么: `projects/A_self_baseline/` 的绝对 Path。
    - 如何验证: 从不同工作目录运行脚本, 打印出的 A 根目录应一致。
    """
    raise NotImplementedError("TODO: locate A project root.")


def get_repo_root() -> Path:
    """Return the monorepo root path.

    TODO(中文):
    - 要解决什么路径问题: A 项目需要访问 `shared/robot_assets/` 和仓库级文档, 因此要能定位总仓库根目录。
    - 为什么 pipeline 需要统一输出管理: 统一区分 A 项目输出和仓库级共享资源, 避免脚本写到错误目录。
    - 输入是什么: 无显式输入, 后续可由 A 项目根目录向上推导。
    - 输出是什么: `/home/ubuntu/Robot_Dynamics_Control` 对应的 Path。
    - 如何验证: 返回路径下应存在 `projects/`, `shared/`, `tools/`, `docs/`。
    """
    raise NotImplementedError("TODO: locate repository root.")


def get_output_dir(a_root: Path, kind: str) -> Path:
    """Return an output subdirectory for a pipeline artifact kind.

    TODO(中文):
    - 要解决什么路径问题: 不同脚本不能各自随意创建 `outputs/foo`, 应统一使用 cache/reports/figures/trajectories/logs/videos。
    - 为什么 pipeline 需要统一输出管理: A04/A05 的轨迹要被 A06 消费, A03 的报告要被 A04/A05 复查。
    - 输入是什么: a_root 是 A 项目根目录, kind 是输出类别, 如 `reports` 或 `trajectories`。
    - 输出是什么: `a_root / "outputs" / kind` 的 Path。
    - 如何验证: kind 合法时目录路径稳定; kind 非法时后续应抛出清晰错误。
    """
    raise NotImplementedError("TODO: return standardized output directory.")


def build_step_output_paths(step_id: str, a_root: Path) -> dict[str, Path]:
    """Build standard output file paths for one A pipeline step.

    TODO(中文):
    - 要解决什么路径问题: 每个步骤需要同时写报告、缓存、图像或轨迹, 文件名应包含 A01-A07 步骤编号。
    - 为什么 pipeline 需要统一输出管理: 后续步骤可以按约定找到前置产物, 不需要猜文件名。
    - 输入是什么: step_id 是 `A01` 到 `A07` 之一, a_root 是 A 项目根目录。
    - 输出是什么: dict, 例如包含 `report`, `cache`, `figure`, `trajectory`, `log`, `video` 等 Path。
    - 如何验证: 对每个 step_id 打印路径, 应全部落在 `projects/A_self_baseline/outputs/` 下。
    """
    raise NotImplementedError("TODO: build standard output paths for one step.")


def write_json_placeholder(path: Path, payload: dict[str, Any]) -> None:
    """Write a lightweight JSON artifact placeholder.

    TODO(中文):
    - 要解决什么路径问题: A01/A03/A05/A07 需要保存轻量结构化结果, 路径和父目录创建应统一处理。
    - 为什么 pipeline 需要统一输出管理: JSON 缓存是后续步骤读取前置结果的桥梁。
    - 输入是什么: path 是目标 JSON 路径, payload 是可序列化的 dict。
    - 输出是什么: 写入磁盘的 JSON 文件; 当前函数返回 None。
    - 如何验证: 写入后重新读取 JSON, 字段和值应与 payload 一致。
    """
    raise NotImplementedError("TODO: write JSON artifact placeholder.")


def write_markdown_placeholder(path: Path, title: str, lines: list[str]) -> None:
    """Write a Markdown report placeholder.

    TODO(中文):
    - 要解决什么路径问题: A01-A07 都需要可复盘报告, 不能把报告散落在脚本目录或根目录。
    - 为什么 pipeline 需要统一输出管理: Markdown 报告是面试复盘和调试记录的主要入口。
    - 输入是什么: path 是目标 Markdown 路径, title 是报告标题, lines 是报告正文行。
    - 输出是什么: 写入磁盘的 Markdown 文件; 当前函数返回 None。
    - 如何验证: 写入后打开文件, 标题、输入、输出、验证结果应完整可读。
    """
    raise NotImplementedError("TODO: write Markdown report placeholder.")
