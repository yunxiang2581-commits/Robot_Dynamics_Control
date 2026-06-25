from __future__ import annotations

import importlib.util
import json
from datetime import datetime, timezone
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "tools" / "daily_hot_report.py"


def load_daily_hot_report_module():
    spec = importlib.util.spec_from_file_location("daily_hot_report", MODULE_PATH)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_rank_prefers_robotics_over_generic_ai():
    mod = load_daily_hot_report_module()
    items = [
        mod.NewsItem(
            title="New humanoid locomotion MPC paper",
            source="arXiv",
            url="https://example.com/robotics",
            published_at=datetime(2026, 6, 22, 2, tzinfo=timezone.utc),
            summary="whole-body control and locomotion",
            category="robotics",
        ),
        mod.NewsItem(
            title="New general AI assistant feature",
            source="Blog",
            url="https://example.com/ai",
            published_at=datetime(2026, 6, 22, 2, tzinfo=timezone.utc),
            summary="general model update",
            category="ai",
        ),
    ]

    ranked = mod.rank_items(items, now=datetime(2026, 6, 22, 8, tzinfo=timezone.utc))
    assert ranked[0].category == "robotics"
    assert ranked[0].score > ranked[1].score


def test_deduplicate_items_keeps_stronger_entry():
    mod = load_daily_hot_report_module()
    items = [
        mod.NewsItem(
            title="Robot locomotion paper",
            source="Unknown",
            url="https://example.com/post?utm_source=feed",
            published_at=datetime(2026, 6, 22, 2, tzinfo=timezone.utc),
            summary="short",
            category="robotics",
            score=4.0,
        ),
        mod.NewsItem(
            title="Robot locomotion paper",
            source="arXiv",
            url="https://example.com/post",
            published_at=datetime(2026, 6, 22, 2, tzinfo=timezone.utc),
            summary="longer summary with more signal",
            category="robotics",
            score=12.0,
        ),
    ]

    deduped = mod.deduplicate_items(items)
    assert len(deduped) == 1
    assert deduped[0].source == "arXiv"


def test_generate_report_writes_html_md_json(tmp_path):
    mod = load_daily_hot_report_module()
    output_dir = tmp_path / "每日热点"

    result = mod.generate_report(
        output_dir=output_dir,
        use_live_network=False,
        now=datetime(2026, 6, 22, 8, tzinfo=timezone.utc),
    )

    assert result.html_path.is_file()
    assert result.md_path.is_file()
    assert result.json_path.is_file()

    html_text = result.html_path.read_text(encoding="utf-8")
    assert "机器人运动控制" in html_text
    assert "今日摘要" in html_text

    payload = json.loads(result.json_path.read_text(encoding="utf-8"))
    assert payload["report_date"] == "2026-06-22"
    assert payload["status"] in {"ready", "degraded"}


def test_generate_report_marks_degraded_when_some_sources_fail(tmp_path):
    mod = load_daily_hot_report_module()
    output_dir = tmp_path / "每日热点"

    def fake_fetch_all_sources(sources, now, lookback_hours):
        return [
            mod.NewsItem(
                title="Humanoid locomotion MPC and whole-body control paper",
                source="arXiv cs.RO",
                url="https://example.com/robot-paper",
                published_at=datetime(2026, 6, 22, 2, tzinfo=timezone.utc),
                summary="legged locomotion, WBC, MPC and sim-to-real",
                category="robotics",
            )
        ], [
            mod.SourceFailure(
                source="Example Feed",
                url="https://example.com/feed",
                reason="timeout",
            )
        ]

    mod.fetch_all_sources = fake_fetch_all_sources
    mod.enrich_top_items_with_metadata = lambda items, limit=16: items

    result = mod.generate_report(
        output_dir=output_dir,
        use_live_network=True,
        now=datetime(2026, 6, 22, 8, tzinfo=timezone.utc),
    )

    payload = json.loads(result.json_path.read_text(encoding="utf-8"))
    assert result.status == "degraded"
    assert payload["status"] == "degraded"
    assert payload["failures"][0]["source"] == "Example Feed"


def test_generate_report_uses_ai_analyzer_when_available(tmp_path):
    mod = load_daily_hot_report_module()
    output_dir = tmp_path / "每日热点"
    raw_item = mod.NewsItem(
        title="Generic raw title",
        source="Raw Source",
        url="https://example.com/raw",
        published_at=datetime(2026, 6, 22, 2, tzinfo=timezone.utc),
        summary="raw candidate summary",
        category="ai",
    )

    def fake_fetch_all_sources(sources, now, lookback_hours):
        return [raw_item], []

    def fake_analyze(candidates, now, max_items, model="test-analyzer"):
        assert [item.url for item in candidates] == [raw_item.url]
        return mod.AnalysisResult(
            items=[
                mod.NewsItem(
                    title="AI picked humanoid locomotion priority",
                    source="Codex Analysis",
                    url="https://example.com/ai-picked",
                    published_at=datetime(2026, 6, 22, 3, tzinfo=timezone.utc),
                    summary="中文分析摘要",
                    category="robotics",
                    score=98.0,
                    why_it_matters="模型判断它直接服务机器人运动控制学习主线。",
                    tags=("运动控制", "MPC"),
                    content_type="robotics",
                )
            ],
            digest=("模型筛选后只保留最高价值机器人运动控制条目。",),
            model="test-analyzer",
            used_ai=True,
        )

    mod.fetch_all_sources = fake_fetch_all_sources
    mod.enrich_top_items_with_metadata = lambda items, limit=16: items
    mod.analyze_candidates_with_openai = fake_analyze

    result = mod.generate_report(
        output_dir=output_dir,
        use_live_network=True,
        use_ai_analysis=True,
        now=datetime(2026, 6, 22, 8, tzinfo=timezone.utc),
    )

    payload = json.loads(result.json_path.read_text(encoding="utf-8"))
    assert payload["analysis"]["used_ai"] is True
    assert payload["analysis"]["model"] == "test-analyzer"
    assert payload["items"][0]["title"] == "AI picked humanoid locomotion priority"
    assert "模型筛选" in result.html_path.read_text(encoding="utf-8")


def test_ai_analysis_falls_back_when_api_key_missing(tmp_path, monkeypatch):
    mod = load_daily_hot_report_module()
    output_dir = tmp_path / "每日热点"
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)

    result = mod.generate_report(
        output_dir=output_dir,
        use_live_network=False,
        use_ai_analysis=True,
        now=datetime(2026, 6, 22, 8, tzinfo=timezone.utc),
    )

    payload = json.loads(result.json_path.read_text(encoding="utf-8"))
    assert payload["analysis"]["used_ai"] is False
    assert payload["analysis"]["model"] == "rule-fallback"
    assert any("OPENAI_API_KEY" in failure["reason"] for failure in payload["failures"])


def test_analyzer_can_read_api_key_from_env_file(tmp_path, monkeypatch):
    mod = load_daily_hot_report_module()
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    env_path = tmp_path / ".env"
    env_path.write_text("OPENAI_API_KEY=from-env-file\n", encoding="utf-8")
    captured = {}

    def fake_post_json(url, payload, api_key, timeout=90):
        captured["api_key"] = api_key
        return {
            "output_text": json.dumps(
                {
                    "digest": ["模型筛选摘要一", "模型筛选摘要二"],
                    "selected_items": [
                        {
                            "url": "https://example.com/robot",
                            "title": "Humanoid locomotion control",
                            "source": "arXiv",
                            "published_at": "2026-06-22T02:00:00+00:00",
                            "summary": "中文摘要",
                            "why_it_matters": "直接关联机器人运动控制。",
                            "tags": ["运动控制", "MPC"],
                            "score": 91,
                            "category": "robotics",
                            "content_type": "robotics",
                            "image_url": None,
                        }
                    ],
                },
                ensure_ascii=False,
            )
        }

    mod.post_json = fake_post_json
    result = mod.analyze_candidates_with_openai(
        [
            mod.NewsItem(
                title="raw",
                source="raw",
                url="https://example.com/robot",
                published_at=datetime(2026, 6, 22, 2, tzinfo=timezone.utc),
                summary="raw",
                category="robotics",
            )
        ],
        now=datetime(2026, 6, 22, 8, tzinfo=timezone.utc),
        max_items=5,
        env_files=(env_path,),
    )

    assert captured["api_key"] == "from-env-file"
    assert result.used_ai is True
    assert result.items[0].title == "Humanoid locomotion control"


def test_arxiv_search_url_contains_motion_control_keywords():
    mod = load_daily_hot_report_module()
    url = mod.build_arxiv_search_url("humanoid locomotion mpc whole-body control")

    assert "search_query=" in url
    assert "humanoid" in url.lower()
    assert "locomotion" in url.lower()
    assert "mpc" in url.lower()
    assert "whole" in url.lower()


def test_arxiv_motion_control_items_keep_weekly_context():
    mod = load_daily_hot_report_module()
    now = datetime(2026, 6, 22, 8, tzinfo=timezone.utc)
    items = [
        mod.NewsItem(
            title="Dexterous manipulation and legged locomotion paper",
            source="arXiv Motion Control Search",
            url="https://example.com/arxiv-paper",
            published_at=datetime(2026, 6, 18, 8, tzinfo=timezone.utc),
            summary="robot dexterous manipulation and locomotion",
            category="robotics",
        )
    ]

    filtered = mod.filter_recent_items(items, now, lookback_hours=30)

    assert filtered == items


def test_cli_help_and_scheduler_script_are_present():
    script_path = MODULE_PATH
    scheduler_path = script_path.with_name("install_daily_hot_report_task.ps1")

    assert script_path.is_file()
    assert scheduler_path.is_file()

    scheduler_text = scheduler_path.read_text(encoding="utf-8")
    assert "New-ScheduledTaskTrigger -Daily -At $StartTime" in scheduler_text
    assert "daily_hot_report.py" in scheduler_text
    assert "AnalysisModel" in scheduler_text
    assert "--analysis-model" in scheduler_text
    assert "DefaultOutputDir" in scheduler_text
    assert "684c" in scheduler_text
    assert "6bcf" in scheduler_text
