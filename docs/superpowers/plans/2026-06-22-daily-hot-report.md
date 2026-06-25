# Daily Hot Report Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a daily Chinese HTML report generator that highlights robot motion control first and AI news second, saves the report to `D:\桌面\每日热点`, and can be scheduled to run every morning at 07:30.

**Architecture:** Keep the feature in one focused Python script with small pure helpers for fetching, normalizing, scoring, deduplicating, and rendering. Use RSS/Atom-first sources with lightweight HTML metadata fallback, then render HTML/Markdown/JSON outputs and a Windows scheduled-task installer for automation.

**Tech Stack:** Python 3.12, `requests`, `beautifulsoup4`, `Jinja2`, `pytest`, Windows Task Scheduler / PowerShell

---

### Task 1: Lock down the core data model and ranking behavior with tests

**Files:**
- Create: `tests/test_daily_hot_report.py`

- [ ] **Step 1: Write the failing test**

```python
from datetime import datetime, timezone
import importlib.util
from pathlib import Path

MODULE_PATH = Path(__file__).resolve().parents[1] / "tools" / "daily_hot_report.py"


def load_daily_hot_report_module():
    spec = importlib.util.spec_from_file_location("daily_hot_report", MODULE_PATH)
    mod = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(mod)
    return mod


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

    ranked = mod.rank_items(items)
    assert ranked[0].category == "robotics"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_daily_hot_report.py -q`

Expected: fail because the module and helpers do not exist yet.

- [ ] **Step 3: Add the minimal model and pure helpers**

```python
@dataclass(frozen=True)
class NewsItem:
    title: str
    source: str
    url: str
    published_at: datetime | None
    summary: str
    category: str
    score: float = 0.0
```

### Task 2: Implement feed fetching, deduplication, scoring, and HTML rendering

**Files:**
- Create: `tools/daily_hot_report.py`
- Modify: `requirements.txt`

- [ ] **Step 1: Write the failing integration test**

```python
def test_generate_report_writes_html_md_json_and_assets(tmp_path):
    mod = load_daily_hot_report_module()
    output_dir = tmp_path / "每日热点"
    result = mod.generate_report(output_dir=output_dir, use_live_network=False)

    assert result.html_path.is_file()
    assert result.md_path.is_file()
    assert result.json_path.is_file()
    assert "机器人运动控制" in result.html_path.read_text(encoding="utf-8")
```

- [ ] **Step 2: Run the test to see missing fetch/render code**

Run: `pytest tests/test_daily_hot_report.py -q`

Expected: fail until the generator, template, and file-writing flow exist.

- [ ] **Step 3: Implement the generator**

```python
def main() -> int:
    output_dir = parse_args().output_dir
    result = generate_report(output_dir=output_dir)
    print(result.html_path)
    print(result.md_path)
    print(result.json_path)
    return 0
```

### Task 3: Add the Windows scheduler installer

**Files:**
- Create: `tools/install_daily_hot_report_task.ps1`

- [ ] **Step 1: Write the failing test**

```powershell
$script = Join-Path $PSScriptRoot "tools/daily_hot_report.py"
python $script --help
```

Expected: the install script should create a task that invokes the generator at 07:30.

- [ ] **Step 2: Implement task registration**

Use `Register-ScheduledTask` or `schtasks` with:

- daily trigger at `07:30`
- action: current Python executable + `tools/daily_hot_report.py`
- output directory: `D:\桌面\每日热点`

### Task 4: Verify end-to-end generation

**Files:**
- Test: `tests/test_daily_hot_report.py`

- [ ] **Step 1: Run the full test suite**

Run: `pytest tests/test_daily_hot_report.py -q`

Expected: pass

- [ ] **Step 2: Run the generator once on live data**

Run: `python tools/daily_hot_report.py --output-dir "D:\桌面\每日热点"`

Expected: report files are created and the HTML file contains the current date

- [ ] **Step 3: Inspect git diff summary**

Run: `git diff --stat`

Expected: only the daily hot report files, the test file, the installer, and dependency updates appear
