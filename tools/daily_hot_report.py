"""Generate a daily robotics motion-control and AI hot-news report.

The script is intentionally self-contained and standard-library only.  It can
run from a Windows scheduled task without depending on the shell's current
working directory or optional Python packages.
"""

import argparse
import hashlib
import json
import math
import os
import re
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, replace
from datetime import datetime, timedelta, timezone
from email.utils import parsedate_to_datetime
from html import escape, unescape
from html.parser import HTMLParser
from pathlib import Path
from typing import Any

try:
    from zoneinfo import ZoneInfo
except ImportError:  # pragma: no cover - Python 3.8 fallback if ever needed.
    ZoneInfo = None


REPORT_BASENAME = "机器人运动控制与AI热点日报"
DEFAULT_OUTPUT_DIR = Path(r"D:\桌面\每日热点")
DEFAULT_LOOKBACK_HOURS = 30
DEFAULT_MAX_ITEMS = 42
FETCH_TIMEOUT_SECONDS = 12
DEFAULT_ANALYSIS_MODEL = "gpt-4.1-mini"


ROBOTICS_KEYWORDS = {
    "robot": 3.0,
    "robotics": 3.0,
    "humanoid": 4.5,
    "legged": 4.2,
    "locomotion": 4.4,
    "biped": 4.0,
    "quadruped": 3.8,
    "whole-body": 4.6,
    "whole body": 4.6,
    "wbc": 4.2,
    "mpc": 4.2,
    "model predictive": 4.2,
    "control": 2.8,
    "controller": 2.8,
    "sim-to-real": 4.0,
    "sim2real": 4.0,
    "reinforcement learning": 3.2,
    "rl control": 3.7,
    "dexterous": 4.0,
    "manipulation": 3.4,
    "embodied": 3.9,
    "ros": 3.0,
    "isaac": 3.2,
    "mujoco": 3.4,
    "genesis": 3.2,
    "lerobot": 3.8,
    "unitree": 4.0,
    "boston dynamics": 4.0,
    "figure ai": 4.0,
    "agility robotics": 4.0,
    "1x": 3.0,
    "physical intelligence": 4.0,
    "skild": 3.6,
    "trajectory": 2.4,
    "contact": 2.4,
    "grasp": 2.4,
}

AI_KEYWORDS = {
    "ai": 2.0,
    "agent": 2.4,
    "llm": 2.3,
    "model": 1.5,
    "multimodal": 2.5,
    "open source": 2.2,
    "benchmark": 1.8,
    "inference": 1.8,
    "developer": 1.6,
    "tool": 1.8,
    "cursor": 2.4,
    "replit": 2.4,
    "hugging face": 2.3,
    "product hunt": 1.7,
    "github": 1.6,
}

EMBODIED_KEYWORDS = {
    "embodied",
    "humanoid",
    "dexterous",
    "manipulation",
    "physical intelligence",
    "world model",
    "robot foundation",
    "vision-language-action",
    "vla",
}

TOOL_KEYWORDS = {
    "tool",
    "developer",
    "sdk",
    "github",
    "open source",
    "agent",
    "workflow",
    "ide",
    "cursor",
    "replit",
    "vercel",
    "langchain",
    "llamaindex",
}

SOURCE_WEIGHTS = {
    "arXiv Motion Control Search": 9.0,
    "arXiv cs.RO": 8.5,
    "arXiv cs.AI": 6.2,
    "arXiv cs.LG": 5.8,
    "arXiv eess.SY": 6.5,
    "IEEE Spectrum Robotics": 7.8,
    "The Robot Report": 7.5,
    "Robohub": 6.6,
    "ROS Discourse": 6.3,
    "NVIDIA Robotics Blog": 7.2,
    "Google DeepMind Blog": 7.2,
    "OpenAI News": 7.0,
    "Anthropic News": 6.6,
    "Meta AI Blog": 6.4,
    "Microsoft AI Blog": 6.0,
    "Hugging Face Blog": 5.9,
    "GitHub Search": 5.8,
    "Hacker News": 4.8,
    "Product Hunt": 4.4,
}


@dataclass
class NewsItem:
    title: str
    source: str
    url: str
    published_at: datetime | None
    summary: str
    category: str
    score: float = 0.0
    why_it_matters: str = ""
    tags: tuple[str, ...] = ()
    image_url: str | None = None
    local_image: str | None = None
    content_type: str = ""


@dataclass
class SourceFailure:
    source: str
    url: str
    reason: str


@dataclass
class ReportResult:
    html_path: Path
    md_path: Path
    json_path: Path
    status: str
    item_count: int
    failures: list[SourceFailure]


@dataclass
class AnalysisResult:
    items: list[NewsItem]
    digest: tuple[str, ...]
    model: str
    used_ai: bool
    raw_text: str = ""


@dataclass(frozen=True)
class SourceConfig:
    name: str
    url: str
    kind: str
    category: str
    query: str = ""


DEFAULT_SOURCES: tuple[SourceConfig, ...] = (
    SourceConfig(
        "arXiv Motion Control Search",
        "https://export.arxiv.org/api/query",
        "arxiv_search",
        "robotics",
        query=(
            "humanoid locomotion OR legged locomotion OR whole-body control OR "
            "model predictive control robot OR sim-to-real robot OR dexterous manipulation"
        ),
    ),
    SourceConfig("arXiv cs.RO", "https://export.arxiv.org/rss/cs.RO", "feed", "robotics"),
    SourceConfig("arXiv cs.AI", "https://export.arxiv.org/rss/cs.AI", "feed", "ai"),
    SourceConfig("arXiv cs.LG", "https://export.arxiv.org/rss/cs.LG", "feed", "ai"),
    SourceConfig("arXiv eess.SY", "https://export.arxiv.org/rss/eess.SY", "feed", "robotics"),
    SourceConfig("IEEE Spectrum Robotics", "https://spectrum.ieee.org/rss/robotics/fulltext", "feed", "robotics"),
    SourceConfig("The Robot Report", "https://www.therobotreport.com/feed/", "feed", "robotics"),
    SourceConfig("Robohub", "https://robohub.org/feed/", "feed", "robotics"),
    SourceConfig("ROS Discourse", "https://discourse.ros.org/latest.rss", "feed", "robotics"),
    SourceConfig("NVIDIA Robotics Blog", "https://developer.nvidia.com/blog/category/robotics/feed/", "feed", "robotics"),
    SourceConfig("Google DeepMind Blog", "https://deepmind.google/discover/blog/rss.xml", "feed", "ai"),
    SourceConfig("OpenAI News", "https://openai.com/news/rss.xml", "feed", "ai"),
    SourceConfig("Anthropic News", "https://www.anthropic.com/news/rss.xml", "feed", "ai"),
    SourceConfig("Meta AI Blog", "https://ai.meta.com/blog/rss/", "feed", "ai"),
    SourceConfig("Microsoft AI Blog", "https://blogs.microsoft.com/ai/feed/", "feed", "ai"),
    SourceConfig("Hugging Face Blog", "https://huggingface.co/blog/feed.xml", "feed", "ai"),
    SourceConfig("Product Hunt", "https://www.producthunt.com/feed", "feed", "tool"),
    SourceConfig(
        "GitHub Search",
        "https://api.github.com/search/repositories",
        "github_search",
        "tool",
        query="robotics humanoid locomotion mpc OR embodied-ai OR lerobot",
    ),
    SourceConfig(
        "GitHub Search",
        "https://api.github.com/search/repositories",
        "github_search",
        "tool",
        query="ai agent developer tool llm multimodal",
    ),
    SourceConfig(
        "Hacker News",
        "https://hn.algolia.com/api/v1/search_by_date",
        "hn_algolia",
        "trend",
        query="robotics OR humanoid OR AI agent OR open source AI",
    ),
)


class TextExtractor(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.parts: list[str] = []

    def handle_data(self, data: str) -> None:
        text = data.strip()
        if text:
            self.parts.append(text)

    def text(self) -> str:
        return " ".join(self.parts)


class OpenGraphExtractor(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.meta: dict[str, str] = {}

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        if tag.lower() != "meta":
            return
        attrs_map = {k.lower(): (v or "") for k, v in attrs}
        key = attrs_map.get("property") or attrs_map.get("name")
        value = attrs_map.get("content")
        if key and value:
            self.meta[key.lower()] = value.strip()


def get_local_now() -> datetime:
    if ZoneInfo is not None:
        return datetime.now(ZoneInfo("Asia/Shanghai"))
    return datetime.now().astimezone()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a Chinese daily robotics motion-control and AI hot-news report."
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=r"Output directory. Default: D:\桌面\每日热点",
    )
    parser.add_argument(
        "--lookback-hours",
        type=int,
        default=DEFAULT_LOOKBACK_HOURS,
        help="Prefer items published within this many hours.",
    )
    parser.add_argument(
        "--max-items",
        type=int,
        default=DEFAULT_MAX_ITEMS,
        help="Maximum ranked items to keep in the archive.",
    )
    parser.add_argument(
        "--no-live-network",
        action="store_true",
        help="Skip network fetching and write a degraded offline report.",
    )
    parser.add_argument(
        "--no-ai-analysis",
        action="store_true",
        help="Skip OpenAI/Codex analysis and use rule-based ranking only.",
    )
    parser.add_argument(
        "--analysis-model",
        default=os.environ.get("OPENAI_DAILY_REPORT_MODEL", DEFAULT_ANALYSIS_MODEL),
        help=f"OpenAI model for candidate analysis. Default: {DEFAULT_ANALYSIS_MODEL}",
    )
    parser.add_argument(
        "--date",
        help="Override report date as YYYY-MM-DD. Mainly useful for tests/backfills.",
    )
    return parser.parse_args(argv)


def fetch_text(url: str, timeout: int = FETCH_TIMEOUT_SECONDS) -> str:
    request = urllib.request.Request(
        url,
        headers={
            "User-Agent": (
                "Mozilla/5.0 (Windows NT 10.0; Win64; x64) "
                "RobotDynamicsControlDailyReport/1.0"
            ),
            "Accept": "application/rss+xml, application/atom+xml, application/json, text/html;q=0.9, */*;q=0.8",
        },
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        data = response.read()
        charset = response.headers.get_content_charset() or "utf-8"
    return data.decode(charset, errors="replace")


def post_json(url: str, payload: dict[str, Any], api_key: str, timeout: int = 90) -> dict[str, Any]:
    data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=data,
        method="POST",
        headers={
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json",
            "User-Agent": "RobotDynamicsControlDailyReport/1.0",
        },
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        text = response.read().decode("utf-8", errors="replace")
    return json.loads(text)


def clean_text(raw: str | None, max_chars: int = 360) -> str:
    if not raw:
        return ""
    parser = TextExtractor()
    try:
        parser.feed(raw)
        text = parser.text() or raw
    except Exception:
        text = raw
    text = unescape(text)
    text = re.sub(r"\s+", " ", text).strip()
    if len(text) <= max_chars:
        return text
    return text[: max_chars - 1].rstrip() + "…"


def local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1].lower()


def child_text(element: ET.Element, *names: str) -> str:
    wanted = {name.lower() for name in names}
    for child in list(element):
        if local_name(child.tag) in wanted and child.text:
            return clean_text(child.text, max_chars=900)
    return ""


def find_entry_link(entry: ET.Element) -> str:
    for child in list(entry):
        if local_name(child.tag) != "link":
            continue
        href = child.attrib.get("href")
        rel = child.attrib.get("rel", "alternate")
        if href and rel in {"alternate", ""}:
            return href.strip()
        if child.text and child.text.strip().startswith("http"):
            return child.text.strip()
    return child_text(entry, "link")


def find_entry_image(entry: ET.Element) -> str | None:
    for child in entry.iter():
        name = local_name(child.tag)
        if name in {"thumbnail", "content"}:
            url = child.attrib.get("url")
            medium = child.attrib.get("medium", "")
            if url and (name == "thumbnail" or medium == "image"):
                return url
        if name == "enclosure":
            url = child.attrib.get("url")
            typ = child.attrib.get("type", "")
            if url and typ.startswith("image/"):
                return url
    return None


def parse_datetime(value: str | None) -> datetime | None:
    if not value:
        return None
    value = value.strip()
    try:
        parsed = parsedate_to_datetime(value)
        if parsed.tzinfo is None:
            parsed = parsed.replace(tzinfo=timezone.utc)
        return parsed
    except (TypeError, ValueError, IndexError, OverflowError):
        pass

    for candidate in (value, value.replace("Z", "+00:00")):
        try:
            parsed = datetime.fromisoformat(candidate)
            if parsed.tzinfo is None:
                parsed = parsed.replace(tzinfo=timezone.utc)
            return parsed
        except ValueError:
            continue
    return None


def parse_feed(xml_text: str, source: SourceConfig) -> list[NewsItem]:
    root = ET.fromstring(xml_text)
    entries = [node for node in root.iter() if local_name(node.tag) in {"item", "entry"}]
    items: list[NewsItem] = []
    for entry in entries:
        title = child_text(entry, "title")
        link = find_entry_link(entry)
        if not title or not link:
            continue
        summary = child_text(entry, "summary", "description", "content", "encoded")
        published_at = parse_datetime(child_text(entry, "published", "updated", "pubDate", "dc:date"))
        items.append(
            NewsItem(
                title=title,
                source=source.name,
                url=link,
                published_at=published_at,
                summary=summary,
                category=source.category,
                image_url=find_entry_image(entry),
            )
        )
    return items


def fetch_github_search(source: SourceConfig, now: datetime, lookback_hours: int) -> list[NewsItem]:
    start_date = (now - timedelta(hours=lookback_hours)).date().isoformat()
    query = f"{source.query} created:>={start_date}"
    params = {
        "q": query,
        "sort": "stars",
        "order": "desc",
        "per_page": "12",
    }
    url = source.url + "?" + urllib.parse.urlencode(params)
    payload = json.loads(fetch_text(url))
    items: list[NewsItem] = []
    for repo in payload.get("items", []):
        title = repo.get("full_name") or repo.get("name")
        html_url = repo.get("html_url")
        if not title or not html_url:
            continue
        stars = repo.get("stargazers_count", 0)
        language = repo.get("language") or "unknown"
        topics = repo.get("topics") or []
        summary = repo.get("description") or ""
        summary = f"{summary} Stars: {stars}. Language: {language}. Topics: {', '.join(topics[:8])}."
        items.append(
            NewsItem(
                title=f"GitHub: {title}",
                source=source.name,
                url=html_url,
                published_at=parse_datetime(repo.get("created_at") or repo.get("updated_at")),
                summary=summary,
                category=classify_category(f"{title} {summary}", source.category),
            )
        )
    return items


def fetch_hn_algolia(source: SourceConfig, now: datetime, lookback_hours: int) -> list[NewsItem]:
    min_created = int((now - timedelta(hours=lookback_hours)).timestamp())
    params = {
        "query": source.query,
        "tags": "story",
        "hitsPerPage": "16",
        "numericFilters": f"created_at_i>{min_created}",
    }
    url = source.url + "?" + urllib.parse.urlencode(params)
    payload = json.loads(fetch_text(url))
    items: list[NewsItem] = []
    for hit in payload.get("hits", []):
        title = hit.get("title") or hit.get("story_title")
        link = hit.get("url") or f"https://news.ycombinator.com/item?id={hit.get('objectID')}"
        if not title or not link:
            continue
        points = hit.get("points") or 0
        comments = hit.get("num_comments") or 0
        summary = f"Hacker News discussion signal. Points: {points}. Comments: {comments}."
        items.append(
            NewsItem(
                title=title,
                source=source.name,
                url=link,
                published_at=parse_datetime(hit.get("created_at")),
                summary=summary,
                category=classify_category(title, source.category),
            )
        )
    return items


def build_arxiv_search_url(query: str, max_results: int = 28) -> str:
    """Build an arXiv API URL for motion-control-oriented paper search."""
    terms = [term.strip() for term in re.split(r"\s+OR\s+", query, flags=re.IGNORECASE) if term.strip()]
    if not terms:
        terms = [query.strip()]
    search_query = " OR ".join(f'all:"{term}"' if " " in term else f"all:{term}" for term in terms)
    params = {
        "search_query": search_query,
        "start": "0",
        "max_results": str(max_results),
        "sortBy": "submittedDate",
        "sortOrder": "descending",
    }
    return "https://export.arxiv.org/api/query?" + urllib.parse.urlencode(params, quote_via=urllib.parse.quote)


def fetch_arxiv_search(source: SourceConfig, now: datetime, lookback_hours: int) -> list[NewsItem]:
    url = build_arxiv_search_url(source.query)
    api_source = SourceConfig(source.name, url, "feed", source.category)
    return parse_feed(fetch_text(url), api_source)


def fetch_source(source: SourceConfig, now: datetime, lookback_hours: int) -> list[NewsItem]:
    if source.kind == "feed":
        return parse_feed(fetch_text(source.url), source)
    if source.kind == "arxiv_search":
        return fetch_arxiv_search(source, now, lookback_hours)
    if source.kind == "github_search":
        return fetch_github_search(source, now, lookback_hours)
    if source.kind == "hn_algolia":
        return fetch_hn_algolia(source, now, lookback_hours)
    raise ValueError(f"Unsupported source kind: {source.kind}")


def fetch_all_sources(
    sources: tuple[SourceConfig, ...],
    now: datetime,
    lookback_hours: int,
) -> tuple[list[NewsItem], list[SourceFailure]]:
    items: list[NewsItem] = []
    failures: list[SourceFailure] = []
    for source in sources:
        try:
            source_items = fetch_source(source, now, lookback_hours)
            items.extend(filter_recent_items(source_items, now, lookback_hours))
            time.sleep(0.15)
        except (urllib.error.URLError, TimeoutError, ET.ParseError, json.JSONDecodeError, OSError, ValueError) as exc:
            failures.append(SourceFailure(source=source.name, url=source.url, reason=str(exc)))
    return items, failures


def filter_recent_items(items: list[NewsItem], now: datetime, lookback_hours: int) -> list[NewsItem]:
    filtered: list[NewsItem] = []
    for item in items:
        source_window_hours = 24 * 7 if item.source == "arXiv Motion Control Search" else max(lookback_hours * 2, 48)
        earliest = now - timedelta(hours=source_window_hours)
        if item.published_at is None:
            filtered.append(item)
            continue
        published = item.published_at.astimezone(now.tzinfo or timezone.utc)
        if published >= earliest:
            filtered.append(item)
    return filtered


def canonicalize_url(url: str) -> str:
    parsed = urllib.parse.urlsplit(url.strip())
    query_pairs = [
        (k, v)
        for k, v in urllib.parse.parse_qsl(parsed.query, keep_blank_values=True)
        if not k.lower().startswith("utm_") and k.lower() not in {"fbclid", "gclid", "mc_cid", "mc_eid"}
    ]
    normalized = parsed._replace(
        scheme=parsed.scheme.lower(),
        netloc=parsed.netloc.lower(),
        query=urllib.parse.urlencode(query_pairs, doseq=True),
        fragment="",
    )
    text = urllib.parse.urlunsplit(normalized).rstrip("/")
    return text


def normalize_title(title: str) -> str:
    text = re.sub(r"[^a-z0-9\u4e00-\u9fff]+", " ", title.lower())
    return re.sub(r"\s+", " ", text).strip()


def deduplicate_items(items: list[NewsItem]) -> list[NewsItem]:
    best: dict[str, NewsItem] = {}
    order: list[str] = []
    for item in items:
        key = canonicalize_url(item.url) if item.url else "title:" + normalize_title(item.title)
        if not key or key == "title:":
            digest = hashlib.sha1(item.title.encode("utf-8")).hexdigest()
            key = f"untitled:{digest}"
        current = best.get(key)
        if current is None:
            best[key] = item
            order.append(key)
            continue
        if item.score > current.score or (
            math.isclose(item.score, current.score) and len(item.summary) > len(current.summary)
        ):
            best[key] = item
    return [best[key] for key in order]


def classify_category(text: str, fallback: str = "ai") -> str:
    lowered = text.lower()
    robotics_hits = sum(1 for key in ROBOTICS_KEYWORDS if key in lowered)
    ai_hits = sum(1 for key in AI_KEYWORDS if key in lowered)
    if robotics_hits >= 1 and robotics_hits >= ai_hits:
        return "robotics"
    if any(key in lowered for key in EMBODIED_KEYWORDS):
        return "embodied"
    if any(key in lowered for key in TOOL_KEYWORDS):
        return "tool"
    return fallback or "ai"


def infer_content_type(item: NewsItem) -> str:
    text = f"{item.title} {item.summary}".lower()
    if item.category == "robotics":
        if any(key in text for key in EMBODIED_KEYWORDS):
            return "embodied"
        return "robotics"
    if item.category == "tool" or any(key in text for key in TOOL_KEYWORDS):
        return "tool"
    if item.category == "trend":
        return "trend"
    if any(key in text for key in EMBODIED_KEYWORDS):
        return "embodied"
    return "ai"


def infer_tags(item: NewsItem) -> tuple[str, ...]:
    text = f"{item.title} {item.summary}".lower()
    tags: list[str] = []
    tag_rules = [
        ("humanoid", "人形机器人"),
        ("legged", "足式机器人"),
        ("locomotion", "运动控制"),
        ("whole-body", "WBC"),
        ("whole body", "WBC"),
        ("mpc", "MPC"),
        ("model predictive", "MPC"),
        ("reinforcement learning", "RL Control"),
        ("sim-to-real", "Sim-to-Real"),
        ("dexterous", "灵巧操作"),
        ("manipulation", "操作控制"),
        ("embodied", "具身智能"),
        ("ros", "ROS"),
        ("isaac", "Isaac"),
        ("mujoco", "MuJoCo"),
        ("genesis", "Genesis"),
        ("lerobot", "LeRobot"),
        ("github", "开源项目"),
        ("agent", "Agent"),
        ("multimodal", "多模态"),
        ("developer", "开发者工具"),
        ("open source", "开源"),
    ]
    for keyword, tag in tag_rules:
        if keyword in text and tag not in tags:
            tags.append(tag)
    if not tags:
        tags.append(
            {
                "robotics": "机器人",
                "embodied": "具身智能",
                "tool": "AI 工具",
                "trend": "趋势信号",
            }.get(item.category, "AI")
        )
    return tuple(tags[:6])


def freshness_score(item: NewsItem, now: datetime) -> float:
    if item.published_at is None:
        return 2.0
    published = item.published_at.astimezone(now.tzinfo or timezone.utc)
    hours = max((now - published).total_seconds() / 3600.0, 0.0)
    if hours <= 12:
        return 8.0
    if hours <= 24:
        return 6.0
    if hours <= 48:
        return 3.5
    return 1.0


def keyword_score(text: str, keywords: dict[str, float]) -> float:
    lowered = text.lower()
    return sum(weight for keyword, weight in keywords.items() if keyword in lowered)


def explain_importance(item: NewsItem) -> str:
    tags = set(item.tags)
    if item.content_type == "robotics":
        if {"MPC", "WBC", "运动控制"} & tags:
            return "它直接关联机器人运动控制主线，可用于跟踪 MPC、WBC、步态和接触控制的新方法。"
        if {"开源项目", "ROS", "MuJoCo", "Isaac", "Genesis", "LeRobot"} & tags:
            return "它可能提供可复现实验、仿真或工程工具，对后续机器人控制学习链条有实用价值。"
        return "它属于机器人核心动态，适合作为今日优先跟踪的控制与系统信号。"
    if item.content_type == "embodied":
        return "它连接机器人本体、感知和大模型能力，是具身智能落地趋势的重要信号。"
    if item.content_type == "tool":
        return "它可能影响 AI 开发工作流、开源生态或自动化工具选择，适合进入工具观察清单。"
    if item.content_type == "trend":
        return "它来自社区讨论或趋势入口，可作为早期热度信号，但仍需要结合原文进一步核验。"
    return "它反映通用 AI 模型、产品或生态的新变化，可作为第二优先级跟踪内容。"


def score_item(item: NewsItem, now: datetime) -> NewsItem:
    text = f"{item.title} {item.summary}"
    category = classify_category(text, item.category)
    base = {
        "robotics": 34.0,
        "embodied": 30.0,
        "tool": 18.0,
        "trend": 14.0,
        "ai": 16.0,
    }.get(category, 12.0)
    source = SOURCE_WEIGHTS.get(item.source, 4.0)
    robot_score = keyword_score(text, ROBOTICS_KEYWORDS)
    ai_score = keyword_score(text, AI_KEYWORDS)
    fresh = freshness_score(item, now)
    image_bonus = 1.2 if item.image_url else 0.0
    category_bonus = 4.0 if category == "robotics" and robot_score > 0 else 0.0
    tags = infer_tags(replace(item, category=category))
    content_type = infer_content_type(replace(item, category=category, tags=tags))
    total = base + source + robot_score + ai_score * 0.45 + fresh + image_bonus + category_bonus
    scored = replace(
        item,
        category=category,
        score=round(total, 2),
        tags=tags,
        content_type=content_type,
    )
    return replace(scored, why_it_matters=explain_importance(scored))


def rank_items(items: list[NewsItem], now: datetime | None = None) -> list[NewsItem]:
    now = now or get_local_now()
    scored = [score_item(item, now) for item in items]
    deduped = deduplicate_items(scored)
    return sorted(
        deduped,
        key=lambda item: (
            item.score,
            1 if item.category == "robotics" else 0,
            item.published_at.timestamp() if item.published_at else 0,
        ),
        reverse=True,
    )


def compact_candidate_for_analysis(item: NewsItem, index: int) -> dict[str, Any]:
    return {
        "id": index,
        "title": item.title,
        "source": item.source,
        "url": item.url,
        "published_at": item.published_at.isoformat() if item.published_at else None,
        "summary": clean_text(item.summary, 520),
        "category_hint": item.category,
        "rule_score": item.score,
        "tags_hint": list(item.tags),
    }


def build_analysis_prompt(candidates: list[NewsItem], now: datetime, max_items: int) -> str:
    payload = [compact_candidate_for_analysis(item, index) for index, item in enumerate(candidates[:80])]
    return (
        "你是 Codex，正在为用户筛选《机器人运动控制 & AI 今日热点报告》。\n"
        "请不要简单复述候选列表，而要做分析筛选：去重、剔除低价值噪声、按用户学习方向排序，"
        "机器人运动控制必须明显优先于通用 AI 消息。\n\n"
        "用户优先级：\n"
        "1. 机器人运动控制：humanoid/legged locomotion、whole-body control、MPC、RL control、"
        "sim-to-real、dexterous manipulation、embodied AI、ROS/Isaac/MuJoCo/Genesis/LeRobot、"
        "机器人控制论文、开源项目、机器人公司动态。\n"
        "2. AI 最新消息：模型、产品、Agent、开发者工具、多模态、开源项目、GitHub/Product Hunt/Hugging Face 趋势。\n\n"
        f"当前时间：{now.isoformat()}。最多输出 {max_items} 条。\n"
        "请返回 JSON，字段必须符合 schema。每个 selected_items 条目都必须来自候选中的原始 url，"
        "可以重写中文摘要、为什么重要、标签、category/content_type 和 score。\n\n"
        "评分标准：robotics/embodied 相关可给 70-100；AI 工具或模型 45-80；弱相关趋势低于 50。"
        "摘要和重要性判断必须用中文，紧凑但有判断。\n\n"
        "候选 JSON：\n"
        + json.dumps(payload, ensure_ascii=False, indent=2)
    )


def analysis_json_schema() -> dict[str, Any]:
    item_schema = {
        "type": "object",
        "additionalProperties": False,
        "properties": {
            "url": {"type": "string"},
            "title": {"type": "string"},
            "source": {"type": "string"},
            "published_at": {"type": ["string", "null"]},
            "summary": {"type": "string"},
            "why_it_matters": {"type": "string"},
            "tags": {"type": "array", "items": {"type": "string"}},
            "score": {"type": "number"},
            "category": {"type": "string", "enum": ["robotics", "embodied", "ai", "tool", "trend"]},
            "content_type": {"type": "string", "enum": ["robotics", "embodied", "ai", "tool", "trend"]},
            "image_url": {"type": ["string", "null"]},
        },
        "required": [
            "url",
            "title",
            "source",
            "published_at",
            "summary",
            "why_it_matters",
            "tags",
            "score",
            "category",
            "content_type",
            "image_url",
        ],
    }
    return {
        "type": "object",
        "additionalProperties": False,
        "properties": {
            "digest": {"type": "array", "items": {"type": "string"}, "minItems": 2, "maxItems": 5},
            "selected_items": {"type": "array", "items": item_schema},
        },
        "required": ["digest", "selected_items"],
    }


def extract_response_text(response: dict[str, Any]) -> str:
    if isinstance(response.get("output_text"), str):
        return response["output_text"]
    texts: list[str] = []
    for output in response.get("output", []):
        for content in output.get("content", []):
            if content.get("type") in {"output_text", "text"} and isinstance(content.get("text"), str):
                texts.append(content["text"])
    if texts:
        return "\n".join(texts)
    raise ValueError("OpenAI response did not contain output text.")


def parse_ai_selected_items(payload: dict[str, Any], original_items: list[NewsItem], now: datetime) -> list[NewsItem]:
    by_url = {canonicalize_url(item.url): item for item in original_items}
    selected: list[NewsItem] = []
    for raw in payload.get("selected_items", []):
        url = str(raw.get("url", "")).strip()
        original = by_url.get(canonicalize_url(url))
        published_at = parse_datetime(raw.get("published_at")) if raw.get("published_at") else None
        if original and published_at is None:
            published_at = original.published_at
        item = NewsItem(
            title=clean_text(raw.get("title"), 220) or (original.title if original else url),
            source=clean_text(raw.get("source"), 120) or (original.source if original else "OpenAI Analysis"),
            url=url or (original.url if original else ""),
            published_at=published_at,
            summary=clean_text(raw.get("summary"), 520) or (original.summary if original else ""),
            category=str(raw.get("category") or (original.category if original else "ai")),
            score=float(raw.get("score", 0.0)),
            why_it_matters=clean_text(raw.get("why_it_matters"), 360),
            tags=tuple(str(tag).strip() for tag in raw.get("tags", []) if str(tag).strip())[:6],
            image_url=raw.get("image_url") or (original.image_url if original else None),
            content_type=str(raw.get("content_type") or ""),
        )
        if not item.url:
            continue
        if not item.content_type:
            item = replace(item, content_type=infer_content_type(item))
        if not item.tags:
            item = replace(item, tags=infer_tags(item))
        if not item.why_it_matters:
            item = replace(item, why_it_matters=explain_importance(item))
        selected.append(item)
    if not selected:
        raise ValueError("OpenAI analysis returned no selected_items.")
    return rank_items(selected, now=now)


def analyze_candidates_with_openai(
    candidates: list[NewsItem],
    now: datetime,
    max_items: int,
    model: str = DEFAULT_ANALYSIS_MODEL,
) -> AnalysisResult:
    api_key = os.environ.get("OPENAI_API_KEY", "").strip()
    if not api_key:
        raise RuntimeError("OPENAI_API_KEY is not set; cannot run OpenAI/Codex analysis.")
    prompt = build_analysis_prompt(candidates, now, max_items)
    payload = {
        "model": model,
        "input": [
            {
                "role": "system",
                "content": (
                    "你是严谨的机器人运动控制与 AI 情报分析助手。"
                    "你必须优先筛选对机器人运动控制学习和研究真正有价值的内容，"
                    "不要让泛泛的 AI 工具消息盖过机器人控制论文、工程工具和具身智能进展。"
                ),
            },
            {"role": "user", "content": prompt},
        ],
        "temperature": 0.2,
        "max_output_tokens": 8000,
        "text": {
            "format": {
                "type": "json_schema",
                "name": "daily_robotics_ai_report_analysis",
                "schema": analysis_json_schema(),
                "strict": True,
            }
        },
    }
    response = post_json("https://api.openai.com/v1/responses", payload, api_key=api_key)
    raw_text = extract_response_text(response)
    analysis_payload = json.loads(raw_text)
    return AnalysisResult(
        items=parse_ai_selected_items(analysis_payload, candidates, now)[:max_items],
        digest=tuple(str(line).strip() for line in analysis_payload.get("digest", []) if str(line).strip()),
        model=model,
        used_ai=True,
        raw_text=raw_text,
    )


def fetch_open_graph(url: str) -> dict[str, str]:
    html_text = fetch_text(url, timeout=8)
    parser = OpenGraphExtractor()
    parser.feed(html_text[:200_000])
    return parser.meta


def enrich_top_items_with_metadata(items: list[NewsItem], limit: int = 16) -> list[NewsItem]:
    enriched: list[NewsItem] = []
    for index, item in enumerate(items):
        if index >= limit or item.image_url:
            enriched.append(item)
            continue
        try:
            meta = fetch_open_graph(item.url)
        except (urllib.error.URLError, TimeoutError, OSError, ValueError):
            enriched.append(item)
            continue
        image = meta.get("og:image") or meta.get("twitter:image")
        description = meta.get("og:description") or meta.get("twitter:description")
        title = meta.get("og:title")
        enriched.append(
            replace(
                item,
                title=clean_text(title, 180) or item.title,
                summary=clean_text(description, 360) or item.summary,
                image_url=image or item.image_url,
            )
        )
        time.sleep(0.08)
    enriched.extend(items[limit:])
    return enriched


def offline_seed_items(now: datetime) -> list[NewsItem]:
    published = now - timedelta(hours=3)
    return [
        NewsItem(
            title="Offline seed: humanoid locomotion MPC and whole-body control watchlist",
            source="Daily Hot Report",
            url="https://example.com/offline-robotics-watchlist",
            published_at=published,
            summary=(
                "网络抓取关闭或失败时的机器人运动控制占位条目。重点提醒关注 humanoid locomotion、"
                "whole-body control、MPC、sim-to-real、MuJoCo/Isaac/Genesis 与开源复现。"
            ),
            category="robotics",
        ),
        NewsItem(
            title="Offline seed: embodied AI and dexterous manipulation watchlist",
            source="Daily Hot Report",
            url="https://example.com/offline-embodied-watchlist",
            published_at=published,
            summary="离线降级条目，用于保留具身智能、人形机器人、灵巧操作和 VLA 模型观察位。",
            category="embodied",
        ),
        NewsItem(
            title="Offline seed: AI developer tools and open-source agents watchlist",
            source="Daily Hot Report",
            url="https://example.com/offline-ai-tools-watchlist",
            published_at=published,
            summary="离线降级条目，用于保留 AI Agent、开发者工具、GitHub Trending 与 Hugging Face 观察位。",
            category="tool",
        ),
    ]


def format_datetime(value: datetime | None) -> str:
    if value is None:
        return "未标注"
    local = value.astimezone(get_local_now().tzinfo)
    return local.strftime("%Y-%m-%d %H:%M")


def item_summary_cn(item: NewsItem) -> str:
    tags = "、".join(item.tags)
    raw = clean_text(item.summary, 240)
    if raw:
        return f"该条目聚焦 {tags}。原文摘要要点：{raw}"
    return f"该条目聚焦 {tags}，建议打开原文核验技术细节与发布时间。"


def split_sections(items: list[NewsItem]) -> dict[str, list[NewsItem]]:
    sections = {
        "robotics": [item for item in items if item.content_type == "robotics"],
        "embodied": [item for item in items if item.content_type == "embodied"],
        "ai": [item for item in items if item.content_type == "ai"],
        "tool": [item for item in items if item.content_type == "tool"],
        "trend": [item for item in items if item.content_type == "trend"],
    }
    if not sections["embodied"]:
        sections["embodied"] = [item for item in items if item.category in {"robotics", "embodied"}][3:8]
    if not sections["trend"]:
        sections["trend"] = items[8:14]
    return sections


def build_digest(items: list[NewsItem], failures: list[SourceFailure], status: str) -> list[str]:
    robotics_count = sum(1 for item in items if item.content_type in {"robotics", "embodied"})
    tool_count = sum(1 for item in items if item.content_type == "tool")
    top = items[:3]
    lines = [
        f"共筛选 {len(items)} 条候选重点，其中机器人/具身智能相关 {robotics_count} 条，AI 工具相关 {tool_count} 条。",
        "机器人运动控制内容已按更高权重排序，优先覆盖 WBC、MPC、足式/人形运动、Sim-to-Real 与开源工具。",
    ]
    if top:
        lines.append("最高优先级信号：" + "；".join(item.title for item in top) + "。")
    if status == "degraded":
        lines.append(f"本次为降级报告：{len(failures)} 个来源未能完整抓取，正文保留已获取内容和失败说明。")
    return lines


def fallback_analysis_result(items: list[NewsItem], now: datetime) -> AnalysisResult:
    ranked = rank_items(items, now=now)
    return AnalysisResult(
        items=ranked,
        digest=tuple(build_digest(ranked, [], "ready")),
        model="rule-fallback",
        used_ai=False,
    )


def render_item_card(item: NewsItem) -> str:
    tags = "".join(f"<span>{escape(tag)}</span>" for tag in item.tags)
    image = ""
    if item.image_url:
        image = f'<img class="thumb" src="{escape(item.image_url)}" alt="">'
    return f"""
        <article class="news-card">
          {image}
          <div class="card-body">
            <div class="meta">
              <span>{escape(item.source)}</span>
              <span>{escape(format_datetime(item.published_at))}</span>
              <strong>{item.score:.1f}</strong>
            </div>
            <h3><a href="{escape(item.url)}" target="_blank" rel="noopener noreferrer">{escape(item.title)}</a></h3>
            <p>{escape(item_summary_cn(item))}</p>
            <p class="why"><b>为什么重要：</b>{escape(item.why_it_matters)}</p>
            <div class="tags">{tags}</div>
          </div>
        </article>
    """


def render_section(title: str, subtitle: str, items: list[NewsItem], limit: int) -> str:
    cards = "\n".join(render_item_card(item) for item in items[:limit])
    if not cards:
        cards = '<p class="empty">本节暂无高置信候选，已在原始来源列表中保留可继续核验的入口。</p>'
    return f"""
      <section class="report-section">
        <div class="section-head">
          <h2>{escape(title)}</h2>
          <p>{escape(subtitle)}</p>
        </div>
        <div class="grid">{cards}</div>
      </section>
    """


def render_html_report(
    items: list[NewsItem],
    failures: list[SourceFailure],
    report_date: str,
    generated_at: datetime,
    status: str,
    analysis: AnalysisResult,
) -> str:
    sections = split_sections(items)
    digest = list(analysis.digest) or build_digest(items, failures, status)
    digest_html = "".join(f"<li>{escape(line)}</li>" for line in digest)
    analysis_mode = "OpenAI/Codex 分析筛选" if analysis.used_ai else "规则排序降级"
    source_rows = "\n".join(
        f'<tr><td>{idx}</td><td>{escape(item.source)}</td><td>{escape(format_datetime(item.published_at))}</td>'
        f'<td><a href="{escape(item.url)}" target="_blank" rel="noopener noreferrer">{escape(item.title)}</a></td>'
        f"<td>{item.score:.1f}</td></tr>"
        for idx, item in enumerate(items, start=1)
    )
    failure_html = ""
    if failures:
        rows = "".join(
            f"<li><b>{escape(f.source)}</b>: {escape(f.reason[:220])} "
            f'<a href="{escape(f.url)}" target="_blank" rel="noopener noreferrer">source</a></li>'
            for f in failures
        )
        failure_html = f"""
          <section class="report-section warning">
            <div class="section-head">
              <h2>降级说明</h2>
              <p>单个来源失败不会阻塞日报生成；下面列出本次未完整抓取的来源。</p>
            </div>
            <ul class="failure-list">{rows}</ul>
          </section>
        """

    return f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{escape(report_date)} {REPORT_BASENAME}</title>
  <style>
    :root {{
      color-scheme: dark;
      --bg: #08090b;
      --panel: #11151b;
      --panel-2: #151b22;
      --text: #eef4f8;
      --muted: #94a3ad;
      --line: #26323b;
      --cyan: #38d4ff;
      --lime: #9cff6e;
      --amber: #ffbf47;
      --rose: #ff5d85;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      background:
        linear-gradient(180deg, rgba(56, 212, 255, .10), transparent 340px),
        repeating-linear-gradient(90deg, rgba(255,255,255,.035) 0, rgba(255,255,255,.035) 1px, transparent 1px, transparent 96px),
        var(--bg);
      color: var(--text);
      font-family: "Segoe UI", "Microsoft YaHei", Arial, sans-serif;
      line-height: 1.6;
    }}
    a {{ color: inherit; text-decoration: none; }}
    a:hover {{ color: var(--cyan); }}
    .shell {{ width: min(1180px, calc(100vw - 32px)); margin: 0 auto; }}
    header {{
      min-height: 54vh;
      display: grid;
      align-items: center;
      padding: 48px 0 32px;
      border-bottom: 1px solid var(--line);
    }}
    .kicker {{
      color: var(--lime);
      font-size: 13px;
      letter-spacing: 0;
      text-transform: uppercase;
      font-weight: 700;
    }}
    h1 {{
      margin: 12px 0 16px;
      font-size: clamp(34px, 6vw, 72px);
      line-height: 1.05;
      letter-spacing: 0;
      max-width: 980px;
    }}
    .hero-text {{ max-width: 820px; color: var(--muted); font-size: 18px; }}
    .metrics {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
      gap: 12px;
      margin-top: 28px;
    }}
    .metric {{
      background: rgba(17, 21, 27, .88);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 16px;
    }}
    .metric strong {{ display: block; font-size: 26px; color: var(--cyan); }}
    .metric span {{ color: var(--muted); font-size: 13px; }}
    main {{ padding: 32px 0 56px; }}
    .digest {{
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 20px 22px;
      margin-bottom: 26px;
    }}
    .digest h2, .section-head h2 {{ margin: 0; font-size: 24px; letter-spacing: 0; }}
    .digest ul {{ margin: 12px 0 0; padding-left: 20px; color: var(--muted); }}
    .analysis-note {{ margin: 8px 0 0; color: var(--cyan); font-size: 14px; }}
    .report-section {{ margin-top: 36px; }}
    .section-head {{
      display: flex;
      justify-content: space-between;
      gap: 18px;
      align-items: end;
      border-bottom: 1px solid var(--line);
      padding-bottom: 12px;
      margin-bottom: 16px;
    }}
    .section-head p {{ margin: 0; color: var(--muted); max-width: 540px; }}
    .grid {{
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 14px;
    }}
    .news-card {{
      display: grid;
      grid-template-columns: 132px minmax(0, 1fr);
      gap: 0;
      min-height: 210px;
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      overflow: hidden;
    }}
    .news-card:not(:has(img)) {{ grid-template-columns: 1fr; }}
    .thumb {{ width: 132px; height: 100%; min-height: 210px; object-fit: cover; background: var(--panel-2); }}
    .card-body {{ padding: 16px; min-width: 0; }}
    .meta {{
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      align-items: center;
      color: var(--muted);
      font-size: 12px;
    }}
    .meta strong {{ color: var(--amber); margin-left: auto; }}
    h3 {{ margin: 10px 0 8px; font-size: 18px; line-height: 1.35; letter-spacing: 0; }}
    .news-card p {{ margin: 8px 0; color: var(--muted); font-size: 14px; }}
    .why {{ color: #cbd6dd !important; }}
    .tags {{ display: flex; flex-wrap: wrap; gap: 6px; margin-top: 12px; }}
    .tags span {{
      border: 1px solid rgba(56, 212, 255, .32);
      color: var(--cyan);
      border-radius: 999px;
      padding: 2px 8px;
      font-size: 12px;
      white-space: nowrap;
    }}
    .source-table {{
      width: 100%;
      border-collapse: collapse;
      overflow: hidden;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: var(--panel);
    }}
    .source-table th, .source-table td {{
      border-bottom: 1px solid var(--line);
      padding: 10px;
      text-align: left;
      vertical-align: top;
      font-size: 13px;
    }}
    .source-table th {{ color: var(--lime); background: var(--panel-2); }}
    .failure-list {{ color: var(--muted); }}
    .warning {{ border-left: 3px solid var(--amber); padding-left: 14px; }}
    .empty {{ color: var(--muted); }}
    footer {{ color: var(--muted); border-top: 1px solid var(--line); padding: 22px 0 36px; font-size: 13px; }}
    @media (max-width: 820px) {{
      header {{ min-height: auto; }}
      .metrics, .grid {{ grid-template-columns: 1fr; }}
      .section-head {{ display: block; }}
      .news-card {{ grid-template-columns: 1fr; }}
      .thumb {{ width: 100%; height: 180px; min-height: 180px; }}
      .source-table {{ display: block; overflow-x: auto; }}
    }}
  </style>
</head>
<body>
  <header>
    <div class="shell">
      <div class="kicker">Daily Robotics Control & AI Radar</div>
      <h1>{escape(REPORT_BASENAME)}</h1>
      <p class="hero-text">面向机器人运动控制学习路线的每日情报流：优先筛选 humanoid / legged locomotion、WBC、MPC、RL control、Sim-to-Real、具身智能与机器人开源生态，再补充 AI 模型、工具和社区趋势。</p>
      <div class="metrics">
        <div class="metric"><strong>{len(items)}</strong><span>入选条目</span></div>
        <div class="metric"><strong>{sum(1 for item in items if item.content_type in {"robotics", "embodied"})}</strong><span>机器人/具身重点</span></div>
        <div class="metric"><strong>{sum(1 for item in items if item.content_type == "tool")}</strong><span>AI 工具信号</span></div>
        <div class="metric"><strong>{escape(status)}</strong><span>抓取状态</span></div>
        <div class="metric"><strong>{escape(analysis_mode)}</strong><span>分析模式</span></div>
      </div>
    </div>
  </header>
  <main class="shell">
    <section class="digest">
      <h2>今日摘要</h2>
      <p class="analysis-note">分析层：{escape(analysis_mode)}；模型：{escape(analysis.model)}。</p>
      <ul>{digest_html}</ul>
    </section>
    {render_section("机器人运动控制重点", "第一优先级：控制、步态、接触、仿真到真实和可复现项目。", sections["robotics"], 10)}
    {render_section("具身智能 / 人形机器人动态", "关注机器人本体、大模型策略、操作能力和产业落地。", sections["embodied"], 8)}
    {render_section("AI 最新消息", "第二优先级：模型、产品、多模态能力和研究生态。", sections["ai"], 8)}
    {render_section("热门 AI 工具", "开发者工具、开源项目、Agent 框架和社区项目。", sections["tool"], 8)}
    {render_section("趋势信号", "来自社区、榜单或讨论区的早期热度，适合快速扫雷。", sections["trend"], 8)}
    {failure_html}
    <section class="report-section">
      <div class="section-head">
        <h2>原始来源链接</h2>
        <p>按综合评分排序，保留标题、来源、发布时间和原文入口，便于二次核验。</p>
      </div>
      <table class="source-table">
        <thead><tr><th>#</th><th>来源</th><th>时间</th><th>标题</th><th>评分</th></tr></thead>
        <tbody>{source_rows}</tbody>
      </table>
    </section>
  </main>
  <footer class="shell">
    生成时间：{escape(generated_at.strftime("%Y-%m-%d %H:%M:%S %Z"))}。建议定时任务每天 07:30 启动，确保 12:00 前完成。
  </footer>
</body>
</html>
"""


def render_markdown_report(
    items: list[NewsItem],
    failures: list[SourceFailure],
    report_date: str,
    generated_at: datetime,
    status: str,
    analysis: AnalysisResult,
) -> str:
    sections = split_sections(items)
    lines: list[str] = [
        f"# {report_date} {REPORT_BASENAME}",
        "",
        f"- 生成时间: `{generated_at.isoformat()}`",
        f"- 抓取状态: `{status}`",
        f"- 分析模式: `{'OpenAI/Codex' if analysis.used_ai else 'rule-fallback'}`",
        f"- 分析模型: `{analysis.model}`",
        f"- 入选条目: `{len(items)}`",
        "",
        "## 今日摘要",
        "",
    ]
    for line in list(analysis.digest) or build_digest(items, failures, status):
        lines.append(f"- {line}")
    lines.append("")

    section_defs = [
        ("机器人运动控制重点", sections["robotics"], 10),
        ("具身智能 / 人形机器人动态", sections["embodied"], 8),
        ("AI 最新消息", sections["ai"], 8),
        ("热门 AI 工具", sections["tool"], 8),
        ("趋势信号", sections["trend"], 8),
    ]
    for title, section_items, limit in section_defs:
        lines.extend([f"## {title}", ""])
        if not section_items:
            lines.extend(["暂无高置信候选。", ""])
            continue
        for idx, item in enumerate(section_items[:limit], start=1):
            lines.append(f"### {idx}. [{item.title}]({item.url})")
            lines.append(f"- 来源: {item.source}")
            lines.append(f"- 发布时间: {format_datetime(item.published_at)}")
            lines.append(f"- 评分: {item.score:.1f}")
            lines.append(f"- 标签: {', '.join(item.tags)}")
            lines.append(f"- 摘要: {item_summary_cn(item)}")
            lines.append(f"- 为什么重要: {item.why_it_matters}")
            lines.append("")

    if failures:
        lines.extend(["## 降级说明", ""])
        for failure in failures:
            lines.append(f"- {failure.source}: {failure.reason} ({failure.url})")
        lines.append("")

    lines.extend(["## 原始来源链接", ""])
    for idx, item in enumerate(items, start=1):
        lines.append(f"{idx}. [{item.title}]({item.url}) - {item.source} - {format_datetime(item.published_at)} - {item.score:.1f}")
    lines.append("")
    return "\n".join(lines)


def item_to_json(item: NewsItem) -> dict[str, Any]:
    data = asdict(item)
    data["published_at"] = item.published_at.isoformat() if item.published_at else None
    data["tags"] = list(item.tags)
    return data


def write_report_files(
    output_dir: Path,
    report_date: str,
    items: list[NewsItem],
    failures: list[SourceFailure],
    generated_at: datetime,
    status: str,
    analysis: AnalysisResult,
) -> ReportResult:
    output_dir.mkdir(parents=True, exist_ok=True)
    basename = f"{report_date}-{REPORT_BASENAME}"
    html_path = output_dir / f"{basename}.html"
    md_path = output_dir / f"{basename}.md"
    json_path = output_dir / f"{basename}.json"

    html_path.write_text(
        render_html_report(items, failures, report_date, generated_at, status, analysis),
        encoding="utf-8",
    )
    md_path.write_text(
        render_markdown_report(items, failures, report_date, generated_at, status, analysis),
        encoding="utf-8",
    )
    payload = {
        "report_date": report_date,
        "generated_at": generated_at.isoformat(),
        "status": status,
        "analysis": {
            "used_ai": analysis.used_ai,
            "model": analysis.model,
            "digest": list(analysis.digest),
        },
        "output_dir": str(output_dir),
        "item_count": len(items),
        "items": [item_to_json(item) for item in items],
        "failures": [asdict(failure) for failure in failures],
    }
    json_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return ReportResult(
        html_path=html_path,
        md_path=md_path,
        json_path=json_path,
        status=status,
        item_count=len(items),
        failures=failures,
    )


def generate_report(
    output_dir: Path = DEFAULT_OUTPUT_DIR,
    use_live_network: bool = True,
    use_ai_analysis: bool = True,
    now: datetime | None = None,
    lookback_hours: int = DEFAULT_LOOKBACK_HOURS,
    max_items: int = DEFAULT_MAX_ITEMS,
    analysis_model: str = DEFAULT_ANALYSIS_MODEL,
) -> ReportResult:
    now = now or get_local_now()
    if now.tzinfo is None:
        now = now.replace(tzinfo=timezone.utc)
    report_date = now.astimezone(get_local_now().tzinfo).date().isoformat()

    failures: list[SourceFailure] = []
    if use_live_network:
        items, failures = fetch_all_sources(DEFAULT_SOURCES, now, lookback_hours)
        candidates = rank_items(items, now=now)
        if candidates:
            candidates = rank_items(enrich_top_items_with_metadata(candidates[: max_items * 2], limit=14), now=now)
        status = "degraded" if failures or not candidates else "ready"
        if not candidates:
            failures.append(
                SourceFailure(
                    source="Daily Hot Report",
                    url="local fallback",
                    reason="所有外部来源均未返回可用条目，已写入离线降级内容。",
                )
            )
            candidates = rank_items(offline_seed_items(now), now=now)
    else:
        failures.append(
            SourceFailure(
                source="Daily Hot Report",
                url="local fallback",
                reason="use_live_network=False，跳过外部抓取并生成离线降级报告。",
            )
        )
        candidates = rank_items(offline_seed_items(now), now=now)
        status = "degraded"

    if use_ai_analysis:
        try:
            analysis = analyze_candidates_with_openai(candidates, now=now, max_items=max_items, model=analysis_model)
            ranked = analysis.items[:max_items]
        except Exception as exc:
            failures.append(
                SourceFailure(
                    source="OpenAI/Codex Analysis",
                    url="https://api.openai.com/v1/responses",
                    reason=str(exc),
                )
            )
            status = "degraded"
            analysis = fallback_analysis_result(candidates, now=now)
            ranked = analysis.items[:max_items]
    else:
        failures.append(
            SourceFailure(
                source="OpenAI/Codex Analysis",
                url="local configuration",
                reason="AI analysis disabled by --no-ai-analysis; using rule-based fallback.",
            )
        )
        status = "degraded"
        analysis = fallback_analysis_result(candidates, now=now)
        ranked = analysis.items[:max_items]

    return write_report_files(
        output_dir=Path(output_dir),
        report_date=report_date,
        items=ranked,
        failures=failures,
        generated_at=now,
        status=status,
        analysis=analysis,
    )


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    now = get_local_now()
    if args.date:
        try:
            date_value = datetime.strptime(args.date, "%Y-%m-%d").date()
        except ValueError as exc:
            raise SystemExit(f"--date must be YYYY-MM-DD: {exc}") from exc
        now = now.replace(year=date_value.year, month=date_value.month, day=date_value.day)
    result = generate_report(
        output_dir=args.output_dir,
        use_live_network=not args.no_live_network,
        use_ai_analysis=not args.no_ai_analysis,
        now=now,
        lookback_hours=args.lookback_hours,
        max_items=args.max_items,
        analysis_model=args.analysis_model,
    )
    print(f"status={result.status}")
    print(result.html_path)
    print(result.md_path)
    print(result.json_path)
    if result.failures:
        print(f"failures={len(result.failures)}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
