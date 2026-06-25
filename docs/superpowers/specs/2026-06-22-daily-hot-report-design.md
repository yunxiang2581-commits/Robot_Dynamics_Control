# 2026-06-22 机器人运动控制 & AI 今日热点日报设计

## 目标

每天自动生成一份中文 HTML 日报，保存到 `D:\桌面\每日热点`，默认文件名为：

`YYYY-MM-DD-机器人运动控制与AI热点日报.html`

并附带同名 `md` 与 `json` 归档文件。日报必须以机器人运动控制为核心，AI 最新消息为第二优先级。若外部抓取失败，也要生成降级报告，说明已获取内容和失败原因。

## 方案对比

### 方案 A: RSS 优先 + HTML 抓取补充

从 arXiv、Hacker News、GitHub Trending、官方博客 RSS/Atom 先抓主内容，再对少量候选页面补抓 `og:title` / `og:description` / `og:image`。

优点：稳定、可维护、适合每日自动跑。

缺点：个别来源没有标准 feed 时需要少量页面抓取补丁。

### 方案 B: 全页面抓取

直接抓各站页面并解析 HTML。

优点：覆盖面广。

缺点：脆弱、维护成本高，站点改版会频繁失效。

### 方案 C: 半人工清单

固定少量来源，人工挑选后写成日报。

优点：最稳。

缺点：不满足“每天自动生成”的目标。

## 选型

采用 **方案 A**。

这是最适合日更的折中：先用 RSS / Atom / 公开页面拿到高价值候选，再做少量 HTML 元信息补充。这样既能自动化，又不会把系统做成脆弱的网页爬虫。

## 组件设计

### 1. 抓取层

职责：
- 维护来源清单
- 拉取 feed 或网页
- 提取标题、链接、发布时间、摘要、来源名、封面图

建议来源分组：
- 机器人运动控制优先：arXiv `cs.RO`、`cs.AI`、`cs.LG`、`eess.SY`，IEEE Spectrum Robotics，The Robot Report，Robohub，ROS Discourse，NVIDIA Robotics Blog，DeepMind Robotics
- AI 次优先：OpenAI、Anthropic、Google DeepMind、Meta AI、Microsoft AI、NVIDIA、Hugging Face、Product Hunt、Hacker News、GitHub Trending

### 2. 归一化与去重层

职责：
- 把不同来源的结果转成统一数据结构
- 按规范化 URL、标题相似度、来源+标题组合去重
- 过滤掉明显过旧或重复转载的内容

### 3. 评分层

职责：
- 对候选信息做排序
- 机器人运动控制内容必须压过通用 AI 内容
- 评分维度建议：
  - 领域匹配
  - 时间新鲜度
  - 来源权重
  - 标题/摘要关键词密度
  - 是否带原始链接与封面图

### 4. 渲染层

职责：
- 输出高科技、动态、简约风格 HTML
- 生成今日摘要、机器人运动控制重点、具身智能/人形机器人动态、AI 最新消息、热门 AI 工具、趋势信号、原始来源链接
- 以卡片方式展示重点条目，支持封面图
- 同步写出 Markdown 与 JSON

### 5. 自动化层

职责：
- 提供一个可手动运行的生成脚本
- 提供一个 Windows 定时任务安装脚本
- 默认每天 07:30 执行

## 数据结构

建议统一结构：

```python
{
  "title": str,
  "source": str,
  "url": str,
  "published_at": str | None,
  "summary": str,
  "why_it_matters": str,
  "tags": list[str],
  "score": float,
  "category": str,
  "image_url": str | None,
  "local_image": str | None,
  "content_type": "robotics" | "embodied" | "ai" | "tool" | "trend",
}
```

## 输出

输出目录固定到：

`D:\桌面\每日热点`

文件：
- `YYYY-MM-DD-机器人运动控制与AI热点日报.html`
- `YYYY-MM-DD-机器人运动控制与AI热点日报.md`
- `YYYY-MM-DD-机器人运动控制与AI热点日报.json`

如果下载了图片，则保存在同目录下的资源子目录中。

## 错误处理

- 单个来源失败不能中断整份报告
- 每个来源失败要写入降级说明
- 若某类来源全失败，报告仍要输出已有内容，并明确标注缺失范围

## 验收标准

- 能在本机生成一份完整 HTML 日报
- 报告里机器人运动控制内容明显占主导
- 内容包含原文链接、来源名、发布时间
- 有可读的中文摘要与重要性判断
- 外部抓取失败时能输出降级报告
- 能通过定时任务每天自动执行
