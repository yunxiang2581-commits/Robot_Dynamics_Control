from __future__ import annotations

from pathlib import Path
import textwrap

from PIL import Image, ImageDraw, ImageEnhance, ImageFilter, ImageFont


OUT_DIR = Path.home() / ".codex" / "generated_images" / "pubg_update_42_1"
BACKGROUND = OUT_DIR / "official_header.jpg"
OUTPUT = OUT_DIR / "pubg_update_42_1_cn_infographic_4k.png"
OUTPUT_PORTRAIT = OUT_DIR / "pubg_update_42_1_cn_infographic_portrait_4k.png"

FONT_HEI = Path("C:/Windows/Fonts/simhei.ttf")
FONT_ARIAL_BOLD = Path("C:/Windows/Fonts/arialbd.ttf")
FONT_BAHN = Path("C:/Windows/Fonts/bahnschrift.ttf")


W, H = 3840, 2160


def font(path: Path, size: int) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(str(path), size=size)


def fit_cover(img: Image.Image, size: tuple[int, int]) -> Image.Image:
    target_w, target_h = size
    scale = max(target_w / img.width, target_h / img.height)
    new_size = (int(img.width * scale), int(img.height * scale))
    img = img.resize(new_size, Image.Resampling.LANCZOS)
    left = (img.width - target_w) // 2
    top = (img.height - target_h) // 2
    return img.crop((left, top, left + target_w, top + target_h))


def rounded_rect(draw: ImageDraw.ImageDraw, box, radius, fill, outline=None, width=1):
    draw.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=width)


def draw_wrapped(
    draw: ImageDraw.ImageDraw,
    text: str,
    xy: tuple[int, int],
    fnt: ImageFont.FreeTypeFont,
    fill,
    max_chars: int,
    line_gap: int = 8,
) -> int:
    x, y = xy
    lines: list[str] = []
    for para in text.split("\n"):
        if not para.strip():
            lines.append("")
            continue
        lines.extend(textwrap.wrap(para, width=max_chars, break_long_words=False, replace_whitespace=False))
    for line in lines:
        draw.text((x, y), line, font=fnt, fill=fill)
        bbox = draw.textbbox((x, y), line or "A", font=fnt)
        y += (bbox[3] - bbox[1]) + line_gap
    return y


def draw_tag(draw, xy, label, fill, text_fill=(18, 18, 18)):
    x, y = xy
    f = font(FONT_HEI, 30)
    bbox = draw.textbbox((0, 0), label, font=f)
    pad_x, pad_y = 24, 10
    box = (x, y, x + bbox[2] + pad_x * 2, y + bbox[3] + pad_y * 2)
    rounded_rect(draw, box, 10, fill)
    draw.text((x + pad_x, y + pad_y - 2), label, font=f, fill=text_fill)
    return box[2]


def draw_card(draw, box, title, items, accent, max_chars=25, item_size=30, title_size=44):
    x1, y1, x2, y2 = box
    rounded_rect(draw, box, 18, (12, 16, 20, 222), outline=(255, 255, 255, 42), width=2)
    draw.rectangle((x1, y1, x1 + 12, y2), fill=accent)
    title_font = font(FONT_HEI, title_size)
    item_font = font(FONT_HEI, item_size)
    draw.text((x1 + 38, y1 + 34), title, font=title_font, fill=(255, 232, 120))
    y = y1 + 108
    for item in items:
        draw.ellipse((x1 + 42, y + 13, x1 + 54, y + 25), fill=accent)
        y = draw_wrapped(draw, item, (x1 + 72, y), item_font, (232, 238, 238), max_chars=max_chars, line_gap=9)
        y += 10


def make_landscape() -> Image.Image:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    if BACKGROUND.exists():
        bg = Image.open(BACKGROUND).convert("RGB")
        bg = fit_cover(bg, (W, H))
    else:
        bg = Image.new("RGB", (W, H), (20, 24, 22))

    bg = ImageEnhance.Contrast(bg).enhance(1.08)
    bg = ImageEnhance.Color(bg).enhance(0.78)
    bg = bg.filter(ImageFilter.GaussianBlur(2.0))

    overlay = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    od = ImageDraw.Draw(overlay)
    # 左侧深色阅读区，右侧保留官方视觉氛围。
    od.rectangle((0, 0, W, H), fill=(7, 9, 10, 136))
    od.rectangle((0, 0, int(W * 0.64), H), fill=(4, 8, 9, 176))
    od.rectangle((0, 0, W, 260), fill=(0, 0, 0, 92))
    od.rectangle((0, H - 160, W, H), fill=(0, 0, 0, 110))
    img = Image.alpha_composite(bg.convert("RGBA"), overlay)
    draw = ImageDraw.Draw(img)

    yellow = (245, 191, 42)
    orange = (236, 113, 35)
    cyan = (62, 205, 220)
    green = (116, 216, 112)
    white = (246, 246, 240)
    muted = (188, 198, 195)

    # Header
    draw_tag(draw, (150, 110), "PUBG: BATTLEGROUNDS", yellow)
    draw.text((150, 190), "更新 42.1 速览", font=font(FONT_HEI, 118), fill=white)
    draw.text((154, 330), "PC 今日上线 · 官方 Patch Notes 2026.06.16", font=font(FONT_HEI, 42), fill=muted)
    draw.text((150, 410), "维护：PC 6月17日 00:00-08:30 UTC ｜ 主机 6月25日 01:00-09:00 UTC", font=font(FONT_HEI, 40), fill=(255, 230, 172))

    # Big feature panel
    rounded_rect(draw, (150, 530, 1420, 1158), 22, (10, 13, 15, 228), outline=(255, 255, 255, 55), width=2)
    draw.rectangle((150, 530, 162, 1158), fill=yellow)
    draw.text((204, 580), "今日重点：Ally Duo 限时 Beta", font=font(FONT_HEI, 58), fill=yellow)
    ally_items = [
        "入口：Play > Arcade > Ally Duo",
        "地图 Sanhok，TPP 双排；单人匹配，入局后与 AI 队友 Ella 组队",
        "PC 服务期：6月17日维护后 - 7月1日 07:00 UTC",
        "需 NVIDIA GeForce RTX GPU；最低 RTX 2080 Ti / 3060，8GB 显存，16GB 内存",
        "语音仅能与 Ella 沟通；观战、死亡回放、Replay、断线重连不可用",
    ]
    y = 670
    for it in ally_items:
        draw.ellipse((210, y + 15, 224, y + 29), fill=yellow)
        y = draw_wrapped(draw, it, (244, y), font(FONT_HEI, 35), (234, 241, 238), max_chars=34, line_gap=10)
        y += 14

    # Rondo future panel
    rounded_rect(draw, (150, 1205, 1420, 1508), 22, (11, 17, 21, 222), outline=(255, 255, 255, 44), width=2)
    draw.text((204, 1244), "后续开放：Rondo 新蓝圈规则", font=font(FONT_HEI, 50), fill=cyan)
    draw_wrapped(
        draw,
        "更快、更密集的 Rondo Arcade LABS 模式。PC：7月1日 07:00 - 7月15日 00:00 UTC；主机：7月9日 - 7月23日。",
        (204, 1322),
        font(FONT_HEI, 34),
        (231, 240, 241),
        max_chars=35,
        line_gap=10,
    )

    # Cards grid
    cards = [
        (
            (1500, 530, 2520, 890),
            "枪械与刷新",
            [
                "SLR 横向后坐力约 -10%",
                "SLR 弹速 840m/s → 870m/s",
                "降低前中段垂直后坐积累，回正略增强",
                "移除世界刷新：Mosin Nagant、R45、DP-28、PP-19 Bizon、P1911、QBU",
            ],
            orange,
        ),
        (
            (2580, 530, 3690, 890),
            "排位 Season 42",
            [
                "赛季延长至 42.1 - 42.3",
                "AWM 排位武器皮肤加入永久段位奖励",
                "排名权重提高；高段位 RP 损失降低",
                "连续两局前4 +5 RP，连续两局第1 +10 RP",
            ],
            green,
        ),
        (
            (1500, 950, 2520, 1320),
            "玩法系统",
            [
                "互动烟雾上线：爆炸、载具撞击可短暂驱散烟雾",
                "手雷、C4、迫击炮、红区、黑区等可影响烟雾",
                "起始飞机与空投飞机视觉材质升级",
                "PC 新增控制器支持；DualSense / DualShock 暂不支持",
            ],
            cyan,
        ),
        (
            (2580, 950, 3690, 1320),
            "蓝圈与治疗",
            [
                "动态蓝圈移除；蓝圈节奏与伤害系统重做",
                "排位 / 电竞总时长 32:50 → 30:10",
                "蓝圈伤害从距离制改为停留时间制",
                "医疗箱重量 20→15，使用 8秒→6秒",
                "绷带 10→12，使用 4秒→3秒；止痛药 10→6",
            ],
            yellow,
        ),
        (
            (1500, 1380, 2520, 1748),
            "Arcade / UGC / 载具",
            [
                "Intense Battle Royale 加入 Haven",
                "UGC 赛车：检查点、计时器、广告牌、车辆参数自定义",
                "新增备用轮胎与 Basic Racing 示例模式",
                "Harley-Davidson 摩托动画与排气效果优化",
            ],
            (195, 142, 255),
        ),
        (
            (2580, 1380, 3690, 1748),
            "商店 / 工坊 / 修复",
            [
                "BATTLEGROUNDS Plus：500 G-Coin 或 20,000 BP",
                "移除 Plus 原含 1,300 G-Coin",
                "黑市 2026 回归；工坊概率结构调整",
                "Hunter's Chest / Archivist's Chest 新增套装",
                "修复 Self-AED 蓝圈处决、M79 乘客位装备等问题",
            ],
            (255, 105, 116),
        ),
    ]
    for c in cards:
        draw_card(draw, *c, max_chars=29, item_size=29, title_size=42)

    # Bottom notes
    rounded_rect(draw, (150, 1570, 1420, 1834), 22, (10, 13, 15, 218), outline=(255, 255, 255, 44), width=2)
    draw.text((204, 1610), "适合发图的一句话总结", font=font(FONT_HEI, 50), fill=yellow)
    draw_wrapped(
        draw,
        "42.1 是一次偏系统向的大更新：PC 新 AI 队友模式先行上线，SLR 明显增强，排位 RP 与奖励调整，烟雾和蓝圈机制都更强调实战决策。",
        (204, 1688),
        font(FONT_HEI, 35),
        (234, 241, 238),
        max_chars=34,
        line_gap=11,
    )

    # Source line and visual title
    draw.text((150, 1992), "资料来源：PUBG 官方 Patch Notes - Update 42.1（2026.06.16）", font=font(FONT_HEI, 30), fill=(180, 188, 185))
    draw.text((2740, 1858), "42.1", font=font(FONT_ARIAL_BOLD, 188), fill=(255, 255, 255, 52))
    draw.text((2750, 2010), "UPDATE BRIEF", font=font(FONT_BAHN, 62), fill=(255, 232, 120, 145))

    # Subtle frame
    draw.rectangle((44, 44, W - 44, H - 44), outline=(255, 255, 255, 48), width=2)
    return img.convert("RGB")


def make_portrait() -> Image.Image:
    pw, ph = 2160, 3840
    if BACKGROUND.exists():
        bg = Image.open(BACKGROUND).convert("RGB")
        bg = fit_cover(bg, (pw, ph))
    else:
        bg = Image.new("RGB", (pw, ph), (18, 22, 20))

    bg = ImageEnhance.Contrast(bg).enhance(1.05)
    bg = ImageEnhance.Color(bg).enhance(0.72)
    bg = bg.filter(ImageFilter.GaussianBlur(2.5))
    overlay = Image.new("RGBA", (pw, ph), (5, 7, 8, 168))
    img = Image.alpha_composite(bg.convert("RGBA"), overlay)
    draw = ImageDraw.Draw(img)

    yellow = (245, 191, 42)
    orange = (236, 113, 35)
    cyan = (62, 205, 220)
    green = (116, 216, 112)
    white = (246, 246, 240)
    muted = (188, 198, 195)

    draw.rectangle((0, 0, pw, 500), fill=(0, 0, 0, 118))
    draw_tag(draw, (110, 112), "PUBG: BATTLEGROUNDS", yellow)
    draw.text((110, 210), "更新 42.1", font=font(FONT_HEI, 130), fill=white)
    draw.text((112, 365), "今日更新内容速览", font=font(FONT_HEI, 58), fill=(255, 232, 120))
    draw.text((110, 455), "PC 6月17日 00:00-08:30 UTC ｜ 主机 6月25日 01:00-09:00 UTC", font=font(FONT_HEI, 34), fill=muted)

    sections = [
        (
            "今日重点：Ally Duo 限时 Beta",
            [
                "Play > Arcade > Ally Duo 进入；Sanhok / TPP / 双排",
                "单人匹配，入局后与 AI 队友 Ella 组队",
                "PC：6月17日维护后 - 7月1日 07:00 UTC",
                "需 RTX GPU；最低 RTX 2080 Ti / 3060，8GB 显存，16GB 内存",
                "语音只和 Ella 沟通；观战、死亡回放、Replay、断线重连不可用",
            ],
            yellow,
        ),
        (
            "枪械与刷新",
            [
                "SLR 横向后坐力约 -10%；弹速 840m/s → 870m/s",
                "降低前中段垂直后坐积累，回正略增强",
                "移除世界刷新：Mosin Nagant、R45、DP-28、PP-19 Bizon、P1911、QBU",
            ],
            orange,
        ),
        (
            "排位 Season 42",
            [
                "赛季延长至 42.1 - 42.3，准备后续排位重做",
                "AWM 排位武器皮肤加入永久段位奖励",
                "排名权重提高；高段位 RP 损失降低",
                "连续两局前4 +5 RP，连续两局第1 +10 RP",
                "Plus 可用 500 G-Coin 或 20,000 BP 购买，移除原 1,300 G-Coin",
            ],
            green,
        ),
        (
            "玩法系统",
            [
                "互动烟雾：爆炸、载具撞击可短暂驱散烟雾",
                "适用于烟雾弹与被摧毁汽油桶产生的烟雾",
                "起始飞机与空投飞机视觉材质升级",
                "PC 新增控制器支持；DualSense / DualShock 暂不支持",
            ],
            cyan,
        ),
        (
            "蓝圈与治疗",
            [
                "动态蓝圈移除；蓝圈节奏与伤害系统重做",
                "排位 / 电竞总时长 32:50 → 30:10",
                "蓝圈伤害从距离制改为停留时间制",
                "医疗箱重量 20→15，使用 8秒→6秒；绷带 10→12，使用 4秒→3秒",
                "止痛药 10→6；Boost 回血间隔缩短，总回复量不变",
            ],
            yellow,
        ),
        (
            "Arcade / UGC / 载具",
            [
                "Rondo 新蓝圈规则 LABS：PC 7月1日 - 7月15日",
                "Intense Battle Royale 加入 Haven",
                "UGC 赛车：检查点、计时器、广告牌、车辆参数自定义",
                "新增备用轮胎与 Basic Racing 示例模式",
            ],
            (195, 142, 255),
        ),
        (
            "商店 / 工坊 / 修复",
            [
                "黑市 2026 回归；工坊概率结构调整",
                "新增宝箱套装；修复 Self-AED、M79 等多项问题",
            ],
            (255, 105, 116),
        ),
    ]

    y = 600
    card_w = 1940
    for title, items, accent in sections:
        est_h = 128 + len(items) * 56
        if title.startswith("今日重点"):
            est_h += 54
        box = (110, y, 110 + card_w, y + est_h)
        draw_card(draw, box, title, items, accent, max_chars=34, item_size=30, title_size=44)
        y += est_h + 24

    summary_bottom = min(y + 222, ph - 210)
    rounded_rect(draw, (110, y + 8, 2050, summary_bottom), 22, (10, 13, 15, 218), outline=(255, 255, 255, 44), width=2)
    draw.text((154, y + 42), "一句话总结", font=font(FONT_HEI, 42), fill=yellow)
    draw_wrapped(
        draw,
        "42.1 是一次偏系统向的大更新：PC 新 AI 队友模式先行上线，SLR 明显增强，排位 RP 与奖励调整，烟雾和蓝圈机制都更强调实战决策。",
        (154, y + 108),
        font(FONT_HEI, 29),
        (234, 241, 238),
        max_chars=42,
        line_gap=8,
    )
    draw.text((110, ph - 126), "资料来源：PUBG 官方 Patch Notes - Update 42.1（2026.06.16）", font=font(FONT_HEI, 28), fill=(178, 188, 185))
    draw.text((1530, ph - 218), "42.1", font=font(FONT_ARIAL_BOLD, 132), fill=(255, 255, 255, 72))
    draw.text((1538, ph - 105), "UPDATE BRIEF", font=font(FONT_BAHN, 48), fill=(255, 232, 120, 150))
    draw.rectangle((36, 36, pw - 36, ph - 36), outline=(255, 255, 255, 48), width=2)
    return img.convert("RGB")


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    landscape = make_landscape()
    landscape.save(OUTPUT, quality=96)
    portrait = make_portrait()
    portrait.save(OUTPUT_PORTRAIT, quality=96)
    print(OUTPUT)
    print(OUTPUT_PORTRAIT)


if __name__ == "__main__":
    main()
