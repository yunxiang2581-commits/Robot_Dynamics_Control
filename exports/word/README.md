# Word 导出目录

## 用途

本目录用于存放从 Markdown 导出的 Word 文档。Windows 优先使用 `scripts/export_md_to_docx.ps1`，Linux/macOS 或 Git Bash 可使用 `scripts/export_md_to_docx.sh`。两个脚本都通过 Pandoc 完成转换。

## 目录说明

| 目录 | 用途 |
| --- | --- |
| `single/` | 存放每个 Markdown 文件单独导出的 `.docx` |
| `combined/` | 存放合并版总文档 |
| `logs/` | 存放导出日志 |

## 运行方式

Windows PowerShell：

```powershell
powershell -ExecutionPolicy Bypass -File scripts/export_md_to_docx.ps1
```

Linux/macOS 或 Git Bash：

```bash
bash scripts/export_md_to_docx.sh
```

脚本会从仓库根目录解析路径，转换以下 Markdown 文件：

| 输入范围 | 说明 |
| --- | --- |
| `README.md` | 项目根 README |
| `docs/**/*.md` | docs 下所有 Markdown 文档 |
| `external/*.md` | external 下一级 Markdown 说明文件 |

## 输出文件

| 类型 | 路径 |
| --- | --- |
| 单文件 Word | `exports/word/single/` |
| 合并版 Word | `exports/word/combined/robot_motion_control_job_project_docs.docx` |
| 导出日志 | `exports/word/logs/export_md_to_docx.log` |

单文件导出会把 Markdown 路径转换成安全文件名。例如：

```text
docs/00_preparation/00_preparation_overview.md
```

会导出为：

```text
exports/word/single/docs__00_preparation__00_preparation_overview.docx
```

## 安装 Pandoc

如果脚本提示 `pandoc` 不存在，请手动安装后重新运行。

| 方式 | 安装命令 |
| --- | --- |
| pip / Anaconda | `python -m pip install pypandoc-binary` |
| Ubuntu/Debian | `sudo apt-get update && sudo apt-get install -y pandoc` |
| macOS | `brew install pandoc` |
| Windows | `winget install --id JohnMacFarlane.Pandoc` |

当前导出脚本会优先查找系统 `pandoc` 命令；如果没有找到，会继续查找 `pypandoc-binary` 自带的 Pandoc。

## 常见问题

| 问题 | 处理建议 |
| --- | --- |
| 表格太宽 | 在 Markdown 中拆分长表格，或导出后在 Word 中把页面改为横向、缩小字体、自动调整表格宽度 |
| 中文字体显示不好 | 在 Word 中统一设置中文字体，例如微软雅黑、宋体或思源黑体；后续可增加 Pandoc reference docx 模板 |
| 公式显示异常 | 优先使用 Pandoc 支持的 LaTeX 数学语法；复杂公式建议导出后在 Word 中检查 |
| 图片路径丢失 | 确认 Markdown 图片路径相对仓库根目录或文档目录可解析；避免引用未提交的大型外部资源 |

## Git 管理建议

导出的 `.docx` 和 `.log` 文件是生成产物，默认不建议提交到 Git。需要归档时，优先提交源 Markdown 和导出说明。
