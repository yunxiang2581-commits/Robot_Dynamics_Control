# 文档导出工具

## 中文版说明

本目录存放仓库级 Markdown 导出工具，用于把 README、准备阶段文档、A/B/C 项目文档、对比文档和面试文档导出为 Word 文件。

这些脚本是仓库级工具，不属于 A 项目算法脚本，也不调用 Pinocchio、MuJoCo、OSQP 或外部控制项目。

## 文件说明

| 文件 | 用途 |
| --- | --- |
| `export_md_to_docx.sh` | Linux、macOS 或 Git Bash 下的导出脚本 |
| `export_md_to_docx.ps1` | Windows PowerShell 下的导出脚本 |
| `README.md` | 当前说明文件 |

## 运行方式

Windows PowerShell：

```powershell
powershell -ExecutionPolicy Bypass -File tools/export/export_md_to_docx.ps1
```

Linux/macOS 或 Git Bash：

```bash
bash tools/export/export_md_to_docx.sh
```

## 输出位置

| 输出 | 路径 |
| --- | --- |
| 单文件 Word | `exports/word/single/` |
| 合并版 Word | `exports/word/combined/robot_motion_control_job_project_docs.docx` |
| 导出日志 | `exports/word/logs/export_md_to_docx.log` |

## English Summary

This directory contains repository-level export tools for converting Markdown documentation to Word documents. The scripts are documentation utilities, not robot control or experiment scripts.
