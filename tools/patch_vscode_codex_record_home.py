"""Patch the VS Code Codex extension to use one fixed Codex record home.

Why this exists:
    The VS Code Codex extension starts an internal ``codex app-server``.
    That app-server reads conversations from ``CODEX_HOME``.  If different
    launchers, URLs, or profiles start it with different environments, the
    thread list appears to be split.  This patch forces the extension-spawned
    app-server to use one stable ``CODEX_HOME``.

The script is intentionally small and reversible:
    - it backs up ``out/extension.js`` before editing;
    - it inserts marked blocks, so re-running is idempotent;
    - it can restore the newest backup with ``--revert``.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
from datetime import datetime
from pathlib import Path


PATCH_NAME = "codex-record-sync"
ENV_START = f"/* {PATCH_NAME}:env:start */"
ENV_END = f"/* {PATCH_NAME}:env:end */"
WSL_START = f"/* {PATCH_NAME}:wsl:start */"
WSL_END = f"/* {PATCH_NAME}:wsl:end */"

DEFAULT_EXTENSION_GLOB = "openai.chatgpt-*"
EXTENSION_RELATIVE_JS = Path("out") / "extension.js"


def default_vscode_extensions_dir() -> Path:
    """Return the normal VS Code extension directory on this machine."""
    return Path.home() / ".vscode" / "extensions"


def default_codex_home() -> Path:
    """Return the stable Codex home that should own the shared records."""
    return Path.home() / ".codex"


def find_latest_extension_js(extensions_dir: Path) -> Path:
    """Find the newest installed OpenAI Codex VS Code extension bundle."""
    candidates = []
    for extension_dir in extensions_dir.glob(DEFAULT_EXTENSION_GLOB):
        extension_js = extension_dir / EXTENSION_RELATIVE_JS
        if extension_js.exists():
            candidates.append(extension_js)
    if not candidates:
        raise FileNotFoundError(
            f"No VS Code Codex extension.js found under {extensions_dir}"
        )
    return max(candidates, key=lambda path: path.stat().st_mtime)


def _remove_marked_block(text: str, start: str, end: str) -> str:
    pattern = re.compile(re.escape(start) + r".*?" + re.escape(end), re.DOTALL)
    return pattern.sub("", text)


def remove_existing_patch(text: str) -> str:
    """Remove this script's previous patch markers without touching other code."""
    text = _remove_marked_block(text, ENV_START, ENV_END)
    text = _remove_marked_block(text, WSL_START, WSL_END)
    return text


def build_env_patch(codex_home: Path) -> str:
    """Build the JavaScript snippet that fixes CODEX_HOME for spawned Codex."""
    codex_home_literal = json.dumps(
        _javascript_path_string(codex_home), ensure_ascii=True
    )
    return (
        ENV_START
        + "try{process.env.CODEX_HOME="
        + "process.env.CODEX_VSCODE_GLOBAL_CODEX_HOME||"
        + codex_home_literal
        + "}catch{}"
        + ENV_END
    )


def _javascript_path_string(path: Path) -> str:
    """Return a stable path string for the JavaScript patch literal."""
    path_text = str(path)
    if re.match(r"^[A-Za-z]:/", path_text):
        return path_text.replace("/", "\\")
    return path_text


def build_wsl_patch() -> str:
    """Build the JavaScript snippet that passes CODEX_HOME into WSL mode."""
    return (
        WSL_START
        + "process.env.CODEX_HOME&&d.push(`CODEX_HOME=${ar(process.env.CODEX_HOME)}`);"
        + WSL_END
    )


def apply_text_patch(text: str, codex_home: Path) -> tuple[str, list[str]]:
    """Return patched extension.js text and a list of applied patch parts."""
    applied: list[str] = []
    text = remove_existing_patch(text)

    env_patch = build_env_patch(codex_home)
    anchor = "function gce(t,e,r){"
    if anchor not in text:
        raise RuntimeError("Could not find Codex spawn function anchor: function gce")
    text = text.replace(anchor, env_patch + anchor, 1)
    applied.append("host CODEX_HOME")

    wsl_patch = build_wsl_patch()
    original_wsl = (
        'd=[`PATH=${c}:$PATH`,"RUST_LOG=warn",'
        '`CODEX_INTERNAL_ORIGINATOR_OVERRIDE=${op}`],f=["-d",o];'
    )
    patched_wsl_anchor = (
        'd=[`PATH=${c}:$PATH`,"RUST_LOG=warn",'
        '`CODEX_INTERNAL_ORIGINATOR_OVERRIDE=${op}`];let f=["-d",o];'
    )
    if original_wsl in text:
        text = text.replace(
            original_wsl,
            original_wsl.replace("],f=", "];" + wsl_patch + "let f="),
            1,
        )
        applied.append("WSL CODEX_HOME")
    elif patched_wsl_anchor in text:
        text = text.replace(
            patched_wsl_anchor,
            patched_wsl_anchor.replace("];let f=", "];" + wsl_patch + "let f="),
            1,
        )
        applied.append("WSL CODEX_HOME")
    else:
        applied.append("WSL CODEX_HOME skipped: anchor not found")

    return text, applied


def backup_file(path: Path) -> Path:
    """Create a timestamped backup next to the target file."""
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    backup = path.with_name(f"{path.name}.bak-{PATCH_NAME}-{stamp}")
    shutil.copy2(path, backup)
    return backup


def newest_backup(path: Path) -> Path:
    """Return the newest backup created by this script for a target file."""
    backups = sorted(
        path.parent.glob(f"{path.name}.bak-{PATCH_NAME}-*"),
        key=lambda candidate: candidate.stat().st_mtime,
        reverse=True,
    )
    if not backups:
        raise FileNotFoundError(f"No {PATCH_NAME} backup found next to {path}")
    return backups[0]


def patch_file(path: Path, codex_home: Path, dry_run: bool) -> None:
    """Patch one installed extension.js file."""
    old_text = path.read_text(encoding="utf-8")
    new_text, applied = apply_text_patch(old_text, codex_home)
    if old_text == new_text:
        print(f"No change needed: {path}")
        return
    print(f"Patch target: {path}")
    print(f"Shared CODEX_HOME: {codex_home}")
    print("Patch parts: " + ", ".join(applied))
    if dry_run:
        print("Dry run only; file was not changed.")
        return
    backup = backup_file(path)
    path.write_text(new_text, encoding="utf-8")
    print(f"Backup written: {backup}")
    print("Patch written. Reload VS Code for the extension to restart app-server.")


def revert_file(path: Path, dry_run: bool) -> None:
    """Restore the newest backup created by this script."""
    backup = newest_backup(path)
    print(f"Restore target: {path}")
    print(f"Newest backup: {backup}")
    if dry_run:
        print("Dry run only; file was not changed.")
        return
    shutil.copy2(backup, path)
    print("Backup restored. Reload VS Code for the extension to restart app-server.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Patch VS Code Codex extension to share one Codex record home."
    )
    parser.add_argument(
        "--extension-js",
        type=Path,
        help="Explicit path to openai.chatgpt.../out/extension.js.",
    )
    parser.add_argument(
        "--extensions-dir",
        type=Path,
        default=default_vscode_extensions_dir(),
        help="VS Code extensions directory used when --extension-js is omitted.",
    )
    parser.add_argument(
        "--codex-home",
        type=Path,
        default=default_codex_home(),
        help="Shared CODEX_HOME that owns Codex records.",
    )
    parser.add_argument("--dry-run", action="store_true", help="Show what would change.")
    parser.add_argument("--revert", action="store_true", help="Restore newest backup.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    extension_js = args.extension_js or find_latest_extension_js(args.extensions_dir)
    extension_js = extension_js.resolve()
    codex_home = args.codex_home.expanduser().resolve()
    if args.revert:
        revert_file(extension_js, args.dry_run)
    else:
        patch_file(extension_js, codex_home, args.dry_run)


if __name__ == "__main__":
    main()
