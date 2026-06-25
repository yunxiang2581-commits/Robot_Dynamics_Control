"""Patch Codex history list requests to include all model providers.

Why this exists:
    Current Codex UI bundles can ask ``thread/list`` with a provider filter
    derived from the active API/provider.  After switching providers, old
    records still exist in the Codex state database but disappear from the
    main history list.  The app-server already supports ``modelProviders: []``
    as "show every provider", so this patch changes only the main list request
    parameter and leaves stored conversation metadata untouched.

The script is intentionally conservative:
    - it patches only text bundles that contain a known ``thread/list`` filter;
    - it backs up every edited target next to the original file;
    - it is idempotent, so re-running after an extension upgrade is safe.
"""

from __future__ import annotations

import argparse
import re
import shutil
from datetime import datetime
from pathlib import Path
from typing import NamedTuple


PATCH_NAME = "codex-thread-list-all-providers"
EXTENSION_RELATIVE_JS = Path("out") / "extension.js"
DEFAULT_EXTENSION_GLOB = "openai.chatgpt-*"
DEFAULT_DESKTOP_GLOB = "OpenAI.Codex_*"

PROVIDER_FILTER_RE = re.compile(r"modelProviders:e\?\[[^\]]+\]:null")
NULL_PROVIDER_RE = re.compile(r"modelProviders:null")
ALL_PROVIDERS_EQUAL_LENGTH = "modelProviders:[]  "


class PatchResult(NamedTuple):
    text: str
    changed_count: int
    has_thread_list: bool
    has_provider_filter: bool


class BytePatchResult(NamedTuple):
    data: bytes
    changed_count: int
    has_thread_list: bool
    has_provider_filter: bool


def patch_thread_list_text(text: str) -> PatchResult:
    """Return text with main ``thread/list`` provider filters set to all providers."""
    has_thread_list = _has_thread_list(text)
    matches = list(PROVIDER_FILTER_RE.finditer(text))
    matches.extend(NULL_PROVIDER_RE.finditer(text))
    if not has_thread_list or not matches:
        return PatchResult(
            text=text,
            changed_count=0,
            has_thread_list=has_thread_list,
            has_provider_filter=bool(matches),
        )

    pieces: list[str] = []
    last = 0
    changed_count = 0
    for match in matches:
        if _is_thread_list_provider_filter(text, match.start(), match.end()):
            pieces.append(text[last : match.start()])
            pieces.append(_replacement_for_match(match.group(0)))
            last = match.end()
            changed_count += 1
    if changed_count == 0:
        return PatchResult(
            text=text,
            changed_count=0,
            has_thread_list=True,
            has_provider_filter=True,
        )

    pieces.append(text[last:])
    return PatchResult(
        text="".join(pieces),
        changed_count=changed_count,
        has_thread_list=True,
        has_provider_filter=True,
    )


def patch_thread_list_bytes(data: bytes) -> BytePatchResult:
    """Patch binary bundles using only equal-length replacements."""
    has_thread_list = _has_thread_list_bytes(data)
    matches = list(re.finditer(b"modelProviders:null", data))
    if not has_thread_list or not matches:
        return BytePatchResult(
            data=data,
            changed_count=0,
            has_thread_list=has_thread_list,
            has_provider_filter=bool(matches),
        )

    replacement = ALL_PROVIDERS_EQUAL_LENGTH.encode("ascii")
    old_value = b"modelProviders:null"
    if len(replacement) != len(old_value):
        raise RuntimeError("Binary replacement must preserve byte length.")

    pieces: list[bytes] = []
    last = 0
    changed_count = 0
    for match in matches:
        if _is_thread_list_provider_filter_bytes(data, match.start(), match.end()):
            pieces.append(data[last : match.start()])
            pieces.append(replacement)
            last = match.end()
            changed_count += 1
    if changed_count == 0:
        return BytePatchResult(
            data=data,
            changed_count=0,
            has_thread_list=True,
            has_provider_filter=True,
        )

    pieces.append(data[last:])
    return BytePatchResult(
        data=b"".join(pieces),
        changed_count=changed_count,
        has_thread_list=True,
        has_provider_filter=True,
    )


def _is_thread_list_provider_filter(text: str, start: int, end: int) -> bool:
    """Check a small minified-code window before replacing a provider filter."""
    window_start = max(0, start - 800)
    window_end = min(len(text), end + 300)
    window = text[window_start:window_end]
    return (
        _has_thread_list(window)
        and "sendRequest" in window
        and ('sortKey:"created_at"' in window or "sortKey:`created_at`" in window or "sortKey:" in window)
    )


def _has_thread_list(text: str) -> bool:
    return '"thread/list"' in text or "`thread/list`" in text


def _has_thread_list_bytes(data: bytes) -> bool:
    return b'"thread/list"' in data or b"`thread/list`" in data


def _is_thread_list_provider_filter_bytes(data: bytes, start: int, end: int) -> bool:
    window_start = max(0, start - 800)
    window_end = min(len(data), end + 300)
    window = data[window_start:window_end]
    return (
        _has_thread_list_bytes(window)
        and b"sendRequest" in window
        and (b"sortKey:" in window)
    )


def _replacement_for_match(value: str) -> str:
    if value == "modelProviders:null":
        return ALL_PROVIDERS_EQUAL_LENGTH
    return "modelProviders:[]"


def default_vscode_extensions_dir() -> Path:
    return Path.home() / ".vscode" / "extensions"


def default_windowsapps_dir() -> Path:
    return Path(r"C:\Program Files\WindowsApps")


def default_backup_dir() -> Path:
    return Path.home() / ".codex" / "patch_backups" / PATCH_NAME


def find_latest_vscode_extension_js(extensions_dir: Path) -> Path | None:
    candidates = []
    for extension_dir in extensions_dir.glob(DEFAULT_EXTENSION_GLOB):
        extension_js = extension_dir / EXTENSION_RELATIVE_JS
        if extension_js.exists():
            candidates.append(extension_js)
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime)


def find_desktop_text_targets(windowsapps_dir: Path) -> list[Path]:
    """Find desktop Codex text bundles that are safe to patch directly."""
    targets: list[Path] = []
    for app_dir in windowsapps_dir.glob(DEFAULT_DESKTOP_GLOB):
        resource_dir = app_dir / "app" / "resources"
        for candidate in [
            resource_dir / "default_app" / "main.js",
            resource_dir / "default_app" / "preload.js",
            resource_dir / "default_app" / "default_app.js",
        ]:
            if candidate.exists():
                targets.append(candidate)
    return sorted(set(targets), key=lambda path: str(path).lower())


def backup_file(path: Path, backup_dir: Path | None = None) -> Path:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    if backup_dir is None:
        backup = path.with_name(f"{path.name}.bak-{PATCH_NAME}-{stamp}")
    else:
        backup_dir.mkdir(parents=True, exist_ok=True)
        safe_parent = re.sub(r"[^A-Za-z0-9_.-]+", "_", path.parent.name)
        backup = backup_dir / f"{safe_parent}-{path.name}.bak-{PATCH_NAME}-{stamp}"
    try:
        shutil.copy2(path, backup)
    except PermissionError:
        if backup_dir is not None:
            raise
        return backup_file(path, default_backup_dir())
    return backup


def newest_backup(path: Path, backup_dir: Path | None = None) -> Path:
    search_dirs = [path.parent]
    if backup_dir is not None:
        search_dirs.append(backup_dir)
    search_dirs.append(default_backup_dir())
    backups = []
    for search_dir in search_dirs:
        if not search_dir.exists():
            continue
        backups.extend(search_dir.glob(f"*{path.name}.bak-{PATCH_NAME}-*"))
    backups = sorted(backups, key=lambda candidate: candidate.stat().st_mtime, reverse=True)
    if not backups:
        raise FileNotFoundError(f"No {PATCH_NAME} backup found next to {path}")
    return backups[0]


def patch_file(path: Path, dry_run: bool, backup_dir: Path | None = None) -> bool:
    try:
        old_text = path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        return patch_binary_file(path, dry_run, backup_dir)

    result = patch_thread_list_text(old_text)
    if result.changed_count == 0:
        print(_scan_message(path, result))
        return False

    print(f"Patch target: {path}")
    print(f"thread/list provider filters changed: {result.changed_count}")
    if dry_run:
        print("Dry run only; file was not changed.")
        return True

    backup = backup_file(path, backup_dir)
    try:
        path.write_text(result.text, encoding="utf-8")
    except PermissionError:
        print(f"Backup written: {backup}")
        print(f"Patch blocked by file permissions: {path}")
        return False
    print(f"Backup written: {backup}")
    print("Patch written. Restart or reload the Codex surface that owns this bundle.")
    return True


def patch_binary_file(path: Path, dry_run: bool, backup_dir: Path | None = None) -> bool:
    old_data = path.read_bytes()
    result = patch_thread_list_bytes(old_data)
    if result.changed_count == 0:
        print(_scan_message(path, result))
        return False

    print(f"Patch target: {path}")
    print(f"binary-safe thread/list provider filters changed: {result.changed_count}")
    if dry_run:
        print("Dry run only; file was not changed.")
        return True

    backup = backup_file(path, backup_dir)
    try:
        path.write_bytes(result.data)
    except PermissionError:
        print(f"Backup written: {backup}")
        print(f"Patch blocked by file permissions: {path}")
        return False
    print(f"Backup written: {backup}")
    print("Patch written. Fully quit and reopen the Codex desktop app.")
    return True


def _scan_message(path: Path, result: PatchResult) -> str:
    if result.has_thread_list and not result.has_provider_filter:
        return f"No provider-filtered thread/list request found: {path}"
    if result.has_provider_filter and not result.has_thread_list:
        return f"Provider filter found without thread/list anchor; skipped: {path}"
    if result.has_thread_list and result.has_provider_filter:
        return f"Provider filter did not match known main-list anchor; skipped: {path}"
    return f"No thread/list request found: {path}"


def revert_file(path: Path, dry_run: bool, backup_dir: Path | None = None) -> None:
    backup = newest_backup(path, backup_dir)
    print(f"Restore target: {path}")
    print(f"Newest backup: {backup}")
    if dry_run:
        print("Dry run only; file was not changed.")
        return
    shutil.copy2(backup, path)
    print("Backup restored. Restart or reload the Codex surface that owns this bundle.")


def collect_targets(args: argparse.Namespace) -> list[Path]:
    if args.target:
        return [target.resolve() for target in args.target]

    targets: list[Path] = []
    if not args.desktop_only:
        vscode = find_latest_vscode_extension_js(args.extensions_dir)
        if vscode is not None:
            targets.append(vscode)
    if args.include_desktop or args.desktop_only:
        targets.extend(find_desktop_text_targets(args.windowsapps_dir))
    return sorted(set(targets), key=lambda path: str(path).lower())


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Patch Codex thread/list requests to show records from all providers."
    )
    parser.add_argument(
        "--target",
        type=Path,
        action="append",
        help="Explicit text bundle to patch. Can be passed more than once.",
    )
    parser.add_argument(
        "--extensions-dir",
        type=Path,
        default=default_vscode_extensions_dir(),
        help="VS Code extensions directory used when --target is omitted.",
    )
    parser.add_argument(
        "--windowsapps-dir",
        type=Path,
        default=default_windowsapps_dir(),
        help="WindowsApps directory used for desktop Codex text-bundle scanning.",
    )
    parser.add_argument(
        "--backup-dir",
        type=Path,
        default=None,
        help="Directory for backups. Defaults to target directory, with a protected-directory fallback under ~/.codex.",
    )
    parser.add_argument(
        "--include-desktop",
        action="store_true",
        help="Also scan directly editable desktop Codex text bundles.",
    )
    parser.add_argument(
        "--desktop-only",
        action="store_true",
        help="Scan only directly editable desktop Codex text bundles.",
    )
    parser.add_argument("--dry-run", action="store_true", help="Show what would change.")
    parser.add_argument("--revert", action="store_true", help="Restore newest backup.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    targets = collect_targets(args)
    if not targets:
        raise FileNotFoundError("No Codex text bundle targets found.")

    for target in targets:
        if args.revert:
            revert_file(target, args.dry_run, args.backup_dir)
        else:
            patch_file(target, args.dry_run, args.backup_dir)


if __name__ == "__main__":
    main()
