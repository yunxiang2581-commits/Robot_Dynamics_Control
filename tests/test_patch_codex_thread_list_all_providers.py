from __future__ import annotations

import importlib.util
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "tools"
    / "patch_codex_thread_list_all_providers.py"
)
SPEC = importlib.util.spec_from_file_location(
    "patch_codex_thread_list_all_providers", SCRIPT_PATH
)
patcher = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(patcher)


MAIN_LIST_SNIPPET = (
    'P$=class{constructor(e){this.codexAppServer=e;let r={onResult:this.onAppServerResult.bind(this)};'
    'e.registerProvider(_le,r)}nextRequestId=0;requestToCallback=new Map;'
    'requestThreadList(e){let r=String(this.nextRequestId++),n=new Promise((o,i)=>{});'
    'return this.codexAppServer.sendRequest(_le,r,"thread/list",'
    '{limit:50,cursor:null,sortKey:"created_at",modelProviders:e?[HS]:null,'
    'archived:!1,sourceKinds:Yf}),n}};'
)


PREVIEW_LOADER_SNIPPET = (
    'GE=class{requestThreadList(e=50){let r=String(this.nextRequestId++),'
    'n=new Promise((i,s)=>{}),o={limit:e,cursor:null,sortKey:"created_at",'
    'modelProviders:[],archived:!1,sourceKinds:Yf};'
    'return this.codexMcpConnection.sendRequest(Hle,r,"thread/list",o),n}};'
)

DESKTOP_RECENT_SNIPPET = (
    "listRecentThreads({cursor:e,limit:t,useStateDbOnly:n=!1}){"
    "return this.params.requestClient.sendRequest(`thread/list`,"
    "{limit:t,cursor:e,sortKey:this.recentConversationSortKey,"
    "modelProviders:null,archived:!1,sourceKinds:Ot,useStateDbOnly:n})}"
)


def test_patch_main_chat_session_provider_uses_all_model_providers() -> None:
    patched = patcher.patch_thread_list_text(MAIN_LIST_SNIPPET)

    assert patched.changed_count == 1
    assert 'modelProviders:[]' in patched.text
    assert 'modelProviders:e?[HS]:null' not in patched.text
    assert 'sendRequest(_le,r,"thread/list"' in patched.text


def test_patch_leaves_already_all_provider_preview_loader_unchanged() -> None:
    patched = patcher.patch_thread_list_text(PREVIEW_LOADER_SNIPPET)

    assert patched.changed_count == 0
    assert patched.text == PREVIEW_LOADER_SNIPPET


def test_patch_desktop_null_provider_filter_uses_equal_length_all_provider_value() -> None:
    patched = patcher.patch_thread_list_text(DESKTOP_RECENT_SNIPPET)

    assert patched.changed_count == 1
    assert "modelProviders:[]  " in patched.text
    assert "modelProviders:null" not in patched.text
    assert len(patched.text) == len(DESKTOP_RECENT_SNIPPET)


def test_patch_leaves_unrelated_null_provider_filter_unchanged() -> None:
    text = "const query={modelProviders:null,sortKey:`updated_at`};"

    patched = patcher.patch_thread_list_text(text)

    assert patched.changed_count == 0
    assert patched.text == text


def test_patch_bytes_uses_equal_length_replacement_for_asar_like_content() -> None:
    original = DESKTOP_RECENT_SNIPPET.encode("utf-8")

    patched = patcher.patch_thread_list_bytes(original)

    assert patched.changed_count == 1
    assert b"modelProviders:[]  " in patched.data
    assert b"modelProviders:null" not in patched.data
    assert len(patched.data) == len(original)


def test_patch_is_idempotent() -> None:
    once = patcher.patch_thread_list_text(MAIN_LIST_SNIPPET)
    twice = patcher.patch_thread_list_text(once.text)

    assert once.changed_count == 1
    assert twice.changed_count == 0
    assert twice.text == once.text
    assert twice.text.count('modelProviders:[]') == 1


def test_patch_reports_missing_thread_list_anchor() -> None:
    patched = patcher.patch_thread_list_text("const unrelated = true;")

    assert patched.changed_count == 0
    assert patched.has_thread_list is False
    assert patched.has_provider_filter is False
