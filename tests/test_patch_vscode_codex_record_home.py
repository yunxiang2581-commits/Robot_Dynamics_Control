from __future__ import annotations

import importlib.util
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "tools" / "patch_vscode_codex_record_home.py"
SPEC = importlib.util.spec_from_file_location("patch_vscode_codex_record_home", SCRIPT_PATH)
patcher = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(patcher)


HOST_SPAWN_SNIPPET = (
    'function gce(t,e,r){let n=process.platform==="win32",o=Q6e();'
    'if(ir())return hce().info("Spawning codex process inside WSL"),X6e(t,e,r,o);'
    'let i=ip(),s=ZR(t),a=n?";":";",c=process.env.PATH+a+tl.Uri.joinPath(t,i).fsPath,'
    'l=[e,...r],u=(0,_L.spawn)(s,l,{stdio:["pipe","pipe","pipe"],'
    'env:{...process.env,...o,PATH:c,RUST_LOG:"warn",CODEX_INTERNAL_ORIGINATOR_OVERRIDE:op}});'
    'if(u.pid==null)throw new sp(`Failed to spawn codex mcp process at ${s}`,{signal:null,exitCode:null});'
    'return u}'
)

WSL_SNIPPET = (
    'function X6e(t,e,r,n){let o=vs();'
    'let i=ip("linux"),s=ar(ZR(t,"linux")),a=tl.Uri.joinPath(t,i).fsPath,c=ar(a),'
    'l=tl.workspace.workspaceFolders?.[0]?.uri.fsPath,u=l?ar(l):void 0,'
    'd=[`PATH=${c}:$PATH`,"RUST_LOG=warn",`CODEX_INTERNAL_ORIGINATOR_OVERRIDE=${op}`],f=["-d",o];'
    'u&&f.push("--cd",u),f.push("--","/usr/bin/bash","-lc");'
    'let h=["/usr/bin/env",...d,s,e];h.push(...r);}'
)


def test_patch_injects_fixed_codex_home_before_app_server_spawn() -> None:
    text = HOST_SPAWN_SNIPPET + WSL_SNIPPET

    patched, applied = patcher.apply_text_patch(text, Path("C:/Users/Administrator/.codex"))

    assert "host CODEX_HOME" in applied
    assert "process.env.CODEX_HOME=process.env.CODEX_VSCODE_GLOBAL_CODEX_HOME" in patched
    assert "C:\\\\Users\\\\Administrator\\\\.codex" in patched
    assert patched.index("process.env.CODEX_HOME") < patched.index("function gce")


def test_patch_passes_codex_home_into_wsl_command_environment() -> None:
    text = HOST_SPAWN_SNIPPET + WSL_SNIPPET

    patched, applied = patcher.apply_text_patch(text, Path("C:/Users/Administrator/.codex"))

    assert "WSL CODEX_HOME" in applied
    assert "process.env.CODEX_HOME&&d.push(`CODEX_HOME=${ar(process.env.CODEX_HOME)}`);" in patched
    assert 'let h=["/usr/bin/env",...d,s,e]' in patched


def test_patch_is_idempotent() -> None:
    text = HOST_SPAWN_SNIPPET + WSL_SNIPPET

    once, _ = patcher.apply_text_patch(text, Path("C:/Users/Administrator/.codex"))
    twice, _ = patcher.apply_text_patch(once, Path("C:/Users/Administrator/.codex"))

    assert twice == once
    assert twice.count(patcher.ENV_START) == 1
    assert twice.count(patcher.WSL_START) == 1
