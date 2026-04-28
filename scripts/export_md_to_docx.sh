#!/usr/bin/env bash
set -euo pipefail

export PATH="/usr/local/bin:/usr/bin:/bin:${PATH:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

EXPORT_ROOT="${REPO_ROOT}/exports/word"
SINGLE_DIR="${EXPORT_ROOT}/single"
COMBINED_DIR="${EXPORT_ROOT}/combined"
LOG_DIR="${EXPORT_ROOT}/logs"
LOG_FILE="${LOG_DIR}/export_md_to_docx.log"
COMBINED_DOCX="${COMBINED_DIR}/robot_motion_control_job_project_docs.docx"

mkdir -p "${SINGLE_DIR}" "${COMBINED_DIR}" "${LOG_DIR}"

timestamp="$(date '+%Y-%m-%d %H:%M:%S %z')"

log() {
  printf '%s\n' "$*" | tee -a "${LOG_FILE}"
}

relative_path() {
  local path="$1"
  printf '%s\n' "${path#"${REPO_ROOT}/"}"
}

safe_docx_name() {
  local rel="$1"
  local safe="${rel//\//__}"
  safe="${safe%.md}.docx"
  printf '%s\n' "${safe}"
}

append_file_if_exists() {
  local file="$1"
  local -n out_array="$2"
  if [[ -f "${file}" ]]; then
    out_array+=("${file}")
  else
    SKIPPED_FILES+=("$(relative_path "${file}") | missing file")
  fi
}

append_find_results() {
  local dir="$1"
  local maxdepth_arg="$2"
  local -n out_array="$3"

  if [[ ! -d "${dir}" ]]; then
    SKIPPED_FILES+=("$(relative_path "${dir}") | missing directory")
    return
  fi

  while IFS= read -r file; do
    out_array+=("${file}")
  done < <(find "${dir}" ${maxdepth_arg} -type f -name '*.md' | sort)
}

print_pandoc_install_help() {
  cat <<'EOF'

Pandoc is required to export Markdown to Word, but it was not found.

Install it manually, then rerun this script:

  Ubuntu/Debian:
    sudo apt-get update
    sudo apt-get install -y pandoc

  macOS:
    brew install pandoc

  Windows:
    winget install --id JohnMacFarlane.Pandoc

EOF
}

resolve_pandoc() {
  if command -v pandoc >/dev/null 2>&1; then
    command -v pandoc
    return 0
  fi

  if command -v python >/dev/null 2>&1; then
    local pypandoc_path
    pypandoc_path="$(python -c 'import os, pypandoc; p=pypandoc.get_pandoc_path(); print(p if os.path.exists(p) else p + ".exe")' 2>/dev/null || true)"
    if [[ -n "${pypandoc_path}" && -f "${pypandoc_path}" ]]; then
      printf '%s\n' "${pypandoc_path}"
      return 0
    fi

    if command -v cygpath >/dev/null 2>&1; then
      local msys_path
      msys_path="$(cygpath -u "${pypandoc_path}" 2>/dev/null || true)"
      if [[ -n "${msys_path}" && -f "${msys_path}" ]]; then
        printf '%s\n' "${msys_path}"
        return 0
      fi
    fi
  fi

  return 1
}

MD_FILES=()
COMBINED_FILES=()
SKIPPED_FILES=()
SUCCESS_FILES=()
FAILED_FILES=()

append_file_if_exists "${REPO_ROOT}/README.md" MD_FILES
append_find_results "${REPO_ROOT}/docs" "" MD_FILES
append_find_results "${REPO_ROOT}/external" "-maxdepth 1" MD_FILES

append_file_if_exists "${REPO_ROOT}/README.md" COMBINED_FILES
append_find_results "${REPO_ROOT}/docs/00_preparation" "-maxdepth 1" COMBINED_FILES
append_find_results "${REPO_ROOT}/docs/01_self_baseline" "-maxdepth 1" COMBINED_FILES
append_find_results "${REPO_ROOT}/docs/02_legged_control" "-maxdepth 1" COMBINED_FILES
append_find_results "${REPO_ROOT}/docs/03_unitree_rl_mjlab" "-maxdepth 1" COMBINED_FILES
append_find_results "${REPO_ROOT}/docs/04_compare" "-maxdepth 1" COMBINED_FILES
append_find_results "${REPO_ROOT}/docs/interview" "-maxdepth 1" COMBINED_FILES
append_find_results "${REPO_ROOT}/external" "-maxdepth 1" COMBINED_FILES

: > "${LOG_FILE}"
log "Markdown to Word export"
log "Export time: ${timestamp}"
log "Repository root: ${REPO_ROOT}"
log "Single output directory: ${SINGLE_DIR}"
log "Combined output path: ${COMBINED_DOCX}"
log ""

PANDOC_BIN="$(resolve_pandoc || true)"

if [[ -z "${PANDOC_BIN}" ]]; then
  log "Pandoc status: missing"
  log ""
  log "Skipped files:"
  if [[ ${#MD_FILES[@]} -eq 0 ]]; then
    log "  - none found"
  else
    for src in "${MD_FILES[@]}"; do
      log "  - $(relative_path "${src}") | pandoc missing"
    done
  fi
  log ""
  log "Successful files:"
  log "  - none"
  log ""
  log "Failed files:"
  log "  - none"
  log ""
  log "Combined document:"
  log "  - not generated | pandoc missing"
  print_pandoc_install_help
  exit 1
fi

log "Pandoc status: ${PANDOC_BIN}"
log ""
log "Starting single-file exports..."

for src in "${MD_FILES[@]}"; do
  rel="$(relative_path "${src}")"
  out="${SINGLE_DIR}/$(safe_docx_name "${rel}")"
  printf 'Exporting %s -> %s\n' "${rel}" "$(relative_path "${out}")"

  if "${PANDOC_BIN}" "${src}" --from markdown --to docx --output "${out}"; then
    SUCCESS_FILES+=("${rel} -> $(relative_path "${out}")")
  else
    FAILED_FILES+=("${rel} -> $(relative_path "${out}")")
  fi
done

log ""
log "Starting combined export..."
if [[ ${#COMBINED_FILES[@]} -eq 0 ]]; then
  SKIPPED_FILES+=("combined document | no markdown files found")
else
  if "${PANDOC_BIN}" "${COMBINED_FILES[@]}" --from markdown --to docx --output "${COMBINED_DOCX}"; then
    SUCCESS_FILES+=("combined -> $(relative_path "${COMBINED_DOCX}")")
  else
    FAILED_FILES+=("combined -> $(relative_path "${COMBINED_DOCX}")")
  fi
fi

log ""
log "Successful files:"
if [[ ${#SUCCESS_FILES[@]} -eq 0 ]]; then
  log "  - none"
else
  for item in "${SUCCESS_FILES[@]}"; do
    log "  - ${item}"
  done
fi

log ""
log "Skipped files:"
if [[ ${#SKIPPED_FILES[@]} -eq 0 ]]; then
  log "  - none"
else
  for item in "${SKIPPED_FILES[@]}"; do
    log "  - ${item}"
  done
fi

log ""
log "Failed files:"
if [[ ${#FAILED_FILES[@]} -eq 0 ]]; then
  log "  - none"
else
  for item in "${FAILED_FILES[@]}"; do
    log "  - ${item}"
  done
fi

log ""
log "Combined document:"
if [[ -f "${COMBINED_DOCX}" ]]; then
  log "  - $(relative_path "${COMBINED_DOCX}")"
else
  log "  - not generated"
fi

log ""
log "Generated Word paths:"
find "${SINGLE_DIR}" "${COMBINED_DIR}" -type f -name '*.docx' | sort | while IFS= read -r docx; do
  log "  - $(relative_path "${docx}")"
done

if [[ ${#FAILED_FILES[@]} -gt 0 ]]; then
  exit 1
fi
