# E04 Download Tool Check

| Tool / Check | Status | Notes |
|---|---|---|
| `git` | present | `/usr/bin/git`, version `2.51.0` |
| `git-lfs` | missing | `git: 'lfs' is not a git command` |
| `huggingface-cli` | missing | not found in PATH |
| `hf` | missing | not found in PATH |
| `wget` | present and validated | successfully downloaded the previously failing Centauro checkpoint in direct test |
| `curl` | present | usable for HF tree API metadata queries |
| `python3` | present | stdlib hashing / manifest generation ok |

## E04 Interpretation

- This retry pass uses `wget` as the primary per-file downloader.
- Missing files are attempted individually; failures are recorded and do not stop later files.
