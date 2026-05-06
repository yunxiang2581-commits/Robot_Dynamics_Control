# 低风险删除候选清单

本文件只列出低风险删除候选。当前不执行删除。

## 可以作为低风险删除候选的内容

### 1. .pytest_cache/

原因：

- pytest 运行缓存。
- 可由测试重新生成。

注意：

- 清理前确认没有人工保存的测试说明混入其中。

### 2. __pycache__/

原因：

- Python 字节码缓存目录。
- 可由 Python 重新生成。

注意：

- 只清理缓存目录，不删除对应 `.py` 源码。

### 3. *.pyc

原因：

- Python 编译缓存文件。
- 可重新生成。

注意：

- 只删除 `.pyc`，不删除 `.py`。

### 4. debug.log

原因：

- 根目录临时日志候选。

注意：

- 删除前人工确认其中没有重要调试记录。

### 5. 明确临时缓存

候选类型：

- `.mypy_cache/`
- `.ruff_cache/`
- `.ipynb_checkpoints/`
- 明确标记为 cache 的临时运行产物。

## 禁止列入删除候选的内容

以下内容本轮不能列入删除候选：

- `.md`
- `.py`
- `.cpp`
- `.h`
- configs
- tests
- `outputs/videos`
- `metrics.json`
- `projects/A_self_baseline`
- `projects/B_mujoco_mpc_study`
- `projects/C_openloong_dyn_control_study`
- `projects/D_legged_control_study`
- `external/open_source_repos`
- `external/mink_upstream`
- `shared/robot_assets`
- `translated-pdfs`

## 执行原则

缓存清理必须单独执行，不能和文档归档、源码整理混在同一次操作中。
