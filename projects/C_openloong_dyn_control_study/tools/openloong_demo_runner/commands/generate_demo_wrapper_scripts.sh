#!/usr/bin/env bash
set -euo pipefail

# 根据总入口脚本支持的 demo 列表，生成每个 demo 的薄包装脚本。

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
GENERATED_DIR="$SCRIPT_DIR/generated"
RUNNER="$SCRIPT_DIR/run_openloong_demo_host.sh"

mkdir -p "$GENERATED_DIR"

while IFS= read -r demo_name; do
    wrapper_path="$GENERATED_DIR/run_${demo_name}_host.sh"
    cat > "$wrapper_path" <<EOF
#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=\$(cd "\$(dirname "\${BASH_SOURCE[0]}")" && pwd)
bash "\$SCRIPT_DIR/../run_openloong_demo_host.sh" "$demo_name" "\$@"
EOF
    chmod +x "$wrapper_path"
done < <(bash "$RUNNER" --list)

echo "Generated wrappers in: $GENERATED_DIR"
