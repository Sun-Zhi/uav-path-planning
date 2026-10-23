#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WEB="$ROOT/web"
DOCS="$ROOT/docs"
mkdir -p "$DOCS"

if ! command -v emcc >/dev/null 2>&1; then
  if [[ ! -d /workspace/emsdk ]]; then
    git clone --depth 1 https://github.com/emscripten-core/emsdk.git /workspace/emsdk
  fi
  # shellcheck disable=SC1091
  cd /workspace/emsdk
  ./emsdk install latest
  ./emsdk activate latest
  # shellcheck disable=SC1091
  source ./emsdk_env.sh
  cd "$ROOT"
else
  # Ensure env is loaded if emsdk exists
  if [[ -f /workspace/emsdk/emsdk_env.sh ]]; then
    # shellcheck disable=SC1091
    source /workspace/emsdk/emsdk_env.sh
  fi
fi

echo "Using emcc: $(command -v emcc)"
emcc --version | head -1

em++ "$WEB/planner_bind.cpp" \
  -O2 \
  -std=c++17 \
  -I"$ROOT/include" \
  --bind \
  -sMODULARIZE=1 \
  -sEXPORT_NAME=createPlanner \
  -sALLOW_MEMORY_GROWTH=1 \
  -sENVIRONMENT=web \
  -o "$DOCS/planner.js"

cp "$ROOT/configs/sample_map.json" "$DOCS/sample_map.json"
echo "Built: $DOCS/planner.js $DOCS/planner.wasm"
ls -la "$DOCS/planner.js" "$DOCS/planner.wasm" "$DOCS/sample_map.json"
