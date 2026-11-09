#!/usr/bin/env bash
# Copyright 2026 The RPent Authors.
# Licensed under the Apache License, Version 2.0.

set -euo pipefail

# 修改这里选择模式：false = 归位预览；true = 按对话指令执行真机动作。
LIVE=false
UV_BIN="/opt/uv/uv"
REASONING_EFFORT="low"

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd -- "$PROJECT_DIR"

mode_args=()
case "$LIVE" in
    true) mode_args+=(--live) ;;
    false) ;;
    *) printf 'LIVE 必须设为 true 或 false。\n' >&2; exit 2 ;;
esac

if [[ ! -x "$UV_BIN" ]]; then
    printf '找不到 uv：%s，请修改脚本中的 UV_BIN。\n' "$UV_BIN" >&2
    exit 1
fi

exec "$UV_BIN" run --no-sync rpent \
    --robot seer_dual_arm --planner api \
    --reasoning-effort "$REASONING_EFFORT" --tui \
    "${mode_args[@]}" "$@"
