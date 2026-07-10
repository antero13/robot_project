#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PIDS=()

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if ((${#PIDS[@]})); then
    kill -TERM "${PIDS[@]}" 2>/dev/null || true
    wait "${PIDS[@]}" 2>/dev/null || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

bash "$SCRIPT_DIR/record_once.sh" camera1 &
PIDS+=("$!")
bash "$SCRIPT_DIR/record_once.sh" camera2 &
PIDS+=("$!")

status=0
set +e
for pid in "${PIDS[@]}"; do
  wait "$pid"
  child_status=$?
  if ((child_status != 0)); then
    status=$child_status
  fi
done
set -e

PIDS=()
trap - EXIT INT TERM
exit "$status"
