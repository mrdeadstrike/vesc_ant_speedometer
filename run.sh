#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BLE_SCRIPT="${SCRIPT_DIR}/start_ble_bridge.sh"
PYTHON_SCRIPT="${SCRIPT_DIR}/speedometer.py"

bridge_pid=""

cleanup() {
  if [[ -n "${bridge_pid}" ]]; then
    kill "${bridge_pid}" 2>/dev/null || true
    wait "${bridge_pid}" 2>/dev/null || true
    bridge_pid=""
  fi
}

trap cleanup EXIT

sleep 10

if [[ -x "${BLE_SCRIPT}" ]]; then
  echo "Запуск BLE-моста..."
  "${BLE_SCRIPT}" &
  bridge_pid=$!
  sleep 10
else
  echo "Внимание: ${BLE_SCRIPT} не найден или не исполняемый. Пропускаю запуск BLE-моста." >&2
fi

echo "Запуск основного приложения..."
python3 "${PYTHON_SCRIPT}"
