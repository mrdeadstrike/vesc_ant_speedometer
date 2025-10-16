#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BLE_VESC_SCRIPT="${SCRIPT_DIR}/start_ble_bridge.sh"
BLE_BMS_SCRIPT="${SCRIPT_DIR}/start_ble_bms_bridge.sh"
PYTHON_SCRIPT="${SCRIPT_DIR}/speedometer.py"

bridge_pid_vesc=""
bridge_pid_bms=""

cleanup() {
  if [[ -n "${bridge_pid_bms}" ]]; then
    kill "${bridge_pid_bms}" 2>/dev/null || true
    wait "${bridge_pid_bms}" 2>/dev/null || true
    bridge_pid_bms=""
  fi
  if [[ -n "${bridge_pid_vesc}" ]]; then
    kill "${bridge_pid_vesc}" 2>/dev/null || true
    wait "${bridge_pid_vesc}" 2>/dev/null || true
    bridge_pid_vesc=""
  fi
}

trap cleanup EXIT

sleep 10

if [[ -x "${BLE_VESC_SCRIPT}" ]]; then
  echo "Запуск BLE-моста VESC..."
  "${BLE_VESC_SCRIPT}" &
  bridge_pid_vesc=$!
else
  echo "Внимание: ${BLE_VESC_SCRIPT} не найден или не исполняемый. Пропускаю запуск BLE-моста VESC." >&2
fi

# sleep 3
#
# if [[ -x "${BLE_BMS_SCRIPT}" ]]; then
#   echo "Запуск BLE-моста BMS..."
#   "${BLE_BMS_SCRIPT}" &
#   bridge_pid_bms=$!
# else
#   echo "Внимание: ${BLE_BMS_SCRIPT} не найден или не исполняемый. Пропускаю запуск BLE-моста BMS." >&2
# fi
#
# sleep 3

echo "Запуск основного приложения..."
python3 "${PYTHON_SCRIPT}"
