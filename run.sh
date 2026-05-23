#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BLE_CONTROLLER_SCRIPT="${SCRIPT_DIR}/start_ble_bridge.sh"
BLE_BMS_SCRIPT="${SCRIPT_DIR}/start_ble_bms_bridge.sh"
PYTHON_SCRIPT="${SCRIPT_DIR}/speedometer.py"
CONTROLLER_TYPE="${CONTROLLER_TYPE:-fardriver}"
FARDRIVER_MASTER_MAC="${FARDRIVER_MASTER_MAC:-E0:00:AC:FB:00:23}"
FARDRIVER_SLAVE_MAC="${FARDRIVER_SLAVE_MAC:-C0:05:EA:1E:00:45}"
FARDRIVER_MASTER_PORT="${FARDRIVER_MASTER_PORT:-/tmp/fardriver-master-ble}"
FARDRIVER_SLAVE_PORT="${FARDRIVER_SLAVE_PORT:-/tmp/fardriver-slave-ble}"

bridge_pid_controller=""
bridge_pid_controller_slave=""
bridge_pid_bms=""

cleanup() {
  if [[ -n "${bridge_pid_bms}" ]]; then
    kill "${bridge_pid_bms}" 2>/dev/null || true
    wait "${bridge_pid_bms}" 2>/dev/null || true
    bridge_pid_bms=""
  fi
  if [[ -n "${bridge_pid_controller}" ]]; then
    kill "${bridge_pid_controller}" 2>/dev/null || true
    wait "${bridge_pid_controller}" 2>/dev/null || true
    bridge_pid_controller=""
  fi
  if [[ -n "${bridge_pid_controller_slave}" ]]; then
    kill "${bridge_pid_controller_slave}" 2>/dev/null || true
    wait "${bridge_pid_controller_slave}" 2>/dev/null || true
    bridge_pid_controller_slave=""
  fi
  rm -f "${FARDRIVER_MASTER_PORT}" "${FARDRIVER_SLAVE_PORT}"
}

trap cleanup EXIT

sleep 10

if [[ -x "${BLE_CONTROLLER_SCRIPT}" ]]; then
  if [[ "${CONTROLLER_TYPE}" == "fardriver" ]]; then
    echo "Запуск BLE-моста FarDriver master..."
    BLE_LABEL="FarDriver master" BLE_DEVICE_MAC="${FARDRIVER_MASTER_MAC}" BLE_VIRTUAL_PORT="${FARDRIVER_MASTER_PORT}" "${BLE_CONTROLLER_SCRIPT}" &
    bridge_pid_controller=$!

    sleep 2

    echo "Запуск BLE-моста FarDriver slave..."
    BLE_LABEL="FarDriver slave" BLE_DEVICE_MAC="${FARDRIVER_SLAVE_MAC}" BLE_VIRTUAL_PORT="${FARDRIVER_SLAVE_PORT}" "${BLE_CONTROLLER_SCRIPT}" &
    bridge_pid_controller_slave=$!

    sleep 5
  else
    echo "Запуск BLE-моста контроллера (${CONTROLLER_TYPE})..."
    CONTROLLER_TYPE="${CONTROLLER_TYPE}" "${BLE_CONTROLLER_SCRIPT}" &
    bridge_pid_controller=$!
  fi
else
  echo "Внимание: ${BLE_CONTROLLER_SCRIPT} не найден или не исполняемый. Пропускаю запуск BLE-моста контроллера." >&2
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
CONTROLLER_TYPE="${CONTROLLER_TYPE}" FARDRIVER_MASTER_PORT="${FARDRIVER_MASTER_PORT}" FARDRIVER_SLAVE_PORT="${FARDRIVER_SLAVE_PORT}" python3 "${PYTHON_SCRIPT}"
