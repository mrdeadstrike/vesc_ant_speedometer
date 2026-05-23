#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BLE_CONTROLLER_SCRIPT="${SCRIPT_DIR}/start_ble_bridge.sh"
BLE_BMS_SCRIPT="${SCRIPT_DIR}/start_ble_bms_bridge.sh"
PYTHON_SCRIPT="${SCRIPT_DIR}/speedometer.py"
CONTROLLER_TYPE="${CONTROLLER_TYPE:-fardriver}"
FARDRIVER_BLE_BACKEND="${FARDRIVER_BLE_BACKEND:-bleak}"
FARDRIVER_MASTER_MAC="${FARDRIVER_MASTER_MAC:-}"
FARDRIVER_SLAVE_MAC="${FARDRIVER_SLAVE_MAC:-}"
FARDRIVER_NAME_PREFIX="${FARDRIVER_NAME_PREFIX:-YuanQuFOC}"
FARDRIVER_SCAN_SECONDS="${FARDRIVER_SCAN_SECONDS:-20}"
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

discover_fardriver_macs() {
  local scan_output
  local found_lines
  local fallback_one=""
  local fallback_two=""

  echo "Ищу FarDriver BLE устройства по имени ${FARDRIVER_NAME_PREFIX} (${FARDRIVER_SCAN_SECONDS} с)..."
  scan_output="$(
    {
      bluetoothctl devices 2>/dev/null || true
      bluetoothctl --timeout "${FARDRIVER_SCAN_SECONDS}" scan on 2>/dev/null || true
    }
  )"

  found_lines="$(printf '%s\n' "${scan_output}" \
    | sed -nE "s/^.*Device ([0-9A-Fa-f:]{17}) (${FARDRIVER_NAME_PREFIX}[^[:space:]]*).*$/\U\1\E \2/p" \
    | awk '!seen[$1]++')"

  if [[ -z "${found_lines}" ]]; then
    echo "Не нашёл BLE-устройства с именем ${FARDRIVER_NAME_PREFIX}." >&2
    echo "Проверь: bluetoothctl scan on" >&2
    return 1
  fi

  echo "Найдены кандидаты FarDriver:"
  printf '%s\n' "${found_lines}"

  while read -r mac name; do
    [[ -z "${mac:-}" ]] && continue
    if [[ -z "${fallback_one}" ]]; then
      fallback_one="${mac}"
    elif [[ -z "${fallback_two}" && "${mac}" != "${fallback_one}" ]]; then
      fallback_two="${mac}"
    fi

    if [[ -z "${FARDRIVER_MASTER_MAC}" && "${mac}" == E0:* ]]; then
      FARDRIVER_MASTER_MAC="${mac}"
    elif [[ -z "${FARDRIVER_SLAVE_MAC}" && "${mac}" == C0:* ]]; then
      FARDRIVER_SLAVE_MAC="${mac}"
    fi
  done <<< "${found_lines}"

  FARDRIVER_MASTER_MAC="${FARDRIVER_MASTER_MAC:-${fallback_one}}"
  if [[ -z "${FARDRIVER_SLAVE_MAC}" || "${FARDRIVER_SLAVE_MAC}" == "${FARDRIVER_MASTER_MAC}" ]]; then
    FARDRIVER_SLAVE_MAC="${fallback_two}"
  fi

  if [[ -z "${FARDRIVER_MASTER_MAC}" || -z "${FARDRIVER_SLAVE_MAC}" ]]; then
    echo "Нужно два FarDriver BLE-устройства, найдено меньше двух." >&2
    return 1
  fi

  echo "FarDriver master MAC: ${FARDRIVER_MASTER_MAC}"
  echo "FarDriver slave  MAC: ${FARDRIVER_SLAVE_MAC}"
}

if [[ -x "${BLE_CONTROLLER_SCRIPT}" ]]; then
  if [[ "${CONTROLLER_TYPE}" == "fardriver" ]]; then
    if [[ -z "${FARDRIVER_MASTER_MAC}" || -z "${FARDRIVER_SLAVE_MAC}" ]]; then
      discover_fardriver_macs
    fi

    if [[ "${FARDRIVER_BLE_BACKEND}" == "bleak" ]]; then
      echo "FarDriver BLE backend: bleak, BLE-мосты не запускаются"
    else
      echo "Запуск BLE-моста FarDriver master..."
      BLE_LABEL="FarDriver master" BLE_DEVICE_MAC="${FARDRIVER_MASTER_MAC}" BLE_VIRTUAL_PORT="${FARDRIVER_MASTER_PORT}" "${BLE_CONTROLLER_SCRIPT}" &
      bridge_pid_controller=$!

      sleep 2

      echo "Запуск BLE-моста FarDriver slave..."
      BLE_LABEL="FarDriver slave" BLE_DEVICE_MAC="${FARDRIVER_SLAVE_MAC}" BLE_VIRTUAL_PORT="${FARDRIVER_SLAVE_PORT}" "${BLE_CONTROLLER_SCRIPT}" &
      bridge_pid_controller_slave=$!

      sleep 5
    fi
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
CONTROLLER_TYPE="${CONTROLLER_TYPE}" FARDRIVER_BLE_BACKEND="${FARDRIVER_BLE_BACKEND}" FARDRIVER_MASTER_MAC="${FARDRIVER_MASTER_MAC}" FARDRIVER_SLAVE_MAC="${FARDRIVER_SLAVE_MAC}" FARDRIVER_MASTER_PORT="${FARDRIVER_MASTER_PORT}" FARDRIVER_SLAVE_PORT="${FARDRIVER_SLAVE_PORT}" python3 "${PYTHON_SCRIPT}"
