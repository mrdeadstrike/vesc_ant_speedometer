#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_LOG="${RUN_LOG:-${SCRIPT_DIR}/run.log}"
BLE_CONTROLLER_SCRIPT="${SCRIPT_DIR}/start_ble_bridge.sh"
BLE_BMS_SCRIPT="${SCRIPT_DIR}/start_ble_bms_bridge.sh"
PYTHON_SCRIPT="${SCRIPT_DIR}/speedometer.py"
CONTROLLER_TYPE="${CONTROLLER_TYPE:-fardriver}"
FARDRIVER_BLE_BACKEND="${FARDRIVER_BLE_BACKEND:-bleak}"
FARDRIVER_MASTER_MAC="${FARDRIVER_MASTER_MAC:-}"
FARDRIVER_SLAVE_MAC="${FARDRIVER_SLAVE_MAC:-}"
FARDRIVER_MASTER_MACS="${FARDRIVER_MASTER_MACS:-}"
FARDRIVER_SLAVE_MACS="${FARDRIVER_SLAVE_MACS:-}"
FARDRIVER_MASTER_NAME="${FARDRIVER_MASTER_NAME:-YuanQuFOC158}"
FARDRIVER_SLAVE_NAME="${FARDRIVER_SLAVE_NAME:-YuanQuFOC690}"
FARDRIVER_NAME_PREFIX="${FARDRIVER_NAME_PREFIX:-YuanQuFOC}"
FARDRIVER_SCAN_SECONDS="${FARDRIVER_SCAN_SECONDS:-30}"
FARDRIVER_SCAN_STEP_SECONDS="${FARDRIVER_SCAN_STEP_SECONDS:-5}"
# The controllers rotate their BLE addresses. Python connects to fresh Bleak scan
# objects by exact advertisement name, so a separate bluetoothctl pre-scan only
# delays startup and fills the process with stale addresses.
FARDRIVER_PREFLIGHT_SCAN="${FARDRIVER_PREFLIGHT_SCAN:-0}"
FARDRIVER_MASTER_PORT="${FARDRIVER_MASTER_PORT:-/tmp/fardriver-master-ble}"
FARDRIVER_SLAVE_PORT="${FARDRIVER_SLAVE_PORT:-/tmp/fardriver-slave-ble}"

bridge_pid_controller=""
bridge_pid_controller_slave=""
bridge_pid_bms=""

exec > >(tee -a "${RUN_LOG}") 2>&1
echo "===== run.sh $(date '+%Y-%m-%d %H:%M:%S') ====="

pause_on_error() {
  local code="$1"
  echo "run.sh завершился с ошибкой ${code}. Лог: ${RUN_LOG}" >&2
  if [[ -t 0 ]]; then
    read -r -p "Нажми Enter, чтобы закрыть окно..." _ || true
  else
    sleep 20
  fi
}

trap 'code=$?; if [[ $code -ne 0 ]]; then pause_on_error "$code"; fi' ERR

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
  local known_output
  local current_output
  local found_lines
  local current_found_lines
  local master_macs
  local slave_macs
  local deadline
  local scan_pass

  echo "Ищу FarDriver BLE устройства по имени ${FARDRIVER_NAME_PREFIX} (${FARDRIVER_SCAN_SECONDS} с)..."
  scan_output=""
  deadline=$((SECONDS + FARDRIVER_SCAN_SECONDS))

  bluetoothctl scan off >/dev/null 2>&1 || true
  scan_pass=0

  while (( SECONDS < deadline )); do
    scan_pass=$((scan_pass + 1))
    echo "FarDriver preflight scan pass ${scan_pass}: bluetoothctl scan ${FARDRIVER_SCAN_STEP_SECONDS}s..."
    current_output="$(
      bluetoothctl --timeout "${FARDRIVER_SCAN_STEP_SECONDS}" scan on 2>&1 || true
    )"
    known_output="$(
      bluetoothctl devices 2>&1 || true
    )"
    current_found_lines="$(printf '%s\n%s\n' "${known_output}" "${current_output}" \
      | sed -nE "s/^.*Device ([0-9A-Fa-f:]{17}) (${FARDRIVER_NAME_PREFIX}[^[:space:]]*).*$/\U\1\E \2/p")"
    if [[ -n "${current_found_lines}" ]]; then
      echo "FarDriver preflight scan pass ${scan_pass}: fresh candidates:"
      printf '%s\n' "${current_found_lines}"
    else
      echo "FarDriver preflight scan pass ${scan_pass}: fresh candidates: none"
    fi

    scan_output="${scan_output}"$'\n'"${known_output}"$'\n'"${current_output}"
    found_lines="$(printf '%s\n' "${scan_output}" \
      | sed -nE "s/^.*Device ([0-9A-Fa-f:]{17}) (${FARDRIVER_NAME_PREFIX}[^[:space:]]*).*$/\U\1\E \2/p")"

    while read -r mac name; do
      [[ -z "${mac:-}" ]] && continue
      if [[ "${name}" == "${FARDRIVER_MASTER_NAME}" ]]; then
        FARDRIVER_MASTER_MAC="${mac}"
      elif [[ "${name}" == "${FARDRIVER_SLAVE_NAME}" ]]; then
        FARDRIVER_SLAVE_MAC="${mac}"
      fi
    done <<< "${found_lines}"

    if [[ -n "${FARDRIVER_MASTER_MAC}" && -n "${FARDRIVER_SLAVE_MAC}" ]]; then
      break
    fi
  done

  bluetoothctl scan off >/dev/null 2>&1 || true

  if [[ -z "${found_lines}" ]]; then
    echo "Не нашёл BLE-устройства с именем ${FARDRIVER_NAME_PREFIX}." >&2
    echo "Проверь: bluetoothctl scan on" >&2
    return 1
  fi

  echo "Найдены кандидаты FarDriver:"
  printf '%s\n' "${found_lines}"

  master_macs="$(printf '%s\n' "${found_lines}" | awk -v target="${FARDRIVER_MASTER_NAME}" '$2 == target { items[++count] = $1 } END { for (i = 1; i <= count; i++) if (!seen[items[i]]++) { out = out (out ? "," : "") items[i] } print out }')"
  slave_macs="$(printf '%s\n' "${found_lines}" | awk -v target="${FARDRIVER_SLAVE_NAME}" '$2 == target { items[++count] = $1 } END { for (i = 1; i <= count; i++) if (!seen[items[i]]++) { out = out (out ? "," : "") items[i] } print out }')"
  if [[ -n "${master_macs}" ]]; then
    FARDRIVER_MASTER_MACS="${master_macs}"
  fi
  if [[ -n "${slave_macs}" ]]; then
    FARDRIVER_SLAVE_MACS="${slave_macs}"
  fi

  if [[ -z "${FARDRIVER_MASTER_MAC}" || -z "${FARDRIVER_SLAVE_MAC}" ]]; then
    echo "Не нашёл оба FarDriver по именам: master=${FARDRIVER_MASTER_NAME}, slave=${FARDRIVER_SLAVE_NAME}." >&2
    return 1
  fi

  echo "FarDriver master: ${FARDRIVER_MASTER_NAME} ${FARDRIVER_MASTER_MAC}"
  echo "FarDriver slave : ${FARDRIVER_SLAVE_NAME} ${FARDRIVER_SLAVE_MAC}"
  echo "FarDriver master candidates: ${FARDRIVER_MASTER_MACS}"
  echo "FarDriver slave candidates : ${FARDRIVER_SLAVE_MACS}"
}

if [[ -x "${BLE_CONTROLLER_SCRIPT}" ]]; then
  if [[ "${CONTROLLER_TYPE}" == "fardriver" ]]; then
    if [[ "${FARDRIVER_BLE_BACKEND}" == "bleak" ]]; then
      if [[ "${FARDRIVER_PREFLIGHT_SCAN}" != "0" && ( -z "${FARDRIVER_MASTER_MAC}" || -z "${FARDRIVER_SLAVE_MAC}" ) ]]; then
        discover_fardriver_macs || echo "Предварительный bluetoothctl scan не нашёл оба контроллера, Python попробует найти их сам"
      fi
      echo "FarDriver BLE backend: bleak, BLE-мосты не запускаются"
      echo "Поиск контроллеров будет внутри Python по именам: master=${FARDRIVER_MASTER_NAME}, slave=${FARDRIVER_SLAVE_NAME}"
    else
      if [[ -z "${FARDRIVER_MASTER_MAC}" || -z "${FARDRIVER_SLAVE_MAC}" ]]; then
        discover_fardriver_macs
      fi

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
CONTROLLER_TYPE="${CONTROLLER_TYPE}" FARDRIVER_BLE_BACKEND="${FARDRIVER_BLE_BACKEND}" FARDRIVER_MASTER_NAME="${FARDRIVER_MASTER_NAME}" FARDRIVER_SLAVE_NAME="${FARDRIVER_SLAVE_NAME}" FARDRIVER_MASTER_MAC="${FARDRIVER_MASTER_MAC}" FARDRIVER_SLAVE_MAC="${FARDRIVER_SLAVE_MAC}" FARDRIVER_MASTER_MACS="${FARDRIVER_MASTER_MACS}" FARDRIVER_SLAVE_MACS="${FARDRIVER_SLAVE_MACS}" FARDRIVER_MASTER_PORT="${FARDRIVER_MASTER_PORT}" FARDRIVER_SLAVE_PORT="${FARDRIVER_SLAVE_PORT}" python3 "${PYTHON_SCRIPT}"
