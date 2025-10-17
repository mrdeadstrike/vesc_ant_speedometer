#!/bin/bash

set -euo pipefail

BLE_DEVICE_MAC="98:DE:1E:84:80:E6"
BLE_VIRTUAL_PORT="/tmp/bms-ble"

BLE_SERIAL_BIN="${BLE_SERIAL_BIN:-$(command -v ble-serial || echo "$HOME/.local/bin/ble-serial")}"

BLE_SERVICE_UUID="${BLE_SERVICE_UUID:-${BLE_SERIAL_SERVICE_UUID:-0000ffe0-0000-1000-8000-00805f9b34fb}}"
BLE_NOTIFY_CHARACTERISTIC="${BLE_NOTIFY_CHARACTERISTIC:-${BLE_SERIAL_NOTIFY_CHARACTERISTIC:-0000fff4-0000-1000-8000-00805f9b34fb}}"
BLE_WRITE_CHARACTERISTIC="${BLE_WRITE_CHARACTERISTIC:-${BLE_SERIAL_WRITE_CHARACTERISTIC:-0000fff2-0000-1000-8000-00805f9b34fb}}"
BLE_WRITE_WITHOUT_RESPONSE_CHARACTERISTIC="${BLE_WRITE_WITHOUT_RESPONSE_CHARACTERISTIC:-${BLE_SERIAL_WRITE_WITHOUT_RESPONSE_CHARACTERISTIC:-0000fff3-0000-1000-8000-00805f9b34fb}}"
BLE_ADAPTER="${BLE_ADAPTER:-}"

if [ ! -x "${BLE_SERIAL_BIN}" ]; then
  echo "ble-serial не найден. Установи его: pip install --user ble-serial" >&2
  exit 1
fi

ble_pid=""

cleanup() {
  if [ -n "${ble_pid}" ]; then
    kill "${ble_pid}" 2>/dev/null || true
    wait "${ble_pid}" 2>/dev/null || true
    ble_pid=""
  fi
}

trap cleanup INT TERM

if command -v bluetoothctl >/dev/null 2>&1; then
  info_output=$(bluetoothctl info "${BLE_DEVICE_MAC}" 2>/dev/null || true)
  if ! printf '%s' "${info_output}" | grep -q "Paired: yes"; then
    echo "⚠️  ${BLE_DEVICE_MAC} не спарено с хостом. Выполни 'bluetoothctl pair ${BLE_DEVICE_MAC}' (PIN/Passkey 12345678), затем 'trust'." >&2
    exit 1
  fi
  if ! printf '%s' "${info_output}" | grep -q "Trusted: yes"; then
    echo "⚠️  ${BLE_DEVICE_MAC} не помечено как доверенное. Добавляю trust..." >&2
    bluetoothctl trust "${BLE_DEVICE_MAC}" >/dev/null 2>&1 || true
  fi
fi

echo "Запускаю BLE-мост для BMS (${BLE_DEVICE_MAC}) на ${BLE_VIRTUAL_PORT}"
if [ -e "${BLE_VIRTUAL_PORT}" ]; then
  echo "Удаляю существующий порт ${BLE_VIRTUAL_PORT}"
  rm -f "${BLE_VIRTUAL_PORT}"
fi

args=(-d "${BLE_DEVICE_MAC}" -p "${BLE_VIRTUAL_PORT}")

if [ -n "${BLE_SERVICE_UUID}" ]; then
  args+=("-s" "${BLE_SERVICE_UUID}")
fi

if [ -n "${BLE_NOTIFY_CHARACTERISTIC}" ]; then
  args+=("-r" "${BLE_NOTIFY_CHARACTERISTIC}")
fi

if [ -n "${BLE_WRITE_WITHOUT_RESPONSE_CHARACTERISTIC}" ]; then
  args+=("-w" "${BLE_WRITE_WITHOUT_RESPONSE_CHARACTERISTIC}")
elif [ -n "${BLE_WRITE_CHARACTERISTIC}" ]; then
  args+=("-w" "${BLE_WRITE_CHARACTERISTIC}")
fi

# passkey для ble-serial не передаём: версия в системе не поддерживает аргумент

if [ -n "${BLE_ADAPTER}" ]; then
  args+=("--adapter" "${BLE_ADAPTER}")
fi

if [ -n "${BLE_SERIAL_FLAGS:-}" ]; then
  # shellcheck disable=SC2206
  extra=(${BLE_SERIAL_FLAGS})
  args+=("${extra[@]}")
fi

set +e
"${BLE_SERIAL_BIN}" "${args[@]}" &
ble_pid=$!
wait "${ble_pid}"
exit_code=$?
set -e

echo "BLE-мост BMS завершён (код ${exit_code})"
exit "${exit_code}"
