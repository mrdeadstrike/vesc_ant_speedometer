#!/bin/bash

set -euo pipefail

BLE_DEVICE_MAC="98:DE:1E:84:80:E6"
BLE_VIRTUAL_PORT="/tmp/bms-ble"

BLE_SERIAL_BIN="${BLE_SERIAL_BIN:-$(command -v ble-serial || echo "$HOME/.local/bin/ble-serial")}"

BLE_NOTIFY_CHARACTERISTIC="${BLE_NOTIFY_CHARACTERISTIC:-${BLE_SERIAL_NOTIFY_CHARACTERISTIC:-}}"
BLE_WRITE_CHARACTERISTIC="${BLE_WRITE_CHARACTERISTIC:-${BLE_SERIAL_WRITE_CHARACTERISTIC:-}}"
BLE_WRITE_WITHOUT_RESPONSE_CHARACTERISTIC="${BLE_WRITE_WITHOUT_RESPONSE_CHARACTERISTIC:-${BLE_SERIAL_WRITE_WITHOUT_RESPONSE_CHARACTERISTIC:-}}"
BLE_ADAPTER="${BLE_ADAPTER:-}"

if [ ! -x "${BLE_SERIAL_BIN}" ]; then
  echo "ble-serial не найден. Установи его: pip install --user ble-serial" >&2
  exit 1
fi

stop_requested=0
ble_pid=""

cleanup() {
  stop_requested=1
  if [ -n "${ble_pid}" ]; then
    kill "${ble_pid}" 2>/dev/null || true
  fi
}

trap cleanup INT TERM

while true; do
  if command -v bluetoothctl >/dev/null 2>&1; then
    info_output=$(bluetoothctl info "${BLE_DEVICE_MAC}" 2>/dev/null || true)
    if ! printf '%s' "${info_output}" | grep -q "Paired: yes"; then
      echo "⚠️  ${BLE_DEVICE_MAC} не спарено с хостом. Выполни 'bluetoothctl pair ${BLE_DEVICE_MAC}' (PIN/Passkey 12345678), затем 'trust'." >&2
      sleep 5
      continue
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

  if [ -n "${BLE_NOTIFY_CHARACTERISTIC}" ]; then
    args+=("--notify-characteristic" "${BLE_NOTIFY_CHARACTERISTIC}")
  fi

  if [ -n "${BLE_WRITE_CHARACTERISTIC}" ]; then
    args+=("--write-characteristic" "${BLE_WRITE_CHARACTERISTIC}")
  fi

  if [ -n "${BLE_WRITE_WITHOUT_RESPONSE_CHARACTERISTIC}" ]; then
    args+=("--write-without-response-characteristic" "${BLE_WRITE_WITHOUT_RESPONSE_CHARACTERISTIC}")
  fi

  # passkey для ble-serial не передаём: некоторые версии не поддерживают аргумент

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
  if [ "${stop_requested}" -eq 1 ]; then
    break
  fi
  echo "BLE-мост BMS отключён (код ${exit_code}), переподключаем через 2 секунды..."
  sleep 2
done
