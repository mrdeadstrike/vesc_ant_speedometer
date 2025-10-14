#!/bin/bash

set -euo pipefail

BLE_DEVICE_MAC="98:DE:1E:84:80:E6"
BLE_VIRTUAL_PORT="/tmp/bms-ble"

BLE_SERIAL_BIN="${BLE_SERIAL_BIN:-$(command -v ble-serial || echo "$HOME/.local/bin/ble-serial")}"

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
  echo "Запускаю BLE-мост для BMS (${BLE_DEVICE_MAC}) на ${BLE_VIRTUAL_PORT}"
  if [ -e "${BLE_VIRTUAL_PORT}" ]; then
    echo "Удаляю существующий порт ${BLE_VIRTUAL_PORT}"
    rm -f "${BLE_VIRTUAL_PORT}"
  fi
  set +e
  "${BLE_SERIAL_BIN}" -d "${BLE_DEVICE_MAC}" -p "${BLE_VIRTUAL_PORT}" &
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
