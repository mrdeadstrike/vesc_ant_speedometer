#!/bin/bash

set -euo pipefail

BLE_DEVICE_MAC="98:DE:1E:84:80:E6"
BLE_VIRTUAL_PORT="/tmp/bms-ble"

BLE_SERIAL_BIN="${BLE_SERIAL_BIN:-$(command -v ble-serial || echo "$HOME/.local/bin/ble-serial")}"

if [ ! -x "${BLE_SERIAL_BIN}" ]; then
  echo "ble-serial не найден. Установи его: pip install --user ble-serial" >&2
  exit 1
fi

echo "Запускаю BLE-мост для BMS (${BLE_DEVICE_MAC}) на ${BLE_VIRTUAL_PORT}"
if [ -e "${BLE_VIRTUAL_PORT}" ]; then
  echo "Удаляю существующий порт ${BLE_VIRTUAL_PORT}"
  rm -f "${BLE_VIRTUAL_PORT}"
fi

exec "${BLE_SERIAL_BIN}" -d "${BLE_DEVICE_MAC}" -p "${BLE_VIRTUAL_PORT}"
