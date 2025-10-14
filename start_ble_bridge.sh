#!/bin/bash
set -euo pipefail

BLE_DEVICE_MAC="F5:00:47:10:37:D2"
BLE_VIRTUAL_PORT="/tmp/vesc-ble"

BLE_SERIAL_BIN="${BLE_SERIAL_BIN:-$(command -v ble-serial || echo "$HOME/.local/bin/ble-serial")}"

if [ ! -x "${BLE_SERIAL_BIN}" ]; then
  echo "ble-serial не найден. Установи его: pip install --user ble-serial" >&2
  exit 1
fi

echo "Запускаю BLE-мост для VESC (${BLE_DEVICE_MAC}) на ${BLE_VIRTUAL_PORT}"
if [ -e "${BLE_VIRTUAL_PORT}" ]; then
  echo "Удаляю существующий порт ${BLE_VIRTUAL_PORT}"
  rm -f "${BLE_VIRTUAL_PORT}"
fi
exec "${BLE_SERIAL_BIN}" -d "${BLE_DEVICE_MAC}" -p "${BLE_VIRTUAL_PORT}"
