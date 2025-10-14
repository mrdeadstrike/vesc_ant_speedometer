#!/bin/bash

set -euo pipefail

BLE_DEVICE_MAC="F5:00:47:10:37:D2"
BLE_VIRTUAL_PORT="/tmp/vesc-ble"
BLE_BAUDRATE="115200"

BLE_SERIAL_BIN="${BLE_SERIAL_BIN:-$(command -v ble-serial || echo "$HOME/.local/bin/ble-serial")}"

if [ ! -x "${BLE_SERIAL_BIN}" ]; then
  echo "ble-serial не найден. Установи его: pip install --user ble-serial" >&2
  exit 1
fi

echo "Запускаю BLE-мост для VESC (${BLE_DEVICE_MAC}) на ${BLE_VIRTUAL_PORT} со скоростью ${BLE_BAUDRATE} бод"
exec "${BLE_SERIAL_BIN}" -d "${BLE_DEVICE_MAC}" -p "${BLE_VIRTUAL_PORT}" -b "${BLE_BAUDRATE}"
