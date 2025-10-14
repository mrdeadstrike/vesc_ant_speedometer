#!/bin/bash

set -euo pipefail

BLE_DEVICE_MAC="F5:00:47:10:37:D2"
BLE_VIRTUAL_PORT="/tmp/vesc-ble"
BLE_BAUD_RATE="115200"

if ! command -v ble-serial >/dev/null 2>&1; then
  echo "ble-serial не установлен. Установи его командой: pip install --user ble-serial" >&2
  exit 1
fi

echo "Запускаю BLE-мост для VESC (${BLE_DEVICE_MAC}) на ${BLE_VIRTUAL_PORT} со скоростью ${BLE_BAUD_RATE} бод"
exec ble-serial -d "${BLE_DEVICE_MAC}" -p "${BLE_VIRTUAL_PORT}" -b "${BLE_BAUD_RATE}" "$@"
