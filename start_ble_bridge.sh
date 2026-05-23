#!/bin/bash
set -euo pipefail

BLE_DEVICE_MAC="${BLE_DEVICE_MAC:-}"
BLE_VIRTUAL_PORT="${BLE_VIRTUAL_PORT:-/tmp/fardriver-ble}"
BLE_LABEL="${BLE_LABEL:-FarDriver}"
BLE_ADAPTER="${BLE_ADAPTER:-hci0}"

BLE_SERIAL_BIN="${BLE_SERIAL_BIN:-$(command -v ble-serial || echo "$HOME/.local/bin/ble-serial")}"

if [ ! -x "${BLE_SERIAL_BIN}" ]; then
  echo "ble-serial не найден. Установи его: pip install --user ble-serial" >&2
  exit 1
fi

if [ -z "${BLE_DEVICE_MAC}" ]; then
  echo "Не задан BLE_DEVICE_MAC для ${BLE_LABEL}." >&2
  echo "Сначала найди MAC: python3 scan_ble.py --timeout 20" >&2
  echo "Потом запусти: BLE_DEVICE_MAC=AA:BB:CC:DD:EE:FF ./start_ble_bridge.sh" >&2
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
  echo "Запускаю BLE-мост для ${BLE_LABEL} (${BLE_DEVICE_MAC}) на ${BLE_VIRTUAL_PORT}"
  if [ -e "${BLE_VIRTUAL_PORT}" ] || [ -L "${BLE_VIRTUAL_PORT}" ]; then
    echo "Удаляю существующий порт ${BLE_VIRTUAL_PORT}"
    rm -f "${BLE_VIRTUAL_PORT}"
  fi
  set +e
  "${BLE_SERIAL_BIN}" -d "${BLE_DEVICE_MAC}" -p "${BLE_VIRTUAL_PORT}" -i "${BLE_ADAPTER}" &
  ble_pid=$!
  wait "${ble_pid}"
  exit_code=$?
  set -e
  if [ "${stop_requested}" -eq 1 ]; then
    break
  fi
  echo "BLE-мост ${BLE_LABEL} отключён (код ${exit_code}), переподключаем через 2 секунды..."
  sleep 2
done
