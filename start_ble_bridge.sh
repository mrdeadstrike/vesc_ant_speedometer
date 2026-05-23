#!/bin/bash
set -euo pipefail

BLE_DEVICE_MAC="${BLE_DEVICE_MAC:-}"
BLE_VIRTUAL_PORT="${BLE_VIRTUAL_PORT:-/tmp/fardriver-ble}"
BLE_LABEL="${BLE_LABEL:-FarDriver}"
BLE_ADAPTER="${BLE_ADAPTER:-}"
BLE_SERVICE_UUID="${BLE_SERVICE_UUID:-}"
BLE_NOTIFY_CHARACTERISTIC="${BLE_NOTIFY_CHARACTERISTIC:-}"
BLE_WRITE_CHARACTERISTIC="${BLE_WRITE_CHARACTERISTIC:-}"
BLE_PRECONNECT="${BLE_PRECONNECT:-1}"

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

prepare_device() {
  if ! command -v bluetoothctl >/dev/null 2>&1; then
    return
  fi

  echo "Готовлю ${BLE_LABEL} в BlueZ: trust/connect ${BLE_DEVICE_MAC}"
  bluetoothctl trust "${BLE_DEVICE_MAC}" >/dev/null 2>&1 || true

  if [ "${BLE_PRECONNECT}" != "0" ]; then
    {
      echo "connect ${BLE_DEVICE_MAC}"
      sleep 5
      echo "info ${BLE_DEVICE_MAC}"
      sleep 1
      echo "disconnect ${BLE_DEVICE_MAC}"
      sleep 1
      echo "quit"
    } | bluetoothctl >"/tmp/${BLE_LABEL// /_}-gatt.txt" 2>/dev/null || true
  else
    bluetoothctl info "${BLE_DEVICE_MAC}" >"/tmp/${BLE_LABEL// /_}-gatt.txt" 2>/dev/null || true
  fi

  if [ -z "${BLE_SERVICE_UUID}" ]; then
    BLE_SERVICE_UUID="$(grep -Eio '0000ffe0-0000-1000-8000-00805f9b34fb|0000fff0-0000-1000-8000-00805f9b34fb' "/tmp/${BLE_LABEL// /_}-gatt.txt" | head -n1 || true)"
  fi
  if [ -z "${BLE_NOTIFY_CHARACTERISTIC}" ]; then
    BLE_NOTIFY_CHARACTERISTIC="$(grep -Eio '0000ffe1-0000-1000-8000-00805f9b34fb|0000fff1-0000-1000-8000-00805f9b34fb|0000fff4-0000-1000-8000-00805f9b34fb' "/tmp/${BLE_LABEL// /_}-gatt.txt" | head -n1 || true)"
  fi
  if [ -z "${BLE_WRITE_CHARACTERISTIC}" ]; then
    BLE_WRITE_CHARACTERISTIC="$(grep -Eio '0000ffe1-0000-1000-8000-00805f9b34fb|0000fff2-0000-1000-8000-00805f9b34fb|0000fff3-0000-1000-8000-00805f9b34fb' "/tmp/${BLE_LABEL// /_}-gatt.txt" | head -n1 || true)"
  fi

  BLE_SERVICE_UUID="${BLE_SERVICE_UUID:-0000ffe0-0000-1000-8000-00805f9b34fb}"
  BLE_NOTIFY_CHARACTERISTIC="${BLE_NOTIFY_CHARACTERISTIC:-0000ffe1-0000-1000-8000-00805f9b34fb}"
  BLE_WRITE_CHARACTERISTIC="${BLE_WRITE_CHARACTERISTIC:-0000ffe1-0000-1000-8000-00805f9b34fb}"

  echo "UUID ${BLE_LABEL}: service=${BLE_SERVICE_UUID} notify=${BLE_NOTIFY_CHARACTERISTIC} write=${BLE_WRITE_CHARACTERISTIC}"
}

while true; do
  echo "Запускаю BLE-мост для ${BLE_LABEL} (${BLE_DEVICE_MAC}) на ${BLE_VIRTUAL_PORT}"
  if [ -e "${BLE_VIRTUAL_PORT}" ] || [ -L "${BLE_VIRTUAL_PORT}" ]; then
    echo "Удаляю существующий порт ${BLE_VIRTUAL_PORT}"
    rm -f "${BLE_VIRTUAL_PORT}"
  fi
  prepare_device
  set +e
  args=(-d "${BLE_DEVICE_MAC}" -p "${BLE_VIRTUAL_PORT}" -s "${BLE_SERVICE_UUID}" -r "${BLE_NOTIFY_CHARACTERISTIC}" -w "${BLE_WRITE_CHARACTERISTIC}")
  if [ -n "${BLE_ADAPTER}" ]; then
    args+=("-i" "${BLE_ADAPTER}")
  fi
  "${BLE_SERIAL_BIN}" "${args[@]}" &
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
