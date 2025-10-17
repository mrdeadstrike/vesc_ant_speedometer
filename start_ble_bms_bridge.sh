#!/bin/bash
set -euo pipefail

BLE_DEVICE_MAC="98:DE:1E:84:80:E6"
BLE_VIRTUAL_PORT="/tmp/bms-ble"
BLE_SERIAL_BIN="${BLE_SERIAL_BIN:-$(command -v ble-serial || echo "$HOME/.local/bin/ble-serial")}"
BLE_ADAPTER="${BLE_ADAPTER:-}"

if [ ! -x "${BLE_SERIAL_BIN}" ]; then
  echo "❌ ble-serial не найден. Установи: pip install --user ble-serial" >&2
  exit 1
fi
if ! command -v bluetoothctl >/dev/null 2>&1; then
  echo "❌ bluetoothctl не найден. Установи пакет bluez." >&2
  exit 1
fi

# cleanup при завершении
ble_pid=""
cleanup() {
  if [ -n "${ble_pid}" ]; then
    kill "${ble_pid}" 2>/dev/null || true
    wait "${ble_pid}" 2>/dev/null || true
    ble_pid=""
  fi
}
trap cleanup INT TERM

echo "🔍 Проверяю статус BLE-устройства ${BLE_DEVICE_MAC}..."
set +e
info_output=$(bluetoothctl info "${BLE_DEVICE_MAC}" 2>/dev/null)
set -e
if ! grep -q "Paired: yes" <<<"$info_output"; then
  echo "⚠️  Устройство не спарено. Сделай:"
  echo "   bluetoothctl"
  echo "   power on"
  echo "   agent KeyboardOnly"
  echo "   default-agent"
  echo "   pair ${BLE_DEVICE_MAC}  # PIN 123456"
  exit 1
fi
if ! grep -q "Trusted: yes" <<<"$info_output"; then
  echo "⚙️  Делаю trust для ${BLE_DEVICE_MAC}..."
  bluetoothctl trust "${BLE_DEVICE_MAC}" >/dev/null 2>&1 || true
fi

# Удаляем старый порт
[ -e "${BLE_VIRTUAL_PORT}" ] && rm -f "${BLE_VIRTUAL_PORT}"

echo "🔗 Подключаюсь к ${BLE_DEVICE_MAC} для получения GATT UUID..."
{
  echo "connect ${BLE_DEVICE_MAC}"
  echo "menu gatt"
  echo "list-attributes"
  echo "back"
  echo "disconnect"
  echo "exit"
} | bluetoothctl > /tmp/bms-gatt.txt 2>/dev/null || true

# Поиск UUID
BLE_SERVICE_UUID=$(grep -m1 -o '0000ffe0-0000-1000-8000-00805f9b34fb' /tmp/bms-gatt.txt || true)
BLE_NOTIFY_CHARACTERISTIC=$(grep -m1 -o '0000fff4-0000-1000-8000-00805f9b34fb' /tmp/bms-gatt.txt || true)
BLE_WRITE_CHARACTERISTIC=$(grep -m1 -o '0000fff3-0000-1000-8000-00805f9b34fb' /tmp/bms-gatt.txt || grep -m1 -o '0000fff2-0000-1000-8000-00805f9b34fb' /tmp/bms-gatt.txt || true)

# fallback если ничего не найдено
BLE_SERVICE_UUID="${BLE_SERVICE_UUID:-0000ffe0-0000-1000-8000-00805f9b34fb}"
BLE_NOTIFY_CHARACTERISTIC="${BLE_NOTIFY_CHARACTERISTIC:-0000fff4-0000-1000-8000-00805f9b34fb}"
BLE_WRITE_CHARACTERISTIC="${BLE_WRITE_CHARACTERISTIC:-0000fff3-0000-1000-8000-00805f9b34fb}"

echo "🧩 Найдены UUID:"
echo "  Service: ${BLE_SERVICE_UUID}"
echo "  Notify : ${BLE_NOTIFY_CHARACTERISTIC}"
echo "  Write  : ${BLE_WRITE_CHARACTERISTIC}"

# Формируем аргументы
args=(-d "${BLE_DEVICE_MAC}" -p "${BLE_VIRTUAL_PORT}" -s "${BLE_SERVICE_UUID}" -r "${BLE_NOTIFY_CHARACTERISTIC}" -w "${BLE_WRITE_CHARACTERISTIC}")
[ -n "${BLE_ADAPTER}" ] && args+=("-i" "${BLE_ADAPTER}")
[ -n "${BLE_SERIAL_FLAGS:-}" ] && args+=(${BLE_SERIAL_FLAGS})

echo "🚀 Запускаю BLE-мост для BMS..."
set +e
"${BLE_SERIAL_BIN}" "${args[@]}" &
ble_pid=$!
wait "${ble_pid}"
exit_code=$?
set -e

echo "🏁 BLE-мост BMS завершён (код ${exit_code})"
exit "${exit_code}"
