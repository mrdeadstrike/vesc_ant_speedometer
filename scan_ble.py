#!/usr/bin/env python3
"""
Simple helper to scan Bluetooth LE devices using bluetoothctl.
Runs `bluetoothctl --timeout <N> scan on` and prints found devices.
If MAC is provided as an argument, highlights it and tries to fetch extra info.
"""

import argparse
import re
import subprocess
import sys
from collections import OrderedDict

SCAN_SECONDS = 10
DEVICE_RE = re.compile(r"^\\s*Device\\s+([0-9A-F:]{17})\\s+(.*)$")


def run_bluetoothctl(args):
  return subprocess.run(args, capture_output=True, text=True, check=False)


def main():
  parser = argparse.ArgumentParser(description="Сканирование BLE-устройств через bluetoothctl")
  parser.add_argument(
    "--mac",
    help="Искомый MAC-адрес (например 98:DE:1E:84:80:E6) — будет выделен в списке",
  )
  parser.add_argument(
    "--timeout",
    type=int,
    default=SCAN_SECONDS,
    help=f"Время сканирования, секунд (по умолчанию {SCAN_SECONDS})",
  )
  args = parser.parse_args()

  try:
    result = run_bluetoothctl(["bluetoothctl", "--timeout", str(args.timeout), "scan", "on"])
  except FileNotFoundError:
    print("bluetoothctl не найден. Установи пакет bluez.", file=sys.stderr)
    return 1

  devices = OrderedDict()
  output = result.stdout or ""
  for line in output.splitlines():
    match = DEVICE_RE.match(line)
    if match:
      mac, name = match.groups()
      devices[mac] = name.strip() or "(без имени)"

  if not devices:
    print(f"За {args.timeout} секунд устройства не найдены.")
    print("Попробуй повторить, убедившись, что BLE-модули в режиме обнаружения.")
    return 0

  target_found = False
  print(f"Найдены BLE устройства (таймаут {args.timeout} с):")
  for mac, name in devices.items():
    marker = ""
    if args.mac and mac.upper() == args.mac.upper():
      marker = "  <-- ИСКОМАЯ BMS"
      target_found = True
    print(f"  {mac:17}  {name}{marker}")

  if args.mac and not target_found:
    print(f"Устройство с MAC {args.mac} не найдено.")
    return 0

  if args.mac and target_found:
    print("\nДополнительная информация по устройству:")
    info = run_bluetoothctl(["bluetoothctl", "info", args.mac])
    print(info.stdout.strip() or "(нет данных)")

  return 0


if __name__ == "__main__":
  raise SystemExit(main())
