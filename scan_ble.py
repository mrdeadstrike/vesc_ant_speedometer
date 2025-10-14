#!/usr/bin/env python3
"""
Simple helper to scan Bluetooth LE devices using bluetoothctl.
Runs `bluetoothctl --timeout <N> scan on` and prints found devices.
"""

import re
import subprocess
import sys
from collections import OrderedDict

SCAN_SECONDS = 10
DEVICE_RE = re.compile(r"^\\s*Device\\s+([0-9A-F:]{17})\\s+(.*)$")


def main():
  try:
    result = subprocess.run(
      ["bluetoothctl", "--timeout", str(SCAN_SECONDS), "scan", "on"],
      capture_output=True,
      text=True,
      check=False,
    )
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
    print(f"За {SCAN_SECONDS} секунд устройства не найдены.")
    print("Попробуй повторить, убедившись, что BLE-модули в режиме обнаружения.")
    return 0

  print(f"Найдены BLE устройства (таймаут {SCAN_SECONDS} с):")
  for mac, name in devices.items():
    print(f"  {mac:17}  {name}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
