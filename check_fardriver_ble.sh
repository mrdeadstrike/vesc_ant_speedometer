#!/bin/bash
set -euo pipefail

MASTER_MAC="${FARDRIVER_MASTER_MAC:-E0:00:AC:FB:00:23}"
SLAVE_MAC="${FARDRIVER_SLAVE_MAC:-C0:05:EA:1E:00:45}"
SCAN_SECONDS="${SCAN_SECONDS:-20}"

echo "== Bluetooth adapters =="
bluetoothctl list || true

echo
echo "== Default adapter =="
bluetoothctl show || true

echo
echo "== rfkill =="
rfkill list bluetooth || true

echo
echo "== Scan ${SCAN_SECONDS}s =="
scan_output="$(bluetoothctl --timeout "${SCAN_SECONDS}" scan on || true)"
echo "${scan_output}"

echo
echo "== Target check =="
for mac in "${MASTER_MAC}" "${SLAVE_MAC}"; do
  if echo "${scan_output}" | grep -qi "${mac}"; then
    echo "FOUND ${mac}"
  else
    echo "NOT FOUND ${mac}"
  fi
  echo "-- bluetoothctl info ${mac}"
  bluetoothctl info "${mac}" || true
  echo
done
