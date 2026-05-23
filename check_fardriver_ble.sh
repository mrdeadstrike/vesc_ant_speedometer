#!/bin/bash
set -euo pipefail

MASTER_MAC="${FARDRIVER_MASTER_MAC:-}"
SLAVE_MAC="${FARDRIVER_SLAVE_MAC:-}"
NAME_PREFIX="${FARDRIVER_NAME_PREFIX:-YuanQuFOC}"
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
printf '%s\n' "${scan_output}" | sed -nE "s/^.*Device ([0-9A-Fa-f:]{17}) (${NAME_PREFIX}[^[:space:]]*).*$/\U\1\E \2/p" | awk '!seen[$1]++'

targets=()
[ -n "${MASTER_MAC}" ] && targets+=("${MASTER_MAC}")
[ -n "${SLAVE_MAC}" ] && targets+=("${SLAVE_MAC}")

for mac in "${targets[@]}"; do
  if echo "${scan_output}" | grep -qi "${mac}"; then
    echo "FOUND ${mac}"
  else
    echo "NOT FOUND ${mac}"
  fi
  echo "-- bluetoothctl info ${mac}"
  bluetoothctl info "${mac}" || true
  echo
done
