#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB_DIR="${SCRIPT_DIR}/speedometer_web"
RUN_DIR="${WEB_DIR}/.run"

BACKEND_PORT="${BACKEND_PORT:-9400}"
FRONTEND_PORT="${FRONTEND_PORT:-4173}"

PID_BACKEND_FILE="${RUN_DIR}/backend.pid"
PID_FRONTEND_FILE="${RUN_DIR}/frontend.pid"
PID_VESC_BRIDGE_FILE="${RUN_DIR}/bridge_vesc.pid"
PID_BMS_BRIDGE_FILE="${RUN_DIR}/bridge_bms.pid"

stop_pid_graceful() {
  local pid="${1:-}"
  if [[ -z "${pid}" ]]; then
    return 0
  fi
  if ! kill -0 "${pid}" 2>/dev/null; then
    return 0
  fi
  kill "${pid}" 2>/dev/null || true
}

stop_pid_force() {
  local pid="${1:-}"
  if [[ -z "${pid}" ]]; then
    return 0
  fi
  if ! kill -0 "${pid}" 2>/dev/null; then
    return 0
  fi
  kill -9 "${pid}" 2>/dev/null || true
}

stop_from_pidfile() {
  local file="$1"
  if [[ ! -f "${file}" ]]; then
    return 0
  fi
  local pid
  pid="$(cat "${file}" 2>/dev/null || true)"
  if [[ -n "${pid}" ]]; then
    echo ">>> Останавливаю PID ${pid} (${file##*/})"
    stop_pid_graceful "${pid}"
  fi
}

force_from_pidfile() {
  local file="$1"
  if [[ ! -f "${file}" ]]; then
    return 0
  fi
  local pid
  pid="$(cat "${file}" 2>/dev/null || true)"
  if [[ -n "${pid}" ]]; then
    stop_pid_force "${pid}"
  fi
}

kill_port() {
  local port="$1"
  if ! command -v lsof >/dev/null 2>&1; then
    return 0
  fi
  local pids
  pids="$(lsof -tiTCP:"${port}" -sTCP:LISTEN 2>/dev/null || true)"
  if [[ -n "${pids}" ]]; then
    echo ">>> Останавливаю процессы на порту ${port}: ${pids//$'\n'/, }"
    for pid in ${pids}; do
      stop_pid_graceful "${pid}"
    done
    sleep 1
    pids="$(lsof -tiTCP:"${port}" -sTCP:LISTEN 2>/dev/null || true)"
    if [[ -n "${pids}" ]]; then
      echo ">>> Принудительно завершаю процессы на порту ${port}: ${pids//$'\n'/, }"
      for pid in ${pids}; do
        stop_pid_force "${pid}"
      done
    fi
  fi
}

kill_by_pattern() {
  local pattern="$1"
  local pids=""
  if command -v pgrep >/dev/null 2>&1; then
    pids="$(pgrep -f "${pattern}" 2>/dev/null || true)"
  else
    pids="$(ps ax -o pid= -o command= | grep -E "${pattern}" | grep -v grep | awk '{print $1}' || true)"
  fi

  if [[ -n "${pids}" ]]; then
    echo ">>> Останавливаю по паттерну '${pattern}': ${pids//$'\n'/, }"
    for pid in ${pids}; do
      stop_pid_graceful "${pid}"
    done
    sleep 1
    for pid in ${pids}; do
      stop_pid_force "${pid}"
    done
  fi
}

stop_from_pidfile "${PID_FRONTEND_FILE}"
stop_from_pidfile "${PID_BACKEND_FILE}"
stop_from_pidfile "${PID_BMS_BRIDGE_FILE}"
stop_from_pidfile "${PID_VESC_BRIDGE_FILE}"

sleep 1

force_from_pidfile "${PID_FRONTEND_FILE}"
force_from_pidfile "${PID_BACKEND_FILE}"
force_from_pidfile "${PID_BMS_BRIDGE_FILE}"
force_from_pidfile "${PID_VESC_BRIDGE_FILE}"

kill_port "${BACKEND_PORT}"
kill_port "${FRONTEND_PORT}"

kill_by_pattern "${SCRIPT_DIR}/start_ble_bridge.sh"
kill_by_pattern "${SCRIPT_DIR}/start_ble_bms_bridge.sh"
kill_by_pattern "${WEB_DIR}/backend.*uvicorn app\\.main:app"
kill_by_pattern "${WEB_DIR}/frontend.*vite preview"

rm -f "${PID_BACKEND_FILE}" "${PID_FRONTEND_FILE}" "${PID_VESC_BRIDGE_FILE}" "${PID_BMS_BRIDGE_FILE}"
rmdir "${RUN_DIR}" 2>/dev/null || true

echo ">>> Web сервис остановлен"
