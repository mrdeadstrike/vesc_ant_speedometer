#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB_DIR="${SCRIPT_DIR}/speedometer_web"
BACKEND_DIR="${WEB_DIR}/backend"
FRONTEND_DIR="${WEB_DIR}/frontend"
VENV_DIR="${BACKEND_DIR}/.venv"

BLE_VESC_SCRIPT="${SCRIPT_DIR}/start_ble_bridge.sh"
BLE_BMS_SCRIPT="${SCRIPT_DIR}/start_ble_bms_bridge.sh"

BACKEND_HOST="${BACKEND_HOST:-0.0.0.0}"
BACKEND_PORT="${BACKEND_PORT:-9400}"
FRONTEND_HOST="${FRONTEND_HOST:-0.0.0.0}"
FRONTEND_PORT="${FRONTEND_PORT:-4173}"
START_BLE_BRIDGES="${START_BLE_BRIDGES:-1}"

MAIN_DATA_FILE="${MAIN_DATA_FILE:-${SCRIPT_DIR}/mainData.txt}"

bridge_pid_vesc=""
bridge_pid_bms=""
backend_pid=""
frontend_pid=""

require_cmd() {
  local cmd="$1"
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "Ошибка: не найдено '${cmd}' в PATH" >&2
    exit 1
  fi
}

stop_pid() {
  local pid="${1:-}"
  if [[ -n "${pid}" ]]; then
    kill "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  fi
}

cleanup() {
  stop_pid "${frontend_pid}"
  stop_pid "${backend_pid}"
  stop_pid "${bridge_pid_bms}"
  stop_pid "${bridge_pid_vesc}"
}

trap cleanup EXIT INT TERM

if [[ ! -d "${WEB_DIR}" ]]; then
  echo "Ошибка: не найдена папка ${WEB_DIR}" >&2
  exit 1
fi

require_cmd python3
require_cmd node
require_cmd npm

free_port() {
  local port="$1"
  if ! command -v lsof >/dev/null 2>&1; then
    echo ">>> lsof не найден, пропускаю автоосвобождение порта ${port}"
    return 0
  fi

  local pids
  pids="$(lsof -tiTCP:"${port}" -sTCP:LISTEN 2>/dev/null || true)"
  if [[ -n "${pids}" ]]; then
    echo ">>> Порт ${port} занят. Останавливаю процесс(ы): ${pids//$'\n'/, }"
    for pid in ${pids}; do
      kill "${pid}" 2>/dev/null || true
    done
    sleep 1

    local remain
    remain="$(lsof -tiTCP:"${port}" -sTCP:LISTEN 2>/dev/null || true)"
    if [[ -n "${remain}" ]]; then
      echo ">>> Принудительно завершаю процесс(ы) на порту ${port}: ${remain//$'\n'/, }"
      for pid in ${remain}; do
        kill -9 "${pid}" 2>/dev/null || true
      done
      sleep 1
    fi

    remain="$(lsof -tiTCP:"${port}" -sTCP:LISTEN 2>/dev/null || true)"
    if [[ -n "${remain}" ]]; then
      echo "Ошибка: не удалось освободить порт ${port} (PID: ${remain//$'\n'/, })" >&2
      return 1
    fi
  fi
  return 0
}

wait_for_children() {
  while true; do
    if [[ -n "${backend_pid}" ]] && ! kill -0 "${backend_pid}" 2>/dev/null; then
      wait "${backend_pid}" || return $?
      return 0
    fi
    if [[ -n "${frontend_pid}" ]] && ! kill -0 "${frontend_pid}" 2>/dev/null; then
      wait "${frontend_pid}" || return $?
      return 0
    fi
    sleep 1
  done
}

free_port "${BACKEND_PORT}"
free_port "${FRONTEND_PORT}"

echo ">>> Подготовка Python окружения"
if [[ ! -d "${VENV_DIR}" ]]; then
  python3 -m venv "${VENV_DIR}"
fi
source "${VENV_DIR}/bin/activate"
python -m pip install --upgrade pip
python -m pip install -r "${BACKEND_DIR}/requirements.txt"

echo ">>> Подготовка фронтенда"
pushd "${FRONTEND_DIR}" >/dev/null
install_frontend_deps() {
  if [[ -f package-lock.json ]]; then
    npm ci
  else
    npm install
  fi
}

if ! install_frontend_deps; then
  echo ">>> npm install не прошёл, повтор с --legacy-peer-deps"
  if [[ -f package-lock.json ]]; then
    npm ci --legacy-peer-deps
  else
    npm install --legacy-peer-deps
  fi
fi
VITE_BACKEND_PORT="${BACKEND_PORT}" npm run build
popd >/dev/null

if [[ "${START_BLE_BRIDGES}" == "1" ]]; then
  if [[ -x "${BLE_VESC_SCRIPT}" ]]; then
    echo ">>> Запуск BLE-моста VESC"
    "${BLE_VESC_SCRIPT}" &
    bridge_pid_vesc=$!
  else
    echo ">>> BLE-мост VESC не запущен: ${BLE_VESC_SCRIPT} не найден или не исполняемый"
  fi

  if [[ -x "${BLE_BMS_SCRIPT}" ]]; then
    echo ">>> Запуск BLE-моста BMS"
    "${BLE_BMS_SCRIPT}" &
    bridge_pid_bms=$!
  else
    echo ">>> BLE-мост BMS не запущен: ${BLE_BMS_SCRIPT} не найден или не исполняемый"
  fi
fi

echo ">>> Запуск backend FastAPI на ${BACKEND_HOST}:${BACKEND_PORT}"
(
  cd "${BACKEND_DIR}"
  MAIN_DATA_FILE="${MAIN_DATA_FILE}" \
  uvicorn app.main:app --host "${BACKEND_HOST}" --port "${BACKEND_PORT}"
) &
backend_pid=$!

echo ">>> Запуск frontend preview на ${FRONTEND_HOST}:${FRONTEND_PORT}"
(
  cd "${FRONTEND_DIR}"
  npm run preview -- --host "${FRONTEND_HOST}" --port "${FRONTEND_PORT}" --strictPort
) &
frontend_pid=$!

echo
echo "Готово:"
echo "- Backend API/WS: http://<IP_RPI>:${BACKEND_PORT}"
echo "- Frontend:       http://<IP_RPI>:${FRONTEND_PORT}"
echo "Остановить: Ctrl+C"
echo

wait_for_children
