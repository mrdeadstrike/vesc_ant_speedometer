# VESC Web Speedometer (React + FastAPI)

В этом проекте телеметрия с VESC/BMS читается в `FastAPI` backend и стримится в браузер по WebSocket (`/ws`).
Звук/озвучка не переносились, только отображение.

## Структура

- `backend/` — чтение `/tmp/vesc-ble` и `/tmp/bms-ble`, API и WebSocket
- `frontend/` — React интерфейс под планшет/браузер

## Быстрый старт

### Запуск одной командой (рекомендуется)

Из корня репозитория:

```bash
cd /Users/deadstrike/wm/WM_DATA/vesc_ant_speedometer
./web_run.sh
```

Скрипт:
- создаёт/обновляет `backend/.venv`
- ставит Python зависимости
- ставит npm зависимости
- собирает фронтенд
- запускает backend (`:9400`) и frontend preview (`:4173`)
- по умолчанию пытается поднять BLE-мосты (`start_ble_bridge.sh`, `start_ble_bms_bridge.sh`)
- автоматически освобождает целевые порты (останавливает процесс, занявший порт)

Опции через env:

- `START_BLE_BRIDGES=0` — не запускать BLE-мосты
- `BACKEND_PORT=8010` / `FRONTEND_PORT=5173` — поменять порты
- `MAIN_DATA_FILE=/path/to/mainData.txt` — файл одометра
- `DEBUG_MOCK=1` — принудительно включить отладочные данные
- `DEBUG_MOCK=0` — отключить отладочные данные

По умолчанию `DEBUG_MOCK=auto`: мок включается автоматически, если нет живых данных от VESC и BMS.

### 1) Backend

```bash
cd /Users/deadstrike/wm/WM_DATA/vesc_ant_speedometer/speedometer_web/backend
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
uvicorn app.main:app --host 0.0.0.0 --port 9400
```

Переменные окружения (опционально):

- `VESC_SERIAL_PORT` (по умолчанию `/tmp/vesc-ble`)
- `BMS_SERIAL_PORT` (по умолчанию `/tmp/bms-ble`)
- `MAIN_DATA_FILE` (файл одометра; если не задан, берётся `../mainData.txt` из корня репозитория)

### 2) Frontend (dev)

```bash
cd /Users/deadstrike/wm/WM_DATA/vesc_ant_speedometer/speedometer_web/frontend
npm install
npm run dev -- --host 0.0.0.0 --port 5173
```

Открыть в браузере планшета: `http://<IP_RPI>:5173`

### 3) Frontend (prod build + отдача из FastAPI)

```bash
cd /Users/deadstrike/wm/WM_DATA/vesc_ant_speedometer/speedometer_web/frontend
npm install
npm run build
```

После этого backend автоматически начнёт раздавать `frontend/dist` на `/`.

## Эндпоинты

- `GET /api/health`
- `GET /api/telemetry` — текущий снапшот
- `WS /ws` — поток обновлений (~20 Гц)

## Примечания

- Логика парсинга VESC/BMS взята из текущего `speedometer.py` и упрощена до нужных экранных показателей.
- Если BLE-мосты не подняты, backend будет пытаться переподключаться.
