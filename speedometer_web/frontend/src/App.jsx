import { useEffect, useMemo, useState } from "react";

const EMPTY = {
  speed_kmh: 0,
  power_w: 0,
  master: { motor_current: 0, battery_current: 0, duty: 0, temp: 0, temp_motor: 0 },
  slave: { motor_current: 0, battery_current: 0, duty: 0, temp: 0, temp_motor: 0 },
  battery: { voltage_v: 0, no_load_voltage_v: 0, sag_v: 0, percent: 0 },
  bms_temp: {
    mosfet_temp: 0,
    balance_temp: 0,
    external_temp_0: 0,
    external_temp_1: 0,
    external_temp_2: 0,
    external_temp_3: 0,
  },
  cells_v: [],
  weak_cell: { index: 0, voltage_v: 0 },
  cell_diff_v: 0,
  trip: {
    odometer_total_km: 0,
    trip_km: 0,
    avg_speed_kmh: 0,
    trip_time: "00:00",
    max_speed_kmh: 0,
    max_power_w: 0,
  },
  status: { vesc_connected: false, bms_lost: true },
};

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function formatNumber(value, digits = 1) {
  if (!Number.isFinite(value)) {
    return "0";
  }
  return Number(value).toFixed(digits);
}

function tempClass(temp) {
  if (temp >= 80) return "is-hot";
  if (temp >= 60) return "is-warm";
  return "is-cool";
}

function batteryClass(percent) {
  if (percent < 25) return "is-hot";
  if (percent < 50) return "is-warm";
  return "is-cool";
}

function diffClass(diff) {
  if (diff >= 0.05) return "is-hot";
  if (diff >= 0.03) return "is-warm";
  return "is-cool";
}

function MetricBar({ label, value, max, suffix = "", alert = false }) {
  const fill = clamp((Math.abs(value) / max) * 100, 0, 100);
  return (
    <div className="metric-bar">
      <div className="metric-head">
        <span>{label}</span>
        <span>{`${Math.round(Math.abs(value))}${suffix}`}</span>
      </div>
      <div className="metric-track">
        <div
          className={`metric-fill ${alert ? "is-alert" : ""}`}
          style={{ width: `${fill}%` }}
        />
      </div>
    </div>
  );
}

function SpeedBar({ speed }) {
  const max = 70;
  const main = clamp((speed / max) * 100, 0, 100);
  const over = speed > max ? clamp(((speed - max) / max) * 100, 0, 100) : 0;
  const overLimit = speed > max;
  return (
    <div className="speed-bar-wrap">
      <div className="speed-track">
        <div className={`speed-fill ${overLimit ? "is-over" : ""}`} style={{ width: `${main}%` }} />
      </div>
      {over > 0 ? <div className="speed-overflow" style={{ width: `${over}%` }} /> : null}
    </div>
  );
}

export default function App() {
  const [snapshot, setSnapshot] = useState(EMPTY);
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    let ws;
    let reconnectTimer;
    let disposed = false;

    const connect = () => {
      const explicit = import.meta.env.VITE_WS_URL;
      const backendPort = import.meta.env.VITE_BACKEND_PORT || "9400";
      const defaultUrl = `${window.location.protocol === "https:" ? "wss" : "ws"}://${window.location.hostname}:${backendPort}/ws`;
      const url = explicit || defaultUrl;

      ws = new WebSocket(url);
      ws.onopen = () => setConnected(true);
      ws.onmessage = (event) => {
        try {
          const next = JSON.parse(event.data);
          setSnapshot(next);
        } catch {
          // ignore malformed payloads
        }
      };
      ws.onerror = () => {
        ws.close();
      };
      ws.onclose = () => {
        setConnected(false);
        if (!disposed) {
          reconnectTimer = setTimeout(connect, 900);
        }
      };
    };

    connect();

    return () => {
      disposed = true;
      clearTimeout(reconnectTimer);
      if (ws && ws.readyState < 2) {
        ws.close();
      }
    };
  }, []);

  const speed = Math.round(snapshot.speed_kmh || 0);
  const bmsSensors = [
    snapshot.bms_temp.external_temp_0,
    snapshot.bms_temp.external_temp_1,
    snapshot.bms_temp.external_temp_2,
    snapshot.bms_temp.external_temp_3,
  ];

  const sortedCells = useMemo(() => {
    const indexed = (snapshot.cells_v || []).map((voltage, index) => ({ index: index + 1, voltage }));
    indexed.sort((a, b) => (a.voltage === b.voltage ? a.index - b.index : a.voltage - b.voltage));
    return indexed;
  }, [snapshot.cells_v]);

  const weakest = sortedCells.slice(0, 4);
  const strongest = [...sortedCells.slice(-4)].reverse();

  const now = new Date();
  const statusLine = snapshot.status.bms_lost ? "BMS Lost" : "BMS Online";

  return (
    <main className="app-shell">
      <section className="top-panel card reveal-up">
        <div className="top-status">
          <span className={`status-dot ${connected ? "is-on" : "is-off"}`} />
          <span>{connected ? "WS connected" : "WS reconnecting"}</span>
          <span className={`status-chip ${snapshot.status.vesc_connected ? "is-on" : "is-off"}`}>
            {snapshot.status.vesc_connected ? "VESC" : "VESC off"}
          </span>
          <span className={`status-chip ${snapshot.status.bms_lost ? "is-off" : "is-on"}`}>{statusLine}</span>
        </div>
        <div className="speed-value-wrap">
          <div className="speed-value">{speed}</div>
          <div className="speed-unit">км/ч</div>
        </div>
        <SpeedBar speed={speed} />
        <div className="power-value">{Math.round(snapshot.power_w)} W</div>
      </section>

      <section className="controls-panel card reveal-up-delay-1">
        <MetricBar label="S Duty" value={snapshot.slave.duty} max={100} />
        <MetricBar label="S Batt" value={snapshot.slave.battery_current} max={80} suffix="A" />
        <MetricBar label="M Batt" value={snapshot.master.battery_current} max={80} suffix="A" />
        <MetricBar label="M Duty" value={snapshot.master.duty} max={100} />
      </section>

      <section className="temp-battery-grid reveal-up-delay-2">
        <article className="card temp-card">
          <h2>Температуры</h2>
          <div className="temp-row">
            <span>Моторы</span>
            <span className={tempClass(snapshot.slave.temp_motor)}>{snapshot.slave.temp_motor}°</span>
            <span className={tempClass(snapshot.master.temp_motor)}>{snapshot.master.temp_motor}°</span>
          </div>
          <div className="temp-row">
            <span>Контроллеры</span>
            <span className={tempClass(snapshot.slave.temp)}>{snapshot.slave.temp}°</span>
            <span className={tempClass(snapshot.master.temp)}>{snapshot.master.temp}°</span>
          </div>
          <div className="temp-row">
            <span>BMS М/Б</span>
            <span className={tempClass(snapshot.bms_temp.mosfet_temp)}>{snapshot.bms_temp.mosfet_temp}°</span>
            <span className={tempClass(snapshot.bms_temp.balance_temp)}>{snapshot.bms_temp.balance_temp}°</span>
          </div>
          <div className="sensor-grid">
            {bmsSensors.map((value, idx) => (
              <span key={idx} className={tempClass(value)}>
                {value}°
              </span>
            ))}
          </div>
        </article>

        <article className="card battery-card">
          <h2>Батарея</h2>
          <div className="voltage-row">
            <span>{formatNumber(snapshot.battery.voltage_v, 1)}V</span>
            <span className={snapshot.battery.sag_v <= -5 ? "is-hot" : snapshot.battery.sag_v <= -2 ? "is-warm" : "is-cool"}>
              {formatNumber(snapshot.battery.sag_v, 1)}V
            </span>
          </div>
          <div className="battery-track">
            <div
              className={`battery-fill ${batteryClass(snapshot.battery.percent)}`}
              style={{ width: `${clamp(snapshot.battery.percent, 0, 100)}%` }}
            />
          </div>
          <div className={`battery-percent ${batteryClass(snapshot.battery.percent)}`}>{snapshot.battery.percent}%</div>
          <div className="weak-row">
            <span>Low {snapshot.weak_cell.index}</span>
            <span>{formatNumber(snapshot.weak_cell.voltage_v, 3)}V</span>
          </div>
          <div className={`diff-row ${diffClass(snapshot.cell_diff_v)}`}>
            <span>Diff</span>
            <span>{formatNumber(snapshot.cell_diff_v, 3)}V</span>
          </div>
        </article>
      </section>

      <section className="bottom-grid reveal-up-delay-3">
        <article className="card cells-card">
          <h2>Ячейки</h2>
          <div className="cells-columns">
            <div>
              <h3>Слабые</h3>
              {weakest.map((cell) => (
                <div className="cell-row weak" key={`w-${cell.index}`}>
                  <span>#{cell.index}</span>
                  <span>{formatNumber(cell.voltage, 3)}V</span>
                </div>
              ))}
            </div>
            <div>
              <h3>Сильные</h3>
              {strongest.map((cell) => (
                <div className="cell-row strong" key={`s-${cell.index}`}>
                  <span>#{cell.index}</span>
                  <span>{formatNumber(cell.voltage, 3)}V</span>
                </div>
              ))}
            </div>
          </div>
        </article>

        <article className="card trip-card">
          <h2>Поездка</h2>
          <div className="trip-row">
            <span>Одометр</span>
            <span>{formatNumber(snapshot.trip.odometer_total_km, 1)} км</span>
          </div>
          <div className="trip-row">
            <span>Дистанция</span>
            <span>{formatNumber(snapshot.trip.trip_km, 1)} км</span>
          </div>
          <div className="trip-row">
            <span>Средняя</span>
            <span>{formatNumber(snapshot.trip.avg_speed_kmh, 1)} км/ч</span>
          </div>
          <div className="trip-row">
            <span>Время</span>
            <span>{snapshot.trip.trip_time}</span>
          </div>
          <div className="trip-row">
            <span>Макс. скорость</span>
            <span>{formatNumber(snapshot.trip.max_speed_kmh, 0)} км/ч</span>
          </div>
          <div className="trip-row">
            <span>Макс. мощность</span>
            <span>{snapshot.trip.max_power_w} W</span>
          </div>
          <div className="clock-line">
            {now.toLocaleDateString("ru-RU", { day: "2-digit", month: "short" })}
            <span>{now.toLocaleTimeString("ru-RU", { hour: "2-digit", minute: "2-digit" })}</span>
          </div>
        </article>
      </section>
    </main>
  );
}
