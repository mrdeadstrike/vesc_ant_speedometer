import { useEffect, useRef, useState } from "react";

const EMPTY = {
  speed_kmh: 0,
  power_w: 0,
  bms_current_a: 0,
  battery: { voltage_v: 0, no_load_voltage_v: 0, sag_v: 0, percent: 0 },
  bms_temp: {
    mosfet_temp: 0,
    balance_temp: 0,
    external_temp_0: 0,
    external_temp_1: 0,
    external_temp_2: 0,
    external_temp_3: 0,
  },
  master: { temp: 0, temp_motor: 0 },
  slave: { temp: 0, temp_motor: 0 },
  weak_cell: { index: 0, voltage_v: 0 },
  cell_diff_v: 0,
  accel: {
    measuring: false,
    elapsed_s: 0,
    current_0_60_s: null,
    current_0_100_s: null,
    last_0_60_s: null,
    last_0_100_s: null,
    best_0_60_s: null,
  },
  status: {
    vesc_connected: false,
    bms_lost: true,
    is_raspberry: false,
    mock_mode: true,
    can_shutdown: false,
  },
};

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function formatNumber(value, digits = 1) {
  if (!Number.isFinite(value)) return "0";
  return Number(value).toFixed(digits);
}

function formatAccel(value) {
  if (value === null || value === undefined || !Number.isFinite(value)) return "-";
  return `${Number(value).toFixed(2)} c`;
}

function tempClass(temp) {
  if (temp >= 80) return "is-hot";
  if (temp >= 60) return "is-warm";
  return "is-cool";
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
  const [theme, setTheme] = useState(() => localStorage.getItem("speedo_theme") || "dark");
  const [locked, setLocked] = useState(false);
  const [actionMessage, setActionMessage] = useState("");

  const latestRef = useRef(EMPTY);
  const uiFps = Math.max(5, Number(import.meta.env.VITE_UI_FPS || 10));
  const backendPort = import.meta.env.VITE_BACKEND_PORT || "9400";
  const backendBase = `${window.location.protocol}//${window.location.hostname}:${backendPort}`;

  useEffect(() => {
    localStorage.setItem("speedo_theme", theme);
  }, [theme]);

  useEffect(() => {
    let ws;
    let reconnectTimer;
    let renderTimer;
    let disposed = false;

    const connect = () => {
      const explicit = import.meta.env.VITE_WS_URL;
      const defaultUrl = `${window.location.protocol === "https:" ? "wss" : "ws"}://${window.location.hostname}:${backendPort}/ws`;
      const url = explicit || defaultUrl;

      ws = new WebSocket(url);
      ws.onopen = () => setConnected(true);
      ws.onmessage = (event) => {
        try {
          latestRef.current = JSON.parse(event.data);
        } catch {
          // ignore malformed payloads
        }
      };
      ws.onerror = () => ws.close();
      ws.onclose = () => {
        setConnected(false);
        if (!disposed) reconnectTimer = setTimeout(connect, 900);
      };
    };

    connect();
    renderTimer = setInterval(() => {
      setSnapshot((prev) => {
        const next = latestRef.current;
        if (!next || next.timestamp === prev.timestamp) return prev;
        return next;
      });
    }, Math.round(1000 / uiFps));

    return () => {
      disposed = true;
      clearTimeout(reconnectTimer);
      clearInterval(renderTimer);
      if (ws && ws.readyState < 2) ws.close();
    };
  }, [uiFps, backendPort]);

  const speed = Math.round(snapshot.speed_kmh || 0);
  const shownSpeed = locked ? "LOCK" : speed;

  const accel = snapshot.accel || EMPTY.accel;
  const accel60 = accel.current_0_60_s ?? accel.last_0_60_s;
  const accel100 = accel.current_0_100_s ?? accel.last_0_100_s;
  const accel60Display =
    accel.measuring && accel.current_0_60_s === null ? `${Number(accel.elapsed_s || 0).toFixed(2)} c...` : formatAccel(accel60);
  const accel100Display =
    accel.measuring && accel.current_0_60_s !== null && accel.current_0_100_s === null
      ? `${Number(accel.elapsed_s || 0).toFixed(2)} c...`
      : formatAccel(accel100);
  const accelLiveDisplay = accel.measuring ? `${Number(accel.elapsed_s || 0).toFixed(2)} c` : "-";

  const bmsTemps = [
    snapshot.bms_temp.mosfet_temp,
    snapshot.bms_temp.balance_temp,
    snapshot.bms_temp.external_temp_0,
    snapshot.bms_temp.external_temp_1,
    snapshot.bms_temp.external_temp_2,
    snapshot.bms_temp.external_temp_3,
  ];

  const voltageLineItems = [
    `V ${formatNumber(snapshot.battery.voltage_v, 1)}`,
    `Sag ${formatNumber(snapshot.battery.sag_v, 1)}`,
    `Cell #${snapshot.weak_cell.index || 0}`,
    `${formatNumber(snapshot.weak_cell.voltage_v, 3)}V`,
    `Diff ${formatNumber(snapshot.cell_diff_v, 3)}`,
  ];

  const handlePower = async () => {
    if (!snapshot.status?.can_shutdown) {
      setActionMessage("Shutdown disabled");
      return;
    }

    setActionMessage("Shutting down...");
    try {
      const response = await fetch(`${backendBase}/api/system/shutdown`, { method: "POST" });
      const result = await response.json();
      setActionMessage(result?.ok ? "Shutdown command sent" : result?.reason || "Shutdown failed");
    } catch {
      setActionMessage("Shutdown request failed");
    }
  };

  const toggleTheme = () => {
    setTheme((prev) => (prev === "dark" ? "light" : "dark"));
  };

  return (
    <main className={`app-shell theme-${theme}`}>
      <header className="top-controls">
        <button className="circle-btn theme-btn" onClick={toggleTheme} aria-label="Toggle theme" />
        <button className={`circle-btn lock-btn ${locked ? "active" : ""}`} onClick={() => setLocked((v) => !v)} aria-label="Lock mode" />
        <button className="circle-btn power-btn" onClick={handlePower} aria-label="Power off" />
      </header>

      <section className="speed-section">
        <div className={`speed-value ${locked ? "lock-text" : ""}`}>{shownSpeed}</div>
        <SpeedBar speed={locked ? 0 : speed} />
      </section>

      <section className="current-section">
        <div className="bms-current-big">{Math.round(locked ? 0 : snapshot.bms_current_a || 0)}</div>
        <div className="power-small">{Math.round(locked ? 0 : snapshot.power_w)} W</div>
      </section>

      <section className="temp-block">
        <div className="temp-line">
          <span className="temp-label">МК</span>
          <span className={tempClass(snapshot.slave.temp_motor)}>{snapshot.slave.temp_motor}°</span>
          <span className={tempClass(snapshot.master.temp_motor)}>{snapshot.master.temp_motor}°</span>
          <span className="temp-label temp-gap">К</span>
          <span className={tempClass(snapshot.slave.temp)}>{snapshot.slave.temp}°</span>
          <span className={tempClass(snapshot.master.temp)}>{snapshot.master.temp}°</span>
        </div>
        <div className="temp-line bms-line">
          <span className="temp-label">BMS/Б</span>
          {bmsTemps.map((value, idx) => (
            <span key={idx} className={tempClass(value)}>
              {value}°
            </span>
          ))}
        </div>
      </section>

      <section className="voltage-line">
        {voltageLineItems.map((item, idx) => (
          <span key={idx}>{item}</span>
        ))}
      </section>

      <section className="accel-block">
        <div className="accel-item">
          <span>0-60</span>
          <strong>{accel60Display}</strong>
        </div>
        <div className="accel-item">
          <span>0-100</span>
          <strong>{accel100Display}</strong>
        </div>
        <div className="accel-item">
          <span>Замер</span>
          <strong>{accelLiveDisplay}</strong>
        </div>
      </section>

      <footer className="status-line">
        <span>{connected ? "WS ok" : "WS reconnect"}</span>
        <span>{snapshot.status?.vesc_connected ? "VESC" : "VESC off"}</span>
        <span>{snapshot.status?.bms_lost ? "BMS lost" : "BMS"}</span>
        <span>{snapshot.status?.mock_mode ? "DEBUG" : "LIVE"}</span>
        {actionMessage ? <span>{actionMessage}</span> : null}
      </footer>
    </main>
  );
}
