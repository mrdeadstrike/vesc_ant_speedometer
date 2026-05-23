from __future__ import annotations

import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

CELL_COUNT = 20
WHEEL_DIAMETER_M = 0.28
WHEEL_CIRCUMFERENCE_M = 3.141592653589793 * WHEEL_DIAMETER_M
ACCEL_MEASURE_TIMEOUT_S = 25.0

# Таблица процента заряда из исходного speedometer.py (для 20s батареи).
_PER_CELL_VOLTAGE_PERCENT_TABLE = [
    (4.17, 100),
    (4.053, 90),
    (3.946, 80),
    (3.845, 70),
    (3.755, 60),
    (3.673, 50),
    (3.624, 40),
    (3.592, 30),
    (3.555, 20),
    (3.477, 10),
    (3.405, 0),
]
VOLTAGE_PERCENT_TABLE = [(v * CELL_COUNT, p) for v, p in _PER_CELL_VOLTAGE_PERCENT_TABLE]


class TelemetryState:
    def __init__(self, data_file: Path):
        self._lock = threading.Lock()
        self._data_file = data_file

        self._record_counter = 1
        self._base_odometer_km = 0.0
        self._load_main_data()

        now = time.time()
        self._state: dict[str, Any] = {
            "speed_kmh": 0.0,
            "master": {
                "motor_current": 0.0,
                "battery_current": 0.0,
                "duty": 0.0,
                "temp": 0,
                "temp_motor": 0,
                "rpm": 0.0,
            },
            "slave": {
                "motor_current": 0.0,
                "battery_current": 0.0,
                "duty": 0.0,
                "temp": 0,
                "temp_motor": 0,
                "rpm": 0.0,
            },
            "battery_voltage_vesc": 0.0,
            "bms_voltage": 0.0,
            "bms_current": 0.0,
            "power": 0,
            "v_without_load": 0.0,
            "voltage_down": 0.0,
            "battery_level": 0,
            "cells_v": [4.0] * CELL_COUNT,
            "unit_diff": 0.0,
            "bad_cell_min": 0.0,
            "bad_cell_index": 0,
            "bms_temp": {
                "mosfet_temp": 0,
                "balance_temp": 0,
                "external_temp_0": 0,
                "external_temp_1": 0,
                "external_temp_2": 0,
                "external_temp_3": 0,
            },
            "trip": {
                "started": False,
                "start_ts": None,
                "trip_odometer_km": 0.0,
                "trip_speed_sum": 0.0,
                "trip_tick": 0,
                "trip_avg_speed_kmh": 0.0,
                "trip_time_s": 0,
                "max_speed_kmh": 0.0,
                "max_power_w": 0,
                "motor1_max_temp": 0,
                "motor2_max_temp": 0,
                "best_0_60_s": None,
            },
            "accel": {
                "ready": True,
                "measuring": False,
                "start_ts": None,
                "elapsed_s": 0.0,
                "current_0_60_s": None,
                "current_0_100_s": None,
                "last_0_60_s": None,
                "last_0_100_s": None,
                "best_0_60_s": None,
            },
            "last_trip_tick_ts": now,
            "last_vesc_update_ts": 0.0,
            "last_bms_update_ts": 0.0,
            "controller_type": "vesc",
        }

    def _load_main_data(self) -> None:
        try:
            text = self._data_file.read_text(encoding="utf-8")
            lines = [line.strip() for line in text.splitlines() if line.strip()]
            if lines:
                self._base_odometer_km = float(lines[0])
            if len(lines) > 1:
                self._record_counter = max(1, int(lines[1]))
        except Exception:
            self._base_odometer_km = 0.0
            self._record_counter = 1

    def save_odometer(self) -> None:
        with self._lock:
            total = self._base_odometer_km + self._state["trip"]["trip_odometer_km"]
            counter = self._record_counter

        self._data_file.parent.mkdir(parents=True, exist_ok=True)
        tmp_file = self._data_file.with_suffix(self._data_file.suffix + ".tmp")
        tmp_file.write_text(f"{total:.1f}\n{counter}\n", encoding="utf-8")
        tmp_file.replace(self._data_file)

    def update_vesc_master(
        self,
        wheel_rpm: float,
        input_current: float,
        duty_cycle: float,
        input_voltage: float,
        motor_current: float,
        mos_temp: float,
        motor_temp: float,
    ) -> None:
        with self._lock:
            speed_mps = (wheel_rpm * WHEEL_CIRCUMFERENCE_M) / 60.0
            speed_kmh = max(0.0, speed_mps * 3.6)
            self._state["speed_kmh"] = speed_kmh

            master = self._state["master"]
            master["motor_current"] = motor_current
            master["battery_current"] = input_current
            master["duty"] = duty_cycle
            master["temp"] = int(mos_temp)
            master["temp_motor"] = int(motor_temp)
            master["rpm"] = wheel_rpm
            self._state["battery_voltage_vesc"] = input_voltage
            self._state["last_vesc_update_ts"] = time.time()
            self._state["controller_type"] = "vesc"

            self._update_trip_locked()

    def update_vesc_slave(
        self,
        input_current: float,
        duty_cycle: float,
        motor_current: float,
        mos_temp: float,
        motor_temp: float,
    ) -> None:
        with self._lock:
            slave = self._state["slave"]
            slave["motor_current"] = motor_current
            slave["battery_current"] = input_current
            slave["duty"] = duty_cycle
            slave["temp"] = int(mos_temp)
            slave["temp_motor"] = int(motor_temp)
            self._state["last_vesc_update_ts"] = time.time()

    def update_fardriver(
        self,
        speed_kmh: float | None,
        battery_current: float | None,
        phase_current: float | None,
        duty_cycle: float | None,
        input_voltage: float | None,
        rpm: float | None,
        mos_temp: float | None,
        motor_temp: float | None,
    ) -> None:
        with self._lock:
            if speed_kmh is not None:
                self._state["speed_kmh"] = max(0.0, speed_kmh)

            master = self._state["master"]
            if phase_current is not None:
                master["motor_current"] = phase_current
            if battery_current is not None:
                master["battery_current"] = battery_current
            if duty_cycle is not None:
                master["duty"] = duty_cycle
            if mos_temp is not None:
                master["temp"] = int(mos_temp)
            if motor_temp is not None:
                master["temp_motor"] = int(motor_temp)
            if rpm is not None:
                master["rpm"] = rpm

            slave = self._state["slave"]
            slave["motor_current"] = 0.0
            slave["battery_current"] = 0.0
            slave["duty"] = 0.0
            slave["temp"] = 0
            slave["temp_motor"] = 0
            slave["rpm"] = 0.0

            if input_voltage is not None:
                self._state["battery_voltage_vesc"] = input_voltage
                if self._state["bms_voltage"] <= 0.0 or (time.time() - self._state["last_bms_update_ts"]) > 2.5:
                    self._state["bms_voltage"] = input_voltage

            if battery_current is not None and self._state["bms_voltage"] > 0.0:
                if (time.time() - self._state["last_bms_update_ts"]) > 2.5:
                    self._state["bms_current"] = battery_current
                    self._state["power"] = int(self._state["bms_voltage"] * battery_current)

            if input_voltage is not None and battery_current is not None and abs(battery_current) < 0.5:
                self._state["v_without_load"] = input_voltage

            if self._state["v_without_load"] > 0.0 and self._state["bms_voltage"] > 0.0:
                self._state["voltage_down"] = self._state["bms_voltage"] - self._state["v_without_load"]

            self._state["last_vesc_update_ts"] = time.time()
            self._state["controller_type"] = "fardriver"
            self._update_battery_level_locked(force=True)
            self._update_trip_locked()

    def update_bms(
        self,
        total_voltage: float,
        current: float,
        temperatures: dict[str, int],
        cells_v: list[float],
    ) -> None:
        with self._lock:
            self._state["bms_voltage"] = total_voltage
            self._state["bms_current"] = current
            self._state["power"] = int(total_voltage * current)
            self._state["bms_temp"] = temperatures
            self._state["cells_v"] = cells_v
            self._state["last_bms_update_ts"] = time.time()

            if abs(current) < 0.5:
                self._state["v_without_load"] = total_voltage

            self._state["voltage_down"] = total_voltage - self._state["v_without_load"]

            self._update_cell_stats_locked()
            self._update_battery_level_locked(force=False)

    def apply_mock_frame(self, phase: float) -> None:
        """
        Генерирует кадр отладочных данных (аналогично старому SetDebugValues),
        чтобы фронт работал без живых VESC/BMS.
        """
        p = max(0.0, min(1.0, float(phase)))
        with self._lock:
            speed = 114.0 * p
            self._state["speed_kmh"] = speed

            self._state["master"]["motor_current"] = 200.0 * p
            self._state["slave"]["motor_current"] = 180.0 * (1.0 - p)
            self._state["master"]["battery_current"] = 60.0 * p
            self._state["slave"]["battery_current"] = 40.0 * (1.0 - p)
            self._state["master"]["duty"] = min(100.0, 95.0 * p)
            self._state["slave"]["duty"] = min(100.0, 90.0 * (1.0 - p))
            self._state["master"]["rpm"] = 5000.0 * p
            self._state["slave"]["rpm"] = 4500.0 * (1.0 - p)

            self._state["master"]["temp_motor"] = int(45 + 30 * p)
            self._state["slave"]["temp_motor"] = int(43 + 28 * (1.0 - p))
            self._state["master"]["temp"] = int(40 + 22 * p)
            self._state["slave"]["temp"] = int(39 + 20 * (1.0 - p))

            bms_voltage = 74.0 - 6.0 * p
            bms_current = 55.0 * p
            self._state["bms_voltage"] = bms_voltage
            self._state["bms_current"] = bms_current
            self._state["power"] = int(bms_voltage * bms_current)

            if self._state["v_without_load"] <= 0.0:
                self._state["v_without_load"] = bms_voltage
            self._state["voltage_down"] = bms_voltage - self._state["v_without_load"]

            self._state["bms_temp"] = {
                "mosfet_temp": int(33 + 18 * p),
                "balance_temp": int(31 + 15 * p),
                "external_temp_0": int(29 + 10 * p),
                "external_temp_1": int(30 + 10 * (1.0 - p)),
                "external_temp_2": int(28 + 9 * p),
                "external_temp_3": int(30 + 8 * (1.0 - p)),
            }

            cells: list[float] = []
            base_cell = (bms_voltage / CELL_COUNT)
            for idx in range(CELL_COUNT):
                wobble = (idx - CELL_COUNT / 2.0) * 0.0008
                drift = 0.012 * (1.0 - p) if idx == 3 else 0.0
                cells.append(round(base_cell + wobble - drift, 3))
            self._state["cells_v"] = cells

            now_ts = time.time()
            self._state["last_vesc_update_ts"] = now_ts
            self._state["last_bms_update_ts"] = now_ts

            self._update_cell_stats_locked()
            self._update_battery_level_locked(force=True)
            self._update_trip_locked()

    def _update_trip_locked(self) -> None:
        trip = self._state["trip"]
        speed = self._state["speed_kmh"]
        now = time.time()

        if (not trip["started"]) and speed > 10:
            trip["started"] = True
            trip["start_ts"] = now
            self._state["last_trip_tick_ts"] = now

        if not trip["started"]:
            self._state["last_trip_tick_ts"] = now
            return

        delta_s = max(0.0, now - self._state["last_trip_tick_ts"])
        self._state["last_trip_tick_ts"] = now

        trip["trip_odometer_km"] += max(0.0, speed) * (delta_s / 3600.0)
        trip["trip_speed_sum"] += max(0.0, speed)
        trip["trip_tick"] += 1
        if trip["trip_tick"] > 0:
            trip["trip_avg_speed_kmh"] = trip["trip_speed_sum"] / trip["trip_tick"]

        if trip["start_ts"] is not None:
            trip["trip_time_s"] = max(0, int(now - trip["start_ts"]))

        trip["max_speed_kmh"] = max(trip["max_speed_kmh"], speed)
        trip["max_power_w"] = max(trip["max_power_w"], self._state["power"])
        trip["motor1_max_temp"] = max(trip["motor1_max_temp"], self._state["slave"]["temp_motor"])
        trip["motor2_max_temp"] = max(trip["motor2_max_temp"], self._state["master"]["temp_motor"])

        self._update_accel_locked(now, speed)

    def _update_accel_locked(self, now: float, speed_kmh: float) -> None:
        accel = self._state["accel"]
        trip = self._state["trip"]

        if accel["ready"] and speed_kmh > 0.5:
            accel["measuring"] = True
            accel["ready"] = False
            accel["start_ts"] = now
            accel["elapsed_s"] = 0.0
            accel["current_0_60_s"] = None
            accel["current_0_100_s"] = None

        if accel["measuring"] and accel["start_ts"] is not None:
            elapsed = max(0.0, now - float(accel["start_ts"]))
            accel["elapsed_s"] = elapsed

            if accel["current_0_60_s"] is None and speed_kmh >= 60.0:
                accel["current_0_60_s"] = elapsed
                accel["last_0_60_s"] = elapsed
                best = accel["best_0_60_s"]
                if best is None or elapsed < best:
                    accel["best_0_60_s"] = elapsed
                    trip["best_0_60_s"] = elapsed
                elif trip["best_0_60_s"] is None:
                    trip["best_0_60_s"] = best

            if accel["current_0_60_s"] is not None and accel["current_0_100_s"] is None and speed_kmh >= 100.0:
                accel["current_0_100_s"] = elapsed
                accel["last_0_100_s"] = elapsed
                accel["measuring"] = False
                accel["start_ts"] = None
                accel["elapsed_s"] = elapsed

            if elapsed > ACCEL_MEASURE_TIMEOUT_S:
                accel["measuring"] = False
                accel["start_ts"] = None

        if speed_kmh <= 0.5:
            accel["ready"] = True
            accel["measuring"] = False
            accel["start_ts"] = None
            accel["elapsed_s"] = 0.0
            accel["current_0_60_s"] = None
            accel["current_0_100_s"] = None

    def _update_cell_stats_locked(self) -> None:
        cells = self._state["cells_v"]
        if not cells:
            return

        indexed = list(enumerate(cells))
        indexed.sort(key=lambda item: (item[1], item[0]))

        bad_idx, bad_val = indexed[0]
        _, good_val = indexed[-1]

        self._state["bad_cell_index"] = bad_idx
        self._state["bad_cell_min"] = bad_val
        self._state["unit_diff"] = max(0.0, good_val - bad_val)

    def _update_battery_level_locked(self, force: bool) -> None:
        sum_motor_current = self._state["master"]["motor_current"] + self._state["slave"]["motor_current"]
        if (not force) and abs(sum_motor_current) >= 1.0:
            return

        v = self._state["bms_voltage"]
        voltages = [item[0] for item in VOLTAGE_PERCENT_TABLE]
        percents = [item[1] for item in VOLTAGE_PERCENT_TABLE]

        if v >= voltages[0]:
            self._state["battery_level"] = 100
            return
        if v <= voltages[-1]:
            self._state["battery_level"] = 0
            return

        for idx in range(len(voltages) - 1):
            v_high, v_low = voltages[idx], voltages[idx + 1]
            p_high, p_low = percents[idx], percents[idx + 1]
            if v_high >= v >= v_low:
                ratio = (v - v_low) / (v_high - v_low)
                self._state["battery_level"] = int(p_low + ratio * (p_high - p_low))
                return

    @staticmethod
    def _format_trip_time(seconds: int) -> str:
        hh = seconds // 3600
        mm = (seconds % 3600) // 60
        ss = seconds % 60
        if hh:
            return f"{hh:02d}:{mm:02d}:{ss:02d}"
        return f"{mm:02d}:{ss:02d}"

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            now = time.time()
            state = self._state
            trip = state["trip"]

            bms_lost = (now - state["last_bms_update_ts"]) > 2.5
            vesc_connected = (now - state["last_vesc_update_ts"]) <= 2.5

            total_odometer = self._base_odometer_km + trip["trip_odometer_km"]
            trip_time_s = int(trip["trip_time_s"])

            return {
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "speed_kmh": round(float(state["speed_kmh"]), 2),
                "power_w": int(state["power"]),
                "bms_current_a": round(float(state["bms_current"]), 1),
                "master": {
                    "motor_current": round(float(state["master"]["motor_current"]), 2),
                    "battery_current": round(float(state["master"]["battery_current"]), 2),
                    "duty": round(float(state["master"]["duty"]), 2),
                    "temp": int(state["master"]["temp"]),
                    "temp_motor": int(state["master"]["temp_motor"]),
                    "rpm": round(float(state["master"]["rpm"]), 1),
                },
                "slave": {
                    "motor_current": round(float(state["slave"]["motor_current"]), 2),
                    "battery_current": round(float(state["slave"]["battery_current"]), 2),
                    "duty": round(float(state["slave"]["duty"]), 2),
                    "temp": int(state["slave"]["temp"]),
                    "temp_motor": int(state["slave"]["temp_motor"]),
                    "rpm": round(float(state["slave"]["rpm"]), 1),
                },
                "battery": {
                    "voltage_v": round(float(state["bms_voltage"]), 2),
                    "no_load_voltage_v": round(float(state["v_without_load"]), 2),
                    "sag_v": round(float(state["voltage_down"]), 2),
                    "percent": int(state["battery_level"]),
                },
                "bms_temp": {
                    "mosfet_temp": int(state["bms_temp"]["mosfet_temp"]),
                    "balance_temp": int(state["bms_temp"]["balance_temp"]),
                    "external_temp_0": int(state["bms_temp"]["external_temp_0"]),
                    "external_temp_1": int(state["bms_temp"]["external_temp_1"]),
                    "external_temp_2": int(state["bms_temp"]["external_temp_2"]),
                    "external_temp_3": int(state["bms_temp"]["external_temp_3"]),
                },
                "cells_v": [round(float(v), 3) for v in state["cells_v"]],
                "weak_cell": {
                    "index": int(state["bad_cell_index"]) + 1,
                    "voltage_v": round(float(state["bad_cell_min"]), 3),
                },
                "cell_diff_v": round(float(state["unit_diff"]), 3),
                "trip": {
                    "started": bool(trip["started"]),
                    "odometer_total_km": round(total_odometer, 2),
                    "trip_km": round(float(trip["trip_odometer_km"]), 2),
                    "avg_speed_kmh": round(float(trip["trip_avg_speed_kmh"]), 2),
                    "trip_time_s": trip_time_s,
                    "trip_time": self._format_trip_time(trip_time_s),
                    "max_speed_kmh": round(float(trip["max_speed_kmh"]), 2),
                    "max_power_w": int(trip["max_power_w"]),
                    "motor1_max_temp": int(trip["motor1_max_temp"]),
                    "motor2_max_temp": int(trip["motor2_max_temp"]),
                    "best_0_60_s": round(float(trip["best_0_60_s"]), 2) if trip["best_0_60_s"] is not None else None,
                },
                "accel": {
                    "measuring": bool(state["accel"]["measuring"]),
                    "elapsed_s": round(float(state["accel"]["elapsed_s"]), 2),
                    "current_0_60_s": round(float(state["accel"]["current_0_60_s"]), 2)
                    if state["accel"]["current_0_60_s"] is not None
                    else None,
                    "current_0_100_s": round(float(state["accel"]["current_0_100_s"]), 2)
                    if state["accel"]["current_0_100_s"] is not None
                    else None,
                    "last_0_60_s": round(float(state["accel"]["last_0_60_s"]), 2) if state["accel"]["last_0_60_s"] is not None else None,
                    "last_0_100_s": round(float(state["accel"]["last_0_100_s"]), 2) if state["accel"]["last_0_100_s"] is not None else None,
                    "best_0_60_s": round(float(state["accel"]["best_0_60_s"]), 2) if state["accel"]["best_0_60_s"] is not None else None,
                },
                "status": {
                    "vesc_connected": vesc_connected,
                    "controller_connected": vesc_connected,
                    "controller_type": state["controller_type"],
                    "bms_lost": bms_lost,
                    "last_vesc_update_ms": int(max(0.0, now - state["last_vesc_update_ts"]) * 1000),
                    "last_bms_update_ms": int(max(0.0, now - state["last_bms_update_ts"]) * 1000),
                },
            }
