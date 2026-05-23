from __future__ import annotations

import asyncio
import glob
import math
import os
import subprocess
import struct
import sys
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Iterator

import serial
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from .telemetry import CELL_COUNT, TelemetryState

PACKET_INDEX_FOR_VESC = 47
SLAVE_CAN_ID = 15
POLE_PAIRS = 15
COMM_GET_VALUES = PACKET_INDEX_FOR_VESC
DEFAULT_VESC_SERIAL_TIMEOUT = 0.5
DEFAULT_FARDRIVER_SERIAL_TIMEOUT = 0.2

FARDRIVER_FLASH_READ_ADDR = [
    0xE2,
    0xE8,
    0xEE,
    0x00,
    0x06,
    0x0C,
    0x12,
    0xE2,
    0xE8,
    0xEE,
    0x18,
    0x1E,
    0x24,
    0x2A,
    0xE2,
    0xE8,
    0xEE,
    0x30,
    0x5D,
    0x63,
    0x69,
    0xE2,
    0xE8,
    0xEE,
    0x7C,
    0x82,
    0x88,
    0x8E,
    0xE2,
    0xE8,
    0xEE,
    0x94,
    0x9A,
    0xA0,
    0xA6,
    0xE2,
    0xE8,
    0xEE,
    0xAC,
    0xB2,
    0xB8,
    0xBE,
    0xE2,
    0xE8,
    0xEE,
    0xC4,
    0xCA,
    0xD0,
    0xE2,
    0xE8,
    0xEE,
    0xD6,
    0xDC,
    0xF4,
    0xFA,
]


class SerialProtocolError(Exception):
    pass


def detect_raspberry_pi() -> bool:
    if sys.platform != "linux":
        return False
    try:
        model_path = Path("/proc/device-tree/model")
        if not model_path.exists():
            return False
        model = model_path.read_text(encoding="utf-8", errors="ignore").lower()
        return "raspberry pi" in model
    except Exception:
        return False


def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def pack_comm_get_values(can_id: int | None = None) -> bytes:
    payload = bytes([COMM_GET_VALUES])
    if can_id is not None:
        payload = bytes([0, can_id]) + payload
    packet = bytearray([2, len(payload)])
    packet.extend(payload)
    checksum = crc16(payload)
    packet.extend([(checksum >> 8) & 0xFF, checksum & 0xFF, 3])
    return bytes(packet)


def pack_packet_slave(payload: bytes) -> bytes:
    packet = bytearray([2, len(payload)])
    packet.extend(payload)
    checksum = crc16(payload)
    packet.extend([(checksum >> 8) & 0xFF, checksum & 0xFF, 3])
    return bytes(packet)


def parse_vesc_payload(payload: bytes) -> tuple[float, float, float, float, float, float, float] | None:
    try:
        if len(payload) < 24:
            return None

        shift = 0
        mos_temp = struct.unpack_from(">h", payload, shift)[0] / 10.0
        shift += 2
        motor_temp = struct.unpack_from(">h", payload, shift)[0] / 10.0
        shift += 2
        motor_current = struct.unpack_from(">i", payload, shift)[0] / 100.0
        shift += 4
        input_current = struct.unpack_from(">i", payload, shift)[0] / 100.0
        shift += 4
        duty_cycle = struct.unpack_from(">h", payload, shift)[0] / 10.0
        shift += 2
        wheel_rpm = struct.unpack_from(">i", payload, shift)[0] / POLE_PAIRS
        shift += 4

        shift += 4  # speed (unused)
        input_voltage = struct.unpack_from(">h", payload, shift)[0] / 10.0

        return (
            wheel_rpm,
            input_current,
            duty_cycle,
            input_voltage,
            motor_current,
            mos_temp,
            motor_temp,
        )
    except Exception:
        return None


def parse_temperatures(packet: bytes) -> dict[str, int]:
    def to_temp(offset: int) -> int:
        return (packet[offset] << 8) | packet[offset + 1]

    return {
        "mosfet_temp": to_temp(91),
        "balance_temp": to_temp(93),
        "external_temp_0": to_temp(95),
        "external_temp_1": to_temp(97),
        "external_temp_2": to_temp(99),
        "external_temp_3": to_temp(101),
    }


def pack_fardriver_old_command(command: int, sub_command: int, value_1: int = 0, value_2: int = 0) -> bytes:
    packet = bytearray(
        [
            0xAA,
            command & 0xFF,
            (~command) & 0xFF,
            sub_command & 0xFF,
            value_1 & 0xFF,
            value_2 & 0xFF,
            0,
            0,
        ]
    )
    checksum = sum(packet[:6]) & 0xFF
    packet[6] = checksum
    packet[7] = (~checksum) & 0xFF
    return bytes(packet)


def _read_i16_le(data: bytes, offset: int) -> int:
    return struct.unpack_from("<h", data, offset)[0]


def _read_u16_le(data: bytes, offset: int) -> int:
    return struct.unpack_from("<H", data, offset)[0]


def _read_u24_be(data: bytes, offset: int) -> int:
    return (data[offset] << 16) | (data[offset + 1] << 8) | data[offset + 2]


class FardriverDecoder:
    """
    FarDriver sends rotating 16-byte status records over the same UART stream that
    the Bluetooth dongle exposes. The public reverse-engineering notes map ids to
    memory addresses; this decoder keeps the latest useful values and emits them
    in the shape expected by TelemetryState.
    """

    def __init__(self, speed_source: str = "frame"):
        self.speed_source = speed_source
        self.speed_kmh: float | None = None
        self.frame_speed_kmh: float | None = None
        self.wheel_speed_kmh: float | None = None
        self.battery_current: float | None = None
        self.phase_current: float | None = None
        self.duty_cycle: float | None = None
        self.input_voltage: float | None = None
        self.rpm: float | None = None
        self.mos_temp: float | None = None
        self.motor_temp: float | None = None
        self._measure_speed: int | None = None
        self._wheel_ratio: int | None = None
        self._wheel_radius: int | None = None
        self._wheel_width: int | None = None
        self._rate_ratio: int | None = None

    def apply_frame(self, frame: bytes) -> dict[str, float | None] | None:
        if len(frame) != 16 or frame[0] != 0xAA:
            return None

        flags = frame[1] >> 6
        frame_id = frame[1] & 0x3F
        if flags != 2:
            return None

        data = frame[2:14]
        if frame_id == 0x37:
            self._apply_gather_frame(data)
        elif frame_id < len(FARDRIVER_FLASH_READ_ADDR):
            self._apply_rotating_frame(FARDRIVER_FLASH_READ_ADDR[frame_id], data)
        else:
            return None

        self._select_speed()
        return self.snapshot()

    def _apply_gather_frame(self, data: bytes) -> None:
        # 010 Editor template: speed / 1000, current / 50 A, voltage / 5 V.
        _, speed_raw, current_raw, voltage_raw, _, modulation_raw, _ = struct.unpack("<hhhhhBB", data)
        self.frame_speed_kmh = max(0.0, speed_raw / 1000.0)
        self.battery_current = current_raw / 50.0
        self.input_voltage = voltage_raw / 5.0
        self.duty_cycle = max(0.0, min(100.0, modulation_raw / 0.2))

    def _apply_rotating_frame(self, addr: int, data: bytes) -> None:
        if addr == 0xE2:
            self.duty_cycle = max(0.0, min(100.0, data[4] * 100.0 / 128.0))
            self._measure_speed = _read_u16_le(data, 6)
            self.rpm = float(self._measure_speed)
            self._recompute_wheel_speed()
        elif addr == 0xE8:
            self.input_voltage = _read_i16_le(data, 0) / 10.0
            self.battery_current = _read_i16_le(data, 4) / 4.0
        elif addr == 0xEE:
            phase_a = self._phase_current_from_raw(_read_u24_be(data, 4))
            phase_c = self._phase_current_from_raw(_read_u24_be(data, 7))
            self.phase_current = max(phase_a, phase_c)
            volts = _read_i16_le(data, 10) / 16.0
            if volts > 0:
                self.input_voltage = volts
        elif addr == 0xD0:
            self._wheel_ratio = data[4]
            self._wheel_radius = data[5]
            self._wheel_width = data[7]
            self._rate_ratio = _read_u16_le(data, 8)
            self._recompute_wheel_speed()
        elif addr == 0xD6:
            self.mos_temp = float(_read_i16_le(data, 10))
        elif addr == 0xF4:
            self.motor_temp = float(_read_i16_le(data, 0))

    @staticmethod
    def _phase_current_from_raw(raw: int) -> float:
        return 1.953125 * math.sqrt(max(0, raw))

    def _recompute_wheel_speed(self) -> None:
        if not all(
            value is not None
            for value in [self._measure_speed, self._wheel_ratio, self._wheel_radius, self._wheel_width, self._rate_ratio]
        ):
            return
        if not self._rate_ratio:
            return

        speed_factor = 0.00376991136 * (
            float(self._wheel_radius) * 1270.0 + float(self._wheel_width) * float(self._wheel_ratio)
        )
        self.wheel_speed_kmh = max(0.0, float(self._measure_speed) * speed_factor / float(self._rate_ratio))

    def _select_speed(self) -> None:
        if self.speed_source == "wheel":
            self.speed_kmh = self.wheel_speed_kmh if self.wheel_speed_kmh is not None else self.frame_speed_kmh
        else:
            self.speed_kmh = self.frame_speed_kmh if self.frame_speed_kmh is not None else self.wheel_speed_kmh

    def snapshot(self) -> dict[str, float | None]:
        return {
            "speed_kmh": self.speed_kmh,
            "battery_current": self.battery_current,
            "phase_current": self.phase_current,
            "duty_cycle": self.duty_cycle,
            "input_voltage": self.input_voltage,
            "rpm": self.rpm,
            "mos_temp": self.mos_temp,
            "motor_temp": self.motor_temp,
        }


class TelemetryEngine:
    def __init__(self, state: TelemetryState):
        self.state = state
        self.stop_event = threading.Event()
        self.threads: list[threading.Thread] = []
        self.is_macos = sys.platform == "darwin"
        self.is_raspberry = detect_raspberry_pi()
        self.force_mock_mode = False
        self.hardware_enabled = True

        self.vesc_port_override = os.getenv("VESC_PORT_OVERRIDE", "/tmp/vesc-ble")
        self.bms_port_override = os.getenv("BMS_PORT_OVERRIDE", "/tmp/bms-ble")
        self.debug_mock_mode = os.getenv("DEBUG_MOCK", "auto").strip().lower()
        self.vesc_serial_timeout = float(os.getenv("VESC_SERIAL_TIMEOUT", "0.12"))
        self.enable_slave_poll = os.getenv("ENABLE_SLAVE_POLL", "1").strip().lower() not in {"0", "false", "off", "no"}
        self.slave_poll_every = max(1, int(os.getenv("SLAVE_POLL_EVERY", "3")))

    def start(self) -> None:
        force_mock = self.debug_mock_mode in {"1", "true", "on", "yes"} or self.is_macos
        self.force_mock_mode = force_mock
        start_hw = not force_mock
        self.hardware_enabled = start_hw

        self.threads = []
        if start_hw:
            self.threads.extend(
                [
                    threading.Thread(target=self._vesc_worker, daemon=True, name="vesc-reader"),
                    threading.Thread(target=self._bms_worker, daemon=True, name="bms-reader"),
                ]
            )
        else:
            print("[ENGINE] mock mode enabled (hardware readers disabled)", flush=True)
        self.threads.extend(
            [
                threading.Thread(target=self._mock_worker, daemon=True, name="mock-reader"),
                threading.Thread(target=self._autosave_worker, daemon=True, name="odometer-autosave"),
            ]
        )
        for thread in self.threads:
            thread.start()

    def runtime_meta(self) -> dict[str, bool]:
        can_shutdown = self.is_raspberry and (not self.force_mock_mode)
        return {
            "is_raspberry": self.is_raspberry,
            "mock_mode": self.force_mock_mode,
            "can_shutdown": can_shutdown,
        }

    def stop(self) -> None:
        self.stop_event.set()
        for thread in self.threads:
            thread.join(timeout=1.0)
        self.state.save_odometer()

    def _is_mac(self) -> bool:
        return self.is_macos

    def _iter_vesc_ports(self) -> Iterator[str]:
        seen: set[str] = set()

        def emit(value: str | None) -> Iterator[str]:
            if value and value not in seen:
                seen.add(value)
                yield value

        yield from emit(os.getenv("VESC_SERIAL_PORT"))
        yield from emit(self.vesc_port_override)

        patterns = ["/dev/tty.usbmodem*", "/dev/tty.*Bluetooth*"] if self._is_mac() else ["/dev/rfcomm*", "/dev/ttyACM*"]
        for pattern in patterns:
            for candidate in sorted(glob.glob(pattern)):
                # На macOS этот системный порт не содержит VESC и всегда дает timeout.
                if "Bluetooth-Incoming-Port" in candidate:
                    continue
                yield from emit(candidate)

        fallback = "/dev/tty.usbmodem3041" if self._is_mac() else "/dev/ttyACM0"
        yield from emit(fallback)

    def _iter_bms_ports(self) -> Iterator[str]:
        seen: set[str] = set()

        def emit(value: str | None) -> Iterator[str]:
            if value and value not in seen:
                seen.add(value)
                yield value

        yield from emit(os.getenv("BMS_SERIAL_PORT"))
        yield from emit(self.bms_port_override)

        for pattern in ["/dev/rfcomm*", "/dev/ttyUSB*", "/dev/ttyS*"]:
            for candidate in sorted(glob.glob(pattern)):
                yield from emit(candidate)

        yield from emit("/dev/ttyUSB0")

    def _request_vesc_payload(self, ser: serial.Serial, packet: bytes) -> bytes:
        ser.write(packet)
        ser.flush()

        header = ser.read(2)
        if len(header) != 2:
            raise SerialProtocolError("VESC timeout")
        if header[0] != 2:
            raise SerialProtocolError(f"VESC invalid frame start: {header[0]}")

        size = header[1]
        frame = ser.read(size + 3)
        if len(frame) != size + 3:
            raise SerialProtocolError("VESC incomplete frame")
        if frame[-1] != 3:
            raise SerialProtocolError("VESC invalid frame end")

        payload = frame[:-3]
        if not payload or payload[0] != COMM_GET_VALUES:
            raise SerialProtocolError("VESC unexpected command in response")

        return payload[1:]

    def _vesc_worker(self) -> None:
        packet_master = pack_comm_get_values()
        slave_payload = struct.pack(">BBB", 0x22, SLAVE_CAN_ID, COMM_GET_VALUES)
        packet_slave = pack_packet_slave(slave_payload)

        while not self.stop_event.is_set():
            ser = None
            for port in self._iter_vesc_ports():
                try:
                    ser = serial.Serial(
                        port,
                        115200,
                        timeout=self.vesc_serial_timeout,
                        write_timeout=self.vesc_serial_timeout,
                    )
                    print(f"[VESC] connected: {port}", flush=True)
                    break
                except Exception:
                    ser = None

            if ser is None:
                time.sleep(1.5)
                continue

            try:
                try:
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()
                except Exception:
                    pass

                consecutive_master_errors = 0
                consecutive_slave_errors = 0
                slave_backoff_until = 0.0
                cycle = 0
                while not self.stop_event.is_set():
                    try:
                        master_raw = self._request_vesc_payload(ser, packet_master)
                        parsed_master = parse_vesc_payload(master_raw)
                        if parsed_master is None:
                            raise SerialProtocolError("VESC master parse failed")
                        self.state.update_vesc_master(*parsed_master)
                        consecutive_master_errors = 0
                    except Exception as exc:
                        consecutive_master_errors += 1
                        if consecutive_master_errors >= 6:
                            raise SerialProtocolError(f"VESC master unstable: {exc}") from exc
                        time.sleep(0.01)
                        continue

                    cycle += 1
                    now_ts = time.time()
                    should_poll_slave = (
                        self.enable_slave_poll
                        and now_ts >= slave_backoff_until
                        and (cycle % self.slave_poll_every == 0)
                    )
                    if should_poll_slave:
                        try:
                            slave_raw = self._request_vesc_payload(ser, packet_slave)
                            parsed_slave = parse_vesc_payload(slave_raw)
                            if parsed_slave is not None:
                                (
                                    _,
                                    input_current,
                                    duty_cycle,
                                    _,
                                    motor_current,
                                    mos_temp,
                                    motor_temp,
                                ) = parsed_slave
                                self.state.update_vesc_slave(
                                    input_current=input_current,
                                    duty_cycle=duty_cycle,
                                    motor_current=motor_current,
                                    mos_temp=mos_temp,
                                    motor_temp=motor_temp,
                                )
                                consecutive_slave_errors = 0
                        except Exception:
                            # slave может временно молчать: не роняем master-поток
                            consecutive_slave_errors += 1
                            if consecutive_slave_errors >= 6:
                                slave_backoff_until = time.time() + 5.0
                                consecutive_slave_errors = 0

                    time.sleep(0.02)
            except Exception as exc:
                print(f"[VESC] reconnect reason: {exc}", flush=True)
                time.sleep(0.4)
            finally:
                try:
                    ser.close()
                except Exception:
                    pass

    def _bms_worker(self) -> None:
        request_frame = b"\x5A\x5A\x00\x00\x00\x00"

        while not self.stop_event.is_set():
            ser = None
            for port in self._iter_bms_ports():
                try:
                    ser = serial.Serial(port, 19200, timeout=0.2, write_timeout=0.2)
                    print(f"[BMS] connected: {port}", flush=True)
                    break
                except Exception:
                    ser = None

            if ser is None:
                time.sleep(1.5)
                continue

            try:
                try:
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()
                except Exception:
                    pass

                while not self.stop_event.is_set():
                    ser.write(request_frame)
                    packet = ser.read(140)

                    if len(packet) != 140 or not packet.startswith(b"\xAA\x55\xAA\xFF"):
                        raise SerialProtocolError("BMS invalid frame")

                    total_voltage = int.from_bytes(packet[4:6], byteorder="big", signed=False) * 0.1
                    current_raw = int.from_bytes(packet[72:74], byteorder="big", signed=True)
                    current = current_raw * 0.1

                    temps = parse_temperatures(packet)
                    cells: list[float] = []
                    for idx in range(CELL_COUNT):
                        raw = int.from_bytes(packet[6 + idx * 2 : 8 + idx * 2], byteorder="big", signed=False)
                        cells.append(raw * 0.001)

                    self.state.update_bms(total_voltage=total_voltage, current=current, temperatures=temps, cells_v=cells)
                    time.sleep(0.08)
            except Exception as exc:
                print(f"[BMS] reconnect reason: {exc}", flush=True)
                time.sleep(1.0)
            finally:
                try:
                    ser.close()
                except Exception:
                    pass

    def _autosave_worker(self) -> None:
        while not self.stop_event.is_set():
            time.sleep(30)
            try:
                self.state.save_odometer()
            except Exception as exc:
                print(f"[SAVE] failed: {exc}", flush=True)

    def _mock_worker(self) -> None:
        if self.is_macos:
            force_mock = True
        elif self.debug_mock_mode in {"0", "false", "off", "no"}:
            return
        elif self.debug_mock_mode in {"1", "true", "on", "yes"}:
            force_mock = True
        else:
            force_mock = False

        while not self.stop_event.is_set():
            snap = self.state.snapshot()
            stale = (not snap["status"]["vesc_connected"]) and bool(snap["status"]["bms_lost"])
            if force_mock or stale:
                phase = (math.sin(time.time() * 0.55) + 1.0) / 2.0
                self.state.apply_mock_frame(phase)
            time.sleep(0.1)


def resolve_main_data_path() -> Path:
    override = os.getenv("MAIN_DATA_FILE")
    if override:
        return Path(override).expanduser().resolve()

    repo_root_candidate = Path(__file__).resolve().parents[3] / "mainData.txt"
    if repo_root_candidate.exists():
        return repo_root_candidate

    return Path(__file__).resolve().parents[1] / "mainData.txt"


@asynccontextmanager
async def lifespan(app: FastAPI):
    state = TelemetryState(resolve_main_data_path())
    engine = TelemetryEngine(state)
    app.state.telemetry_state = state
    app.state.telemetry_engine = engine
    engine.start()
    try:
        yield
    finally:
        engine.stop()


app = FastAPI(title="VESC Web Speedometer", lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/api/health")
async def health() -> dict[str, bool]:
    return {"ok": True}


@app.get("/api/telemetry")
async def telemetry_snapshot() -> dict:
    payload = app.state.telemetry_state.snapshot()
    payload.setdefault("status", {}).update(app.state.telemetry_engine.runtime_meta())
    return payload


@app.post("/api/system/shutdown")
async def system_shutdown() -> dict:
    engine: TelemetryEngine = app.state.telemetry_engine
    meta = engine.runtime_meta()
    if not meta["can_shutdown"]:
        return {"ok": False, "performed": False, "reason": "shutdown disabled in this mode"}

    try:
        subprocess.Popen(["sudo", "shutdown", "now"])
        return {"ok": True, "performed": True}
    except Exception as exc:
        return {"ok": False, "performed": False, "reason": str(exc)}


@app.websocket("/ws")
async def telemetry_ws(websocket: WebSocket) -> None:
    ws_hz = max(1.0, float(os.getenv("WS_HZ", "12")))
    interval = 1.0 / ws_hz
    await websocket.accept()
    try:
        while True:
            payload = app.state.telemetry_state.snapshot()
            payload.setdefault("status", {}).update(app.state.telemetry_engine.runtime_meta())
            await websocket.send_json(payload)
            await asyncio.sleep(interval)
    except WebSocketDisconnect:
        return


dist_dir = Path(__file__).resolve().parents[2] / "frontend" / "dist"
if dist_dir.exists():
    app.mount("/", StaticFiles(directory=dist_dir, html=True), name="frontend")
