from __future__ import annotations

import asyncio
import glob
import math
import os
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


class SerialProtocolError(Exception):
    pass


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


class TelemetryEngine:
    def __init__(self, state: TelemetryState):
        self.state = state
        self.stop_event = threading.Event()
        self.threads: list[threading.Thread] = []

        self.vesc_port_override = os.getenv("VESC_PORT_OVERRIDE", "/tmp/vesc-ble")
        self.bms_port_override = os.getenv("BMS_PORT_OVERRIDE", "/tmp/bms-ble")
        self.debug_mock_mode = os.getenv("DEBUG_MOCK", "auto").strip().lower()

    def start(self) -> None:
        self.threads = [
            threading.Thread(target=self._vesc_worker, daemon=True, name="vesc-reader"),
            threading.Thread(target=self._bms_worker, daemon=True, name="bms-reader"),
            threading.Thread(target=self._mock_worker, daemon=True, name="mock-reader"),
            threading.Thread(target=self._autosave_worker, daemon=True, name="odometer-autosave"),
        ]
        for thread in self.threads:
            thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        for thread in self.threads:
            thread.join(timeout=1.0)
        self.state.save_odometer()

    def _is_mac(self) -> bool:
        return sys.platform == "darwin"

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
                        timeout=DEFAULT_VESC_SERIAL_TIMEOUT,
                        write_timeout=DEFAULT_VESC_SERIAL_TIMEOUT,
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

                while not self.stop_event.is_set():
                    master_raw = self._request_vesc_payload(ser, packet_master)
                    parsed_master = parse_vesc_payload(master_raw)
                    if parsed_master is None:
                        raise SerialProtocolError("VESC master parse failed")

                    self.state.update_vesc_master(*parsed_master)

                    slave_raw = self._request_vesc_payload(ser, packet_slave)
                    parsed_slave = parse_vesc_payload(slave_raw)
                    if parsed_slave is None:
                        raise SerialProtocolError("VESC slave parse failed")

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

                    time.sleep(0.05)
            except Exception as exc:
                print(f"[VESC] reconnect reason: {exc}", flush=True)
                time.sleep(1.0)
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
        if self.debug_mock_mode in {"0", "false", "off", "no"}:
            return

        if self.debug_mock_mode in {"1", "true", "on", "yes"}:
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
    return app.state.telemetry_state.snapshot()


@app.websocket("/ws")
async def telemetry_ws(websocket: WebSocket) -> None:
    await websocket.accept()
    try:
        while True:
            payload = app.state.telemetry_state.snapshot()
            await websocket.send_json(payload)
            await asyncio.sleep(0.05)
    except WebSocketDisconnect:
        return


dist_dir = Path(__file__).resolve().parents[2] / "frontend" / "dist"
if dist_dir.exists():
    app.mount("/", StaticFiles(directory=dist_dir, html=True), name="frontend")
