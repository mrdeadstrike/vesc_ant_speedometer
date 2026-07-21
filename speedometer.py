import datetime
import asyncio
import signal
import socket
import select
import pty
import subprocess
import numpy as np
import pygame
import threading
import serial
import struct
import math
import time
import uuid
import traceback
import re
import json

import glob
import urllib
import platform
import os


PACKET_INDEX_FOR_VESC = 47#4
COMM_FORWARD_CAN = 34
COMM_SET_MCCONF_TEMP = 48
COMM_SET_MCCONF_TEMP_SETUP = 49

ECO_SPEED_LIMIT_KMH = 49.0
ECO_SPEED_LIMIT_ERPM = 14200.0
DEFAULT_NORMAL_ERPM = 1000000.0
ECO_POWER_DISPLAY_LIMIT = 2400
LOCK_PASSWORD = "1024"
LOCK_KEYPAD_LAYOUT = [
  ["1", "2", "3"],
  ["4", "5", "6"],
  ["7", "8", "9"],
  ["", "0", "←"]
]

SLAVE_CAN_ID = 15

CELL_COUNT = 20

# Укажи порт вручную, если нужно (например "/dev/rfcomm0")
VESC_PORT_OVERRIDE = "/tmp/vesc-ble"
FARDRIVER_PORT_OVERRIDE = os.environ.get("FARDRIVER_SERIAL_PORT", "/tmp/fardriver-ble")
FARDRIVER_MASTER_PORT = os.environ.get("FARDRIVER_MASTER_PORT", "/tmp/fardriver-master-ble")
FARDRIVER_SLAVE_PORT = os.environ.get("FARDRIVER_SLAVE_PORT", "/tmp/fardriver-slave-ble")
BMS_PORT_OVERRIDE = "/tmp/bms-ble"
DEFAULT_VESC_SERIAL_TIMEOUT = 0.5
DEFAULT_FARDRIVER_SERIAL_TIMEOUT = 0.2
CONTROLLER_TYPE = os.environ.get("CONTROLLER_TYPE", "fardriver").strip().lower()
PYGAME_FULLSCREEN = os.environ.get("PYGAME_FULLSCREEN", "0").strip().lower() in {"1", "true", "on", "yes"}
ENABLE_BMS = os.environ.get("ENABLE_BMS", "1").strip().lower() not in {"0", "false", "off", "no"}
BMS_BACKEND = os.environ.get("BMS_BACKEND", "bleak" if CONTROLLER_TYPE == "fardriver" else "serial").strip().lower()
BMS_BLE_NAME_PREFIX = os.environ.get("BMS_BLE_NAME_PREFIX", "ANT-BLE").strip()
BMS_BLE_MAC = os.environ.get("BMS_BLE_MAC", "98:DE:1E:84:80:E6").strip()
BMS_BLE_SERVICE_UUID = os.environ.get("BMS_BLE_SERVICE_UUID", "0000ffe0-0000-1000-8000-00805f9b34fb")
BMS_BLE_NOTIFY_UUID = os.environ.get("BMS_BLE_NOTIFY_UUID", "0000ffe1-0000-1000-8000-00805f9b34fb")
BMS_BLE_WRITE_UUID = os.environ.get("BMS_BLE_WRITE_UUID", BMS_BLE_NOTIFY_UUID)
BMS_BLE_SCAN_TIMEOUT = float(os.environ.get("BMS_BLE_SCAN_TIMEOUT", "8"))
BMS_BLE_CONNECT_TIMEOUT = float(os.environ.get("BMS_BLE_CONNECT_TIMEOUT", "15"))
BMS_BLE_RESPONSE_TIMEOUT = float(os.environ.get("BMS_BLE_RESPONSE_TIMEOUT", "8"))
BMS_BLE_POLL_INTERVAL = float(os.environ.get("BMS_BLE_POLL_INTERVAL", "1"))
BMS_BLE_RECONNECT_DELAY = float(os.environ.get("BMS_BLE_RECONNECT_DELAY", "2"))
BMS_BLE_DIRECT_CONNECT = os.environ.get("BMS_BLE_DIRECT_CONNECT", "1").strip().lower() not in {"0", "false", "off", "no"}
BMS_BLE_ALLOW_SINGLE_ANONYMOUS = os.environ.get("BMS_BLE_ALLOW_SINGLE_ANONYMOUS", "1").strip().lower() not in {"0", "false", "off", "no"}
BMS_BLE_PAIR = os.environ.get("BMS_BLE_PAIR", "0").strip().lower() in {"1", "true", "on", "yes"}
BMS_BLE_DEBUG_NOTIFY = int(os.environ.get("BMS_BLE_DEBUG_NOTIFY", "4"))
BMS_BLE_DEBUG_SCAN = os.environ.get("BMS_BLE_DEBUG_SCAN", "1").strip().lower() not in {"0", "false", "off", "no"}
BMS_BLE_DEBUG_BUFFER_BYTES = int(os.environ.get("BMS_BLE_DEBUG_BUFFER_BYTES", "80"))
BMS_BLE_STARTUP_PRIORITY_TIMEOUT = float(os.environ.get("BMS_BLE_STARTUP_PRIORITY_TIMEOUT", "12"))
BMS_LOG_KEYWORD = os.environ.get("BMS_LOG_KEYWORD", "[BMS]")
FARDRIVER_BLE_BACKEND = os.environ.get("FARDRIVER_BLE_BACKEND", "bleak").strip().lower()
FARDRIVER_MASTER_MAC = os.environ.get("FARDRIVER_MASTER_MAC", "").strip()
FARDRIVER_SLAVE_MAC = os.environ.get("FARDRIVER_SLAVE_MAC", "").strip()
FARDRIVER_MASTER_MACS = os.environ.get("FARDRIVER_MASTER_MACS", "").strip()
FARDRIVER_SLAVE_MACS = os.environ.get("FARDRIVER_SLAVE_MACS", "").strip()
FARDRIVER_MASTER_NAME = os.environ.get("FARDRIVER_MASTER_NAME", "YuanQuFOC158").strip()
FARDRIVER_SLAVE_NAME = os.environ.get("FARDRIVER_SLAVE_NAME", "YuanQuFOC690").strip()
FARDRIVER_SERVICE_UUID = os.environ.get("FARDRIVER_SERVICE_UUID", "0000ffe0-0000-1000-8000-00805f9b34fb")
FARDRIVER_NOTIFY_UUID = os.environ.get("FARDRIVER_NOTIFY_UUID", "0000ffe1-0000-1000-8000-00805f9b34fb")
FARDRIVER_WRITE_UUID = os.environ.get("FARDRIVER_WRITE_UUID", FARDRIVER_NOTIFY_UUID)
FARDRIVER_NAME_PREFIX = os.environ.get("FARDRIVER_NAME_PREFIX", "YuanQuFOC")
FARDRIVER_SPEED_CONTROLLER = os.environ.get("FARDRIVER_SPEED_CONTROLLER", "master").strip().lower()
FARDRIVER_SPEED_SOURCE = os.environ.get("FARDRIVER_SPEED_SOURCE", "rpm").strip().lower()
FARDRIVER_RPM_DIVIDER = float(os.environ.get("FARDRIVER_RPM_DIVIDER", "4"))
FARDRIVER_INIT_COMMANDS_HEX = os.environ.get(
  "FARDRIVER_INIT_COMMANDS_HEX",
  "aa13ec070000b04f,aa07f8000000a956"
)
FARDRIVER_DEBUG_NOTIFY = int(os.environ.get("FARDRIVER_DEBUG_NOTIFY", "8"))
FARDRIVER_SCAN_TIMEOUT = float(os.environ.get("FARDRIVER_SCAN_TIMEOUT", "5"))
FARDRIVER_CONNECT_TIMEOUT = float(os.environ.get("FARDRIVER_CONNECT_TIMEOUT", "6"))
FARDRIVER_TRY_CONNECT_TIMEOUT = float(os.environ.get("FARDRIVER_TRY_CONNECT_TIMEOUT", "2"))
FARDRIVER_SERVICE_DISCOVERY_TIMEOUT = float(os.environ.get("FARDRIVER_SERVICE_DISCOVERY_TIMEOUT", "6"))
FARDRIVER_SERVICE_DISCOVERY_INTERVAL = float(os.environ.get("FARDRIVER_SERVICE_DISCOVERY_INTERVAL", "0.75"))
FARDRIVER_FAST_CONNECT_CANDIDATES = int(os.environ.get("FARDRIVER_FAST_CONNECT_CANDIDATES", "6"))
FARDRIVER_FINAL_CONNECT_TIMEOUT = float(os.environ.get("FARDRIVER_FINAL_CONNECT_TIMEOUT", str(FARDRIVER_CONNECT_TIMEOUT)))
FARDRIVER_FINAL_CONNECT_CANDIDATES = int(os.environ.get("FARDRIVER_FINAL_CONNECT_CANDIDATES", "2"))
# FarDriver advertises a resolvable private address, so an address from a previous
# scan is not a stable controller identifier. The exact advertised name is stable.
FARDRIVER_BLUETOOTHCTL_SCAN = os.environ.get("FARDRIVER_BLUETOOTHCTL_SCAN", "1").strip().lower() not in {"0", "false", "off", "no"}
FARDRIVER_BLUETOOTHCTL_SCAN_SECONDS = int(float(os.environ.get("FARDRIVER_BLUETOOTHCTL_SCAN_SECONDS", "8")))
FARDRIVER_LIVE_SCAN_CACHE_SECONDS = float(os.environ.get("FARDRIVER_LIVE_SCAN_CACHE_SECONDS", "0"))
FARDRIVER_ALLOW_STALE_ADDRESS_FALLBACK = os.environ.get("FARDRIVER_ALLOW_STALE_ADDRESS_FALLBACK", "0").strip().lower() in {"1", "true", "on", "yes"}
FARDRIVER_CACHE_FILE = os.environ.get("FARDRIVER_CACHE_FILE", os.path.join(os.path.dirname(os.path.abspath(__file__)), "fardriver_ble_cache.json"))
FARDRIVER_DEBUG_SCAN_ALL = os.environ.get("FARDRIVER_DEBUG_SCAN_ALL", "0").strip().lower() in {"1", "true", "on", "yes"}
FARDRIVER_DEBUG_TRACEBACK = os.environ.get("FARDRIVER_DEBUG_TRACEBACK", "1").strip().lower() not in {"0", "false", "off", "no"}
FARDRIVER_RECONNECT_DELAY = float(os.environ.get("FARDRIVER_RECONNECT_DELAY", "1"))
GPS_ENABLED_ENV = os.environ.get("GPS_ENABLED")
GPS_ENABLED = (GPS_ENABLED_ENV if GPS_ENABLED_ENV is not None else ("0" if platform.system() == "Darwin" else "1")).strip().lower() not in {"0", "false", "off", "no"}
GPS_SERIAL_PORT = os.environ.get("GPS_SERIAL_PORT", "").strip()
GPS_BAUDRATE = int(os.environ.get("GPS_BAUDRATE", "9600"))
GPS_SERIAL_TIMEOUT = float(os.environ.get("GPS_SERIAL_TIMEOUT", "1"))
GPS_STALE_SECONDS = float(os.environ.get("GPS_STALE_SECONDS", "5"))
GPS_MAX_REASONABLE_SPEED_KMH = float(os.environ.get("GPS_MAX_REASONABLE_SPEED_KMH", "200"))

GREEN_COLOR = (0, 160, 0)
GREEN_LIGHT = (0, 210, 0)
ORANGE_COLOR = (230, 135, 0)
RED_COLOR = (255, 0, 0)
GRAY = (180, 180, 180)

THEME_LIGHT = "light"
THEME_DARK = "dark"
THEMES = {
  THEME_LIGHT: {
    "background": (254, 254, 254),
    "lock_background": (250, 250, 250),
    "text_primary": (10, 10, 10),
    "text_secondary": (120, 120, 120),
  },
  THEME_DARK: {
    "background": (15, 15, 18),
    "lock_background": (20, 20, 25),
    "text_primary": (240, 240, 240),
    "text_secondary": (180, 180, 190),
  }
}
TEXT_COLOR_ROLES = {
  (0, 0, 0): "text_primary",
  (10, 10, 10): "text_primary",
  (20, 20, 20): "text_primary",
  (30, 30, 30): "text_primary",
  (80, 80, 80): "text_secondary",
}
current_theme = THEME_LIGHT

# Вертикальный сдвиг основной отрисовки (кроме верхних и нижних элементов)
CONTENT_Y_OFFSET = 130

def get_theme_color(key, default=None):
  theme = THEMES.get(current_theme, THEMES[THEME_LIGHT])
  return theme.get(key, default)

def themed_text_color(color):
  if color is None or current_theme == THEME_LIGHT:
    return color
  role = TEXT_COLOR_ROLES.get(color)
  if role:
    return THEMES[current_theme].get(role, color)
  return color

def is_dark_theme():
  return current_theme == THEME_DARK

def toggle_theme():
  global current_theme
  current_theme = THEME_DARK if current_theme == THEME_LIGHT else THEME_LIGHT

BMS_LOST = False

PREV_VALS = {
  'bms_lost': False,
  'trip_mins': 0, 
  'page_name': "SPEEDOMETER",
  'bms_temp': {
    'mosfet_temp': 0,
    'balance_temp': 0,
    'external_temp_0': 0,
    'external_temp_1': 0,
    'external_temp_2': 0,
    'external_temp_3': 0,
  },
  'motor1_temp': 0,
  'motor1_temp_get_last_time': time.time(),
  'motor2_temp': 0,
  'motor2_temp_get_last_time': time.time(),
  'mosfet_temp_get_last_time': time.time(),
  'balance_temp_get_last_time': time.time(),
  'external_temp_0_get_last_time': time.time(),
  'external_temp_1_get_last_time': time.time(),
  'external_temp_2_get_last_time': time.time(),
  'external_temp_3_get_last_time': time.time(),
  'bms_last_update': 0,
}

EXTERNAL_TEMP_KEYS = [
  'external_temp_0',
  'external_temp_1',
  'external_temp_2',
  'external_temp_3',
]

IS_RASPBERY = False
IS_MAC = False
def is_raspberry_pi():
  if platform.system() != "Linux":
    return False
  try:
    with open("/proc/device-tree/model", "r") as f:
      model = f.read().lower()
    return "raspberry pi" in model
  except:
    return False

font_y_shift = 0
if is_raspberry_pi():
  #print("✅ Это Raspberry Pi")
  font_y_shift += 2
  IS_RASPBERY = True
elif platform.system() == "Darwin":
  print("🍎 Это macOS (MacBook)")
  IS_MAC = True
else:
  print("🤔 Что-то другое")
  font_y_shift += 2

# Увеличиваем громкость TWS
def increase_tws_volume():
  sink_name = "bluez_output.41_42_D4_2C_8B_75.1"

  # Ждём, пока PipeWire активируется и sink появится
  time.sleep(10)

  # Устанавливаем громкость
  subprocess.run(["pactl", "set-sink-volume", sink_name, "120%"])
  add_speak_message("К поездке все готово")
  try:
    mirror_controller.apply_pose('unfolded')
    mirror_controller.request_update()
  except Exception as exc:
    print(f"Не удалось разложить зеркала после установки громкости: {exc}")

if IS_RASPBERY:
  threading.Thread(target=increase_tws_volume, daemon=True).start()


# Данные контроллеров
trip_start_odometer = None
trip_distance_km = 0.0
trip_avg_speed = 0.0
trip_time_start = time.time()

saved_odometer = 0.0
video_record_counter = 1
try:
  with open("mainData.txt", "r") as f:
    lines = [line.strip() for line in f.readlines() if line.strip() != ""]
    if lines:
      saved_odometer = float(lines[0])
    if len(lines) > 1:
      try:
        video_record_counter = int(lines[1])
      except ValueError:
        video_record_counter = 1
    if video_record_counter < 1:
      video_record_counter = 1
    print(f"Загружен одометр: {saved_odometer:.1f} км")
    print(f"Счётчик записей: {video_record_counter}")
except:
  saved_odometer = 0.0
  video_record_counter = 1

data = {
  'speed': 0.0,
  'speed_source_controller': 'master',
  'master': {'motor_current': 0, 'battery_current': 0, 'duty': 0, 'temp': 0, 'temp_motor': 0, 'rpm': 0},
  'slave': {'motor_current': 0, 'battery_current': 0, 'duty': 0, 'temp': 0, 'temp_motor': 0, 'rpm': 0},
  'battery_voltage': 0,
  'v_without_nagruzka': 0,
  'battery_level': 0,
  'odometer': saved_odometer,
  'trip_odometer': 0.0,
  'trip_tick': 0,
  'trip_speed_sum': 0.0,
  'trip_avg_speed': 0.0,
  'trip_time': "00:00",
  'cells_v': [4.0] * CELL_COUNT,
  'unit_diff': 0.0,
  'bad_cell_min': 0.0,
  'bad_cell_min_peak': 5,
  'bad_cell_index': 0,
  'bad_cell_min_peak_index': 0,
  'bms_temp': {
    'mosfet_temp': 0,
    'balance_temp': 0,
    'external_temp_0': 0,
    'external_temp_1': 0,
    'external_temp_2': 0,
    'external_temp_3': 0,
  },
  'bms_current': 0,
  'power': 0,
  'bms_voltage': 0,
  'voltage_down': 0,
  'gps_speed': 0.0,
  'gps_satellites': 0,
  'gps_fix': False,
  'gps_speed_last_update': 0.0,
  'gps_satellites_last_update': 0.0,
}

FARDRIVER_LATEST_LOCK = threading.Lock()
FARDRIVER_LATEST = {
  'master': {},
  'slave': {},
}
FARDRIVER_BLE_SCAN_LOCK = threading.Lock()
FARDRIVER_BLE_CONNECT_LOCK = threading.Lock()
FARDRIVER_BLE_CONTROLLER_SETUP_LOCK = threading.Lock()
BMS_INITIAL_ATTEMPT_DONE = threading.Event()
FARDRIVER_LIVE_SCAN_CACHE = {
  'updated_at': 0.0,
  'by_name': {},
}

data_trip = {
  'max_speed': 0,
  'max_power': 0,
  'best_time_0_60': 100,
  'trip_start_bettery_perc': 0,
  'max_voltage_down': 0,
  'min_cell_v': 5,
  'min_cell_v_index': 0,
  'max_unit_diff': 0,
  'motor1_max_temp': 0,
  'motor2_max_temp': 0,
}


# Таблица для расчёта процента заряда батареи (значения на ячейку)
_per_cell_voltage_percent_table = [
  (4.17, 100), (4.053, 90), (3.946, 80),
  (3.845, 70), (3.755, 60),
  (3.673, 50), (3.624, 40), (3.592, 30),
  (3.555, 20), (3.477, 10), (3.405, 0)
]
voltage_percent_table = [
  (v * CELL_COUNT, p) for v, p in _per_cell_voltage_percent_table
]

class SerialGetError(Exception):
  pass

def bms_log(message):
  print(f"{BMS_LOG_KEYWORD} {message}", flush=True)


############## VOICE RECOGNITION ###############
import subprocess
import numpy as np
import queue
import threading
import json
from vosk import Model, KaldiRecognizer

vesc_command_queue = queue.Queue()

BLUETOOTH_SUPPORTED = hasattr(socket, "AF_BLUETOOTH") and hasattr(socket, "BTPROTO_RFCOMM")

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# Настройки подключения и диапазоны держим прямо в коде
MIRROR_BT_ADDRESS = "78:1C:3C:2C:31:16"
MIRROR_BT_CHANNEL = 1
MIRROR_MIN_ANGLE = -90
MIRROR_MAX_ANGLE = 270
MIRROR_DEFAULT_LEFT = 120
MIRROR_DEFAULT_RIGHT = 120
MIRROR_FINE_STEP = 1
MIRROR_COARSE_STEP = 5

# Второй ESP32 с четырёхразрядным индикатором. По умолчанию адрес ищется
# автоматически по имени Bluetooth Classic (SPP). Указывать MAC нужно только
# если в зоне видны несколько плат с тем же именем.
SPEED_DISPLAY_BT_NAME = os.environ.get("SPEED_DISPLAY_BT_NAME", "SpeedDisplay").strip()
SPEED_DISPLAY_BT_ADDRESS = os.environ.get("SPEED_DISPLAY_BT_ADDRESS", "").strip()
SPEED_DISPLAY_BT_CHANNEL = int(os.environ.get("SPEED_DISPLAY_BT_CHANNEL", "1"))
SPEED_DISPLAY_SEND_INTERVAL = float(os.environ.get("SPEED_DISPLAY_SEND_INTERVAL", "0.05"))
SPEED_DISPLAY_SCAN_SECONDS = int(float(os.environ.get("SPEED_DISPLAY_SCAN_SECONDS", "8")))
SPEED_DISPLAY_RECONNECT_DELAY = float(os.environ.get("SPEED_DISPLAY_RECONNECT_DELAY", "3"))

# Отдельно храним сохранённые позы (сложено/разложено) в отдельном файле состояния
MIRROR_STATE_PATH = os.path.join(BASE_DIR, "mirror_state.json")
_default_mirror_state = {
  "folded": {
    "left": MIRROR_DEFAULT_LEFT,
    "right": MIRROR_DEFAULT_RIGHT
  },
  "unfolded": {
    "left": MIRROR_DEFAULT_LEFT,
    "right": MIRROR_DEFAULT_RIGHT
  }
}

def load_mirror_state():
  try:
    with open(MIRROR_STATE_PATH, "r", encoding="utf-8") as state_file:
      state = json.load(state_file)
      if isinstance(state, dict):
        return state
  except FileNotFoundError:
    pass
  except Exception as exc:
    print(f"Не удалось загрузить {MIRROR_STATE_PATH}: {exc}")
  return json.loads(json.dumps(_default_mirror_state))

def save_mirror_state(state):
  try:
    with open(MIRROR_STATE_PATH, "w", encoding="utf-8") as state_file:
      json.dump(state, state_file, ensure_ascii=False, indent=2)
  except Exception as exc:
    print(f"Не удалось сохранить {MIRROR_STATE_PATH}: {exc}")

mirror_state = load_mirror_state()
save_mirror_state(mirror_state)

def wait_mouse_release(max_wait=0.6):
  start = time.time()
  while pygame.mouse.get_pressed()[0]:
    pygame.event.pump()
    if time.time() - start > max_wait:
      break
    time.sleep(0.01)

class MirrorController:
  def __init__(self, bt_address, channel, min_angle, max_angle, default_left, default_right):
    self.bt_address = bt_address
    self.channel = channel
    self.min_angle = min_angle
    self.max_angle = max_angle
    self._pose_tolerance = 2

    self._lock = threading.Lock()
    self._target = {
      'left': self._clamp(default_left),
      'right': self._clamp(default_right)
    }
    self._actual = {
      'left': None,
      'right': None
    }
    self._connected = False
    self._status = self._initial_status()
    self._last_message = ""

    self._sock = None
    self._rx_buffer = b""
    self._should_run = True
    self._command_queue = queue.Queue()
    self._state = mirror_state
    self._poses = {
      'folded': self._load_pose('folded', default_left, default_right),
      'unfolded': self._load_pose('unfolded', default_left, default_right)
    }

    threading.Thread(target=self._worker_loop, daemon=True).start()

  def _initial_status(self):
    if not BLUETOOTH_SUPPORTED:
      return "Bluetooth недоступен"
    if not self.bt_address:
      return "Укажи bt_address в speedometer.py"
    return "Подключение..."

  def _clamp(self, angle):
    return max(self.min_angle, min(self.max_angle, int(angle)))

  def _load_pose(self, pose_name, fallback_left, fallback_right):
    pose_data = self._state.get(pose_name) or {}
    left = pose_data.get("left")
    right = pose_data.get("right")
    pose = {
      'left': self._clamp(left) if left is not None else self._clamp(fallback_left),
      'right': self._clamp(right) if right is not None else self._clamp(fallback_right)
    }
    self._state[pose_name] = {
      'left': pose['left'],
      'right': pose['right']
    }
    return pose

  def get_snapshot(self):
    with self._lock:
      return {
        'connected': self._connected,
        'status': self._status,
        'target': dict(self._target),
        'actual': dict(self._actual),
        'last_message': self._last_message,
        'poses': {
          'folded': dict(self._poses['folded']),
          'unfolded': dict(self._poses['unfolded'])
        }
      }

  def adjust_target(self, side, delta):
    if side not in self._target:
      return
    with self._lock:
      new_angle = self._target[side] + delta
    self.set_target(side, new_angle)

  def set_target(self, side, angle):
    if side not in self._target:
      return
    clamped = self._clamp(angle)
    with self._lock:
      self._target[side] = clamped
    self._enqueue_command(f"SET {self._encode_side(side)} {clamped}")

  def request_update(self):
    self._enqueue_command("GET ALL")

  def apply_pose(self, pose_name):
    if pose_name not in self._poses:
      return
    pose = self._poses[pose_name]
    for side, angle in pose.items():
      if angle is not None:
        self.set_target(side, angle)

  def save_pose(self, pose_name):
    if pose_name not in self._poses:
      return
    with self._lock:
      if pose_name == 'folded':
        self._poses['folded'] = {
          'left': self._target['left'],
          'right': self._target['right']
        }
      elif pose_name == 'unfolded':
        self._poses['unfolded'] = {
          'left': self._target['left'],
          'right': self._target['right']
        }
      self._state[pose_name] = {
        'left': self._poses[pose_name]['left'],
        'right': self._poses[pose_name]['right']
      }
    save_mirror_state(self._state)
    self._sync_pose(pose_name)

  def _encode_side(self, side):
    return 'L' if side == 'left' else 'R'

  def _decode_side(self, token):
    if token == 'L':
      return 'left'
    if token == 'R':
      return 'right'
    return None

  def _enqueue_command(self, line):
    if not line:
      return
    self._command_queue.put(line)

  def _sync_pose(self, pose_name):
    pose = self._poses.get(pose_name)
    if not pose:
      return
    left = pose.get('left')
    right = pose.get('right')
    if left is None or right is None:
      return
    self._enqueue_command(
      f"POSE {pose_name.upper()} {int(left)} {int(right)}"
    )

  def _pose_matches(self, pose_name):
    pose = self._poses.get(pose_name)
    if not pose:
      return False
    with self._lock:
      left_target = self._target['left']
      right_target = self._target['right']
    return (
      pose.get('left') is not None and
      pose.get('right') is not None and
      abs(left_target - pose['left']) <= self._pose_tolerance and
      abs(right_target - pose['right']) <= self._pose_tolerance
    )

  def toggle_pose(self):
    if self._pose_matches('folded'):
      self.apply_pose('unfolded')
    else:
      self.apply_pose('folded')

  def _worker_loop(self):
    while self._should_run:
      if not BLUETOOTH_SUPPORTED:
        time.sleep(5)
        continue

      if not self.bt_address:
        with self._lock:
          self._connected = False
          self._status = "Укажи bt_address в speedometer.py"
        time.sleep(5)
        continue

      try:
        self._connect()
        self._send_initial_sync()
        self._main_loop()
      except Exception as exc:
        with self._lock:
          self._connected = False
          self._status = f"Ошибка: {exc}"
        self._close_socket()
        time.sleep(3)

  def _connect(self):
    if self._sock:
      return

    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    sock.settimeout(8.0)
    sock.connect((self.bt_address, self.channel))
    sock.settimeout(0.2)
    self._sock = sock
    self._rx_buffer = b""
    with self._lock:
      self._connected = True
      self._status = "Подключено"

  def _send_initial_sync(self):
    for pose_name in ('folded', 'unfolded'):
      self._sync_pose(pose_name)
    for side in ('left', 'right'):
      with self._lock:
        target = self._target[side]
      self._enqueue_command(f"SET {self._encode_side(side)} {target}")
    self._enqueue_command("GET ALL")

  def _main_loop(self):
    while self._should_run and self._sock:
      self._flush_commands()
      self._poll_socket()

  def _flush_commands(self):
    if not self._sock:
      return
    try:
      while True:
        cmd = self._command_queue.get_nowait()
        payload = (cmd + "\n").encode("ascii", errors="ignore")
        self._sock.sendall(payload)
    except queue.Empty:
      pass

  def _poll_socket(self):
    if not self._sock:
      return
    try:
      chunk = self._sock.recv(128)
      if not chunk:
        raise ConnectionError("ESP32 разорвал соединение")
      self._rx_buffer += chunk
      while b"\n" in self._rx_buffer:
        line, self._rx_buffer = self._rx_buffer.split(b"\n", 1)
        decoded = line.decode("utf-8", errors="ignore").strip()
        if decoded:
          self._handle_line(decoded)
    except socket.timeout:
      pass

  def _handle_line(self, line):
    with self._lock:
      self._last_message = line
    parts = line.split()
    if not parts:
      return

    cmd = parts[0].upper()
    if cmd in ("ANGLE", "OK") and len(parts) >= 3:
      side = self._decode_side(parts[1].upper())
      if side is None:
        return
      try:
        angle = int(float(parts[2]))
      except ValueError:
        return
      with self._lock:
        self._actual[side] = self._clamp(angle)
    elif cmd == "HELLO" and len(parts) >= 3:
      try:
        left = int(float(parts[1]))
        right = int(float(parts[2]))
      except ValueError:
        return
      with self._lock:
        self._actual['left'] = self._clamp(left)
        self._actual['right'] = self._clamp(right)
        self._status = "Подключено"
    elif cmd == "POSE" and len(parts) >= 4:
      pose_name = parts[1].lower()
      try:
        left = int(float(parts[2]))
        right = int(float(parts[3]))
      except ValueError:
        return
      if pose_name in self._poses:
        with self._lock:
          self._poses[pose_name] = {
            'left': self._clamp(left),
            'right': self._clamp(right)
          }
          self._state[pose_name] = dict(self._poses[pose_name])
        save_mirror_state(self._state)
    elif cmd == "BUTTON" and len(parts) >= 2:
      event = parts[1].lower()
      if event == "toggle":
        self.toggle_pose()
    elif cmd == "ERR":
      with self._lock:
        self._status = f"Ошибка: {' '.join(parts[1:])}"
    elif cmd == "PONG":
      with self._lock:
        self._status = "Подключено"

  def _close_socket(self):
    if self._sock:
      try:
        self._sock.close()
      except:
        pass
    self._sock = None
    self._rx_buffer = b""


class SpeedDisplayController:
  """Streams the already calculated dashboard speed to the small ESP32 display.

  The protocol is deliberately tiny: an ESP32 named ``SpeedDisplay`` exposes
  Bluetooth Classic SPP and receives newline-terminated ``SPEED <kmh>`` lines.
  """

  _device_pattern = re.compile(r"Device\s+([0-9A-Fa-f:]{17})\s+(.+)$")

  def __init__(self, speed_provider, bt_name, bt_address, channel):
    self._speed_provider = speed_provider
    self._bt_name = bt_name
    self._bt_address = bt_address
    self._channel = channel
    self._cached_address = bt_address or None
    self._sock = None
    self._connected = False
    self._status = "Ожидание Bluetooth"
    self._last_error = ""
    self._lock = threading.Lock()
    self._should_run = True
    threading.Thread(target=self._worker_loop, daemon=True).start()

  def get_snapshot(self):
    with self._lock:
      return {
        "connected": self._connected,
        "status": self._status,
        "address": self._cached_address,
        "last_error": self._last_error,
      }

  def _set_status(self, status, connected=False, error=""):
    with self._lock:
      self._connected = connected
      self._status = status
      self._last_error = error

  @classmethod
  def _addresses_from_bluetoothctl(cls, output, wanted_name):
    addresses = []
    for raw_line in output.splitlines():
      match = cls._device_pattern.search(raw_line.strip())
      if not match:
        continue
      address, name = match.groups()
      if name.strip().casefold() == wanted_name.casefold():
        addresses.append(address.upper())
    return list(dict.fromkeys(addresses))

  def _bluetoothctl(self, args, timeout):
    try:
      completed = subprocess.run(
        ["bluetoothctl", *args],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=timeout,
        check=False,
      )
      return completed.stdout or ""
    except FileNotFoundError:
      raise RuntimeError("bluetoothctl не найден: установи bluez")
    except subprocess.TimeoutExpired as exc:
      return (exc.stdout or "") if isinstance(exc.stdout, str) else ""

  def _discover_address(self):
    if self._bt_address:
      return self._bt_address
    if not self._bt_name:
      raise RuntimeError("не задано имя SpeedDisplay")

    known = self._bluetoothctl(["devices"], timeout=5)
    candidates = self._addresses_from_bluetoothctl(known, self._bt_name)
    if candidates:
      return candidates[0]

    self._set_status(f"Ищу {self._bt_name}…")
    scan_output = ""
    try:
      scan_output = self._bluetoothctl(
        ["--timeout", str(SPEED_DISPLAY_SCAN_SECONDS), "scan", "on"],
        timeout=SPEED_DISPLAY_SCAN_SECONDS + 5,
      )
    finally:
      try:
        self._bluetoothctl(["scan", "off"], timeout=5)
      except RuntimeError:
        pass

    candidates = self._addresses_from_bluetoothctl(scan_output, self._bt_name)
    candidates.extend(self._addresses_from_bluetoothctl(
      self._bluetoothctl(["devices"], timeout=5), self._bt_name
    ))
    candidates = list(dict.fromkeys(candidates))
    if not candidates:
      raise RuntimeError(f"{self._bt_name} не найден")
    return candidates[0]

  def _connect(self):
    address = self._cached_address or self._discover_address()
    self._cached_address = address
    self._set_status(f"Подключение к {self._bt_name} ({address})…")

    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    sock.settimeout(8.0)
    sock.connect((address, self._channel))
    sock.setblocking(False)
    self._sock = sock
    self._set_status("Подключено", connected=True)
    print(f"SpeedDisplay: подключено к {address}", flush=True)

  def _read_speed(self):
    try:
      value = float(self._speed_provider())
    except (TypeError, ValueError):
      value = 0.0
    return max(0, min(999, int(round(value))))

  def _drain_responses(self):
    if not self._sock:
      return
    readable, _, _ = select.select([self._sock], [], [], 0)
    if not readable:
      return
    response = self._sock.recv(128)
    if not response:
      raise ConnectionError("SpeedDisplay разорвал соединение")

  def _stream_loop(self):
    last_sent_at = 0.0
    while self._should_run and self._sock:
      now = time.monotonic()
      if now - last_sent_at >= SPEED_DISPLAY_SEND_INTERVAL:
        self._sock.sendall(f"SPEED {self._read_speed()}\n".encode("ascii"))
        last_sent_at = now
      self._drain_responses()
      time.sleep(0.01)

  def _close_socket(self):
    if self._sock:
      try:
        self._sock.close()
      except OSError:
        pass
    self._sock = None

  def _worker_loop(self):
    while self._should_run:
      if not BLUETOOTH_SUPPORTED:
        self._set_status("Bluetooth Classic недоступен")
        time.sleep(10)
        continue
      try:
        self._connect()
        self._stream_loop()
      except Exception as exc:
        self._set_status(f"Ошибка: {exc}", error=str(exc))
        print(f"SpeedDisplay: {exc}", flush=True)
      finally:
        self._close_socket()
      time.sleep(SPEED_DISPLAY_RECONNECT_DELAY)


mirror_controller = MirrorController(
  bt_address=MIRROR_BT_ADDRESS,
  channel=MIRROR_BT_CHANNEL,
  min_angle=MIRROR_MIN_ANGLE,
  max_angle=MIRROR_MAX_ANGLE,
  default_left=MIRROR_DEFAULT_LEFT,
  default_right=MIRROR_DEFAULT_RIGHT
)
speed_display_controller = SpeedDisplayController(
  speed_provider=lambda: data.get('speed', 0.0),
  bt_name=SPEED_DISPLAY_BT_NAME,
  bt_address=SPEED_DISPLAY_BT_ADDRESS,
  channel=SPEED_DISPLAY_BT_CHANNEL,
)
mirror_fold_done = False

def fold_mirrors_on_exit():
  global mirror_fold_done
  if mirror_fold_done:
    return
  try:
    mirror_controller.apply_pose('folded')
    mirror_controller.request_update()
    mirror_fold_done = True
  except Exception as exc:
    print(f"Не удалось сложить зеркала при выходе: {exc}")

ERROR_LOG_PATH = "speedometer_errors.log"

def log_exception(context: str, exc: Exception):
  """
  Пишем стек-трейс в консоль и файл, чтобы не искать слепую причину зависания.
  """
  timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
  trace = "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))
  message = f"[{timestamp}] {context}: {exc}\n{trace}"
  print(message, flush=True)
  try:
    with open(ERROR_LOG_PATH, "a") as log_file:
      log_file.write(message + "\n")
  except Exception as log_exc:
    print(f"Не удалось записать лог {ERROR_LOG_PATH}: {log_exc}", flush=True)

def perform_exit(reason: str):
  """
  Унифицированное завершение приложения (и системы при полном выключении).
  """
  global running, can_start_record, recorder_proc
  if not running:
    return
  print(f"Завершаем работу: {reason}", flush=True)
  if not can_start_record and recorder_proc is not None:
    try:
      recorder_proc.send_signal(signal.SIGINT)
      recorder_proc.wait(timeout=2)
      print(">>> Запись остановлена")
    except Exception as exc:
      log_exception("Остановка записи", exc)
    finally:
      recorder_proc = None
      can_start_record = True
  try:
    fold_mirrors_on_exit()
  except Exception as exc:
    log_exception("fold_mirrors_on_exit", exc)
  time.sleep(0.4)
  try:
    SaveData()
  except Exception as exc:
    log_exception("SaveData", exc)
  running = False
  pygame.quit()
  if full_off:
    try:
      print("OFF")
      os.system('sudo shutdown now')
    except Exception as exc:
      log_exception("shutdown command", exc)

eco_mode = False
current_speed_limit_kmh = None
current_speed_limit_erpm = None
eco_toggle_was_pressed = False
theme_toggle_was_pressed = False
NORMAL_SPEED_LIMIT_KMH = None
lock_active = False
lock_input = ""
lock_message = ""
lock_restore_speed = None
lock_keypad_pressed = False
lock_prev_page = "SPEEDOMETER"
PAGE_LOCK = "LOCK"
PAGE_MIRROR = "MIRROR_ADJUST"

# === НАСТРОЙКИ ===
MODEL_PATH = "vosk-model-ru"  # путь к модели
if IS_RASPBERY:
  MODEL_PATH = "/home/dead/Documents/vesc_ant_speedometer/vosk-model-ru"
KEYWORDS = ["напряжени", "температур", "статистик"]
DEVICE_NAME = "bluez_input.71_BE_AE_97_D4_73_0"  # ← твой TWS микрофон
INPUT_SAMPLE_RATE = 8000
TARGET_SAMPLE_RATE = 16000

q = queue.Queue()

from scipy.signal import resample
def audio_reader():
  cmd = [
    "parecord",
    "--raw",
    f"--device={DEVICE_NAME}",
    "--format=s16le",
    "--channels=1",
    f"--rate={INPUT_SAMPLE_RATE}"
  ]
  with subprocess.Popen(cmd, stdout=subprocess.PIPE) as proc:
    while True:
      raw = proc.stdout.read(1600)  # 100мс при 8000 Гц
      if not raw:
        break
      samples = np.frombuffer(raw, dtype=np.int16)
      samples = samples * 2.5  # 🔊 усиление
      samples = np.clip(samples, -32768, 32767)
      resampled = resample(samples, int(len(samples) * TARGET_SAMPLE_RATE / INPUT_SAMPLE_RATE))
      resampled = np.clip(resampled, -32768, 32767).astype(np.int16)
      q.put(resampled.tobytes())

# === ОБРАБОТЧИК КОМАНД ===
def handle_command(command):
  if command == "температур":
    add_speak_message("Температура БМС")
    add_speak_message(f"Мосфеты {data['bms_temp']['mosfet_temp']} градусов")
    add_speak_message(f"Балансиры {data['bms_temp']['balance_temp']} градусов")
    for sensor_index, key in enumerate(EXTERNAL_TEMP_KEYS, start=1):
      add_speak_message(
        f"Батарея датчик {sensor_index}... {data['bms_temp'][key]} градусов"
      )
  elif command == "напряжени":
    add_speak_message(f"Напряжение " + f"{data['bms_voltage']:.1f}".replace(".", " и ") + " вольт")
    add_speak_message(f"Просадка " + f"{data['voltage_down']:.1f}".replace(".", " и ") + " вольт")
    add_speak_message(f"Слабейший ряд... " + f"{data['bad_cell_index'] + 1}")
    add_speak_message(f"минимальный заряд... " + f"{data['bad_cell_min']:.2f}".replace(".", " и ") + " вольт")
    add_speak_message(f"разбаланс... " + f"{data['unit_diff']:.2f}".replace(".", " и ") + " вольт")
  # можно добавлять другие действия

def recognition_loop():
  model = Model(MODEL_PATH)
  recognizer = KaldiRecognizer(model, TARGET_SAMPLE_RATE)
  print("🎤 Готов слушать команды!")

  while True:
    data = q.get()
    if recognizer.AcceptWaveform(data):
      result = json.loads(recognizer.Result())
      text = result.get("text", "")
      print("✅", text)
      for keyword in KEYWORDS:
        if keyword in text:
          print(f"🚨 КОМАНДА: {keyword.upper()}")
          handle_command(keyword)
    else:
      partial = json.loads(recognizer.PartialResult())
      #print("🟡", partial.get("partial", ""))

def start_voice_thread():
  threading.Thread(target=audio_reader, daemon=True).start()
  threading.Thread(target=recognition_loop, daemon=True).start()


#start_voice_thread()

############## VOICE SPEAK #####################
#sudo apt install rhvoice-russian
def speak_run(text, voice='anna', pitch=0.0, rate=0.05, volume=0.0):
  command = f'echo "{text}" | RHVoice-client -s {voice} -p {pitch} -r {rate} -v {volume} | aplay'
  subprocess.run(command, shell=True)

MESSAGES_READ_INDEX = 0
MESSAGES_TO_SPEAK = []
message_processing = False

def speak(text, on_complete=None):
  def _run():
    if IS_MAC:
      proc = subprocess.Popen(['say', text])
      proc.wait()  # ← блокирует поток до завершения воспроизведения
      if on_complete:
        on_complete()
    else:
      # например, RHVoice или pyttsx3
      speak_run(text)
      if on_complete:
        on_complete()

  threading.Thread(target=_run, daemon=True).start()

def add_speak_message(text):
  pass
  #global MESSAGES_TO_SPEAK
  #MESSAGES_TO_SPEAK.append(text)

def message_voice_done():
  global message_processing
  message_processing = False

def message_speaker():
  global message_processing
  global MESSAGES_READ_INDEX
  global MESSAGES_TO_SPEAK
  while True:
    if not message_processing:
      if len(MESSAGES_TO_SPEAK) > MESSAGES_READ_INDEX:
        message_processing = True
        speak("... ".join(MESSAGES_TO_SPEAK[MESSAGES_READ_INDEX:len(MESSAGES_TO_SPEAK)]), message_voice_done)
        MESSAGES_READ_INDEX = len(MESSAGES_TO_SPEAK)
    time.sleep(0.25)

threading.Thread(target=message_speaker, daemon=True).start()

# Параметры колеса
wheel_diameter_m = float(os.environ.get("WHEEL_DIAMETER_M", "0.28"))  # 280 мм = 0.28 м
wheel_circumference_m = math.pi * wheel_diameter_m
pole_pairs = 15

def crc16(data: bytes):
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

def pack_comm_get_values(can_id=None):
  payload = bytes([PACKET_INDEX_FOR_VESC])
  if can_id is not None:
    payload = bytes([0]) + bytes([can_id]) + payload
  payload_length = len(payload)
  packet = bytes([2]) + bytes([payload_length]) + payload
  crc = crc16(payload)
  packet += bytes([crc >> 8, crc & 0xFF])
  packet += bytes([3])
  return packet

def build_vesc_packet(payload):
  length = len(payload)
  if length > 255:
    raise ValueError("Payload too large for short VESC frame")
  packet = bytearray([2, length])
  packet.extend(payload)
  crc = crc16(payload)
  packet.extend([(crc >> 8) & 0xFF, crc & 0xFF, 3])
  return bytes(packet)

def enqueue_vesc_command(payload, can_id=None):
  if can_id is not None:
    payload = bytes([COMM_FORWARD_CAN, can_id]) + payload
  packet = build_vesc_packet(payload)
  vesc_command_queue.put(packet)

def build_mcconf_temp_payload_erpm(max_erpm, min_erpm=None,
                              is_setup=True, store=False,
                              forward_can=True, divide_by_controllers=False,
                              ack=False):
  if min_erpm is None:
    min_erpm = -max_erpm if max_erpm != 0 else 0.0

  payload = bytearray()
  payload.append(COMM_SET_MCCONF_TEMP_SETUP if is_setup else COMM_SET_MCCONF_TEMP)
  payload.append(1 if store else 0)
  payload.append(1 if forward_can else 0)
  payload.append(1 if ack else 0)
  payload.append(1 if divide_by_controllers else 0)

  values = (
    1.0,             # current_min_scale
    1.0,             # current_max_scale
    float(min_erpm),
    float(max_erpm),
    0.05,            # duty_min (default value)
    0.95,            # duty_max (default value)
    -200000.0,       # watt_min
    200000.0         # watt_max
  )

  for val in values:
    payload.extend(struct.pack('>f', float(val)))

  return bytes(payload)

def get_normal_speed_limit_kmh():
  global NORMAL_SPEED_LIMIT_KMH
  if NORMAL_SPEED_LIMIT_KMH is None:
    wheel_rpm_per_erpm = 1.0 / pole_pairs
    wheel_speed_per_rpm = wheel_circumference_m / 60.0
    base_speed_mps = DEFAULT_NORMAL_ERPM * wheel_rpm_per_erpm * wheel_speed_per_rpm
    NORMAL_SPEED_LIMIT_KMH = base_speed_mps * 3.6
  return NORMAL_SPEED_LIMIT_KMH

def speed_kmh_to_erpm(speed_kmh):
  if speed_kmh is None:
    return DEFAULT_NORMAL_ERPM
  speed_mps = speed_kmh / 3.6
  wheel_speed_per_rpm = wheel_circumference_m / 60.0
  wheel_rpm = speed_mps / wheel_speed_per_rpm
  return wheel_rpm * pole_pairs

ERPM_REL_TOL = 1e-6
ERPM_ABS_TOL = 1e-3
SPEED_REL_TOL = 1e-6
SPEED_ABS_TOL = 1e-3

def _erpm_isclose(a, b):
  if a is None or b is None:
    return False
  return math.isclose(a, b, rel_tol=ERPM_REL_TOL, abs_tol=ERPM_ABS_TOL)

def _speed_isclose(a, b):
  if a is None or b is None:
    return False
  return math.isclose(a, b, rel_tol=SPEED_REL_TOL, abs_tol=SPEED_ABS_TOL)

def queue_speed_limit_erpm(target_erpm, display_speed_kmh, force=False, reason=None):
  global current_speed_limit_kmh, current_speed_limit_erpm
  if CONTROLLER_TYPE == "fardriver":
    print("FarDriver: VESC-команда ограничения скорости не отправляется", flush=True)
    return
  if target_erpm is None:
    return
  #if (
  #  not force
  #  and _erpm_isclose(current_speed_limit_erpm, target_erpm)
  #  and _speed_isclose(current_speed_limit_kmh, display_speed_kmh)
  #):
  #  return
  payload = build_mcconf_temp_payload_erpm(target_erpm)
  enqueue_vesc_command(payload)
  current_speed_limit_kmh = display_speed_kmh
  current_speed_limit_erpm = target_erpm
  log_reason = f" ({reason})" if reason else ""
  print(f"➡️  VESC speed limit -> ERPM {target_erpm}, отображаем {display_speed_kmh} км/ч{log_reason}", flush=True)
  print(f"    (force={force}, eco_mode={eco_mode}, lock_active={lock_active})", flush=True)

def queue_speed_limit_command(max_speed_kmh, force=False, reason=None):
  if max_speed_kmh is None:
    display_speed_kmh = get_normal_speed_limit_kmh()
    target_erpm = DEFAULT_NORMAL_ERPM
  else:
    display_speed_kmh = max_speed_kmh
    target_erpm = speed_kmh_to_erpm(max_speed_kmh)
  queue_speed_limit_erpm(
    target_erpm,
    display_speed_kmh,
    force=force,
    reason=reason
  )

def apply_speed_limit_erpm(target_erpm, reason=None):
  if target_erpm is None:
    return
  if _erpm_isclose(target_erpm, 0.0):
    display_speed_kmh = 0.0
  elif _erpm_isclose(target_erpm, ECO_SPEED_LIMIT_ERPM):
    display_speed_kmh = ECO_SPEED_LIMIT_KMH
  elif _erpm_isclose(target_erpm, DEFAULT_NORMAL_ERPM):
    display_speed_kmh = get_normal_speed_limit_kmh()
  else:
    display_speed_kmh = get_normal_speed_limit_kmh()
  queue_speed_limit_erpm(
    target_erpm,
    display_speed_kmh,
    force=True,
    reason=reason
  )

def get_display_power():
  power = data.get('power', 0)
  if eco_mode and power > ECO_POWER_DISPLAY_LIMIT:
    return ECO_POWER_DISPLAY_LIMIT
  return power

def activate_lock():
  global lock_active, lock_input, lock_message, lock_restore_speed, lock_prev_page, PAGE_NAME
  if lock_active:
    return
  lock_active = True
  lock_input = ""
  lock_message = ""
  if current_speed_limit_erpm is not None and current_speed_limit_kmh is not None:
    lock_restore_speed = (current_speed_limit_kmh, current_speed_limit_erpm)
  else:
    lock_restore_speed = None
  apply_speed_limit_erpm(0.0, reason="lock ON")
  add_speak_message("Самокат заблокирован")
  lock_prev_page = PAGE_NAME
  PAGE_NAME = PAGE_LOCK

def deactivate_lock(unlocked):
  global lock_active, lock_input, lock_message, lock_restore_speed, lock_keypad_pressed, lock_prev_page, PAGE_NAME
  if not lock_active:
    return
  lock_active = False
  lock_input = ""
  lock_message = ""
  restore_speed = lock_restore_speed
  lock_restore_speed = None
  lock_keypad_pressed = False
  if restore_speed and restore_speed[0] is not None and restore_speed[1] is not None:
    queue_speed_limit_erpm(restore_speed[1], restore_speed[0], force=True)
  else:
    apply_speed_limit_erpm(DEFAULT_NORMAL_ERPM, reason="lock OFF")
  if unlocked:
    add_speak_message("Самокат разблокирован")
  PAGE_NAME = lock_prev_page if lock_prev_page else "SPEEDOMETER"

def handle_lock_digit(digit):
  global lock_input, lock_message
  if len(lock_input) >= 4:
    return
  lock_input += digit
  if len(lock_input) == 4:
    if lock_input == LOCK_PASSWORD:
      deactivate_lock(True)
    else:
      lock_message = "Пароль неверный"
      lock_input = ""
  else:
    lock_message = ""

def draw_lock_page():
  global lock_keypad_pressed
  screen.fill(get_theme_color("lock_background", (250, 250, 250)))
  prompt_y = HEIGHT * 0.2
  draw_text_center(screen, "Самокат заблокирован", font_medium, (30, 30, 30), prompt_y)
  draw_text_center(screen, "Введите пароль", font_small, (80, 80, 80), prompt_y + 70)

  dots = "*" * len(lock_input) if lock_input else "—"
  draw_text_center(screen, dots, font_medium, (10, 10, 10), prompt_y + 130)
  if lock_message:
    draw_text_center(screen, lock_message, font_small, (200, 50, 50), prompt_y + 180)

  keypad_top = prompt_y + 240
  keypad_left = WIDTH * 0.5 - 120
  key_w = 80
  key_h = 80
  key_gap = 15

  mouse = pygame.mouse.get_pos()
  click = pygame.mouse.get_pressed()
  if not click[0]:
    lock_keypad_pressed = False

  for row_idx, row in enumerate(LOCK_KEYPAD_LAYOUT):
    for col_idx, key in enumerate(row):
      if key == "":
        continue
      key_x = keypad_left + col_idx * (key_w + key_gap)
      key_y = keypad_top + row_idx * (key_h + key_gap)
      key_rect = pygame.Rect(key_x, key_y, key_w, key_h)
      is_backspace = key == "←"
      key_color = (200, 200, 200) if not is_backspace else (255, 180, 180)
      pygame.draw.rect(screen, key_color, key_rect, border_radius=18)
      label = font_medium.render(key if not is_backspace else "-", True, (20, 20, 20))
      screen.blit(label, label.get_rect(center=key_rect.center))

      if key_rect.collidepoint(mouse) and click[0] and not lock_keypad_pressed:
        if is_backspace:
          if lock_input:
            handle_backspace_lock()
        else:
          handle_lock_digit(key)
        lock_keypad_pressed = True

def handle_backspace_lock():
  global lock_input, lock_message
  if lock_input:
    lock_input = lock_input[:-1]
    lock_message = ""

def process_pending_vesc_commands(ser):
  try:
    while True:
      packet = vesc_command_queue.get_nowait()
      print(f"Очередь VESC: отправляем пакет {packet.hex()}", flush=True)
      ser.write(packet)
      ser.flush()
  except queue.Empty:
    pass

def set_eco_mode(enabled):
  global eco_mode
  global block_touch
  if lock_active:
    return
  if block_touch:
    return
  if enabled == eco_mode:
    return

  if enabled:
    apply_speed_limit_erpm(27.7, reason="eco ON")
    add_speak_message("Эко режим активирован")
  else:
    apply_speed_limit_erpm(DEFAULT_NORMAL_ERPM, reason="eco OFF")
    add_speak_message("Нормальный режим")

  eco_mode = enabled

def parse_vesc_payload(payload, forwarded=False):
  try:
    if forwarded:
      pass
      #payload = payload[2:]
    if len(payload) < 24:
      return None

    shift = 0

    #values.temp_mos = vb.vbPopFrontDouble16(1e1);
    mos_temp = struct.unpack_from('>h', payload, shift)[0] / 10.0
    shift += 2
    #values.temp_motor = vb.vbPopFrontDouble16(1e1);
    motor_temp = struct.unpack_from('>h', payload, shift)[0] / 10.0
    shift += 2
    #values.current_motor = vb.vbPopFrontDouble32(1e2);
    motor_current = struct.unpack_from('>i', payload, shift)[0] / 100.0
    shift += 4
    #values.current_in = vb.vbPopFrontDouble32(1e2);
    input_current = struct.unpack_from('>i', payload, shift)[0] / 100.0
    shift += 4
    #values.duty_now = vb.vbPopFrontDouble16(1e3);
    duty_cycle = struct.unpack_from('>h', payload, shift)[0] / 10.0
    shift += 2
    #values.rpm = vb.vbPopFrontDouble32(1e0);
    rpm = struct.unpack_from('>i', payload, shift)[0] / pole_pairs
    shift += 4
    #values.speed = vb.vbPopFrontDouble32(1e3);
    shift += 4
    #values.v_in = vb.vbPopFrontDouble16(1e1);
    input_voltage = struct.unpack_from('>h', payload, shift)[0] / 10.0
    shift += 2
    #values.battery_level = vb.vbPopFrontDouble16(1e3);
    battery_level = struct.unpack_from('>h', payload, shift)[0] / 10.0
    shift += 2
    #values.amp_hours = vb.vbPopFrontDouble32(1e4);
    shift += 4
    #values.amp_hours_charged = vb.vbPopFrontDouble32(1e4);
    shift += 4
    #values.watt_hours = vb.vbPopFrontDouble32(1e4);
    shift += 4
    #values.watt_hours_charged = vb.vbPopFrontDouble32(1e4);
    shift += 4
    #values.tachometer = vb.vbPopFrontDouble32(1e3);
    shift += 4
    #values.tachometer_abs = vb.vbPopFrontDouble32(1e3);
    shift += 4
    #values.position = vb.vbPopFrontDouble32(1e6);
    shift += 4
    #values.fault_code = mc_fault_code(vb.vbPopFrontInt8());
    #values.fault_str = faultToStr(values.fault_code);
    shift += 1
    #values.vesc_id = vb.vbPopFrontUint8();
    shift += 1
    #values.num_vescs = vb.vbPopFrontUint8();
    shift += 1
    #values.battery_wh = vb.vbPopFrontDouble32(1e3);
    shift += 4
    #values.odometer = vb.vbPopFrontUint32();
    odometer = int(struct.unpack_from('>I', payload, shift)[0] / 1000.0)
    shift += 4
    #values.uptime_ms = vb.vbPopFrontUint32();
    shift += 4

    return rpm, input_current, duty_cycle, input_voltage, motor_current, mos_temp, motor_temp, battery_level, odometer
  except Exception as e:
    print("Ошибка парсинга payload:", e)
    return None
  
def crc16_slave(data):
  crc = 0
  for b in data:
    crc ^= b << 8
    for _ in range(8):
      if (crc & 0x8000):
        crc = (crc << 1) ^ 0x1021
      else:
        crc <<= 1
    crc &= 0xFFFF
  return crc

def pack_packet_slave(payload):
  start = b'\x02'
  end = b'\x03'
  length = struct.pack('>B', len(payload))
  crc = struct.pack('>H', crc16_slave(payload))
  return start + length + payload + crc + end

FARDRIVER_FLASH_READ_ADDR = [
  0xE2, 0xE8, 0xEE, 0x00, 0x06, 0x0C, 0x12,
  0xE2, 0xE8, 0xEE, 0x18, 0x1E, 0x24, 0x2A,
  0xE2, 0xE8, 0xEE, 0x30, 0x5D, 0x63, 0x69,
  0xE2, 0xE8, 0xEE, 0x7C, 0x82, 0x88, 0x8E,
  0xE2, 0xE8, 0xEE, 0x94, 0x9A, 0xA0, 0xA6,
  0xE2, 0xE8, 0xEE, 0xAC, 0xB2, 0xB8, 0xBE,
  0xE2, 0xE8, 0xEE, 0xC4, 0xCA, 0xD0,
  0xE2, 0xE8, 0xEE, 0xD6, 0xDC, 0xF4, 0xFA,
]

def read_i16_le(raw, offset):
  return struct.unpack_from('<h', raw, offset)[0]

def read_u16_le(raw, offset):
  return struct.unpack_from('<H', raw, offset)[0]

def read_u24_be(raw, offset):
  return (raw[offset] << 16) | (raw[offset + 1] << 8) | raw[offset + 2]

def fardriver_old_command(command, sub_command, value_1=0, value_2=0):
  packet = bytearray([
    0xAA,
    command & 0xFF,
    (~command) & 0xFF,
    sub_command & 0xFF,
    value_1 & 0xFF,
    value_2 & 0xFF,
    0,
    0,
  ])
  crc = sum(packet[:6]) & 0xFF
  packet[6] = crc
  packet[7] = (~crc) & 0xFF
  return bytes(packet)

def fardriver_phase_current(raw_value):
  return 1.953125 * math.sqrt(max(0, raw_value))

def fardriver_init_packets():
  packets = []
  for item in FARDRIVER_INIT_COMMANDS_HEX.split(','):
    item = item.strip().replace(' ', '')
    if not item:
      continue
    try:
      packets.append(bytes.fromhex(item))
    except ValueError as exc:
      print(f"FarDriver: некорректный init hex {item}: {exc}", flush=True)
  return packets

def choose_fardriver_characteristics(client, controller_key):
  known_notify = {
    FARDRIVER_NOTIFY_UUID.lower(),
    "0000ffe1-0000-1000-8000-00805f9b34fb",
    "0000ffec-0000-1000-8000-00805f9b34fb",
    "0000fff1-0000-1000-8000-00805f9b34fb",
    "0000fff4-0000-1000-8000-00805f9b34fb",
  }
  known_write = {
    FARDRIVER_WRITE_UUID.lower(),
    "0000ffe1-0000-1000-8000-00805f9b34fb",
    "0000ffec-0000-1000-8000-00805f9b34fb",
    "0000fff1-0000-1000-8000-00805f9b34fb",
    "0000fff2-0000-1000-8000-00805f9b34fb",
    "0000fff3-0000-1000-8000-00805f9b34fb",
  }

  notify_char = None
  write_char = None
  fallback_notify = None
  fallback_write = None

  for service in client.services:
    service_uuid = service.uuid.lower()
    vendor_service = service_uuid == FARDRIVER_SERVICE_UUID.lower() or service_uuid.startswith("0000ff")
    print(f"FarDriver {controller_key}: service {service.uuid}", flush=True)
    for char in service.characteristics:
      props = set(char.properties or [])
      uuid_lower = char.uuid.lower()
      print(f"FarDriver {controller_key}: char {char.uuid} props={','.join(sorted(props))}", flush=True)

      if "notify" in props or "indicate" in props:
        if vendor_service or uuid_lower.startswith("0000ff"):
          fallback_notify = fallback_notify or char
        if uuid_lower in known_notify:
          notify_char = char
      if "write" in props or "write-without-response" in props:
        if vendor_service or uuid_lower.startswith("0000ff"):
          fallback_write = fallback_write or char
        if uuid_lower in known_write:
          write_char = char

  notify_char = notify_char or fallback_notify
  write_char = write_char or fallback_write or notify_char

  if notify_char is None:
    raise SerialGetError("FarDriver BLE notify characteristic not found")
  if write_char is None:
    raise SerialGetError("FarDriver BLE write characteristic not found")

  print(f"FarDriver {controller_key}: selected notify={notify_char.uuid} write={write_char.uuid}", flush=True)
  return notify_char, write_char

async def wait_for_fardriver_characteristics(client, controller_key):
  deadline = time.monotonic() + max(0.0, FARDRIVER_SERVICE_DISCOVERY_TIMEOUT)
  attempt = 0
  last_error = None

  while True:
    attempt += 1

    # Older Bleak exposes get_services() on the client, newer releases keep it
    # on the backend. Re-query BlueZ because ServicesResolved can lag connect().
    get_services = getattr(client, "get_services", None)
    if not callable(get_services):
      get_services = getattr(getattr(client, "_backend", None), "get_services", None)
    if callable(get_services):
      try:
        await get_services()
      except Exception as exc:
        print(
          f"FarDriver {controller_key}: GATT refresh attempt={attempt} failed: {format_exception(exc)}",
          flush=True
        )

    try:
      return choose_fardriver_characteristics(client, controller_key)
    except SerialGetError as exc:
      last_error = exc

    remaining = deadline - time.monotonic()
    if not client.is_connected or remaining <= 0:
      break

    delay = min(max(0.05, FARDRIVER_SERVICE_DISCOVERY_INTERVAL), remaining)
    print(
      f"FarDriver {controller_key}: GATT services not ready attempt={attempt}; "
      f"retry_in={delay:.2f}s remaining={remaining:.2f}s",
      flush=True
    )
    await asyncio.sleep(delay)

  raise SerialGetError(
    f"FarDriver BLE characteristics not found after {attempt} GATT attempts: {last_error}"
  )

def normalize_ble_mac(mac):
  return (mac or "").strip().upper()

def split_ble_macs(value):
  result = []
  seen = set()
  for item in (value or "").replace(";", ",").split(","):
    mac = normalize_ble_mac(item)
    if mac and mac not in seen:
      result.append(mac)
      seen.add(mac)
  return result

def format_exception(exc):
  return f"{type(exc).__name__}: {repr(exc)}"

def load_fardriver_cache():
  try:
    with open(FARDRIVER_CACHE_FILE, "r") as cache_file:
      loaded = json.load(cache_file)
    if isinstance(loaded, dict):
      return loaded
  except FileNotFoundError:
    return {}
  except Exception as exc:
    print(f"FarDriver BLE cache read failed: {exc}", flush=True)
  return {}

def save_fardriver_cache_entry(controller_key, target_name, address, name=None, source=None):
  address = normalize_ble_mac(address)
  if not address:
    return
  cache = load_fardriver_cache()
  cache[controller_key] = {
    "address": address,
    "target_name": target_name,
    "name": name or target_name or "",
    "source": source or "",
    "updated_at": time.time(),
  }
  try:
    with open(FARDRIVER_CACHE_FILE, "w") as cache_file:
      json.dump(cache, cache_file, indent=2, sort_keys=True)
    print(f"FarDriver {controller_key}: BLE cache saved {address} -> {FARDRIVER_CACHE_FILE}", flush=True)
  except Exception as exc:
    print(f"FarDriver {controller_key}: BLE cache save failed: {exc}", flush=True)

def append_fardriver_candidate(candidates, seen, source, device=None, mac=None, name=None, rssi=None, priority=0):
  address = normalize_ble_mac(getattr(device, "address", None) if device is not None else mac)
  if not address or address in seen:
    return
  seen.add(address)
  candidates.append({
    'device': device,
    'address': address,
    'name': (getattr(device, "name", None) if device is not None else name) or "",
    'rssi': getattr(device, "rssi", rssi) if device is not None else rssi,
    'source': source,
    'priority': priority,
  })

def print_bleak_devices(controller_key, devices, target_name):
  print(f"FarDriver {controller_key}: BleakScanner found {len(devices)} devices", flush=True)
  interesting = []
  for item in devices:
    name = item.name or ""
    if (
      FARDRIVER_DEBUG_SCAN_ALL
      or name.startswith(FARDRIVER_NAME_PREFIX)
      or (target_name and name.startswith(target_name))
    ):
      interesting.append(item)

  if not interesting:
    print(f"FarDriver {controller_key}: BleakScanner FarDriver-like devices: none", flush=True)
    return

  print(f"FarDriver {controller_key}: BleakScanner FarDriver-like devices:", flush=True)
  for index, item in enumerate(interesting, 1):
    metadata = getattr(item, "metadata", None)
    details = getattr(item, "details", None)
    print(
      f"  scan[{index}] address={getattr(item, 'address', None) or '-'} name={getattr(item, 'name', None) or '-'} "
      f"rssi={getattr(item, 'rssi', 'n/a')} metadata={metadata if FARDRIVER_DEBUG_SCAN_ALL else '-'} "
      f"details_type={type(details).__name__ if details is not None else '-'}",
      flush=True
    )

def bluetoothctl_fardriver_macs(target_name, controller_key):
  if not FARDRIVER_BLUETOOTHCTL_SCAN or not target_name:
    return []

  now = time.time()
  cached_by_name = FARDRIVER_LIVE_SCAN_CACHE['by_name']
  cached_macs = cached_by_name.get(target_name, [])
  if cached_macs and now - FARDRIVER_LIVE_SCAN_CACHE['updated_at'] <= FARDRIVER_LIVE_SCAN_CACHE_SECONDS:
    print(
      f"FarDriver {controller_key}: use current-session scan cache for {target_name}: {', '.join(cached_macs)}",
      flush=True
    )
    return list(cached_macs)

  print(
    f"FarDriver {controller_key}: bluetoothctl live scan name={target_name} timeout={FARDRIVER_BLUETOOTHCTL_SCAN_SECONDS}s",
    flush=True
  )
  master_fd = None
  slave_fd = None
  process = None
  scan_chunks = []
  try:
    # bluetoothctl emits [NEW]/[CHG] events only when it owns a terminal. A
    # plain subprocess pipe receives no device lines on Raspberry Pi, while
    # `bluetoothctl devices` is only BlueZ's long-lived stale-address cache.
    master_fd, slave_fd = pty.openpty()
    process = subprocess.Popen(
      ["bluetoothctl"],
      stdin=slave_fd,
      stdout=slave_fd,
      stderr=slave_fd,
      close_fds=True,
    )
    os.close(slave_fd)
    slave_fd = None
    time.sleep(0.2)
    os.write(master_fd, b"scan on\n")

    scan_started = time.monotonic()
    deadline = time.monotonic() + FARDRIVER_BLUETOOTHCTL_SCAN_SECONDS
    direct_target_pattern = re.compile(
      rf"Device\s+[0-9A-Fa-f:]{{17}}\s+{re.escape(target_name)}(?:\s|$)"
    )
    live_ansi_pattern = re.compile(r"\x1b\[[0-9;?]*[ -/]*[@-~]")
    while True:
      remaining = deadline - time.monotonic()
      if remaining <= 0:
        break
      readable, _, _ = select.select([master_fd], [], [], remaining)
      if not readable:
        continue
      try:
        chunk = os.read(master_fd, 4096)
      except OSError:
        break
      if not chunk:
        break
      scan_chunks.append(chunk.decode("utf-8", errors="replace"))
      partial_text = live_ansi_pattern.sub("", "".join(scan_chunks)).replace("\r", "")
      if direct_target_pattern.search(partial_text):
        print(
          f"FarDriver {controller_key}: fresh advertisement for {target_name} "
          f"seen after {time.monotonic() - scan_started:.2f}s; stop scan",
          flush=True
        )
        break

    os.write(master_fd, b"scan off\nquit\n")
    process.wait(timeout=3)
  except Exception as exc:
    print(f"FarDriver {controller_key}: bluetoothctl live scan failed: {format_exception(exc)}", flush=True)
  finally:
    if process is not None and process.poll() is None:
      try:
        process.terminate()
        process.wait(timeout=2)
      except Exception:
        pass
    if slave_fd is not None:
      try:
        os.close(slave_fd)
      except OSError:
        pass
    if master_fd is not None:
      try:
        os.close(master_fd)
      except OSError:
        pass

  ansi_pattern = re.compile(r"\x1b\[[0-9;?]*[ -/]*[@-~]")
  scan_text = ansi_pattern.sub("", "\n".join(scan_chunks)).replace("\r", "")
  live_lines = [line.strip() for line in scan_text.splitlines() if "Device " in line]
  touched_addresses = []
  live_names = {}
  event_pattern = re.compile(r"Device\s+([0-9A-Fa-f:]{17})(?:\s+([^\s]+))?")
  for line in live_lines:
    match = event_pattern.search(line)
    if not match:
      continue
    addr = normalize_ble_mac(match.group(1))
    if addr in touched_addresses:
      touched_addresses.remove(addr)
    touched_addresses.append(addr)
    possible_name = match.group(2) or ""
    if possible_name.startswith(FARDRIVER_NAME_PREFIX):
      live_names[addr] = possible_name

  known_names = {}
  known_order = []
  try:
    known = subprocess.run(
      ["bluetoothctl", "devices"],
      capture_output=True,
      text=True,
      timeout=5,
    )
    known_pattern = re.compile(r"Device\s+([0-9A-Fa-f:]{17})\s+([^\s]+)")
    for line in (known.stdout or "").splitlines():
      match = known_pattern.search(line)
      if not match:
        continue
      addr = normalize_ble_mac(match.group(1))
      known_names[addr] = match.group(2)
      known_order.append(addr)
    print(
      f"FarDriver {controller_key}: bluetoothctl live events={len(live_lines)} "
      f"touched={len(touched_addresses)} known_devices={len(known_names)}",
      flush=True
    )
  except Exception as exc:
    print(f"FarDriver {controller_key}: bluetoothctl devices map failed: {format_exception(exc)}", flush=True)

  for index, line in enumerate(live_lines[-30:], 1):
    print(f"  live[{index}] {line}", flush=True)

  by_name = {}
  matched_lines = []
  for addr in touched_addresses:
    name = live_names.get(addr) or known_names.get(addr, "")
    if not name.startswith(FARDRIVER_NAME_PREFIX):
      continue
    addresses = by_name.setdefault(name, [])
    if addr in addresses:
      addresses.remove(addr)
    addresses.append(addr)
    if name == target_name:
      matched_lines.append(f"Device {addr} {name} (touched in current scan)")

  # Some BlueZ builds suppress live device events even in a PTY. In that case
  # consider only cached addresses that still expose live RSSI/connection state.
  if not by_name.get(target_name):
    scored = []
    target_addresses = [addr for addr in known_order if known_names.get(addr) == target_name][-16:]
    for addr in target_addresses:
      try:
        info = subprocess.run(
          ["bluetoothctl", "info", addr],
          capture_output=True,
          text=True,
          timeout=2,
        )
      except Exception as exc:
        print(f"FarDriver {controller_key}: bluetoothctl info {addr} failed: {format_exception(exc)}", flush=True)
        continue
      info_text = info.stdout or ""
      connected = bool(re.search(r"Connected:\s+yes", info_text, re.IGNORECASE))
      resolved = bool(re.search(r"ServicesResolved:\s+yes", info_text, re.IGNORECASE))
      rssi_match = re.search(r"RSSI:\s+(-?\d+)", info_text, re.IGNORECASE)
      rssi = int(rssi_match.group(1)) if rssi_match else None
      print(
        f"FarDriver {controller_key}: cached-live-check {target_name} {addr} "
        f"connected={connected} resolved={resolved} rssi={rssi if rssi is not None else '-'}",
        flush=True
      )
      if connected or resolved or rssi is not None:
        score = (10000 if connected else 0) + (1000 if resolved else 0) + (rssi if rssi is not None else -999)
        scored.append((score, addr))
    if scored:
      scored.sort()
      by_name[target_name] = [addr for _, addr in scored]
      matched_lines.append("BlueZ cached address with live Connected/ServicesResolved/RSSI state")

  FARDRIVER_LIVE_SCAN_CACHE['updated_at'] = time.time()
  FARDRIVER_LIVE_SCAN_CACHE['by_name'] = by_name
  macs = by_name.get(target_name, [])

  if matched_lines:
    print(f"FarDriver {controller_key}: bluetoothctl live matched lines for {target_name}:", flush=True)
    for index, line in enumerate(matched_lines[-20:], 1):
      print(f"  bt[{index}] {line}", flush=True)
  else:
    print(f"FarDriver {controller_key}: bluetoothctl live matched lines for {target_name}: none", flush=True)

  if macs:
    print(f"FarDriver {controller_key}: bluetoothctl live candidates for {target_name}: {', '.join(macs)}", flush=True)
  else:
    print(f"FarDriver {controller_key}: bluetoothctl live candidates for {target_name}: none", flush=True)
  return macs

async def find_fardriver_candidates(mac, target_name, controller_key):
  from bleak import BleakScanner

  candidates = []
  seen = set()
  configured_macs = split_ble_macs(FARDRIVER_MASTER_MACS if controller_key == "master" else FARDRIVER_SLAVE_MACS)
  explicit_mac = normalize_ble_mac(mac)
  if explicit_mac and explicit_mac not in configured_macs:
    configured_macs.append(explicit_mac)
  print(
    f"FarDriver {controller_key}: configured_macs={configured_macs if configured_macs else []} explicit_mac={explicit_mac or '-'}",
    flush=True
  )

  cache = load_fardriver_cache()
  cached = cache.get(controller_key, {})
  cached_mac = normalize_ble_mac(cached.get("address") if isinstance(cached, dict) else "")
  cached_target_name = cached.get("target_name") if isinstance(cached, dict) else None
  print(
    f"FarDriver {controller_key}: cache address={cached_mac or '-'} target_name={cached_target_name or '-'} cache_file={FARDRIVER_CACHE_FILE}",
    flush=True
  )
  if not FARDRIVER_ALLOW_STALE_ADDRESS_FALLBACK and (configured_macs or cached_mac):
    print(
      f"FarDriver {controller_key}: configured/cache addresses are ignored because FarDriver rotates BLE addresses",
      flush=True
  )
  if FARDRIVER_BLUETOOTHCTL_SCAN:
    # bluetoothctl uses the same adapter-wide discovery session as Bleak.
    # Do not let master and slave issue scan on/off at the same time.
    with FARDRIVER_BLE_SCAN_LOCK:
      fresh_macs = bluetoothctl_fardriver_macs(target_name, controller_key)
    for index, fresh_mac in enumerate(fresh_macs, 1):
      append_fardriver_candidate(candidates, seen, "bluetoothctl-live", mac=fresh_mac, name=target_name, priority=1200 + index)

  if not candidates:
    print(
      f"FarDriver {controller_key}: fresh Bleak scan fallback name={target_name} timeout={FARDRIVER_SCAN_TIMEOUT}s",
      flush=True
    )
    # BlueZ accepts only one discovery session at a time. Keep this lock limited
    # to the scan itself: the other controller can scan while the first connects.
    with FARDRIVER_BLE_SCAN_LOCK:
      devices = await BleakScanner.discover(timeout=FARDRIVER_SCAN_TIMEOUT)
    print_bleak_devices(controller_key, devices, target_name)
    for item in devices:
      item_name = item.name or ""
      item_addr = normalize_ble_mac(item.address)
      if not item_addr:
        continue
      if target_name and item_name == target_name:
        append_fardriver_candidate(candidates, seen, "bleak-fresh-exact-name", device=item, priority=1000)
      elif target_name and item_name.startswith(target_name):
        append_fardriver_candidate(candidates, seen, "bleak-fresh-name-prefix", device=item, priority=900)
      elif (not target_name) and item_name.startswith(FARDRIVER_NAME_PREFIX):
        append_fardriver_candidate(candidates, seen, "bleak-fresh-prefix", device=item, priority=800)

  if not candidates and FARDRIVER_ALLOW_STALE_ADDRESS_FALLBACK:
    print(
      f"FarDriver {controller_key}: fresh name was not seen; stale-address fallback is enabled",
      flush=True
    )
    scanned_by_mac = {normalize_ble_mac(item.address): item for item in devices if normalize_ble_mac(item.address)}
    if cached_mac and (not target_name or cached_target_name == target_name):
      scanned = scanned_by_mac.get(cached_mac)
      if scanned is not None:
        append_fardriver_candidate(candidates, seen, "cache+scan", device=scanned, priority=300)
      else:
        append_fardriver_candidate(candidates, seen, "cache", mac=cached_mac, name=target_name, priority=200)
    for configured_mac in reversed(configured_macs):
      scanned = scanned_by_mac.get(configured_mac)
      if scanned is not None:
        append_fardriver_candidate(candidates, seen, "configured+scan", device=scanned, priority=100)
      else:
        append_fardriver_candidate(candidates, seen, "configured", mac=configured_mac, name=target_name, priority=50)

  if candidates:
    candidates.sort(
      key=lambda item: (
        item['priority'],
        item['rssi'] if item['rssi'] is not None else -999
      ),
      reverse=True
    )
    print(f"FarDriver {controller_key}: BLE candidates:", flush=True)
    for index, item in enumerate(candidates, 1):
      print(
        f"  {index}. {item['address']} {item['name'] or '-'} rssi={item['rssi'] if item['rssi'] is not None else 'n/a'} source={item['source']} priority={item['priority']}",
        flush=True
      )
    return candidates

  names = ", ".join(f"{item.address} {item.name}" for item in devices if (item.name or "").startswith(FARDRIVER_NAME_PREFIX))
  raise SerialGetError(f"FarDriver BLE device not found: name={target_name} mac={mac}. Seen: {names}")

class FarDriverTelemetry:
  def __init__(self, controller_key='master'):
    self.controller_key = controller_key
    self.speed_source = FARDRIVER_SPEED_SOURCE
    self.speed_kmh = None
    self.frame_speed_kmh = None
    self.rpm_speed_kmh = None
    self.wheel_speed_kmh = None
    self.battery_current = None
    self.phase_current = None
    self.duty = None
    self.voltage = None
    self.rpm = None
    self.mos_temp = None
    self.motor_temp = None
    self.measure_speed = None
    self.wheel_ratio = None
    self.wheel_radius = None
    self.wheel_width = None
    self.rate_ratio = None

  def apply_frame(self, frame):
    if len(frame) != 16 or frame[0] != 0xAA:
      return False

    flags = frame[1] >> 6
    frame_id = frame[1] & 0x3F
    if flags != 2:
      return False

    payload = frame[2:14]
    if frame_id == 0x37:
      self.apply_gather_payload(payload)
    elif frame_id < len(FARDRIVER_FLASH_READ_ADDR):
      self.apply_rotating_payload(FARDRIVER_FLASH_READ_ADDR[frame_id], payload)
    else:
      return False

    self.select_speed()
    self.flush_to_data()
    return True

  def apply_gather_payload(self, payload):
    # Реверс из fardriver.bt: Speed/1000, Curr/50A, Vol/5V, Mod/0.2.
    _, speed_raw, current_raw, voltage_raw, _, modulation_raw, _ = struct.unpack('<hhhhhBB', payload)
    self.frame_speed_kmh = max(0.0, speed_raw / 1000.0)
    self.battery_current = current_raw / 50.0
    self.voltage = voltage_raw / 5.0
    self.duty = max(0.0, min(100.0, modulation_raw / 0.2))

  def apply_rotating_payload(self, addr, payload):
    if addr == 0xE2:
      self.duty = max(0.0, min(100.0, payload[4] * 100.0 / 128.0))
      self.measure_speed = read_u16_le(payload, 6)
      self.rpm = float(self.measure_speed) / FARDRIVER_RPM_DIVIDER if FARDRIVER_RPM_DIVIDER else float(self.measure_speed)
      self.rpm_speed_kmh = max(0.0, self.rpm * wheel_circumference_m * 60.0 / 1000.0)
      self.recompute_wheel_speed()
    elif addr == 0xE8:
      self.voltage = read_i16_le(payload, 0) / 10.0
      self.battery_current = read_i16_le(payload, 4) / 4.0
    elif addr == 0xEE:
      phase_a = fardriver_phase_current(read_u24_be(payload, 4))
      phase_c = fardriver_phase_current(read_u24_be(payload, 7))
      self.phase_current = max(phase_a, phase_c)
      volts = read_i16_le(payload, 10) / 16.0
      if volts > 0:
        self.voltage = volts
    elif addr == 0xD0:
      self.wheel_ratio = payload[4]
      self.wheel_radius = payload[5]
      self.wheel_width = payload[7]
      self.rate_ratio = read_u16_le(payload, 8)
      self.recompute_wheel_speed()
    elif addr == 0xD6:
      self.mos_temp = float(read_i16_le(payload, 10))
    elif addr == 0xF4:
      self.motor_temp = float(read_i16_le(payload, 0))

  def recompute_wheel_speed(self):
    if None in (self.measure_speed, self.wheel_ratio, self.wheel_radius, self.wheel_width, self.rate_ratio):
      return
    if not self.rate_ratio:
      return
    speed_factor = 0.00376991136 * (
      float(self.wheel_radius) * 1270.0 + float(self.wheel_width) * float(self.wheel_ratio)
    )
    self.wheel_speed_kmh = max(0.0, float(self.measure_speed) * speed_factor / float(self.rate_ratio))

  def select_speed(self):
    if self.speed_source == "rpm":
      self.speed_kmh = self.rpm_speed_kmh if self.rpm_speed_kmh is not None else self.frame_speed_kmh
    elif self.speed_source == "wheel":
      self.speed_kmh = self.wheel_speed_kmh if self.wheel_speed_kmh is not None else self.frame_speed_kmh
    else:
      self.speed_kmh = self.frame_speed_kmh if self.frame_speed_kmh is not None else (self.rpm_speed_kmh if self.rpm_speed_kmh is not None else self.wheel_speed_kmh)

  def flush_to_data(self):
    now = time.time()
    snapshot = {
      'speed_kmh': self.speed_kmh,
      'rpm_speed_kmh': self.rpm_speed_kmh,
      'frame_speed_kmh': self.frame_speed_kmh,
      'wheel_speed_kmh': self.wheel_speed_kmh,
      'battery_current': self.battery_current,
      'phase_current': self.phase_current,
      'duty': self.duty,
      'voltage': self.voltage,
      'rpm': self.rpm,
      'mos_temp': self.mos_temp,
      'motor_temp': self.motor_temp,
      'updated_at': now,
    }

    with FARDRIVER_LATEST_LOCK:
      FARDRIVER_LATEST[self.controller_key] = snapshot
      fresh = {
        key: value
        for key, value in FARDRIVER_LATEST.items()
        if value.get('updated_at') and now - value['updated_at'] <= 3
      }

      for key in ['master', 'slave']:
        item = fresh.get(key, FARDRIVER_LATEST.get(key, {}))
        if item.get('phase_current') is not None:
          data[key]['motor_current'] = item['phase_current']
        if item.get('battery_current') is not None:
          data[key]['battery_current'] = item['battery_current']
        if item.get('duty') is not None:
          data[key]['duty'] = item['duty']
        if item.get('mos_temp') is not None:
          data[key]['temp'] = int(item['mos_temp'])
        if item.get('motor_temp') is not None:
          data[key]['temp_motor'] = int(item['motor_temp'])
        if item.get('rpm') is not None:
          data[key]['rpm'] = item['rpm']

      speed_source = None
      speed_source_controller = None
      if fresh.get('master', {}).get('speed_kmh') is not None:
        speed_source = fresh['master']
        speed_source_controller = 'master'
      elif fresh.get('slave', {}).get('speed_kmh') is not None:
        speed_source = fresh['slave']
        speed_source_controller = 'slave'
      else:
        speed_source = {}
      if speed_source.get('speed_kmh') is not None:
        data['speed'] = speed_source['speed_kmh']
        data['speed_source_controller'] = speed_source_controller or data.get('speed_source_controller', 'master')

      voltages = [item['voltage'] for item in fresh.values() if item.get('voltage') is not None]
      battery_currents = [item['battery_current'] for item in fresh.values() if item.get('battery_current') is not None]
      if voltages:
        voltage = sum(voltages) / len(voltages)
        data['battery_voltage'] = voltage
        if data.get('bms_voltage', 0) <= 0 or now - PREV_VALS.get('bms_last_update', 0) > 3:
          data['bms_voltage'] = voltage

        if battery_currents:
          total_battery_current = sum(battery_currents)
          if now - PREV_VALS.get('bms_last_update', 0) > 3:
            data['bms_current'] = total_battery_current
            data['power'] = int(voltage * total_battery_current)
          if abs(total_battery_current) < 0.5:
            data['v_without_nagruzka'] = voltage
          if data.get('v_without_nagruzka', 0) > 0:
            data['voltage_down'] = voltage - data['v_without_nagruzka']

def read_fardriver_serial(ser, controller_key='master'):
  telemetry = FarDriverTelemetry(controller_key)
  buffer = bytearray()
  status_poll = fardriver_old_command(0x13, 0x07)
  send_status_poll = os.environ.get("FARDRIVER_STATUS_POLL", "1").strip().lower() not in {"0", "false", "off", "no"}
  next_poll_at = 0.0
  last_frame_at = time.time()

  ser.reset_input_buffer()
  ser.reset_output_buffer()
  print(f"Буферы FarDriver {controller_key} очищены, начинаем чтение потока", flush=True)

  while True:
    now = time.time()
    if send_status_poll and now >= next_poll_at:
      ser.write(status_poll)
      ser.flush()
      next_poll_at = now + 2.0

    chunk = ser.read(64)
    if chunk:
      buffer.extend(chunk)
    elif now - last_frame_at > 8:
      raise SerialGetError("FarDriver timeout")

    while len(buffer) >= 16:
      try:
        start = buffer.index(0xAA)
      except ValueError:
        buffer.clear()
        break
      if start:
        del buffer[:start]
      if len(buffer) < 16:
        break
      candidate = bytes(buffer[:16])
      if (candidate[1] >> 6) != 2:
        del buffer[0]
        continue
      del buffer[:16]
      if telemetry.apply_frame(candidate):
        last_frame_at = time.time()

async def fardriver_ble_loop(mac, controller_key='master'):
  try:
    from bleak import BleakClient, BleakScanner
  except Exception as exc:
    print(f"FarDriver {controller_key}: bleak не установлен ({exc}). Установи: pip install --user bleak", flush=True)
    return

  status_poll = fardriver_old_command(0x13, 0x07)
  init_packets = fardriver_init_packets()
  notify_uuid = FARDRIVER_NOTIFY_UUID
  write_uuid = FARDRIVER_WRITE_UUID
  target_name = FARDRIVER_MASTER_NAME if controller_key == "master" else FARDRIVER_SLAVE_NAME

  while True:
    telemetry = FarDriverTelemetry(controller_key)
    buffer = bytearray()
    last_frame_at = time.time()
    debug_notify_left = FARDRIVER_DEBUG_NOTIFY

    def on_notify(_, chunk):
      nonlocal last_frame_at, debug_notify_left
      if not chunk:
        return
      if debug_notify_left > 0:
        print(f"FarDriver {controller_key}: notify {bytes(chunk).hex()}", flush=True)
        debug_notify_left -= 1
      buffer.extend(bytes(chunk))
      while len(buffer) >= 16:
        try:
          start = buffer.index(0xAA)
        except ValueError:
          buffer.clear()
          break
        if start:
          del buffer[:start]
        if len(buffer) < 16:
          break
        candidate = bytes(buffer[:16])
        if (candidate[1] >> 6) != 2:
          del buffer[0]
          continue
        del buffer[:16]
        if telemetry.apply_frame(candidate):
          last_frame_at = time.time()

    setup_lock_held = False
    try:
      print(f"FarDriver {controller_key}: waiting for fresh-address setup slot", flush=True)
      FARDRIVER_BLE_CONTROLLER_SETUP_LOCK.acquire()
      setup_lock_held = True
      print(f"FarDriver {controller_key}: fresh-address setup slot acquired", flush=True)
      candidates = await find_fardriver_candidates(mac, target_name, controller_key)
      fast_candidates = candidates[:max(1, FARDRIVER_FAST_CONNECT_CANDIDATES)]
      final_candidates = candidates[:max(1, FARDRIVER_FINAL_CONNECT_CANDIDATES)]
      attempts = [
        ("fast", selected, FARDRIVER_TRY_CONNECT_TIMEOUT)
        for selected in fast_candidates
      ] + [
        ("final", selected, FARDRIVER_FINAL_CONNECT_TIMEOUT)
        for selected in final_candidates
      ]
      client = None
      selected = None
      notify_char = None
      write_char = None
      attempt_errors = []
      for attempt_index, (attempt_kind, candidate_row, connect_timeout) in enumerate(attempts, 1):
        target = candidate_row['device'] if candidate_row['device'] is not None else candidate_row['address']
        print(
          f"FarDriver {controller_key}: BLE connect attempt {attempt_index}/{len(attempts)} "
          f"kind={attempt_kind} address={candidate_row['address']} name={candidate_row['name'] or '-'} "
          f"source={candidate_row['source']} timeout={connect_timeout}s "
          f"target_type={'BLEDevice' if candidate_row['device'] is not None else 'address'}",
          flush=True
        )
        candidate_client = BleakClient(target, timeout=connect_timeout)
        try:
          # Keep the whole setup atomic at adapter level. On Raspberry Pi a
          # concurrent scan/connect while BlueZ resolves GATT can leave only
          # the generic system services visible and hide FarDriver's FFE1.
          with FARDRIVER_BLE_SCAN_LOCK:
            with FARDRIVER_BLE_CONNECT_LOCK:
              await candidate_client.connect()
              print(
                f"FarDriver {controller_key}: BLE link established address={candidate_row['address']}; "
                "resolving GATT",
                flush=True
              )
              candidate_notify, candidate_write = await wait_for_fardriver_characteristics(
                candidate_client,
                controller_key
              )
              await candidate_client.start_notify(candidate_notify, on_notify)
              print(f"FarDriver {controller_key}: notify {candidate_notify.uuid}", flush=True)

              for packet in init_packets:
                print(f"FarDriver {controller_key}: init write {packet.hex()}", flush=True)
                try:
                  await candidate_client.write_gatt_char(candidate_write, packet, response=False)
                except Exception:
                  await candidate_client.write_gatt_char(candidate_write, packet, response=True)
                await asyncio.sleep(0.2)
        except Exception as exc:
          attempt_errors.append(f"{candidate_row['address']}={format_exception(exc)}")
          print(
            f"FarDriver {controller_key}: BLE setup failed attempt={attempt_index}/{len(attempts)} "
            f"address={candidate_row['address']} error={format_exception(exc)}",
            flush=True
          )
          if FARDRIVER_DEBUG_TRACEBACK:
            traceback.print_exc()
          try:
            await candidate_client.disconnect()
          except Exception:
            pass
          continue
        client = candidate_client
        selected = candidate_row
        notify_char = candidate_notify
        write_char = candidate_write
        print(
          f"FarDriver {controller_key}: BLE setup success attempt={attempt_index}/{len(attempts)} "
          f"address={selected['address']}",
          flush=True
        )
        break

      if client is None or selected is None or notify_char is None or write_char is None:
        raise SerialGetError(
          f"FarDriver BLE all candidates failed: {'; '.join(attempt_errors)}"
        )

      with FARDRIVER_BLE_CONNECT_LOCK:
        save_fardriver_cache_entry(controller_key, target_name, selected['address'], selected['name'], selected['source'])

      FARDRIVER_BLE_CONTROLLER_SETUP_LOCK.release()
      setup_lock_held = False
      print(f"FarDriver {controller_key}: fresh-address setup slot released", flush=True)

      try:
        while True:
          try:
            await client.write_gatt_char(write_char, status_poll, response=False)
          except Exception:
            await client.write_gatt_char(write_char, status_poll, response=True)

          await asyncio.sleep(2.0)
          if time.time() - last_frame_at > 12:
            raise SerialGetError("FarDriver BLE no telemetry")
      finally:
        try:
          await client.disconnect()
        except Exception:
          pass
    except Exception as exc:
      print(f"FarDriver {controller_key}: BLE reconnect reason: {format_exception(exc)}", flush=True)
      if FARDRIVER_DEBUG_TRACEBACK:
        traceback.print_exc()
    finally:
      if setup_lock_held:
        FARDRIVER_BLE_CONTROLLER_SETUP_LOCK.release()
        print(f"FarDriver {controller_key}: fresh-address setup slot released after error", flush=True)
        setup_lock_held = False
    await asyncio.sleep(FARDRIVER_RECONNECT_DELAY)

def read_fardriver_ble(mac, controller_key='master'):
  target_name = FARDRIVER_MASTER_NAME if controller_key == "master" else FARDRIVER_SLAVE_NAME
  if not mac and not target_name:
    print(f"FarDriver {controller_key}: MAC и имя не заданы", flush=True)
    return
  if ENABLE_BMS and BMS_BACKEND in {"ble", "bleak", "bluetooth"} and BMS_BLE_STARTUP_PRIORITY_TIMEOUT > 0:
    print(
      f"FarDriver {controller_key}: ждём первый запуск BMS до "
      f"{BMS_BLE_STARTUP_PRIORITY_TIMEOUT:.1f}s",
      flush=True
    )
    bms_attempted = BMS_INITIAL_ATTEMPT_DONE.wait(timeout=BMS_BLE_STARTUP_PRIORITY_TIMEOUT)
    print(f"FarDriver {controller_key}: BMS initial attempt finished={bms_attempted}", flush=True)
  if controller_key == "slave":
    time.sleep(float(os.environ.get("FARDRIVER_SLAVE_CONNECT_DELAY", "0")))
  asyncio.run(fardriver_ble_loop(mac, controller_key))


######### CONTROLLER READ ##########
def read_serial(ser):
  packet_master = pack_comm_get_values()
  packet_slave = pack_comm_get_values(can_id=15)

  ser.reset_input_buffer()
  ser.reset_output_buffer()
  print("Буферы VESC очищены, начинаем цикл чтения", flush=True)

  while True:
    #GET_INFO
    print(f"VESC master: отправляем {len(packet_master)} байт -> {packet_master.hex()}", flush=True)
    ser.write(packet_master)
    ser.flush()
    print(f"VESC master: после отправки in_waiting={ser.in_waiting}", flush=True)
    header = ser.read(2)
    if not header:
      print(f"VESC master: пустой ответ (timeout), in_waiting={ser.in_waiting}", flush=True)
      if ser.in_waiting:
        leftover = ser.read(ser.in_waiting)
        if leftover:
          print(f"VESC master: неожиданные данные {leftover.hex()}", flush=True)
      process_pending_vesc_commands(ser)
      time.sleep(0.1)
      continue
    if header[0] != 2:
      print(f"VESC master: неожиданный заголовок {header!r}, in_waiting={ser.in_waiting}", flush=True)
      ser.reset_input_buffer()
      time.sleep(0.1)
      continue
    controllerAnswerError = True
    if header and header[0] == 2:
      size = header[1]
      frame = ser.read(size + 3)
      if len(frame) == size + 3 and frame[-1:] == b'\x03':
        payload = frame[:-3]
        crc = frame[-3:-1]
        if payload and payload[0] == PACKET_INDEX_FOR_VESC:
          real_payload = payload[1:]
          parsed = parse_vesc_payload(real_payload, forwarded=False)
          if parsed:
            controllerAnswerError = False
            rpm, input_current, duty_cycle, volt, motor_current, mos_temp, motor_temp, battery_level, odometer = parsed
            wheel_rpm = rpm
            speed_mps = (wheel_rpm * wheel_circumference_m) / 60
            data['speed'] = speed_mps * 3.6
              
            data['master']['motor_current'] = motor_current
            data['master']['battery_current'] = input_current
            data['master']['duty'] = duty_cycle
            data['master']['temp'] = int(mos_temp)
            data['master']['temp_motor'] = int(motor_temp)
            data['battery_voltage'] = volt
            #data['battery_level'] = battery_level
            #data['odometer'] = odometer

    if controllerAnswerError:
      #add_speak_message("Ошибка данных контроллера мастер")
      raise SerialGetError("Error")
      

    process_pending_vesc_commands(ser)
    time.sleep(0.1)#0.05

    slave_id = 15
    command = PACKET_INDEX_FOR_VESC#4  # COMM_GET_VALUES

    payload = struct.pack('>BBB', 0x22, slave_id, command)
    packet = pack_packet_slave(payload)

    #ser.write(packet)
    #response = ser.read(1024)
    #print(response)
    controllerAnswerError = True
    print(f"VESC slave: отправляем {len(packet)} байт -> {packet.hex()}", flush=True)
    ser.write(packet)
    ser.flush()
    print(f"VESC slave: после отправки in_waiting={ser.in_waiting}", flush=True)
    header = ser.read(2)
    if not header:
      print(f"VESC slave: пустой ответ (timeout), in_waiting={ser.in_waiting}", flush=True)
      if ser.in_waiting:
        leftover = ser.read(ser.in_waiting)
        if leftover:
          print(f"VESC slave: неожиданные данные {leftover.hex()}", flush=True)
      process_pending_vesc_commands(ser)
      time.sleep(0.1)
      continue
    if header[0] != 2:
      print(f"VESC slave: неожиданный заголовок {header!r}, in_waiting={ser.in_waiting}", flush=True)
      ser.reset_input_buffer()
      time.sleep(0.1)
      continue
    if header and header[0] == 2:
      size = header[1]
      frame = ser.read(size + 3)
      if len(frame) == size + 3 and frame[-1:] == b'\x03':
        payload = frame[0:-3]
        #print(payload)
        crc = frame[-3:-1]
        if payload and payload[0] == PACKET_INDEX_FOR_VESC:
          real_payload = payload[1:]
          parsed = parse_vesc_payload(real_payload, forwarded=True)
          if parsed:
            controllerAnswerError = False
            rpm, input_current, duty_cycle, volt, motor_current, mos_temp, motor_temp, battery_level, odometer = parsed
            data['slave']['motor_current'] = motor_current
            data['slave']['battery_current'] = input_current
            data['slave']['duty'] = duty_cycle
            data['slave']['temp'] = int(mos_temp)
            data['slave']['temp_motor'] = int(motor_temp)

    if controllerAnswerError:
      #add_speak_message("Ошибка данных контроллера слейв")
      raise SerialGetError("Error")

    process_pending_vesc_commands(ser)
    time.sleep(0.1)#0.05


def iter_vesc_port_candidates(explicit_port=None):
  seen = set()
  if explicit_port:
    print(f"Используем порт из параметров: {explicit_port}", flush=True)
    yield explicit_port
    seen.add(explicit_port)

  if VESC_PORT_OVERRIDE and VESC_PORT_OVERRIDE not in seen:
    print(f"Используем VESC_PORT_OVERRIDE: {VESC_PORT_OVERRIDE}", flush=True)
    yield VESC_PORT_OVERRIDE
    seen.add(VESC_PORT_OVERRIDE)

  env_port = os.environ.get("VESC_SERIAL_PORT")
  if env_port and env_port not in seen:
    print(f"Используем порт из окружения VESC_SERIAL_PORT={env_port}", flush=True)
    yield env_port
    seen.add(env_port)

  patterns = []
  if IS_MAC:
    patterns.extend(['/dev/tty.usbmodem*', '/dev/tty.*Bluetooth*'])
  else:
    patterns.extend(['/dev/rfcomm*', '/dev/ttyACM*'])

  for pattern in patterns:
    for candidate in sorted(glob.glob(pattern)):
      if candidate not in seen:
        print(f"Найден кандидат порта {candidate} по шаблону {pattern}", flush=True)
        yield candidate
        seen.add(candidate)

  fallback = '/dev/tty.usbmodem3041' if IS_MAC else '/dev/ttyACM0'
  if fallback not in seen:
    yield fallback

def iter_fardriver_port_candidates(explicit_port=None, controller_key='master'):
  seen = set()

  def emit(candidate, reason=None):
    if candidate and candidate not in seen:
      if reason:
        print(f"Используем FarDriver порт {candidate} ({reason})", flush=True)
      seen.add(candidate)
      return candidate
    return None

  if controller_key == 'slave':
    configured = [
      (explicit_port, "параметр"),
      (os.environ.get("FARDRIVER_SLAVE_PORT"), "FARDRIVER_SLAVE_PORT"),
      (FARDRIVER_SLAVE_PORT, "FARDRIVER_SLAVE_PORT default"),
      ("/tmp/fardriver-slave-ble", "default"),
    ]
  else:
    configured = [
      (explicit_port, "параметр"),
      (os.environ.get("FARDRIVER_MASTER_PORT"), "FARDRIVER_MASTER_PORT"),
      (FARDRIVER_MASTER_PORT, "FARDRIVER_MASTER_PORT default"),
      (os.environ.get("FARDRIVER_SERIAL_PORT"), "FARDRIVER_SERIAL_PORT"),
      (FARDRIVER_PORT_OVERRIDE, "FARDRIVER_PORT_OVERRIDE"),
      ("/tmp/fardriver-master-ble", "default"),
      ("/tmp/fardriver-ble", "совместимость со старым BLE-мостом"),
      ("/tmp/vesc-ble", "совместимость со старым BLE-мостом"),
    ]

  for candidate, reason in configured:
    value = emit(candidate, reason)
    if value:
      yield value

  patterns = []
  if IS_MAC:
    patterns.extend(['/dev/tty.*Bluetooth*', '/dev/tty.usbserial*', '/dev/tty.usbmodem*'])
  else:
    patterns.extend(['/dev/rfcomm*', '/dev/ttyUSB*', '/dev/ttyS*', '/dev/ttyACM*'])

  for pattern in patterns:
    for candidate in sorted(glob.glob(pattern)):
      value = emit(candidate, f"шаблон {pattern}")
      if value:
        yield value

  fallback = '/dev/ttyUSB0'
  value = emit(fallback, "fallback")
  if value:
    yield value


def iter_bms_port_candidates(explicit_port=None):
  seen = set()
  if explicit_port:
    bms_log(f"serial: используем порт из параметров: {explicit_port}")
    yield explicit_port
    seen.add(explicit_port)

  if BMS_PORT_OVERRIDE and BMS_PORT_OVERRIDE not in seen:
    bms_log(f"serial: используем BMS_PORT_OVERRIDE: {BMS_PORT_OVERRIDE}")
    yield BMS_PORT_OVERRIDE
    seen.add(BMS_PORT_OVERRIDE)

  env_port = os.environ.get("BMS_SERIAL_PORT")
  if env_port and env_port not in seen:
    bms_log(f"serial: используем порт из окружения BMS_SERIAL_PORT={env_port}")
    yield env_port
    seen.add(env_port)

  default_paths = ["/tmp/bms-ble"]
  for path in default_paths:
    if path not in seen:
      yield path
      seen.add(path)

  patterns = ['/dev/rfcomm*', '/dev/ttyUSB*', '/dev/ttyS*']
  for pattern in patterns:
    for candidate in sorted(glob.glob(pattern)):
      if candidate not in seen:
        bms_log(f"serial: найден кандидат порта {candidate} по шаблону {pattern}")
        yield candidate
        seen.add(candidate)

  fallback = '/dev/ttyUSB0'
  if fallback not in seen:
    yield fallback


def read_сontrollers(
                port_name=None,
                baudrate=115200,
                controller_key='master'):
  if IS_RASPBERY or not IS_MAC:
    while True:
      ser = None
      current_port = None
      is_fardriver = CONTROLLER_TYPE == "fardriver"
      candidates = iter_fardriver_port_candidates(port_name, controller_key) if is_fardriver else iter_vesc_port_candidates(port_name)
      current_baudrate = int(os.environ.get("FARDRIVER_BAUDRATE", "19200")) if is_fardriver else baudrate
      current_timeout = DEFAULT_FARDRIVER_SERIAL_TIMEOUT if is_fardriver else DEFAULT_VESC_SERIAL_TIMEOUT
      for candidate in candidates:
        print(f"Пытаемся открыть порт {candidate}", flush=True)
        try:
          ser = serial.Serial(
            candidate,
            current_baudrate,
            timeout=current_timeout,
            write_timeout=current_timeout
          )
          current_port = candidate
          print(f"{'FarDriver' if is_fardriver else 'VESC'} port open: {candidate}", flush=True)
          break
        except Exception as e:
          print(f"Не удалось открыть порт {candidate}: {e}", flush=True)
          if candidate in {VESC_PORT_OVERRIDE, FARDRIVER_PORT_OVERRIDE}:
            print("Проверь запущен ли BLE-мост (./start_ble_bridge.sh) и присутствует ли виртуальный порт", flush=True)
          ser = None
      if ser is None:
        print("Порты не открылись, повтор через 2 секунды", flush=True)
        time.sleep(2)
        continue

      try:
        print("Пауза после открытия порта 0.5 c", flush=True)
        time.sleep(0.5)
        if is_fardriver:
          read_fardriver_serial(ser, controller_key)
        else:
          read_serial(ser)
      except Exception as exc:
        print(f"Ошибка чтения {'FarDriver' if is_fardriver else 'VESC'} ({current_port}): {exc}", flush=True)
      finally:
        try:
          ser.close()
        except Exception:
          ser = None
      time.sleep(2)

if CONTROLLER_TYPE == "fardriver":
  if FARDRIVER_BLE_BACKEND == "bleak":
    threading.Thread(
      target=read_fardriver_ble,
      args=(FARDRIVER_MASTER_MAC, "master"),
      daemon=True
    ).start()
    threading.Thread(
      target=read_fardriver_ble,
      args=(FARDRIVER_SLAVE_MAC, "slave"),
      daemon=True
    ).start()
  else:
    threading.Thread(
      target=read_сontrollers,
      kwargs={"port_name": FARDRIVER_MASTER_PORT, "controller_key": "master"},
      daemon=True
    ).start()
    threading.Thread(
      target=read_сontrollers,
      kwargs={"port_name": FARDRIVER_SLAVE_PORT, "controller_key": "slave"},
      daemon=True
    ).start()
else:
  threading.Thread(target=read_сontrollers, kwargs={"port_name": VESC_PORT_OVERRIDE}, daemon=True).start()

######### BMS READ ############
def parse_temperatures(data):
  def to_temp(offset):
    return (data[offset] << 8 | data[offset + 1])

  return {
    'mosfet_temp': to_temp(91),
    'balance_temp': to_temp(93),
    'external_temp_0': to_temp(95),
    'external_temp_1': to_temp(97),
    'external_temp_2': to_temp(99),
    'external_temp_3': to_temp(101),
  }

def parse_current(bms_data: bytes) -> float | None:
  """
  Извлекает ток из пакета данных BMS. Возвращает ток в амперах (float),
  или None, если данные явно невалидные (например, 0x0000).
  """
  raw = (bms_data[108] << 8) | bms_data[109]

  # Отсекаем заведомо мусорные значения
  if raw == 0 or raw < 1000 or raw > 60000:
    return 0

  # ANT BMS: ток кодируется с offset = 30000, шаг = 0.1A
  current = (raw - 30000) * 0.1
  return round(current, 2)


def ant_crc16_modbus(payload: bytes) -> int:
  crc = 0xFFFF
  for byte in payload:
    crc ^= byte
    for _ in range(8):
      if crc & 0x0001:
        crc = (crc >> 1) ^ 0xA001
      else:
        crc >>= 1
  return crc & 0xFFFF


def build_ant_bms_ble_command(function: int, address: int, value: int) -> bytes:
  frame = bytearray([
    0x7E,
    0xA1,
    function & 0xFF,
    address & 0xFF,
    (address >> 8) & 0xFF,
    value & 0xFF,
  ])
  crc = ant_crc16_modbus(bytes(frame[1:]))
  frame.extend([crc & 0xFF, (crc >> 8) & 0xFF, 0xAA, 0x55])
  return bytes(frame)


ANT_BMS_BLE_STATUS_COMMAND = build_ant_bms_ble_command(0x01, 0x0000, 0xBE)


def parse_ant_bms_ble_status(frame: bytes) -> None:
  global BMS_LOST
  if len(frame) < 10 or not frame.startswith(b"\x7E\xA1") or frame[2] != 0x11:
    raise SerialGetError(f"unexpected ANT BLE status frame: len={len(frame)} head={frame[:6].hex()}")

  expected_len = 6 + frame[5] + 4
  if len(frame) != expected_len:
    raise SerialGetError(f"ANT BLE status length mismatch: expected {expected_len}, got {len(frame)}")

  crc = ant_crc16_modbus(frame[1:expected_len - 4])
  expected_crc = frame[expected_len - 4] | (frame[expected_len - 3] << 8)
  if crc != expected_crc:
    raise SerialGetError(f"ANT BLE status CRC mismatch: got {crc:04x}, expected {expected_crc:04x}")

  def u16(offset):
    return int.from_bytes(frame[offset:offset + 2], byteorder="little", signed=False)

  def i16(offset):
    return int.from_bytes(frame[offset:offset + 2], byteorder="little", signed=True)

  temperature_count = min(frame[8], 4)
  cell_count = min(frame[9], CELL_COUNT)
  offset = 34

  cell_voltages = []
  for i in range(cell_count):
    cell_voltages.append(u16(offset + i * 2) * 0.001)

  while len(cell_voltages) < CELL_COUNT:
    cell_voltages.append(0.0)

  offset += frame[9] * 2
  external_temps = []
  for i in range(temperature_count):
    raw_temp = i16(offset + i * 2)
    external_temps.append(0 if raw_temp <= -40 else raw_temp)

  while len(external_temps) < 4:
    external_temps.append(0)

  offset += frame[8] * 2
  mosfet_temp = i16(offset)
  balance_temp = i16(offset + 2)
  total_voltage = u16(offset + 4) * 0.01
  current = i16(offset + 6) * 0.1
  soc = u16(offset + 8)

  data['bms_temp'] = {
    'mosfet_temp': mosfet_temp,
    'balance_temp': balance_temp,
    'external_temp_0': external_temps[0],
    'external_temp_1': external_temps[1],
    'external_temp_2': external_temps[2],
    'external_temp_3': external_temps[3],
  }
  data['bms_current'] = current
  data['power'] = int(total_voltage * current)
  data['bms_voltage'] = total_voltage
  data['cells_v'] = cell_voltages
  if 0 <= soc <= 100:
    data['battery_level'] = soc
  PREV_VALS['bms_last_update'] = time.time()
  BMS_LOST = False


def assemble_ant_bms_ble_frame(buffer: bytearray, chunk: bytes) -> bytes | None:
  if chunk.startswith(b"\x7E\xA1"):
    buffer.clear()
  buffer.extend(chunk)

  start = buffer.find(b"\x7E\xA1")
  if start > 0:
    del buffer[:start]
  elif start < 0:
    buffer.clear()
    return None

  if len(buffer) < 10:
    return None

  expected_len = 6 + buffer[5] + 4
  if expected_len < 10 or expected_len > 220:
    bms_log(f"BLE frame: некорректная длина кадра {expected_len}, buffer={buffer.hex()}")
    buffer.clear()
    return None

  if len(buffer) < expected_len:
    return None

  frame = bytes(buffer[:expected_len])
  del buffer[:expected_len]
  if not frame.endswith(b"\xAA\x55"):
    bms_log(f"BLE frame: кадр без хвоста AA55: {frame.hex()}")
    return None
  return frame


def bms_ble_hex_preview(blob: bytes | bytearray, limit: int = BMS_BLE_DEBUG_BUFFER_BYTES) -> str:
  raw = bytes(blob)
  if len(raw) <= limit:
    return raw.hex()
  return f"{raw[:limit].hex()}...(+{len(raw) - limit} bytes)"


def bms_ble_device_match_reason(device) -> tuple[bool, str]:
  wanted_mac = BMS_BLE_MAC.lower()
  address = (getattr(device, "address", "") or "").lower()
  name = (getattr(device, "name", "") or "")
  name_lower = name.lower()
  metadata = getattr(device, "metadata", {}) or {}
  service_uuids = [str(u).lower() for u in metadata.get("uuids", [])]
  has_service = BMS_BLE_SERVICE_UUID.lower() in service_uuids

  if wanted_mac:
    if address == wanted_mac:
      return True, "MAC совпал с BMS_BLE_MAC"
    return False, f"MAC не совпал: {address or '-'} != {wanted_mac}"
  if BMS_BLE_NAME_PREFIX and name.startswith(BMS_BLE_NAME_PREFIX):
    return True, f"имя начинается с {BMS_BLE_NAME_PREFIX}"
  if has_service and "yuanqufoc" not in name_lower and "fardriver" not in name_lower:
    return True, f"рекламирует service {BMS_BLE_SERVICE_UUID}"
  if name_lower.startswith("yuanqufoc") or "fardriver" in name_lower:
    return False, "похоже на FarDriver, не BMS"
  if BMS_BLE_NAME_PREFIX and name:
    return False, f"имя не начинается с {BMS_BLE_NAME_PREFIX}"
  if BMS_BLE_NAME_PREFIX and not name:
    return False, f"нет имени и не видно service {BMS_BLE_SERVICE_UUID}"
  return False, f"нет service {BMS_BLE_SERVICE_UUID}"


def bms_ble_device_matches(device) -> bool:
  matched, _ = bms_ble_device_match_reason(device)
  return matched


async def find_ant_bms_ble_device():
  from bleak import BleakScanner

  bms_log(f"BLE scan: start timeout={BMS_BLE_SCAN_TIMEOUT:.1f}s name_prefix={BMS_BLE_NAME_PREFIX or '-'} mac={BMS_BLE_MAC or '-'}")
  # FarDriver and BMS share one BlueZ discovery session. Scanning concurrently
  # causes each device to receive incomplete or empty scan results.
  try:
    with FARDRIVER_BLE_SCAN_LOCK:
      devices = await asyncio.wait_for(
        BleakScanner.discover(timeout=BMS_BLE_SCAN_TIMEOUT),
        timeout=BMS_BLE_SCAN_TIMEOUT + 3.0
      )
  except Exception as exc:
    bms_log(f"BLE scan error: {format_exception(exc)}; повторим поиск")
    return None

  bms_log(f"BLE scan: найдено устройств: {len(devices)}")
  candidate_rows = []
  for device in devices:
    metadata = getattr(device, "metadata", {}) or {}
    uuids = ",".join(metadata.get("uuids", []) or [])
    rssi = getattr(device, "rssi", None)
    matched, reason = bms_ble_device_match_reason(device)
    if matched:
      candidate_rows.append(device)
    if BMS_BLE_DEBUG_SCAN or matched:
      bms_log(
        f"BLE scan: seen match={matched} reason={reason} "
        f"name={getattr(device, 'name', None)} address={getattr(device, 'address', None)} rssi={rssi} uuids={uuids or '-'}"
      )

  candidates = candidate_rows
  if not candidates and BMS_BLE_ALLOW_SINGLE_ANONYMOUS:
    anonymous = [
      device for device in devices
      if not (getattr(device, "name", "") or "").strip()
    ]
    if len(anonymous) == 1:
      candidates = anonymous
      bms_log(
        "BLE scan: выбран единственный безымянный кандидат "
        f"address={anonymous[0].address}; старый BMS_BLE_MAC не совпал, "
        "проверим устройством ANT status"
      )
  if not candidates:
    bms_log(
      "BLE scan: кандидатов нет. Если BMS виден в bluetoothctl под другим именем, "
      "запусти с BMS_BLE_NAME_PREFIX=<начало_имени> или BMS_BLE_MAC=<адрес>."
    )
    return None

  candidates.sort(key=lambda d: getattr(d, "rssi", -999) if getattr(d, "rssi", None) is not None else -999, reverse=True)
  bms_log("BLE scan: кандидаты:")
  for device in candidates:
    bms_log(f"BLE scan: candidate name={device.name} address={device.address} rssi={getattr(device, 'rssi', None)}")
  bms_log(f"BLE scan: выбран name={candidates[0].name} address={candidates[0].address}")
  return candidates[0]


async def read_bms_ble_async():
  global BMS_LOST
  from bleak import BleakClient

  while True:
    target = await find_ant_bms_ble_device()
    if target is None:
      BMS_INITIAL_ATTEMPT_DONE.set()
      BMS_LOST = True
      bms_log("BLE state: ANT-BLE устройство не найдено, повтор")
      await asyncio.sleep(BMS_BLE_RECONNECT_DELAY)
      continue

    buffer = bytearray()
    pending_status = {"future": None}
    notify_counter = 0

    def on_notify(sender, chunk: bytearray):
      nonlocal notify_counter
      notify_counter += 1
      chunk_bytes = bytes(chunk)
      if notify_counter <= BMS_BLE_DEBUG_NOTIFY:
        bms_log(f"BLE notify {notify_counter}: sender={sender} data={chunk_bytes.hex()}")

      frame = assemble_ant_bms_ble_frame(buffer, chunk_bytes)
      if not frame:
        if notify_counter <= BMS_BLE_DEBUG_NOTIFY:
          bms_log(f"BLE frame: пока не собран, buffer_len={len(buffer)} buffer={bms_ble_hex_preview(buffer)}")
        return

      bms_log(f"BLE frame: собран len={len(frame)} func=0x{frame[2]:02x} data_len={frame[5]}")
      if frame[2] != 0x11:
        bms_log(f"BLE frame: пропущен кадр func=0x{frame[2]:02x} len={len(frame)} frame={bms_ble_hex_preview(frame)}")
        return

      future = pending_status.get("future")
      if future is not None and not future.done():
        future.set_result(frame)

    try:
      bms_log(f"BLE state: connect start name={target.name} address={target.address} timeout={BMS_BLE_CONNECT_TIMEOUT:.1f}s")
      with FARDRIVER_BLE_CONNECT_LOCK:
        client = BleakClient(target, timeout=BMS_BLE_CONNECT_TIMEOUT)
        await client.connect()
      bms_log(f"BLE state: connect ok connected={client.is_connected}")
      BMS_INITIAL_ATTEMPT_DONE.set()

      try:
        services = await client.get_services()
      except Exception:
        services = client.services
      bms_log("BLE services: connected, discovered services:")
      for service in services:
        bms_log(f"BLE services: service {service.uuid}")
        for char in service.characteristics:
          bms_log(f"BLE services: char {char.uuid} props={','.join(char.properties)}")

      notify_char = BMS_BLE_NOTIFY_UUID
      write_char = BMS_BLE_WRITE_UUID
      bms_log(f"BLE state: start_notify {notify_char}")
      await client.start_notify(notify_char, on_notify)
      bms_log(f"BLE state: notify started {notify_char}, write {write_char}")
      BMS_LOST = False

      try:
        while client.is_connected:
          loop = asyncio.get_running_loop()
          pending_status["future"] = loop.create_future()
          bms_log(f"BLE poll: send status command {ANT_BMS_BLE_STATUS_COMMAND.hex()}")
          await client.write_gatt_char(write_char, ANT_BMS_BLE_STATUS_COMMAND, response=False)
          bms_log(f"BLE poll: status command sent, wait response timeout={BMS_BLE_RESPONSE_TIMEOUT:.1f}s")
          try:
            frame = await asyncio.wait_for(pending_status["future"], timeout=BMS_BLE_RESPONSE_TIMEOUT)
            parse_ant_bms_ble_status(frame)
            bms_log(
              f"BLE data: {data['bms_voltage']:.1f}V {data['bms_current']:.1f}A "
              f"{int(data['battery_level'])}% cells={sum(1 for v in data['cells_v'] if v > 0)}"
            )
          except asyncio.TimeoutError:
            BMS_LOST = True
            bms_log(
              f"BLE timeout: нет ответа на status, connected={client.is_connected}, "
              f"notify_count={notify_counter}, buffer_len={len(buffer)}, buffer={bms_ble_hex_preview(buffer)}"
            )
            break
          except Exception as exc:
            BMS_LOST = True
            bms_log(
              f"BLE parse error: {type(exc).__name__}: {exc}; "
              f"last_frame={bms_ble_hex_preview(frame) if 'frame' in locals() else '-'}"
            )
            if os.environ.get("BMS_BLE_DEBUG_TRACEBACK", "1").strip().lower() not in {"0", "false", "off", "no"}:
              traceback.print_exc()
            break
          await asyncio.sleep(BMS_BLE_POLL_INTERVAL)
      finally:
        bms_log("BLE state: disconnect start")
        try:
          await client.stop_notify(notify_char)
        except Exception:
          pass
        await client.disconnect()
        bms_log("BLE state: disconnected")
    except Exception as exc:
      BMS_INITIAL_ATTEMPT_DONE.set()
      BMS_LOST = True
      bms_log(f"BLE error: {type(exc).__name__}: {exc}")
      if os.environ.get("BMS_BLE_DEBUG_TRACEBACK", "1").strip().lower() not in {"0", "false", "off", "no"}:
        traceback.print_exc()

    await asyncio.sleep(BMS_BLE_RECONNECT_DELAY)


def read_bms_ble():
  try:
    asyncio.run(read_bms_ble_async())
  except Exception as exc:
    bms_log(f"BLE fatal: {type(exc).__name__}: {exc}")


def read_bms_data(ser):
  while True:
    ser.write(b'\x5A\x5A\x00\x00\x00\x00')
    bms_data = ser.read(140)

    if len(bms_data) != 140 or not bms_data.startswith(b'\xAA\x55\xAA\xFF'):
      bms_log(f"serial: некорректный ответ len={len(bms_data)} head={bms_data[:8].hex()}")
      #add_speak_message("Некорректный ответ от BMS")
      raise SerialGetError("Error")

    # Общий вольтаж: bms_data[4] и bms_data[5], шаг 0.1 В
    total_voltage = (bms_data[4] << 8 | bms_data[5]) * 0.1
    raw = int.from_bytes(bms_data[72:74], byteorder='big', signed=True)
    current = raw * 0.1  # ← без смещения!

    temp_info = parse_temperatures(bms_data)
    data['bms_temp'] = temp_info

    data['bms_current'] = current
    data['power'] = int(total_voltage * current)
    data['bms_voltage'] = total_voltage
    PREV_VALS['bms_last_update'] = time.time()

    # Вольтаж каждой ячейки: bms_data[6]..bms_data[69], по 2 байта на ячейку, шаг 1 мВ
    cell_voltages = []
    for i in range(CELL_COUNT):  # ANT BMS отдаёт до 32 значений, забираем нужное количество
      high = bms_data[6 + i * 2]
      low = bms_data[6 + i * 2 + 1]
      voltage = (high << 8 | low) * 0.001  # в В
      cell_voltages.append(float(voltage))

    bms___ = {
      "total_voltage": total_voltage,
      "current": current,
      "cell_voltages": cell_voltages
    }

    data['cells_v'] = cell_voltages




    time.sleep(0.1)

def read_bms(
                port_name=None,
                baudrate=19200):
  global BMS_LOST
  if IS_RASPBERY or not IS_MAC:
    while True:
      ser = None
      current_port = None
      for candidate in iter_bms_port_candidates(port_name):
        try:
          ser = serial.Serial(candidate, baudrate, timeout=0.1)
          current_port = candidate
          bms_log(f"serial: port open {candidate}")
          BMS_LOST = False
          time.sleep(0.5)
          try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
          except Exception:
            pass
          break
        except Exception as e:
          bms_log(f"serial: не удалось открыть порт {candidate}: {e}")
          if candidate == (port_name or BMS_PORT_OVERRIDE) or candidate == BMS_PORT_OVERRIDE:
            bms_log("serial: проверь BLE-мост для BMS (./start_ble_bms_bridge.sh) и наличие /tmp/bms-ble")
          ser = None
      if ser is None:
        BMS_LOST = True
        bms_log("serial: порты не открылись, повтор через 2 секунды")
        time.sleep(2)
        continue

      try:
        read_bms_data(ser)
      except Exception as exc:
        BMS_LOST = True
        bms_log(f"serial: ошибка чтения ({current_port}): {exc}")
        time.sleep(2)
      finally:
        try:
          ser.close()
        except Exception:
          ser = None
      time.sleep(2)


def nmea_float(value):
  try:
    return float(value)
  except (TypeError, ValueError):
    return None

def nmea_int(value):
  try:
    return int(value)
  except (TypeError, ValueError):
    return None

def nmea_checksum_ok(line):
  if "*" not in line:
    return True
  payload, checksum = line[1:].split("*", 1)
  checksum = checksum[:2]
  try:
    expected = int(checksum, 16)
  except ValueError:
    return False
  actual = 0
  for char in payload:
    actual ^= ord(char)
  return actual == expected

def gps_accept_speed(speed_kmh):
  if speed_kmh is None:
    return
  if speed_kmh < 0 or speed_kmh > GPS_MAX_REASONABLE_SPEED_KMH:
    print(f"GPS: отброшена подозрительная скорость {speed_kmh:.1f} км/ч", flush=True)
    return
  data['gps_speed'] = max(0.0, speed_kmh)
  data['gps_speed_last_update'] = time.time()

def parse_nmea_sentence(line):
  if not line.startswith("$"):
    return
  if not nmea_checksum_ok(line):
    print(f"GPS: отброшена строка с плохой checksum: {line}", flush=True)
    return
  body = line[1:].split("*", 1)[0]
  parts = body.split(",")
  if not parts:
    return
  sentence_type = parts[0][-3:]
  now = time.time()

  if sentence_type == "RMC":
    # $GPRMC,time,status,lat,N,lon,E,speed_knots,...
    mode = parts[12] if len(parts) > 12 else ""
    valid = parts[2] == "A" and mode != "N"
    data['gps_fix'] = valid
    if len(parts) > 7 and valid:
      speed_knots = nmea_float(parts[7])
      if speed_knots is not None:
        gps_accept_speed(speed_knots * 1.852)
  elif sentence_type == "VTG":
    # $GPVTG,...,speed_knots,N,speed_kmh,K
    mode = parts[9] if len(parts) > 9 else ""
    if len(parts) > 7 and data.get('gps_fix', False) and mode != "N":
      speed_kmh = nmea_float(parts[7])
      if speed_kmh is not None:
        gps_accept_speed(speed_kmh)
  elif sentence_type in {"GGA", "GNS"}:
    # GGA/GNS: satellites used in fix.
    if sentence_type == "GGA" and len(parts) > 6:
      data['gps_fix'] = (nmea_int(parts[6]) or 0) > 0
    elif sentence_type == "GNS" and len(parts) > 6:
      data['gps_fix'] = bool(parts[6].strip()) and parts[6].strip("N") != ""
    if len(parts) > 7:
      satellites = nmea_int(parts[7])
      if satellites is not None:
        data['gps_satellites'] = max(0, satellites)
        data['gps_satellites_last_update'] = now
  elif sentence_type == "GSV":
    # GSV: total satellites in view.
    if len(parts) > 3:
      satellites = nmea_int(parts[3])
      if satellites is not None:
        data['gps_satellites'] = max(0, satellites)
        data['gps_satellites_last_update'] = now

def iter_gps_port_candidates(explicit_port=None):
  seen = set()

  def emit(candidate, reason=None):
    if candidate and candidate not in seen:
      if reason:
        print(f"GPS: кандидат порта {candidate} ({reason})", flush=True)
      seen.add(candidate)
      return candidate
    return None

  configured = [
    (explicit_port, "параметр"),
    (GPS_SERIAL_PORT, "GPS_SERIAL_PORT"),
  ]
  for candidate, reason in configured:
    value = emit(candidate, reason)
    if value:
      yield value

  patterns = []
  if IS_MAC:
    patterns.extend(['/dev/tty.usbmodem*', '/dev/tty.usbserial*'])
  else:
    patterns.extend([
      '/dev/serial/by-id/*u-blox*',
      '/dev/serial/by-id/*U-Blox*',
      '/dev/serial/by-id/*GPS*',
      '/dev/serial/by-id/*gps*',
      '/dev/ttyACM*',
      '/dev/ttyUSB*',
    ])

  for pattern in patterns:
    for candidate in sorted(glob.glob(pattern)):
      value = emit(candidate, f"шаблон {pattern}")
      if value:
        yield value

def read_gps(port_name=None, baudrate=GPS_BAUDRATE):
  while True:
    ser = None
    current_port = None
    for candidate in iter_gps_port_candidates(port_name):
      try:
        ser = serial.Serial(candidate, baudrate, timeout=GPS_SERIAL_TIMEOUT)
        current_port = candidate
        print(f"GPS port open: {candidate} baud={baudrate}", flush=True)
        try:
          ser.reset_input_buffer()
        except Exception:
          pass
        break
      except Exception as exc:
        print(f"GPS: не удалось открыть {candidate}: {exc}", flush=True)
        ser = None

    if ser is None:
      print("GPS: порты не открылись, повтор через 5 секунд", flush=True)
      time.sleep(5)
      continue

    last_sentence_at = time.time()
    try:
      while True:
        raw_line = ser.readline()
        if not raw_line:
          if time.time() - last_sentence_at > 10:
            raise SerialGetError("GPS NMEA timeout")
          continue
        line = raw_line.decode("ascii", errors="ignore").strip()
        if not line:
          continue
        if line.startswith("$"):
          last_sentence_at = time.time()
          parse_nmea_sentence(line)
    except Exception as exc:
      print(f"GPS ошибка чтения ({current_port}): {exc}", flush=True)
      time.sleep(2)
    finally:
      try:
        ser.close()
      except Exception:
        pass
    time.sleep(2)



if ENABLE_BMS:
  if BMS_BACKEND in {"ble", "bleak", "bluetooth"}:
    bms_log("start: чтение включено через BLE/bleak")
    threading.Thread(target=read_bms_ble, daemon=True).start()
  else:
    bms_log("start: чтение включено через serial/rfcomm")
    threading.Thread(target=read_bms, kwargs={"port_name": BMS_PORT_OVERRIDE}, daemon=True).start()
else:
  bms_log("start: чтение отключено (ENABLE_BMS=0)")

if GPS_ENABLED:
  threading.Thread(target=read_gps, kwargs={"port_name": GPS_SERIAL_PORT or None}, daemon=True).start()
else:
  print("GPS чтение отключено (GPS_ENABLED=0)", flush=True)

######## INTERFACE ###########
import pygame
import math
import time

pygame.init()

WIDTH, HEIGHT = 600, 1010
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption('VESC ANT Speedometer')

class ThemedFont:
  def __init__(self, base_font):
    self._font = base_font

  def render(self, text, antialias, color, background=None):
    color = themed_text_color(color)
    background = themed_text_color(background)
    return self._font.render(text, antialias, color, background)

  def __getattr__(self, attr):
    return getattr(self._font, attr)

font_large = ThemedFont(pygame.font.SysFont('Arial', 310))
font_medium = ThemedFont(pygame.font.SysFont('Arial', 50, True))
font_small = ThemedFont(pygame.font.SysFont('Arial', 40, True))
font_tick = ThemedFont(pygame.font.SysFont('Arial', 30, True))
font_gps = ThemedFont(pygame.font.SysFont('Arial', 34, True))

def render_force_color(font_obj, text, color, background=None):
  base_font = font_obj._font if isinstance(font_obj, ThemedFont) else font_obj
  return base_font.render(text, True, color, background)
clock = pygame.time.Clock()

setDebugValues = False
needSetValues = False

def draw_progress_bar(surface, x, y, width, height, value, max_value, text, color, active = True):
  back_color = (200, 200, 200)
  if not active:
    back_color = (240, 240, 240)
  pygame.draw.rect(surface, back_color, (x, y, width, height), border_radius=10)

  cur_val = value
  if cur_val > max_value:
    cur_val = max_value

  fill_width = int(width * min(cur_val / max_value, 1.0))
  fill_color = color
  fill_x = x
  fill_y = y
  fill_height = height
  fill_radius = 10
  if is_dark_theme() and color == (0, 0, 0):
    vertical_inset = min(height - 4, int(height * 0.4))
    horizontal_inset = min(width - 4, int(width * 0.1))
    fill_y += vertical_inset // 2
    fill_x += horizontal_inset // 2
    fill_height = max(2, height - vertical_inset)
    fill_width = max(2, fill_width - horizontal_inset)
    fill_radius = max(2, fill_radius - max(1, vertical_inset // 3))
  if fill_width > 0:
    pygame.draw.rect(surface, fill_color, (fill_x, fill_y, fill_width, fill_height), border_radius=fill_radius)
  if text != "":
    draw_text(surface, text, font_medium, color, x + width * 0.5, y + height + 30)

def draw_filled_arc(surface, color, center, radius, start_angle, end_angle, segments=100):
  points = [center]
  for i in range(segments + 1):
    angle = start_angle + (end_angle - start_angle) * i / segments
    x = center[0] + radius * math.cos(angle)
    y = center[1] + radius * math.sin(angle)
    points.append((x, y))
  pygame.draw.polygon(surface, color, points)
  pygame.draw.rect(surface, color, (165, 150, 270, 120))

def draw_speed_arc(surface, center, radius, speed, max_speed, up_gap):
  #temp disable slave
  #av_duty = int((data['slave']['duty'] + data['master']['duty']) / 2)
  av_duty = int(data['master']['duty'])

  #if speed > 0:
  #  if av_duty >= 85:
  #    draw_filled_arc(surface, (255, 180, 180), center, radius, math.pi * 0.15, -math.pi * 1.15)

  pygame.draw.arc(surface, (200, 200, 200), (center[0]-radius, center[1]-radius, radius*2, radius*2),
                  -math.pi * 0.15, math.pi * 1.15, 20)
  end_angle = math.pi * 1.15 - ((speed) / max_speed) * math.pi * 1.3
  if speed > 0:
    speedColor = GREEN_LIGHT
    if av_duty >= 80:
      speedColor = (255, 0, 0)

    pygame.draw.arc(surface, speedColor, (center[0]-radius, center[1]-radius, radius*2, radius*2),
                    end_angle, math.pi * 1.15, 20)
    
    end_angle = math.pi * 1.15 - ((speed) / max_speed) * math.pi * 1.3
    # Маленький зелёный маркер на дуге
    marker_outer_x = center[0] + (radius - 1)* math.cos(end_angle)
    marker_outer_y = center[1] - (radius - 1) * math.sin(end_angle)
    marker_inner_x = center[0] + (radius - 50) * math.cos(end_angle)
    marker_inner_y = center[1] - (radius - 50) * math.sin(end_angle)
    pygame.draw.line(surface, speedColor, (marker_inner_x, marker_inner_y), (marker_outer_x, marker_outer_y), 10)

  # Отметки скорости
  for mark in range(0, int(max_speed) + 1, 20):
    angle = math.pi * 0.85 + (mark / max_speed) * math.pi * 1.3
    x_outer = center[0] + (radius + 5) * math.cos(angle)
    y_outer = center[1] + (radius + 5) * math.sin(angle)
    x_inner = center[0] + (radius - 25) * math.cos(angle)
    y_inner = center[1] + (radius - 25) * math.sin(angle)
    pygame.draw.line(surface, (60, 60, 60), (x_inner, y_inner), (x_outer, y_outer), 3)

    x = center[0] + radius * 1.17 * math.cos(angle)
    y = center[1] + radius * 1.17 * math.sin(angle)
    label = font_tick.render(str(mark), True, (0, 0, 0))
    label_rect = label.get_rect(center=(x, y))
    surface.blit(label, label_rect)

  # TEXT
  text_color = (0, 0, 0)
  #if av_duty >= 85:
  #  text_color = (255, 255, 255)
  draw_text_center(screen, f"{int(data['speed'])}", font_large, text_color, 180 + up_gap)


def draw_arc(text, surface, center, radius, speed, max_speed, color):
  draw_text(screen, text, font_medium, color, center[0], center[1] - 5)
  pygame.draw.arc(surface, (200, 200, 200), (center[0]-radius, center[1]-radius, radius*2, radius*2),
                  -math.pi * 0.15, math.pi * 1.15, 15)
  end_angle = math.pi * 1.15 - (speed / max_speed) * math.pi * 1.3
  if speed > 0:
    pygame.draw.arc(surface, color, (center[0]-radius, center[1]-radius, radius*2, radius*2),
                    end_angle, math.pi * 1.15, 15)
    
    return
    # Маленький зелёный маркер на дуге
    marker_outer_x = center[0] + (radius - 1)* math.cos(end_angle)
    marker_outer_y = center[1] - (radius - 1) * math.sin(end_angle)
    marker_inner_x = center[0] + (radius - 50) * math.cos(end_angle)
    marker_inner_y = center[1] - (radius - 50) * math.sin(end_angle)
    pygame.draw.line(surface, GREEN_COLOR, (marker_inner_x, marker_inner_y), (marker_outer_x, marker_outer_y), 10)

  return
  # Отметки скорости
  for mark in [0, 20, 40, 60]:
    angle = math.pi * 0.85 + (mark / max_speed) * math.pi * 1.3
    x_outer = center[0] + (radius + 5) * math.cos(angle)
    y_outer = center[1] + (radius + 5) * math.sin(angle)
    x_inner = center[0] + (radius - 20) * math.cos(angle)
    y_inner = center[1] + (radius - 20) * math.sin(angle)
    pygame.draw.line(surface, (60, 60, 60), (x_inner, y_inner), (x_outer, y_outer), 3)

    x = center[0] + radius * 1.2 * math.cos(angle)
    y = center[1] + radius * 1.2 * math.sin(angle)
    label = font_small.render(str(mark), True, (0, 0, 0))
    label_rect = label.get_rect(center=(x, y))
    surface.blit(label, label_rect)

def draw_text_center(surface, text, font, color, y):
  render = font.render(text, True, color)
  rect = render.get_rect(center=(WIDTH//2, y + font_y_shift))
  surface.blit(render, rect)

def draw_text(surface, text, font, color, x, y):
  render = font.render(text, True, color)
  rect = render.get_rect(center=(x, y + font_y_shift))
  surface.blit(render, rect)

def draw_text_left(surface, text, font, color, x, y):
  render = font.render(text, True, color)
  rect = render.get_rect(topleft=(x, y + font_y_shift))
  surface.blit(render, rect)

def draw_text_right(surface, text, font, color, x, y):
  render = font.render(text, True, color)
  rect = render.get_rect(topright=(x, y + font_y_shift))
  surface.blit(render, rect)

def draw_gps_overlay(surface):
  now = time.time()
  speed_stale = now - data.get('gps_speed_last_update', 0) > GPS_STALE_SECONDS
  sats_stale = now - data.get('gps_satellites_last_update', 0) > GPS_STALE_SECONDS
  speed_text = "--" if speed_stale else f"{data.get('gps_speed', 0.0):.1f}"
  sats_text = "--" if sats_stale else f"{int(data.get('gps_satellites', 0))}"
  draw_text_left(surface, speed_text, font_gps, RED_COLOR, 10, 58)
  draw_text_right(surface, sats_text, font_gps, RED_COLOR, WIDTH - 10, 58)

def draw_cells_block(screen, startY):
  x_shift = WIDTH * 0.425
  y_shift = startY

  cells_with_index = list(enumerate(data['cells_v']))
  if not cells_with_index:
    return

  sorted_cells = sorted(cells_with_index, key=lambda item: (item[1], item[0]))

  bad_cell_index, bad_cell_min = sorted_cells[0]
  good_cell_index, good_cell_max = sorted_cells[-1]

  def chunk_cells(cells):
    padded = list(cells) + [None] * max(0, 4 - len(cells))
    return [padded[0:2], padded[2:4]]

  worst_chunks = chunk_cells(sorted_cells[:4])
  best_chunks = chunk_cells(sorted_cells[-4:])

  rows = worst_chunks + [[{'type': 'placeholder'}, {'type': 'placeholder'}]] + best_chunks

  def prepare_slot(entry):
    if isinstance(entry, dict):
      return entry
    if entry is None:
      return {'type': 'empty'}
    index, voltage = entry
    return {
      'type': 'cell',
      'index': index,
      'voltage': voltage,
    }

  rows = [[prepare_slot(slot) for slot in row] for row in rows]

  for row in rows:
    for col, slot in enumerate(row):
      left_boost = 10 if col == 0 else 190
      rect = (x_shift + left_boost - 15, y_shift + 2, 155, 38)

      if slot['type'] == 'placeholder':
        pygame.draw.rect(screen, (200, 200, 200), rect, width=2, border_radius=10)
        draw_text(screen, '...', font_small, (150, 150, 150), x_shift + left_boost + 70, y_shift + 20)
        continue

      if slot['type'] == 'empty':
        pygame.draw.rect(screen, (200, 200, 200), rect, width=2, border_radius=10)
        continue

      cell_index = slot['index']
      cell_voltage = slot['voltage']

      cell_color = (200, 200, 200)
      cell_index_color = (0, 0, 0)
      cell_v_color = (150, 150, 150)

      if cell_index == good_cell_index:
        cell_color = GREEN_COLOR
        cell_index_color = cell_color
        cell_v_color = cell_color
      if cell_index == bad_cell_index:
        cell_color = (255, 0, 0)
        cell_index_color = cell_color
        cell_v_color = cell_color

      pygame.draw.rect(screen, cell_color, rect, width=2, border_radius=10)
      draw_text(screen, f"{cell_index + 1}", font_small, cell_index_color, x_shift + left_boost + 15, y_shift + 20)
      draw_text(screen, f"{cell_voltage:.2f}", font_small, cell_v_color, x_shift + left_boost + 90, y_shift + 20)

    y_shift += 43

  data['unit_diff'] = good_cell_max - bad_cell_min
  data['bad_cell_min'] = bad_cell_min
  data['bad_cell_index'] = bad_cell_index
  if data['bad_cell_min_peak'] > data['bad_cell_min']:
    data['bad_cell_min_peak'] = data['bad_cell_min']
    data['bad_cell_min_peak_index'] = bad_cell_index


def get_battery_color(level):
  if level < 25:
    return (255, 0, 0)
  elif level < 50:
    return ORANGE_COLOR
  else:
    return GREEN_COLOR
  
def get_unit_diff_color(volt):
  if volt >= 0.05:
    return (255, 0, 0)
  elif volt >= 0.03:
    return ORANGE_COLOR
  else:
    return GREEN_COLOR
  
def get_battery_temp_color(temp):
  if temp >= 55:
    return (255, 0, 0)
  elif temp >= 40:
    return ORANGE_COLOR
  else:
    return GREEN_COLOR
  
def get_motor_temp_color(temp):
  if temp >= 80:
    return RED_COLOR
  elif temp >= 60:
    return ORANGE_COLOR
  else:
    return GREEN_COLOR
  
def SaveData():
  try:
    with open("mainData.txt", "w") as f:
      total_odometer = data['odometer'] + data['trip_odometer']
      f.write(f"{total_odometer:.1f}\n{video_record_counter}")
  except Exception as e:
    print("Ошибка сохранения данных:", e)

def SetDebugValues():
  #DEBUG_VISUAL_TEST
  changeV = time.time() % 5 / 5

  data['master']['motor_current'] = 200 * changeV
  data['slave']['motor_current'] = 180 * (1 - changeV)
  data['bms_current'] = 50 * changeV
  data['speed'] = 120 * changeV
  data['master']['duty'] = 300 * changeV
  data['slave']['duty'] = 280 * (1 - changeV)
  data['master']['battery_current'] = 60 * changeV
  data['slave']['battery_current'] = 40 * (1 - changeV)
  data['bms_voltage'] = 60 - 10 * changeV
  if data['master']['duty'] > 200:
    data['master']['duty'] = 200
  if data['slave']['duty'] > 200:
    data['slave']['duty'] = 200

# Переменные для замера разгона 0-60 км/ч
start_time = None
measured_time = None
measured_time_0_100 = None
ready = True
measuring = False
trip_start_time = None
timer_power_off = None
block_touch = False
prev_speed = 0
zamer_success = False
zamer_0_100_success = False
zamer_success_prev = False
ACCEL_MEASURE_TIMEOUT = 25.0

can_start_record = True
recorder_proc = None

full_off = False

PAGE_NAME = "SPEEDOMETER"

miganie = False
miganie_tick = 0

trip_end_datetime_str = ""
trip_end_datetime_str_full = ""

# === УСКОРЕНИЕ (график во время замера 0-60) ===
accel_history = []          # список значений ускорения (м/с^2)
accel_last_sample_time = None
accel_last_speed_mps = 0.0  # последняя скорость в м/с для расчёта dV
ACCEL_SAMPLE_PERIOD = 0.30  # 100 мс
ACCEL_HISTORY_MAX = 300     # ~30 сек при шаге 0.1с

running = True
#FULL_SCREEN
if IS_RASPBERY and PYGAME_FULLSCREEN:
  screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)

while running:
  for event in pygame.event.get():
    if event.type == pygame.QUIT:
      running = False
    elif PAGE_NAME == PAGE_LOCK and event.type == pygame.KEYDOWN:
      if event.key == pygame.K_RETURN:
        if len(lock_input) == 4 and lock_input == LOCK_PASSWORD:
          deactivate_lock(True)
        else:
          lock_message = "Пароль неверный"
          lock_input = ""
      elif event.key == pygame.K_BACKSPACE:
        handle_backspace_lock()
      else:
        ch = event.unicode
        if ch.isdigit():
          handle_lock_digit(ch)
  if PAGE_NAME == PAGE_LOCK:
    draw_lock_page()
    pygame.display.flip()
    clock.tick(30)
    continue

  screen.fill(get_theme_color("background", (254, 254, 254)))

  if miganie_tick > 3:
    miganie = not miganie
    miganie_tick = 0
  else:
    miganie_tick += 1

  if not IS_RASPBERY:
    SetDebugValues()

  up_gap = 25
  if PREV_VALS['page_name'] == "SPEEDOMETER" and PAGE_NAME == "TRIP_STAT":
    #add_speak_message("Статистика поездки:")
    #add_speak_message("Приехал " + trip_end_datetime_str_full)
    trip_info = data['trip_time']
    if trip_info[0] == '0':
      trip_info = trip_info[1:]
      trip_info = trip_info.replace(":", " и ")
    add_speak_message("Время в пути " + trip_info)
    add_speak_message("Приехал " + trip_end_datetime_str_full)
    add_speak_message("Максимальные температуры моторов..." + f"{int(data_trip['motor1_max_temp'])} ... и {int(data_trip['motor2_max_temp'])} градусов")
    #add_speak_message("Расстояние " + f"{data['trip_odometer']:.1f}".replace(".", " и ") + " километров")
    #add_speak_message("Средняя скорость " + f"{data['trip_avg_speed']:.1f}".replace(".", " и ") + " километров в час")
    add_speak_message("Максимальная скорость " + f"{int(data_trip['max_speed'])} километров в час")
    #add_speak_message("Лучшее время от 0 до 60 составило " + f"{data_trip['best_time_0_60']:.2f}".replace(".", " и ") + " секунд")
    #add_speak_message("Максимальная мощность " + f"{int(data_trip['max_power'])} ват")
    #add_speak_message("Потрачено заряда " + f"{int(data_trip['trip_start_bettery_perc'] - data['battery_level'])} процентов")
    #add_speak_message("Максимальная просадка " + f"{data_trip['max_voltage_down']:.1f}".replace(".", " и ") + " вольт")
    add_speak_message("Слабейший ряд " + f"{data_trip['min_cell_v_index'] + 1}")
    add_speak_message("Минимальный вольтаж в ряду " + f"{data_trip['min_cell_v']:.2f}".replace(".", " и ") + " вольт")
    #add_speak_message("Максимальный разбаланс " + f"{data_trip['max_unit_diff']:.2f}".replace(".", " и ") + " вольт")
    add_speak_message("До свидания")

  PREV_VALS['page_name'] = PAGE_NAME
  if PAGE_NAME == "SPEEDOMETER":
    # 1. Скорость полукруг 
    #average_duty = int((data['slave']['duty'] + data['master']['duty']) / 2)
    average_duty = int(data['master']['duty'])
    speed_color = (0, 0, 0)
    #if average_duty >= 85:
    #  speed_color = (255, 0, 0)

    #draw_speed_arc(screen, (WIDTH//2, 180 + up_gap + CONTENT_Y_OFFSET), 150, int(data['speed']), 100, up_gap)
    speed_y = 180 + up_gap + CONTENT_Y_OFFSET - 115
    draw_text_center(screen, f"{int(data['speed'])}", font_large, (0, 0, 0), speed_y + 10)

    # Прогресс-бар скорости (0-100 км/ч) над цифрой
    bar_max_speed = 70
    bar_width = 350
    bar_height = 25
    bar_x = (WIDTH - bar_width) / 2
    bar_y = speed_y - font_large.get_height() / 2 - bar_height - 8 + 35
    pygame.draw.rect(screen, (200, 200, 200), (bar_x, bar_y, bar_width, bar_height), width=2, border_radius=15)

    cur_speed = max(0, data['speed'])
    base_fill = min(cur_speed, bar_max_speed)
    base_width = bar_width * (base_fill / bar_max_speed)
    blinking_over_limit = cur_speed > bar_max_speed
    base_color = GREEN_COLOR if not blinking_over_limit else ((255, 0, 0) if miganie else (160, 160, 160))
    if base_width > 0:
      pygame.draw.rect(
        screen,
        base_color,
        (bar_x, bar_y, base_width, bar_height),
        border_radius=9
      )

    if cur_speed > bar_max_speed:
      overflow = cur_speed - bar_max_speed
      overflow_width = bar_width * (overflow / bar_max_speed)
      overflow_color = (255, 0, 0) if miganie else (160, 160, 160)
      pygame.draw.rect(
        screen,
        overflow_color,
        (bar_x + bar_width + 5, bar_y, overflow_width, bar_height),
        border_radius=9
      )

    # 2. Показатели контроллеров мастер и слейв
    summ_current = data['slave']['motor_current'] + data['master']['motor_current']
    #if summ_current > 200:
    #  summ_current = 200
    
    up_gap += 20
    stats_bar_width = 100
    stats_bar_height = 15
    stats_bar_spacing = 140
    stats_block_y = speed_y + 155  # опустили полосы на 50px ниже текущей позиции
    bar_offsets = [
      -1.7 * stats_bar_spacing,
      -0.5 * stats_bar_spacing - 50,
      0.5 * stats_bar_spacing + 50,
      1.7 * stats_bar_spacing,
    ]

    slave_motor_current = int(abs(data['slave']['motor_current']))
    slave_battery_current = int(abs(data['slave']['battery_current']))
    slave_duty = int(abs(data['slave']['duty']))

    master_motor_current = int(abs(data['master']['motor_current']))
    master_battery_current = int(abs(data['master']['battery_current']))
    master_duty = int(abs(data['master']['duty']))
    rpm_source_key = data.get('speed_source_controller', 'master')
    rpm_source = data.get(rpm_source_key, data['master'])
    rpm_display = int(abs(rpm_source.get('rpm', 0)))

    bar_lineup = [
      (slave_duty, 100, f"{slave_duty}", (0, 0, 0)),
      (slave_battery_current, 80, f"{slave_battery_current}A", (0, 0, 255)),
      (master_battery_current, 80, f"{master_battery_current}A", (0, 0, 255)),
      (master_duty, 100, f"{master_duty}", (0, 0, 0)),
    ]

    for (value, max_val, text, color), dx in zip(bar_lineup, bar_offsets):
      x = WIDTH * 0.5 + dx - stats_bar_width / 2
      draw_progress_bar(screen, x, stats_block_y, stats_bar_width, stats_bar_height, value, max_val, text, color)

    draw_text_center(screen, f"{rpm_display}", font_small, (0, 0, 0), stats_block_y + stats_bar_height / 2)

    # Временный overlay мощности/оборотов/фазного тока пока скрыт.
    # rpm_display = int(abs(data['master'].get('rpm', 0)))
    # phase_current_display = int(abs(data['master']['motor_current']))
    # draw_text_center(
    #   screen,
    #   f"{get_display_power()}W  {rpm_display}rpm  Ф{phase_current_display}A",
    #   font_small,
    #   (0, 0, 0),
    #   295 + CONTENT_Y_OFFSET
    # )

    # Когда ослабление магнитного поля активно рисуем рамку
    #if average_duty >= 85:
    #  pygame.draw.rect(screen, (255, 0, 0), (0, 0, WIDTH, HEIGHT), width=12, border_radius=0)

    #Температура всего
    temp_y = 345 + CONTENT_Y_OFFSET
    border_r = 10
    pygame.draw.rect(screen, (200, 200, 200), (15, temp_y - 22, WIDTH * 0.46, 44), width=2, border_radius=border_r)
    draw_text(screen, f"МК", font_small, (200, 200, 200), WIDTH * 0.1, temp_y)
    motor1_temp_color = get_motor_temp_color(int(data['slave']['temp_motor']))
    draw_text(screen, f"{int(data['slave']['temp_motor'])}°", font_small, motor1_temp_color, WIDTH * 0.25, temp_y)
    motor2_temp_color = get_motor_temp_color(int(data['master']['temp_motor']))
    draw_text(screen, f"{int(data['master']['temp_motor'])}°", font_small, motor2_temp_color, WIDTH * 0.4, temp_y)

    temp_y += 50
    pygame.draw.rect(screen, (200, 200, 200), (15, temp_y - 22, WIDTH * 0.46, 44), width=2, border_radius=border_r)
    draw_text(screen, f"К", font_small, (200, 200, 200), WIDTH * 0.1, temp_y)
    draw_text(screen, f"{int(data['slave']['temp'])}°", font_small, GREEN_COLOR, WIDTH * 0.25, temp_y)
    draw_text(screen, f"{int(data['master']['temp'])}°", font_small, GREEN_COLOR, WIDTH * 0.4, temp_y)

    temp_y -= 50
    block_x = WIDTH * 0.5 + 6
    block_w = WIDTH * 0.46 + 4
    pygame.draw.rect(screen, (200, 200, 200), (block_x, temp_y - 22, block_w, 44), width=2, border_radius=border_r)
    draw_text(screen, f"М/Б", font_small, (200, 200, 200), WIDTH * 0.6, temp_y)
    mos_color = get_battery_temp_color(int(data['bms_temp']['mosfet_temp']))
    draw_text(screen, f"{int(data['bms_temp']['mosfet_temp'])}°", font_small, mos_color, WIDTH * 0.75, temp_y)
    bal_color = get_battery_temp_color(int(data['bms_temp']['balance_temp']))
    draw_text(screen, f"{int(data['bms_temp']['balance_temp'])}°", font_small, bal_color, WIDTH * 0.9, temp_y)

    temp_y += 50
    block_x = WIDTH * 0.5 + 6
    block_w = WIDTH * 0.46 + 4
    pygame.draw.rect(screen, (200, 200, 200), (block_x, temp_y - 22, block_w, 44), width=2, border_radius=border_r)
    for idx, key in enumerate(EXTERNAL_TEMP_KEYS):
      sensor_temp = int(data['bms_temp'][key])
      sensor_color = get_battery_temp_color(sensor_temp)
      center_x = block_x + block_w * ((idx + 0.5) / len(EXTERNAL_TEMP_KEYS)) + 5
      draw_text(screen, f"{sensor_temp}°", font_small, sensor_color, center_x, temp_y)

    # temp alarm
    # bms
    cur_t = time.time()
    if cur_t - PREV_VALS['mosfet_temp_get_last_time'] > 30:
      if data['bms_temp']['mosfet_temp'] >= 40 and PREV_VALS['bms_temp']['mosfet_temp'] < 40:
        add_speak_message(f"Температура мосфетов БМС достигла... {data['bms_temp']['mosfet_temp']} градусов")
        PREV_VALS['mosfet_temp_get_last_time'] = cur_t
      if data['bms_temp']['mosfet_temp'] >= 55 and PREV_VALS['bms_temp']['mosfet_temp'] < 55:
        add_speak_message(f"Внимание... Температура мосфетов БМС достигла... {data['bms_temp']['mosfet_temp']} градусов")
        PREV_VALS['mosfet_temp_get_last_time'] = cur_t

    if cur_t - PREV_VALS['balance_temp_get_last_time'] > 30:
      if data['bms_temp']['balance_temp'] >= 40 and PREV_VALS['bms_temp']['balance_temp'] < 40:
        add_speak_message(f"Температура балансиров БМС достигла... {data['bms_temp']['balance_temp']} градусов")
        PREV_VALS['balance_temp_get_last_time'] = cur_t
      if data['bms_temp']['balance_temp'] >= 55 and PREV_VALS['bms_temp']['balance_temp'] < 55:
        add_speak_message(f"Внимание... Температура балансиров БМС достигла... {data['bms_temp']['balance_temp']} градусов")
        PREV_VALS['balance_temp_get_last_time'] = cur_t

    for sensor_index, key in enumerate(EXTERNAL_TEMP_KEYS, start=1):
      timer_key = f"{key}_get_last_time"
      if cur_t - PREV_VALS[timer_key] > 30:
        current_val = data['bms_temp'][key]
        prev_val = PREV_VALS['bms_temp'][key]
        if current_val >= 40 and prev_val < 40:
          add_speak_message(
            f"Температура батареи датчик {sensor_index}... достигла... {current_val} градусов"
          )
          PREV_VALS[timer_key] = cur_t
        if current_val >= 55 and prev_val < 55:
          add_speak_message(
            f"Внимание... Температура батареи датчик {sensor_index}... достигла... {current_val} градусов"
          )
          PREV_VALS[timer_key] = cur_t

    # температура колес дергается поэтому после смены ждем время до следующего уведомления
    if cur_t - PREV_VALS['motor1_temp_get_last_time'] > 30:
      if data['slave']['temp_motor'] >= 60 and PREV_VALS['motor1_temp'] < 60:
        add_speak_message(f"Температура переднего колеса достигла... {int(data['slave']['temp_motor'])} градусов")
        PREV_VALS['motor1_temp_get_last_time'] = cur_t
      if data['slave']['temp_motor'] >= 80 and PREV_VALS['motor1_temp'] < 80:
        add_speak_message(f"Температура переднего колеса достигла... {int(data['slave']['temp_motor'])} градусов")
        PREV_VALS['motor1_temp_get_last_time'] = cur_t
      if data['slave']['temp_motor'] >= 90 and PREV_VALS['motor1_temp'] < 90:
        add_speak_message(f"Внимание... Температура переднего колеса достигла... {int(data['slave']['temp_motor'])} градусов")
        PREV_VALS['motor1_temp_get_last_time'] = cur_t
      if data['slave']['temp_motor'] >= 100 and PREV_VALS['motor1_temp'] < 100:
        add_speak_message(f"Опсано перегрев... Температура переднего колеса достигла... {int(data['slave']['temp_motor'])} градусов")
        PREV_VALS['motor1_temp_get_last_time'] = cur_t

    if cur_t - PREV_VALS['motor2_temp_get_last_time'] > 30:
      if data['master']['temp_motor'] >= 60 and PREV_VALS['motor2_temp'] < 60:
        add_speak_message(f"Температура заднего колеса достигла... {int(data['master']['temp_motor'])} градусов")
        PREV_VALS['motor2_temp_get_last_time'] = cur_t
      if data['master']['temp_motor'] >= 80 and PREV_VALS['motor2_temp'] < 80:
        add_speak_message(f"Температура заднего колеса достигла... {int(data['master']['temp_motor'])} градусов")
        PREV_VALS['motor2_temp_get_last_time'] = cur_t
      if data['master']['temp_motor'] >= 90 and PREV_VALS['motor2_temp'] < 90:
        add_speak_message(f"Внимание... Температура заднего колеса достигла... {int(data['master']['temp_motor'])} градусов")
        PREV_VALS['motor2_temp_get_last_time'] = cur_t
      if data['master']['temp_motor'] >= 100 and PREV_VALS['motor2_temp'] < 100:
        add_speak_message(f"Опасно перегрев... Температура заднего колеса достигла... {int(data['master']['temp_motor'])} градусов")
        PREV_VALS['motor2_temp_get_last_time'] = cur_t



    PREV_VALS['bms_temp'] = {
      'mosfet_temp': data['bms_temp']['mosfet_temp'],
      'balance_temp': data['bms_temp']['balance_temp'],
      'external_temp_0': data['bms_temp']['external_temp_0'],
      'external_temp_1': data['bms_temp']['external_temp_1'],
      'external_temp_2': data['bms_temp']['external_temp_2'],
      'external_temp_3': data['bms_temp']['external_temp_3'],
    }

    PREV_VALS['motor1_temp'] = data['slave']['temp_motor']
    PREV_VALS['motor2_temp'] = data['master']['temp_motor']

    #ВОЛЬТАЖ

    # 4. Вольтаж батареи и заряд
    boostDown = 50
    v_y = 450 + CONTENT_Y_OFFSET
    # запоминаем вольтаж без нагрузки и рекуперации
    if int(summ_current) == 0:
      data['v_without_nagruzka'] = data['bms_voltage']

    voltage_down = (data['bms_voltage'] - data['v_without_nagruzka'])
    data['voltage_down'] = voltage_down
    voltage_down_color = GREEN_COLOR
    if voltage_down < -5:
      voltage_down_color = (255, 0, 0)
    elif voltage_down < -2:
      voltage_down_color = ORANGE_COLOR

    pygame.draw.rect(screen, (200, 200, 200), (15, v_y - 27, WIDTH - 30, 54), width=2, border_radius=border_r)
    draw_text(screen, f"{data['bms_voltage']:.1f}", font_medium, (0, 100, 255), WIDTH * 0.1275, v_y)
    draw_text(screen, f"{voltage_down:.1f}", font_medium, voltage_down_color, WIDTH * 0.38, v_y)
    #draw_text_left(screen, f"{data['v_without_nagruzka']:.1f}V", font_medium, (0, 100, 255), WIDTH * 0.5, v_y)

    #battery_text = font_medium.render(f"{data['bms_voltage']:.1f}V  {data['v_without_nagruzka']:.1f}V {int(data['battery_level'])}%", True, (0, 100, 255))
    #battery_rect = battery_text.get_rect(center=(WIDTH//2 - 40, 800 + boostDown))
    #screen.blit(battery_text, battery_rect)

    # Расчёт процента заряда батареи
    if int(summ_current) == 0:
      voltages = [v for v, _ in voltage_percent_table]
      percents = [p for _, p in voltage_percent_table]

      if data['bms_voltage'] >= voltages[0]:
        data['battery_level'] = 100
      elif data['bms_voltage'] <= voltages[-1]:
        data['battery_level'] = 0
      else:
        data['battery_level'] = 0
        for i in range(len(voltages) - 1):
          v_high, v_low = voltages[i], voltages[i + 1]
          p_high, p_low = percents[i], percents[i + 1]
          if v_high >= data['bms_voltage'] >= v_low:
            # Линейная интерполяция между двумя ближайшими точками
            ratio = (data['bms_voltage'] - v_low) / (v_high - v_low)
            data['battery_level'] = int(p_low + ratio * (p_high - p_low))
            break

    battery_color = get_battery_color(data['battery_level'])
    #draw_arc(f"{int(data['battery_level'])}%", screen, (battery_rect.right + 10, 800 - 15 + boostDown), 80, average_duty, 100, (0, 0, 0))
    draw_progress_bar(screen, WIDTH * 0.73, v_y - 15, 135, 30, data['battery_level'], 100, "", battery_color)
    draw_text(screen, f"{int(data['battery_level'])}%", font_medium, (0, 0, 0), WIDTH * 0.62, v_y)

    if not BMS_LOST:
      v_y += 55
      pygame.draw.rect(screen, (200, 200, 200), (15, v_y - 20, WIDTH * 0.34, 82), width=2, border_radius=border_r)
      v_y -= 2
      weak_color = (200, 200, 200)
      if data['bad_cell_min'] < 3.3 and miganie:
        weak_color = (255, 0, 0)
      draw_text(screen, f"Low {data['bad_cell_index'] + 1}", font_small, weak_color, WIDTH * 0.19, v_y)
      v_y += 40
      draw_text(screen, f"{(data['bad_cell_min']):.3f}", font_small, (0, 0, 0), WIDTH * 0.19, v_y + 3)
      v_y -= 59
      draw_cells_block(screen, v_y)

      v_y += (54) * 2
      pygame.draw.rect(screen, (200, 200, 200), (15, v_y - 20, WIDTH * 0.34, 82), width=2, border_radius=border_r)
      draw_text(screen, f"Diff", font_small, (200, 200, 200), WIDTH * 0.19, v_y)
      v_y += 38
      unit_diff_color = get_unit_diff_color(data['unit_diff'])
      draw_text(screen, f"{(data['unit_diff']):.3f}", font_small, unit_diff_color, WIDTH * 0.19, v_y + 3)
      v_y -= 59
    else:
      v_y += 55
      pygame.draw.rect(screen, (200, 200, 200), (15, v_y - 20, WIDTH * 0.34, 42), width=2, border_radius=border_r)
      draw_text(screen, f"BMS Lost", font_small, (255, 0, 0), WIDTH * 0.19, v_y)
      
    #if not PREV_VALS['bms_lost'] and BMS_LOST:
    #  add_speak_message("Связь с бмс потеряна")
    #if PREV_VALS['bms_lost'] and not BMS_LOST:
    #  add_speak_message("Связь с бмс восстановлена")
    PREV_VALS['bms_lost'] = BMS_LOST


    # блокируем тач при движении
    if int(data['bms_current']) != 0:
      block_touch = True
    else:
      block_touch = False

    if trip_start_time is None and data['speed'] > 10:
      add_speak_message("Погнали епта бля насрал")
      trip_start_time = time.time()
      data_trip['trip_start_bettery_perc'] = data['battery_level']

    # 3. Замер времени разгона 0-60 и 0-100 км/ч
    if ready and data['speed'] > 0:
      start_time = time.time()
      measured_time = None
      measured_time_0_100 = None
      ready = False
      measuring = True
      zamer_success = False
      zamer_0_100_success = False

    elapsed_since_start = None
    if start_time is not None:
      elapsed_since_start = time.time() - start_time

    if measuring and start_time is not None:
      if not zamer_success and data['speed'] >= 60:
        measured_time = elapsed_since_start
        zamer_success = True

      if zamer_success and (not zamer_0_100_success) and data['speed'] >= 100:
        measured_time_0_100 = elapsed_since_start
        zamer_0_100_success = True
        measuring = False
        start_time = None

      if elapsed_since_start is not None and elapsed_since_start > ACCEL_MEASURE_TIMEOUT:
        measuring = False
        start_time = None
        if not zamer_success:
          measured_time = None
        measured_time_0_100 = None
        zamer_0_100_success = False

    if int(data['speed']) == 0:
      start_time = None
      measured_time = None
      measured_time_0_100 = None
      ready = True
      measuring = False
      zamer_success = False
      zamer_0_100_success = False
    prev_speed = int(data['speed'])

    if not zamer_success_prev and zamer_success:
      add_speak_message("Разгон")
      add_speak_message(f"{measured_time:.2f}".replace(".", " и ") + " секунды")
      #add_speak_message("секунды")
    zamer_success_prev = zamer_success

    # === УСКОРЕНИЕ: сбор данных каждые 300 мс во время замера ===
    try:
      def draw_acceleration_graph(surface, x, y, width, height, values):
        # рамка
        pygame.draw.rect(surface, (220, 220, 220), (x, y, width, height), width=2, border_radius=10)

        # базовая линия 0 внизу (отрицательных значений не ожидается)
        base_y = y + height - 5
        #pygame.draw.line(surface, (200, 200, 200), (x + 5, base_y), (x + width - 5, base_y), 1)

        if not values or len(values) < 2:
          return

        # динамический масштаб от 0 до max(values)
        v_max = max(values)
        scale = max(v_max, 0.5) * 1.2

        # отображаем последние значения так, чтобы умещались по ширине
        max_points = max(2, width - 10)
        start_index = 0 if len(values) <= max_points else (len(values) - max_points)
        view = values[start_index:]
        if len(view) < 2:
          return
        dx = (width - 10) / (len(view) - 1)
        pts = []
        for i, a in enumerate(view):
          a = max(0.0, a)  # защита от отрицательных значений
          norm = min(1.0, a / scale)
          py = base_y - int(norm * (height - 10))
          px = x + 5 + int(i * dx)
          pts.append((px, py))
        pygame.draw.lines(surface, (160, 0, 255), False, pts, 5)

      # начало/сброс истории при первом кадре активного замера
      if measuring and accel_last_sample_time is None:
        accel_history = []
        accel_last_sample_time = None
        accel_last_speed_mps = data['speed'] / 3.6

      # сбор значений ускорения во время активного замера
      if measuring:
        now_t = time.time()
        cur_speed_mps = data['speed'] / 3.6
        if accel_last_sample_time is None:
          accel_last_sample_time = now_t
          accel_last_speed_mps = cur_speed_mps
        elif (now_t - accel_last_sample_time) >= ACCEL_SAMPLE_PERIOD:
          dt = now_t - accel_last_sample_time
          if dt > 0:
            a = (cur_speed_mps - accel_last_speed_mps) / dt
            if a < 0:
              a = 0.0
            accel_history.append(a)
            if len(accel_history) > ACCEL_HISTORY_MAX:
              accel_history = accel_history[-ACCEL_HISTORY_MAX:]
          accel_last_sample_time = now_t
          accel_last_speed_mps = cur_speed_mps

        # # рисуем график ускорения внизу экрана
        # g_x, g_y, g_w, g_h = 15, 660, WIDTH * 0.34, 120
        # draw_acceleration_graph(screen, g_x, g_y, g_w, g_h, accel_history)
        pass
      elif measured_time is not None:
        # # после завершения замера продолжаем показывать последний график
        # g_x, g_y, g_w, g_h = 15, 660, WIDTH * 0.34, 120
        # draw_acceleration_graph(screen, g_x, g_y, g_w, g_h, accel_history)
        pass
    except:
      pass

    # подпись замера 0-60 / 0-100
    razg_boost = 260
    razg_y = 600 + razg_boost
    result_0_60_x = WIDTH // 2 - 175
    result_0_60_y = razg_y - 58
    if measuring and start_time is not None:
      current_elapsed = time.time() - start_time
      if zamer_success and measured_time is not None:
        draw_text(screen, f"0-60: {measured_time:.2f} с", font_small, (0, 0, 0), result_0_60_x, result_0_60_y)
        draw_text_center(screen, f"0-100: {current_elapsed:.2f} с", font_medium, (0, 0, 0), razg_y)
      else:
        draw_text_center(screen, f"Разгон: {current_elapsed:.2f} с", font_medium, (0, 0, 0), razg_y)
    elif measured_time_0_100 is not None and measured_time is not None:
      draw_text(screen, f"0-60: {measured_time:.2f} с", font_small, (0, 0, 0), result_0_60_x, result_0_60_y)
      draw_text_center(screen, f"0-100: {measured_time_0_100:.2f} с", font_medium, (0, 0, 0), razg_y)
    elif measured_time is not None:
      draw_text_center(screen, f"0-60: {measured_time:.2f} с", font_medium, (0, 0, 0), razg_y)
    else:
      draw_text_center(screen, "-", font_medium, (0, 0, 0), razg_y)
    # сбрасываем маркер первого кадра замера, когда замера нет
    if not measuring:
      accel_last_sample_time = None

    # 5. Одометр
    trip_y = 835 + boostDown
    trip_y_new_line = 39
    draw_text_center(screen, f"{(data['odometer'] + data['trip_odometer']):.1f} км", font_small, (150, 150, 0), 935 + boostDown)
    if trip_start_time is not None:
      # Расчёт дистанции и средней скорости поездки
      now = time.time()
      if 'trip_odometer' not in data:
        data['trip_odometer'] = 0.0
      
      if 'last_time' not in data:
        # Первый вызов — просто устанавливаем last_time, расстояние не увеличиваем
        data['last_time'] = now
      else:
        # Вычисляем прошедшее время с последнего тика
        delta_time = now - data['last_time']
        data['last_time'] = now
        # Обновляем одометр (расстояние), интегрируя скорость по времени
        data['trip_odometer'] += data['speed'] * (delta_time / 3600.0)

      elapsed_time = time.time() - trip_time_start
      data['trip_speed_sum'] += data['speed']
      data['trip_tick'] += 1
      data['trip_avg_speed'] = data['trip_speed_sum'] / data['trip_tick']

      # поездка
      draw_text_right(screen, f"{data['trip_odometer']:.1f} км", font_small, (0, 0, 0), WIDTH - 10, trip_y)
      trip_y += trip_y_new_line
      draw_text_right(screen, f"{data['trip_avg_speed']:.1f} км/ч", font_small, (0, 0, 0), WIDTH - 10, trip_y)

      trip_time = time.time() - trip_start_time
      minutes = int(trip_time // 60)
      seconds = int(trip_time % 60)

      ############ ОЗВУЧКА ######### Озвучиваем статистику поездки каждую минуту ###########
      if PREV_VALS['trip_mins'] != minutes:
        PREV_VALS['trip_mins'] = minutes
        add_speak_message(f"Время в пути {minutes} минут")
        if minutes == 15:
          add_speak_message(f"Опять еле едем из-за долбаебов на дороге")
        add_speak_message(f"Температура моторов... {int(data_trip['motor1_max_temp'])}... и {int(data_trip['motor2_max_temp'])} градусов")
        #add_speak_message(f"Средняя скорость" + f" {data['trip_avg_speed']:.1f}".replace(".", " и ") + " километров в час")
        add_speak_message(f"Заряд {data['battery_level']} процентов")
        add_speak_message(f"Слабейший ряд... {data['bad_cell_min_peak_index'] + 1}... минимальный заряд... " + f"{data['bad_cell_min_peak']:.2f}".replace(".", " и ") + "... вольт")
        
      trip_y += trip_y_new_line
      draw_text_right(screen, f"{minutes:02d}:{seconds:02d}", font_small, (0, 0, 0), WIDTH - 10, trip_y)
      trip_y -= trip_y_new_line * 2


    # Отображение даты и времени
    now = datetime.datetime.now()
    weekdays = ['Пн', 'Вт', 'Ср', 'Чт', 'Пт', 'Сб', 'Вс']
    months = ['янв', 'фев', 'мар', 'апр', 'мая', 'июн', 'июл', 'авг', 'сен', 'окт', 'ноя', 'дек']
    weekdays_full = ['Понедельник', 'Вторник', 'Среда', 'Четверг', 'Пятница', 'Суббота', 'Воскресенье']
    months_full = ['января', 'февраля', 'марта', 'апреля', 'мая', 'июня', 'июля', 'августа', 'сентября', 'октября', 'ноября', 'декабря']
    draw_text_left(screen, f"{weekdays[now.weekday()]}", font_small, (0, 0, 0), 10, trip_y)
    trip_y += trip_y_new_line
    draw_text_left(screen, f"{now.day} {months[now.month-1]}", font_small, (0, 0, 0), 10, trip_y)
    trip_y += trip_y_new_line
    draw_text_left(screen, f"{now.hour:02d}:{now.minute:02d}", font_small, (0, 0, 0), 10, trip_y)
    trip_end_datetime_str = f"{weekdays[now.weekday()]} {now.day} {months[now.month-1]} {now.hour:02d}:{now.minute:02d}"
    #trip_end_datetime_str_full = f"{weekdays_full[now.weekday()]} {now.day} {months_full[now.month-1]} {now.hour:02d} часов {now.minute:02d} минут"
    trip_end_datetime_str_full = f"{now.hour:02d} часов {now.minute:02d} минут"
  

    button_size = 37
    button_spacing = 15
    btn_x = 12
    btn_y = 12

    # Кнопка выключения системы
    button_rect = pygame.Rect(btn_x, btn_y, button_size, button_size)
    pygame.draw.rect(screen, (255, 95, 87), button_rect, border_radius=20)
    button_text = font_small.render("", True, (0, 0, 0))
    screen.blit(button_text, button_text.get_rect(center=button_rect.center))

    mouse = pygame.mouse.get_pos()
    click = pygame.mouse.get_pressed()
    if button_rect.collidepoint(mouse) and click[0] and (not block_touch or not IS_RASPBERY):
      if trip_start_time is None:
        trip_start_time = time.time()

      full_off = True
      PAGE_NAME = "TRIP_STAT"
      trip_time = time.time() - trip_start_time
      minutes = int(trip_time // 60)
      seconds = int(trip_time % 60)
      data['trip_time'] = f"{minutes:02d}:{seconds:02d}"
      timer_power_off = time.time()
      SaveData()

    # Обновляем данные за поездку
    if data_trip['max_speed'] < data['speed']:
      data_trip['max_speed'] = data['speed']
    if data_trip['max_power'] < data['power']:
      data_trip['max_power'] = data['power']

    if zamer_success and data_trip['best_time_0_60'] > measured_time:
      data_trip['best_time_0_60'] = measured_time

    if data_trip['max_voltage_down'] > data['voltage_down']:
      data_trip['max_voltage_down'] = data['voltage_down']

    if data_trip['min_cell_v'] > data['bad_cell_min']:
      data_trip['min_cell_v'] = data['bad_cell_min']
      data_trip['min_cell_v_index'] = data['bad_cell_index']

    if data_trip['max_unit_diff'] < data['unit_diff']:
      data_trip['max_unit_diff'] = data['unit_diff']
    
    if data_trip['motor1_max_temp'] < data['slave']['temp_motor']:
      data_trip['motor1_max_temp'] = data['slave']['temp_motor']
    if data_trip['motor2_max_temp'] < data['master']['temp_motor']:
      data_trip['motor2_max_temp'] = data['master']['temp_motor']

    # Кнопка выключения программы
    btn_x += button_size + button_spacing
    button_rect = pygame.Rect(btn_x, btn_y, button_size, button_size)
    pygame.draw.rect(screen, (255, 188, 46), button_rect, border_radius=20)
    button_text = font_small.render("", True, (0, 0, 0))
    screen.blit(button_text, button_text.get_rect(center=button_rect.center))

    mouse = pygame.mouse.get_pos()
    click = pygame.mouse.get_pressed()
    if button_rect.collidepoint(mouse) and click[0] and (not block_touch or not IS_RASPBERY):
      if trip_start_time is None:
        trip_start_time = time.time()

      full_off = False
      PAGE_NAME = "TRIP_STAT"
      trip_time = time.time() - trip_start_time
      minutes = int(trip_time // 60)
      seconds = int(trip_time % 60)
      data['trip_time'] = f"{minutes:02d}:{seconds:02d}"
      timer_power_off = time.time()
      SaveData()

    # Кнопка начала записи
    # Кнопка начала записи
    btn_x += button_size + button_spacing
    record_rect = pygame.Rect(btn_x, btn_y, button_size, button_size)
    if can_start_record:
      pygame.draw.rect(screen, (40, 200, 64), record_rect, border_radius=20)
      button_text = font_small.render("", True, (0, 0, 0))
      screen.blit(button_text, button_text.get_rect(center=record_rect.center))

      mouse = pygame.mouse.get_pos()
      click = pygame.mouse.get_pressed()
      if record_rect.collidepoint(mouse) and click[0] and IS_RASPBERY:
        # Старт записи
        unique_suffix = uuid.uuid4().hex[:8]
        filename = f"trip_{unique_suffix}_{video_record_counter}.mp4"
        try:
          recorder_proc = subprocess.Popen(["wf-recorder", "-f", filename])
        except Exception as exc:
          log_exception("Старт записи", exc)
        else:
          video_record_counter += 1
          SaveData()
          can_start_record = False
          print(f">>> Запись началась: {filename}")

    # Кнопка блокировки
    btn_x += button_size + button_spacing
    lock_rect = pygame.Rect(btn_x, btn_y, button_size, button_size)
    lock_color = (60, 140, 230) if lock_active else (90, 170, 255)
    pygame.draw.rect(screen, lock_color, lock_rect, border_radius=20)

    mouse = pygame.mouse.get_pos()
    click = pygame.mouse.get_pressed()
    if (not lock_active) and lock_rect.collidepoint(mouse) and click[0] and (not block_touch or not IS_RASPBERY):
      activate_lock()

    # Кнопка переключения темы между зелёной и синей, но ниже
    theme_button_radius = button_size // 2
    same_row_y = btn_y + theme_button_radius
    # Размещаем тему справа от синей кнопки на том же уровне по высоте
    theme_button_center = (lock_rect.right + button_spacing + theme_button_radius, same_row_y)
    theme_button_color = (0, 0, 0) if not is_dark_theme() else (255, 255, 255)
    theme_button_border = (200, 200, 200) if is_dark_theme() else (60, 60, 60)
    pygame.draw.circle(screen, theme_button_color, theme_button_center, theme_button_radius)
    pygame.draw.circle(screen, theme_button_border, theme_button_center, theme_button_radius, width=2)

    mouse = pygame.mouse.get_pos()
    click = pygame.mouse.get_pressed()
    inside_theme_button = math.hypot(
      mouse[0] - theme_button_center[0],
      mouse[1] - theme_button_center[1]
    ) <= theme_button_radius
    theme_button_pressed = inside_theme_button and click[0] and (not block_touch or not IS_RASPBERY)
    if theme_button_pressed and not theme_toggle_was_pressed:
      toggle_theme()
    theme_toggle_was_pressed = theme_button_pressed

    # Кнопка зеркал в верхней части экрана
    mirror_snapshot = mirror_controller.get_snapshot()
    mirror_top_width = 90
    mirror_top_height = 40
    eco_rect = pygame.Rect(WIDTH - mirror_top_width - 12, 12, mirror_top_width, mirror_top_height)
    mirror_top_rect = pygame.Rect(eco_rect.left - mirror_top_width - 10, 12, mirror_top_width, mirror_top_height)
    mirror_top_color = (90, 170, 255) if mirror_snapshot['connected'] else (210, 210, 210)
    pygame.draw.rect(screen, mirror_top_color, mirror_top_rect, border_radius=15)
    mirror_top_label = render_force_color(font_small, "З", (0, 0, 0))
    screen.blit(mirror_top_label, mirror_top_label.get_rect(center=mirror_top_rect.center))

    status_text = "Э" if eco_mode else "Н"
    status_color = (200, 200, 200)
    pygame.draw.rect(screen, status_color, eco_rect, border_radius=20)
    status_label = render_force_color(font_small, status_text, (0, 0, 0))
    screen.blit(status_label, status_label.get_rect(center=eco_rect.center))

    mouse = pygame.mouse.get_pos()
    click = pygame.mouse.get_pressed()
    status_pressed = eco_rect.collidepoint(mouse) and click[0] and (not block_touch or not IS_RASPBERY)
    if status_pressed and not eco_toggle_was_pressed:
      set_eco_mode(not eco_mode)
    eco_toggle_was_pressed = status_pressed

    if mirror_top_rect.collidepoint(mouse) and click[0] and (not block_touch or not IS_RASPBERY):
      mirror_controller.request_update()
      PAGE_NAME = PAGE_MIRROR

    draw_gps_overlay(screen)

  elif PAGE_NAME == PAGE_MIRROR:
    mirror_snapshot = mirror_controller.get_snapshot()
    touch_allowed = (not block_touch or not IS_RASPBERY)

    mouse = pygame.mouse.get_pos()
    click = pygame.mouse.get_pressed()

    back_rect = pygame.Rect(12, 12, 160, 60)
    pygame.draw.rect(screen, (220, 220, 220), back_rect, border_radius=20)
    back_label = font_small.render("Назад", True, (0, 0, 0))
    screen.blit(back_label, back_label.get_rect(center=back_rect.center))
    if back_rect.collidepoint(mouse) and click[0]:
      PAGE_NAME = "SPEEDOMETER"
      wait_mouse_release()
      continue

    draw_text_center(screen, "Регулировка зеркал", font_small, (0, 0, 0), 90)

    status_color = (0, 110, 0) if mirror_snapshot['connected'] else (160, 60, 60)
    status_text = mirror_snapshot['status']
    if len(status_text) > 40:
      status_text = status_text[:37] + "..."
    draw_text_center(screen, status_text, font_tick, status_color, 135)

    range_text = f"Диапазон: {MIRROR_MIN_ANGLE}° - {MIRROR_MAX_ANGLE}°"
    draw_text_center(screen, range_text, font_tick, (90, 90, 90), 170)

    poses = mirror_snapshot.get('poses', {})
    folded_pose = poses.get('folded', {})
    unfolded_pose = poses.get('unfolded', {})
    def _pose_val(value):
      return "--" if value is None else str(value)

    folded_text = f"Сложено: Л {_pose_val(folded_pose.get('left'))}°  П {_pose_val(folded_pose.get('right'))}°"
    unfolded_text = f"Разложено: Л {_pose_val(unfolded_pose.get('left'))}°  П {_pose_val(unfolded_pose.get('right'))}°"
    draw_text_center(screen, folded_text, font_tick, (70, 70, 70), 200)
    draw_text_center(screen, unfolded_text, font_tick, (70, 70, 70), 225)

    last_message = mirror_snapshot.get('last_message')
    if last_message:
      msg = last_message
      if len(msg) > 42:
        msg = msg[:39] + "..."
      draw_text_center(screen, msg, font_tick, (120, 120, 120), 255)

    def draw_mirror_section(label_text, side, center_x):
      panel_rect = pygame.Rect(int(center_x - 150), 240, 300, 360)
      pygame.draw.rect(screen, (240, 240, 240), panel_rect, border_radius=25)

      title = font_small.render(label_text, True, (0, 0, 0))
      screen.blit(title, title.get_rect(center=(panel_rect.centerx, panel_rect.top + 35)))

      target = mirror_snapshot['target'].get(side)
      actual = mirror_snapshot['actual'].get(side)

      target_text = f"Цель: {target}°" if target is not None else "Цель: --°"
      actual_text = f"Факт: {actual}°" if actual is not None else "Факт: --°"

      target_label = font_tick.render(target_text, True, (0, 0, 0))
      actual_label = font_tick.render(actual_text, True, (0, 0, 0))
      screen.blit(target_label, target_label.get_rect(center=(panel_rect.centerx, panel_rect.top + 95)))
      screen.blit(actual_label, actual_label.get_rect(center=(panel_rect.centerx, panel_rect.top + 135)))

      buttons = [
        (pygame.Rect(panel_rect.left + 15, panel_rect.top + 175, 120, 70), f"-{MIRROR_COARSE_STEP}°", -MIRROR_COARSE_STEP),
        (pygame.Rect(panel_rect.right - 135, panel_rect.top + 175, 120, 70), f"+{MIRROR_COARSE_STEP}°", MIRROR_COARSE_STEP),
        (pygame.Rect(panel_rect.left + 15, panel_rect.top + 265, 120, 70), f"-{MIRROR_FINE_STEP}°", -MIRROR_FINE_STEP),
        (pygame.Rect(panel_rect.right - 135, panel_rect.top + 265, 120, 70), f"+{MIRROR_FINE_STEP}°", MIRROR_FINE_STEP),
      ]

      for rect, text, delta in buttons:
        btn_color = (90, 170, 255) if touch_allowed else (210, 210, 210)
        pygame.draw.rect(screen, btn_color, rect, border_radius=20)
        btn_label = font_tick.render(text, True, (0, 0, 0))
        screen.blit(btn_label, btn_label.get_rect(center=rect.center))
        if touch_allowed and rect.collidepoint(mouse) and click[0]:
          mirror_controller.adjust_target(side, delta)
          mirror_controller.request_update()

    draw_mirror_section("Левое зеркало", 'left', WIDTH * 0.28)
    draw_mirror_section("Правое зеркало", 'right', WIDTH * 0.72)

    refresh_rect = pygame.Rect(WIDTH * 0.5 - 130, HEIGHT - 320, 260, 70)
    refresh_color = (200, 200, 200) if touch_allowed else (230, 230, 230)
    pygame.draw.rect(screen, refresh_color, refresh_rect, border_radius=25)
    refresh_label = font_small.render("Обновить", True, (0, 0, 0))
    screen.blit(refresh_label, refresh_label.get_rect(center=refresh_rect.center))
    if touch_allowed and refresh_rect.collidepoint(mouse) and click[0]:
      mirror_controller.request_update()

    action_y = HEIGHT - 220
    btn_width = 230
    btn_height = 70
    btn_spacing_x = 40

    fold_rect = pygame.Rect(int(WIDTH * 0.5 - btn_width - btn_spacing_x / 2), action_y, btn_width, btn_height)
    unfold_rect = pygame.Rect(int(WIDTH * 0.5 + btn_spacing_x / 2), action_y, btn_width, btn_height)
    save_fold_rect = pygame.Rect(fold_rect.x, action_y + btn_height + 20, btn_width, btn_height)
    save_unfold_rect = pygame.Rect(unfold_rect.x, action_y + btn_height + 20, btn_width, btn_height)

    def draw_button(rect, caption, enabled=True):
      color_active = (90, 170, 255)
      color_disabled = (210, 210, 210)
      color = color_active if enabled else color_disabled
      pygame.draw.rect(screen, color, rect, border_radius=25)
      label = font_tick.render(caption, True, (0, 0, 0))
      screen.blit(label, label.get_rect(center=rect.center))

    draw_button(fold_rect, "Сложить", touch_allowed)
    draw_button(unfold_rect, "Разложить", touch_allowed)
    draw_button(save_fold_rect, "Запомнить сложено", touch_allowed)
    draw_button(save_unfold_rect, "Запомнить разложено", touch_allowed)

    if touch_allowed and fold_rect.collidepoint(mouse) and click[0]:
      mirror_controller.apply_pose('folded')
      mirror_controller.request_update()
    if touch_allowed and unfold_rect.collidepoint(mouse) and click[0]:
      mirror_controller.apply_pose('unfolded')
      mirror_controller.request_update()
    if touch_allowed and save_fold_rect.collidepoint(mouse) and click[0]:
      mirror_controller.save_pose('folded')
      mirror_controller.request_update()
    if touch_allowed and save_unfold_rect.collidepoint(mouse) and click[0]:
      mirror_controller.save_pose('unfolded')
      mirror_controller.request_update()

    if not touch_allowed and IS_RASPBERY:
      draw_text_center(screen, "Остановись чтобы управлять зеркалами", font_tick, (180, 60, 60), HEIGHT - 40)

  #################### PAGE TRIP_STAT ###########################
  # добавить температуру моторов
  elif PAGE_NAME == "TRIP_STAT":
    try:
      y_trip_start = 80
      draw_text_center(screen, "Статистика поездки:", font_small, GRAY, y_trip_start)
      y_trip_start += 60
      y_trip_shift = 40
      draw_text_left(screen, "Время в пути ", font_small, GRAY, 10, y_trip_start - 2)
      draw_text_right(screen, data['trip_time'], font_small, (0, 0, 0), WIDTH - 20, y_trip_start)
      y_trip_start += y_trip_shift
      draw_text_left(screen, "Приехал ", font_small, GRAY, 10, y_trip_start - 2)
      draw_text_right(screen, trip_end_datetime_str, font_small, (0, 0, 0), WIDTH - 20, y_trip_start)
      y_trip_start += y_trip_shift
      draw_text_left(screen, "Макс. t° моторов ", font_small, GRAY, 10, y_trip_start - 2)
      draw_text_right(screen, f"{int(data_trip['motor1_max_temp'])}° {int(data_trip['motor2_max_temp'])}°", font_small, (0, 0, 0), WIDTH - 20, y_trip_start)
      y_trip_start += y_trip_shift
      draw_text_left(screen, "Расстояние ", font_small, GRAY, 10, y_trip_start - 2)
      draw_text_right(screen, f"{data['trip_odometer']:.1f} км", font_small, (0, 0, 0), WIDTH - 20, y_trip_start)
      y_trip_start += y_trip_shift
      draw_text_left(screen, "Средняя скорость ", font_small, GRAY, 10, y_trip_start - 2)
      draw_text_right(screen, f"{data['trip_avg_speed']:.1f} км/ч", font_small, (0, 0, 0), WIDTH - 20, y_trip_start)
      y_trip_start += y_trip_shift
      draw_text_left(screen, "Макс. скорость ", font_small, GRAY, 10, y_trip_start - 2)
      draw_text_right(screen, f"{int(data_trip['max_speed'])} км/ч", font_small, (0, 0, 0), WIDTH - 20, y_trip_start)
      y_trip_start += y_trip_shift
      draw_text_left(screen, "Лучшее 0-60 ", font_small, GRAY, 10, y_trip_start - 2)
      time_0_60 = f"{data_trip['best_time_0_60']:.2f} с"
      if int(data_trip['best_time_0_60']) == 100:
        time_0_60 = "-"
      draw_text_right(screen, time_0_60, font_small, (0, 0, 0), WIDTH - 20, y_trip_start)
      y_trip_start += y_trip_shift
      draw_text_left(screen, "Макс. мощность ", font_small, GRAY, 10, y_trip_start - 2)
      draw_text_right(screen, f"{int(data_trip['max_power'])} Вт", font_small, (0, 0, 0), WIDTH - 20, y_trip_start)
      y_trip_start += y_trip_shift
      draw_text_left(screen, "Потрачено заряда ", font_small, GRAY, 10, y_trip_start - 2)
      draw_text_right(screen, f"{int(data_trip['trip_start_bettery_perc'] - data['battery_level'])} %", font_small, (0, 0, 0), WIDTH - 20, y_trip_start)
      y_trip_start += y_trip_shift
      draw_text_left(screen, "Макс. просадка ", font_small, GRAY, 10, y_trip_start - 2)
      draw_text_right(screen, f"{data_trip['max_voltage_down']:.1f}V", font_small, (0, 0, 0), WIDTH - 20, y_trip_start)
      y_trip_start += y_trip_shift
      draw_text_left(screen, "Слабейший ряд ", font_small, GRAY, 10, y_trip_start - 2)
      draw_text_right(screen, f"{data_trip['min_cell_v_index'] + 1}", font_small, (0, 0, 0), WIDTH - 20, y_trip_start)
      y_trip_start += y_trip_shift
      draw_text_left(screen, "Мин. V в ряду ", font_small, GRAY, 10, y_trip_start - 2)
      draw_text_right(screen, f"{data_trip['min_cell_v']:.3f}V", font_small, (0, 0, 0), WIDTH - 20, y_trip_start)
      y_trip_start += y_trip_shift
      draw_text_left(screen, "Макс. разбаланс ", font_small, GRAY, 10, y_trip_start - 2)
      draw_text_right(screen, f"{data_trip['max_unit_diff']:.3f}V", font_small, (0, 0, 0), WIDTH - 20, y_trip_start)

      sec_to_exit = 20

      if timer_power_off is not None:
        remaining = sec_to_exit - (time.time() - timer_power_off)
        if remaining < 0:
          remaining = 0
        timer_label = "До выключения" if full_off else "До закрытия"
        timer_off_t = f"{timer_label}: {remaining:.0f} сек"
        draw_text(screen, timer_off_t, font_small, (0, 0, 0), WIDTH * 0.5, 730)

      # Кнопка раннего выключения системы
      button_rect = pygame.Rect(WIDTH * 0.5 - 210, 820, 410, 60)
      pygame.draw.rect(screen, (200, 200, 200), button_rect, border_radius=15)
      button_text = font_small.render("Выключить сейчас", True, (0, 0, 0))
      screen.blit(button_text, button_text.get_rect(center=button_rect.center))

      mouse = pygame.mouse.get_pos()
      click = pygame.mouse.get_pressed()
      if button_rect.collidepoint(mouse) and click[0] and (not block_touch or not IS_RASPBERY):
        sec_to_exit = 0

      should_exit_now = False
      if timer_power_off is not None:
        if sec_to_exit == 0:
          should_exit_now = True
        elif (time.time() - timer_power_off) > sec_to_exit:
          should_exit_now = True

      if should_exit_now:
        perform_exit("shutdown timer")
        break
    except Exception as exc:
      log_exception("TRIP_STAT", exc)
      if timer_power_off is not None:
        perform_exit("trip_stat exception")
        break
      draw_text_center(screen, "Ошибка статистики, см. лог", font_small, RED_COLOR, HEIGHT // 2)
      pygame.display.flip()
      clock.tick(5)
      continue


  pygame.display.flip()
  clock.tick(30)



fold_mirrors_on_exit()
time.sleep(0.4)
pygame.quit()
