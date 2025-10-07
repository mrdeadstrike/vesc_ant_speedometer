import datetime
import signal
import socket
import subprocess
import numpy as np
import pygame
import threading
import serial
import struct
import math
import time

import urllib


PACKET_INDEX_FOR_VESC = 47#4
COMM_FORWARD_CAN = 34
COMM_SET_MCCONF_TEMP = 48
COMM_SET_MCCONF_TEMP_SETUP = 49

ECO_SPEED_LIMIT_KMH = 49.0
DEFAULT_NORMAL_ERPM = 100000.0
ECO_POWER_DISPLAY_LIMIT = 2400

SLAVE_CAN_ID = 15

CELL_COUNT = 20

GREEN_COLOR = (0, 160, 0)
GREEN_LIGHT = (0, 210, 0)
ORANGE_COLOR = (230, 135, 0)
RED_COLOR = (255, 0, 0)
GRAY = (180, 180, 180)

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
}

EXTERNAL_TEMP_KEYS = [
  'external_temp_0',
  'external_temp_1',
  'external_temp_2',
  'external_temp_3',
]

import platform
import os

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

if IS_RASPBERY:
  threading.Thread(target=increase_tws_volume, daemon=True).start()


# Данные контроллеров
trip_start_odometer = None
trip_distance_km = 0.0
trip_avg_speed = 0.0
trip_time_start = time.time()

saved_odometer = 0.0
try:
  with open("mainData.txt", "r") as f:
    saved_odometer = float(f.read().strip())
    print(f"Загружен одометр: {saved_odometer:.1f} км")
except:
  saved_odometer = 0.0

data = {
  'speed': 0.0,
  'master': {'motor_current': 0, 'battery_current': 0, 'duty': 0, 'temp': 0, 'temp_motor': 0},
  'slave': {'motor_current': 0, 'battery_current': 0, 'duty': 0, 'temp': 0, 'temp_motor': 0},
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


############## VOICE RECOGNITION ###############
import subprocess
import numpy as np
import queue
import threading
import json
from vosk import Model, KaldiRecognizer

vesc_command_queue = queue.Queue()
eco_mode = False
current_speed_limit_kmh = None
eco_toggle_was_pressed = False
NORMAL_SPEED_LIMIT_KMH = None

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
  global MESSAGES_TO_SPEAK
  MESSAGES_TO_SPEAK.append(text)
  #print(MESSAGES_TO_SPEAK)

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
wheel_diameter_m = 0.28  # 280 мм = 0.28 м 11 дюймов
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

def build_mcconf_temp_payload(max_speed_mps, min_speed_mps=None,
                              is_setup=True, store=False,
                              forward_can=True, divide_by_controllers=False,
                              ack=False):
  if min_speed_mps is None:
    min_speed_mps = -max_speed_mps if max_speed_mps != 0 else 0.0

  payload = bytearray()
  payload.append(COMM_SET_MCCONF_TEMP_SETUP if is_setup else COMM_SET_MCCONF_TEMP)
  payload.append(1 if store else 0)
  payload.append(1 if forward_can else 0)
  payload.append(1 if ack else 0)
  payload.append(1 if divide_by_controllers else 0)

  values = (
    1.0,           # current_min_scale
    1.0,           # current_max_scale
    float(min_speed_mps),
    float(max_speed_mps),
    0.05,          # duty_min (default value)
    0.95,          # duty_max (default value)
    -200000.0,     # watt_min
    200000.0       # watt_max
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

def queue_speed_limit_command(max_speed_kmh, force=False):
  global current_speed_limit_kmh
  if not force and max_speed_kmh == current_speed_limit_kmh:
    return

  if max_speed_kmh is None:
    max_speed_kmh = get_normal_speed_limit_kmh()

  max_speed_mps = max_speed_kmh / 3.6
  payload = build_mcconf_temp_payload(max_speed_mps)
  enqueue_vesc_command(payload)
  current_speed_limit_kmh = max_speed_kmh

def get_display_power():
  power = data.get('power', 0)
  if eco_mode and power > ECO_POWER_DISPLAY_LIMIT:
    return ECO_POWER_DISPLAY_LIMIT
  return power

def process_pending_vesc_commands(ser):
  try:
    while True:
      packet = vesc_command_queue.get_nowait()
      ser.write(packet)
  except queue.Empty:
    pass

def set_eco_mode(enabled):
  global eco_mode
  global block_touch
  if block_touch:
    return
  if enabled == eco_mode:
    return

  if enabled:
    queue_speed_limit_command(ECO_SPEED_LIMIT_KMH)
    add_speak_message("Эко режим активирован")
  else:
    queue_speed_limit_command(None)
    add_speak_message("Нормальный режим")

  eco_mode = enabled

def requeue_current_speed_limit():
  if current_speed_limit_kmh is None:
    return
  queue_speed_limit_command(current_speed_limit_kmh, force=True)

def parse_vesc_payload(payload, forwarded=False):
  try:
    if forwarded:
      if len(payload) <= 2:
        return None
      payload = payload[2:]
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


######### CONTROLLER READ ##########
def read_serial(ser):
  packet_master = pack_comm_get_values()
  packet_slave = pack_comm_get_values(can_id=15)

  requeue_current_speed_limit()

  while True:
    #GET_INFO
    ser.write(packet_master)
    header = ser.read(2)
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
    ser.write(packet)
    header = ser.read(2)
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


def read_сontrollers(
                #port_name='/dev/tty.usbmodem3041', #MAC
                port_name='/dev/ttyACM0', #Raspbery PI
                baudrate=115200):
  if IS_RASPBERY or not IS_MAC:
    while True:
      try:
        ser = serial.Serial(port_name, baudrate, timeout=0.1)
      except Exception as e:
        try:
          ser.close()
          #add_speak_message("Отладка БМС 1")
        except:
          #add_speak_message("Отладка БМС 2")
          ser = None
        print("Не удалось открыть порт:", e)
        time.sleep(2)
        continue

      try:
        read_serial(ser)
      except:
        try:
          ser.close()
          #add_speak_message("Ошибка БМС 1")
        except:
          ser = None
          #add_speak_message("Ошибка БМС 2")
        time.sleep(2)
      time.sleep(2)

threading.Thread(target=read_сontrollers, daemon=True).start()

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


def read_bms_data(ser):
  while True:
    ser.write(b'\x5A\x5A\x00\x00\x00\x00')
    bms_data = ser.read(140)

    if len(bms_data) != 140 or not bms_data.startswith(b'\xAA\x55\xAA\xFF'):
      print("❌ Некорректный ответ от BMS")
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
                port_name='/dev/ttyUSB0', #Ubuntu
                #port_name='/dev/ttyUSB0', #Raspbery PI
                baudrate=19200):
  global BMS_LOST
  if IS_RASPBERY or not IS_MAC:
    while True:
      try:
        ser = serial.Serial(port_name, baudrate, timeout=0.1)
        print("bms port open")
        BMS_LOST = False
      except Exception as e:
        BMS_LOST = True
        try:
          ser.close()
          #add_speak_message("Отладка БМС 1")
        except:
          #add_speak_message("Отладка БМС 2")
          ser = None
        print("Не удалось открыть порт:", e)
        time.sleep(2)
        continue

      try:
        read_bms_data(ser)
      except:
        try:
          ser.close()
          #add_speak_message("Ошибка БМС 1")
        except:
          ser = None
          #add_speak_message("Ошибка БМС 2")
        BMS_LOST = True
        print("bms lost")
        time.sleep(2)
      time.sleep(2)



threading.Thread(target=read_bms, daemon=True).start()

######## INTERFACE ###########
import pygame
import math
import time

pygame.init()

WIDTH, HEIGHT = 600, 1010
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption('VESC ANT Speedometer')

font_large = pygame.font.SysFont('Arial', 150)
font_medium = pygame.font.SysFont('Arial', 50, True)
font_small = pygame.font.SysFont('Arial', 40, True)
font_tick = pygame.font.SysFont('Arial', 30, True)
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
  if fill_width > 0:
    pygame.draw.rect(surface, color, (x, y, fill_width, height), border_radius=10)
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
      f.write(f"{data['odometer'] + data['trip_odometer']:.1f}")
  except Exception as e:
    print("Ошибка сохранения одометра:", e)

def SetDebugValues():
  #DEBUG_VISUAL_TEST
  changeV = time.time() % 5 / 5

  data['master']['motor_current'] = 200 * changeV
  data['slave']['motor_current'] = 180 * (1 - changeV)
  data['bms_current'] = 50 * changeV
  data['speed'] = 70 * changeV
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
ready = True
measuring = False
trip_start_time = None
timer_power_off = None
block_touch = False
prev_speed = 0
zamer_success = False
zamer_success_prev = False

can_start_record = True

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
if IS_RASPBERY:
  screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)

while running:
  for event in pygame.event.get():
    if event.type == pygame.QUIT:
      running = False

  screen.fill((254, 254, 254))

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

    draw_speed_arc(screen, (WIDTH//2, 180 + up_gap), 150, int(data['speed']), 100, up_gap)

    # 2. Показатели контроллеров мастер и слейв
    y_offset = 360
    spacing_x = 250

    summ_current = data['slave']['motor_current'] + data['master']['motor_current']
    #if summ_current > 200:
    #  summ_current = 200
    
    up_gap += 20

    stats_bar_start = 30 + up_gap
    stats_bar_step = 85
    stats_bar_width = 100
    stats_bar_height = 15
    left_bar_x = 10
    right_bar_x = WIDTH - stats_bar_width - 10

    slave_motor_current = int(abs(data['slave']['motor_current']))
    slave_battery_current = int(abs(data['slave']['battery_current']))
    slave_duty = int(abs(data['slave']['duty']))

    master_motor_current = int(abs(data['master']['motor_current']))
    master_battery_current = int(abs(data['master']['battery_current']))
    master_duty = int(abs(data['master']['duty']))

    draw_progress_bar(screen, left_bar_x, stats_bar_start, stats_bar_width, stats_bar_height, slave_motor_current, 200, f"{slave_motor_current}A", (255, 0, 0))
    draw_progress_bar(screen, left_bar_x, stats_bar_start + stats_bar_step, stats_bar_width, stats_bar_height, slave_battery_current, 80, f"{slave_battery_current}A", (0, 0, 255))
    draw_progress_bar(screen, left_bar_x, stats_bar_start + stats_bar_step * 2, stats_bar_width, stats_bar_height, slave_duty, 100, f"{slave_duty}", (0, 0, 0))

    draw_progress_bar(screen, right_bar_x, stats_bar_start, stats_bar_width, stats_bar_height, master_motor_current, 200, f"{master_motor_current}A", (255, 0, 0))
    draw_progress_bar(screen, right_bar_x, stats_bar_start + stats_bar_step, stats_bar_width, stats_bar_height, master_battery_current, 80, f"{master_battery_current}A", (0, 0, 255))
    draw_progress_bar(screen, right_bar_x, stats_bar_start + stats_bar_step * 2, stats_bar_width, stats_bar_height, master_duty, 100, f"{master_duty}", (0, 0, 0))

    draw_text_center(screen, str(get_display_power()) + " Вт", font_small, (0, 0, 0), 295)

    # Когда ослабление магнитного поля активно рисуем рамку
    #if average_duty >= 85:
    #  pygame.draw.rect(screen, (255, 0, 0), (0, 0, WIDTH, HEIGHT), width=12, border_radius=0)

    #Температура всего
    temp_y = 345
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
    v_y = 450
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

    # 3. Замер времени разгона 0-60 км/ч
    if ready and data['speed'] > 0:
      start_time = time.time()
      ready = False
      measuring = True

    if measuring and data['speed'] >= 60:
      measured_time = time.time() - start_time
      measuring = False
      zamer_success = True

    if int(data['speed']) == 0 or (not zamer_success and (start_time is not None and (time.time() - start_time) > 25)):
      start_time = None
      measured_time = None
      if zamer_success or int(data['speed']) == 0:
        ready = True
      measuring = False
      zamer_success = False
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

        # рисуем график ускорения внизу экрана
        g_x, g_y, g_w, g_h = 15, 660, WIDTH * 0.34, 120
        draw_acceleration_graph(screen, g_x, g_y, g_w, g_h, accel_history)
      elif measured_time is not None:
        # после завершения замера продолжаем показывать последний график
        g_x, g_y, g_w, g_h = 15, 660, WIDTH * 0.34, 120
        draw_acceleration_graph(screen, g_x, g_y, g_w, g_h, accel_history)
    except:
      pass

    # подпись замера 0-60
    razg_boost = 260
    if measuring:
      current_elapsed = time.time() - start_time
      draw_text_center(screen, f"Разгон: {current_elapsed:.2f} сек", font_medium, (0, 0, 0), 600 + razg_boost)
    elif measured_time is not None:
      draw_text_center(screen, f"0-60: {measured_time:.2f} сек", font_medium, (0, 0, 0), 600 + razg_boost)
    else:
      draw_text_center(screen, "-", font_medium, (0, 0, 0), 600 + razg_boost)
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
  

    # Кнопка выключения системы
    button_rect = pygame.Rect(12, 12, 40, 40)
    pygame.draw.rect(screen, (255, 95, 87), button_rect, border_radius=25)
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
      if timer_power_off is None:
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
    button_rect = pygame.Rect(72, 12, 40, 40)
    pygame.draw.rect(screen, (255, 188, 46), button_rect, border_radius=25)
    button_text = font_small.render("", True, (0, 0, 0))
    screen.blit(button_text, button_text.get_rect(center=button_rect.center))

    mouse = pygame.mouse.get_pos()
    click = pygame.mouse.get_pressed()
    if button_rect.collidepoint(mouse) and click[0] and (not block_touch or not IS_RASPBERY):
      if trip_start_time is None:
        trip_start_time = time.time()
    
      PAGE_NAME = "TRIP_STAT"
      trip_time = time.time() - trip_start_time
      minutes = int(trip_time // 60)
      seconds = int(trip_time % 60)
      data['trip_time'] = f"{minutes:02d}:{seconds:02d}"
      if timer_power_off is None:
        timer_power_off = time.time()
      SaveData()

    # Кнопка начала записи
    if can_start_record:
      button_rect = pygame.Rect(132, 12, 40, 40)
      pygame.draw.rect(screen, (40, 200, 64), button_rect, border_radius=25)
      button_text = font_small.render("", True, (0, 0, 0))
      screen.blit(button_text, button_text.get_rect(center=button_rect.center))

      mouse = pygame.mouse.get_pos()
      click = pygame.mouse.get_pressed()
      if button_rect.collidepoint(mouse) and click[0] and IS_RASPBERY:
        # Старт записи
        filename = "trip.mp4"
        if os.path.exists(filename):
          os.remove(filename)
        can_start_record = False
        recorder_proc = subprocess.Popen(["wf-recorder", "-f", filename])
        print(">>> Запись началась")

    # Переключатель ЭКО / НОРМА
    status_rect = pygame.Rect(WIDTH - 170, 12, 160, 40)
    status_text = "Э" if eco_mode else "Н"
    status_color = (200, 200, 200)
    pygame.draw.rect(screen, status_color, status_rect, border_radius=25)
    status_label = font_small.render(status_text, True, (0, 0, 0))
    screen.blit(status_label, status_label.get_rect(center=status_rect.center))

    mouse = pygame.mouse.get_pos()
    click = pygame.mouse.get_pressed()
    status_pressed = status_rect.collidepoint(mouse) and click[0] and (not block_touch or not IS_RASPBERY)
    if status_pressed and not eco_toggle_was_pressed:
      set_eco_mode(not eco_mode)
    eco_toggle_was_pressed = status_pressed

  #################### PAGE TRIP_STAT ###########################
  # добавить температуру моторов
  elif PAGE_NAME == "TRIP_STAT":
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

    #data_trip = {
    #  'max_speed': 0,
    #  'max_power': 0,
    #  'best_time_0_60': 0,
    #  'trip_start_bettery_perc': 0,
    #}

    sec_to_exit = 20#5

    timer_off_t = f"До выключения: {sec_to_exit - (time.time() - timer_power_off):.0f} сек"
    timer_off = font_small.render(timer_off_t, True, (0, 0, 0))
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

    if time.time() - timer_power_off > sec_to_exit:
      if not can_start_record:
        # Остановить запись
        recorder_proc.send_signal(signal.SIGINT)
        recorder_proc.wait()
        print(">>> Запись остановлена")
      SaveData()
      pygame.quit()
      if full_off:
        import os
        print("OFF")
        os.system('sudo shutdown now')


  pygame.display.flip()
  clock.tick(30)

  

pygame.quit()
