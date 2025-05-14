import datetime
import signal
import socket
import subprocess
import pygame
import threading
import serial
import struct
import math
import time

import urllib


PACKET_INDEX_FOR_VESC = 47#4

GREEN_COLOR = (0, 160, 0)
GREEN_LIGHT = (0, 210, 0)
ORANGE_COLOR = (230, 135, 0)
GRAY = (180, 180, 180)

BMS_LOST = False

PREV_VALS = {
  'bms_lost': False,
  'trip_mins': 0, 
  'page_name': "SPEEDOMETER",
}

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
  'master': {'motor_current': 0, 'battery_current': 0, 'duty': 0, 'temp': 0},
  'slave': {'motor_current': 0, 'battery_current': 0, 'duty': 0, 'temp': 0},
  'battery_voltage': 0,
  'v_without_nagruzka': 0,
  'battery_level': 0,
  'odometer': saved_odometer,
  'trip_odometer': 0.0,
  'trip_tick': 0,
  'trip_speed_sum': 0.0,
  'trip_avg_speed': 0.0,
  'trip_time': "00:00",
  'cells_v': [3.99, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4.01],
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
}


# Таблица для расчёта процента заряда батареи 15S
voltage_percent_table = [
  (4.17 * 15, 100), (4.053 * 15, 90), (3.946 * 15, 80),
  (3.845 * 15, 70), (3.755 * 15, 60),
  (3.673 * 15, 50), (3.624 * 15, 40), (3.592 * 15, 30),
  (3.555 * 15, 20), (3.477 * 15, 10), (3.405 * 15, 0)
]

############## VOICE SPEAK #####################
#sudo apt install rhvoice-russian
def speak_run(text, voice='anna', pitch=0.0, rate=0.15, volume=0.0):
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
  print(MESSAGES_TO_SPEAK)

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

    return rpm, input_current, duty_cycle, input_voltage, motor_current, mos_temp, battery_level, odometer
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
def read_serial(
                #port_name='/dev/tty.usbmodem3041', #MAC
                port_name='/dev/ttyACM0', #Raspbery PI
                baudrate=115200):
  try:
    ser = serial.Serial(port_name, baudrate, timeout=0.1)
  except Exception as e:
    print("Не удалось открыть порт:", e)
    return

  packet_master = pack_comm_get_values()
  packet_slave = pack_comm_get_values(can_id=15)

  while True:
    try:
      #GET_INFO
      ser.write(packet_master)
      header = ser.read(2)
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
              rpm, input_current, duty_cycle, volt, motor_current, mos_temp, battery_level, odometer = parsed
              wheel_rpm = rpm
              speed_mps = (wheel_rpm * wheel_circumference_m) / 60
              data['speed'] = speed_mps * 3.6
                
              data['master']['motor_current'] = motor_current
              data['master']['battery_current'] = input_current
              data['master']['duty'] = duty_cycle
              data['master']['temp'] = mos_temp
              data['battery_voltage'] = volt
              #data['battery_level'] = battery_level
              #data['odometer'] = odometer

      time.sleep(0.1)#0.05

      slave_id = 15
      command = PACKET_INDEX_FOR_VESC#4  # COMM_GET_VALUES

      payload = struct.pack('>BBB', 0x22, slave_id, command)
      packet = pack_packet_slave(payload)

      #ser.write(packet)
      #response = ser.read(1024)
      #print(response)
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
              rpm, input_current, duty_cycle, volt, motor_current, mos_temp, battery_level, odometer = parsed
              data['slave']['motor_current'] = motor_current
              data['slave']['battery_current'] = input_current
              data['slave']['duty'] = duty_cycle
              data['slave']['temp'] = mos_temp

    except Exception as e:
      print("Ошибка чтения данных:", e)

threading.Thread(target=read_serial, daemon=True).start()

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
      time.sleep(0.1)
      continue

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
    for i in range(15):  # для 15s
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
        except:
          ser = None
        print("Не удалось открыть порт:", e)
        time.sleep(2)
        continue

      try:
        read_bms_data(ser)
      except:
        try:
          ser.close()
        except:
          ser = None
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
  av_duty = int((data['slave']['duty'] + data['master']['duty']) / 2)
  #if speed > 0:
  #  if av_duty >= 85:
  #    draw_filled_arc(surface, (255, 180, 180), center, radius, math.pi * 0.15, -math.pi * 1.15)

  pygame.draw.arc(surface, (200, 200, 200), (center[0]-radius, center[1]-radius, radius*2, radius*2),
                  -math.pi * 0.15, math.pi * 1.15, 20)
  end_angle = math.pi * 1.15 - ((speed) / max_speed) * math.pi * 1.3
  if speed > 0:
    speedColor = GREEN_LIGHT
    if av_duty >= 85:
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
  for mark in [0, 20, 40, 60, 80]:
    angle = math.pi * 0.85 + (mark / max_speed) * math.pi * 1.3
    x_outer = center[0] + (radius + 5) * math.cos(angle)
    y_outer = center[1] + (radius + 5) * math.sin(angle)
    x_inner = center[0] + (radius - 25) * math.cos(angle)
    y_inner = center[1] + (radius - 25) * math.sin(angle)
    pygame.draw.line(surface, (60, 60, 60), (x_inner, y_inner), (x_outer, y_outer), 3)

    x = center[0] + radius * 1.22 * math.cos(angle)
    y = center[1] + radius * 1.22 * math.sin(angle)
    label = font_small.render(str(mark), True, (0, 0, 0))
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
  is_left = True
  x_shift = WIDTH * 0.425
  y_shift = startY

  good_cell_index = 0
  good_cell_max = 0
  bad_cell_index = 0
  bad_cell_min = 5
  for i in range(len(data['cells_v'])):
    cell_v = data['cells_v'][i]
    if cell_v > good_cell_max:
      good_cell_max = cell_v
      good_cell_index = i
    if cell_v < bad_cell_min:
      bad_cell_min = cell_v
      bad_cell_index = i

  cell_ind = 0
  for cell_v in data['cells_v']:
    left_boost = 10
    if not is_left:
      left_boost = 190

    cell_color = (200, 200, 200)
    cell_index_color = (0, 0, 0)
    cell_v_color = (150, 150, 150)
    if cell_ind == good_cell_index:
      cell_color = GREEN_COLOR
      cell_index_color = cell_color
      cell_v_color = cell_color
    if cell_ind == bad_cell_index:
      cell_color = (255, 0, 0)
      cell_index_color = cell_color
      cell_v_color = cell_color

    pygame.draw.rect(screen, cell_color, (x_shift + left_boost - 15, y_shift + 2, 155, 38), width=2, border_radius=10)
    draw_text(screen, f"{cell_ind + 1}", font_small, cell_index_color, x_shift + left_boost + 15, y_shift + 20)
    draw_text(screen, f"{cell_v:.2f}", font_small, cell_v_color, x_shift + left_boost + 90, y_shift + 20)

    if not is_left:
      y_shift += 43
      
    is_left = not is_left
    cell_ind += 1
  
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
  data['bms_current'] = 50 * changeV
  data['speed'] = 70 * changeV
  data['master']['duty'] = 300 * changeV
  data['battery_voltage'] = 60 - 10 * changeV
  if data['master']['duty'] > 200:
    data['master']['duty'] = 200

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
    add_speak_message("Статистика поездки:")
    add_speak_message("Приехал " + trip_end_datetime_str)
    add_speak_message("Время в пути " + data['trip_time'])
    add_speak_message("Расстояние " + f"{data['trip_odometer']:.1f}".replace(".", " и ") + " километров")
    add_speak_message("Средняя скорость " + f"{data['trip_avg_speed']:.1f}".replace(".", " и ") + " километров в час")
    add_speak_message("Максимальная скорость " + f"{int(data_trip['max_speed'])} километров в час")
    add_speak_message("Лучшее время от 0 до 60 составило " + f"{data_trip['best_time_0_60']:.2f}".replace(".", " и ") + " секунд")
    add_speak_message("Максимальная мощность " + f"{int(data_trip['max_power'])} ват")
    add_speak_message("Потрачено заряда " + f"{int(data_trip['trip_start_bettery_perc'] - data['battery_level'])} процентов")
    add_speak_message("Максимальная просадка " + f"{data_trip['max_voltage_down']:.1f}".replace(".", " и ") + " вольт")
    add_speak_message("Слабейший ряд " + f"{data_trip['min_cell_v_index'] + 1}")
    add_speak_message("Минимальный вольтаж в ряду " + f"{data_trip['min_cell_v']:.3f}".replace(".", " и ") + " вольт")
    add_speak_message("Максимальный разбаланс " + f"{data_trip['max_unit_diff']:.3f}".replace(".", " и ") + " вольт")

  PREV_VALS['page_name'] = PAGE_NAME
  if PAGE_NAME == "SPEEDOMETER":
    # 1. Скорость полукруг
    average_duty = int((data['slave']['duty'] + data['master']['duty']) / 2)
    speed_color = (0, 0, 0)
    #if average_duty >= 85:
    #  speed_color = (255, 0, 0)

    draw_speed_arc(screen, (WIDTH//2, 180 + up_gap), 150, int(data['speed']), 80, up_gap)

    # 2. Показатели контроллеров мастер и слейв
    y_offset = 360
    spacing_x = 250

    summ_current = data['slave']['motor_current'] + data['master']['motor_current']
    if summ_current > 200:
      summ_current = 200
    
    up_gap += 20

    if average_duty >= 85:
      draw_progress_bar(screen, 15, 40 + up_gap, 110, 15, 100 if miganie else 0, 100, "FW", (255, 0, 0))
    else:
      draw_progress_bar(screen, 15, 40 + up_gap, 110, 15, 100 if miganie else 0, 100, "FW", (240, 240, 240), False)

    #draw_arc(f"{int(summ_current)}A", screen, (WIDTH * 0.9, 100 + up_gap), 80, summ_current, 200, (255, 0, 0))
    draw_progress_bar(screen, WIDTH * 0.8, 40 + up_gap, 110, 15, int(summ_current), 200, str(int(summ_current)) + "A", (255, 0, 0))
    summ_battery = int(((data['slave']['battery_current'] + data['master']['battery_current']) / 2))
    if summ_battery > 50:
      summ_battery = 50

    bms_current = data['bms_current']

    #draw_progress_bar(screen, WIDTH * 0.8, 60 + up_gap, 110, 15, int(summ_battery), 50, f"{int(summ_battery)}A", (0, 0, 255))
    draw_progress_bar(screen, WIDTH * 0.8, 130 + up_gap, 110, 15, int(bms_current), 50, f"{int(bms_current)}A", (0, 0, 255))

    draw_progress_bar(screen, 15, 130 + up_gap, 110, 15, int(average_duty), 100, f"{int(average_duty)}%", (0, 0, 0))

    draw_text_center(screen, str(data['power']) + " Вт", font_small, (0, 0, 0), 295)

    #draw_arc(f"{int(summ_battery)}A", screen, (WIDTH * 0.9, 220 + up_gap), 80, summ_battery, 50, (0, 0, 255))
    #draw_arc(f"{int(average_duty)}%", screen, (WIDTH * 0.1, 220 + up_gap), 80, average_duty, 100, (0, 0, 0))

    # Когда ослабление магнитного поля активно рисуем рамку
    #if average_duty >= 85:
    #  pygame.draw.rect(screen, (255, 0, 0), (0, 0, WIDTH, HEIGHT), width=12, border_radius=0)

    #Температура всего
    temp_y = 345
    border_r = 10
    pygame.draw.rect(screen, (200, 200, 200), (15, temp_y - 22, WIDTH * 0.46, 44), width=2, border_radius=border_r)
    draw_text(screen, f"МК", font_small, (200, 200, 200), WIDTH * 0.1, temp_y)
    draw_text(screen, f"?°", font_small, GREEN_COLOR, WIDTH * 0.25, temp_y)
    draw_text(screen, f"?°", font_small, GREEN_COLOR, WIDTH * 0.4, temp_y)

    temp_y += 50
    pygame.draw.rect(screen, (200, 200, 200), (15, temp_y - 22, WIDTH * 0.46, 44), width=2, border_radius=border_r)
    draw_text(screen, f"К", font_small, (200, 200, 200), WIDTH * 0.1, temp_y)
    draw_text(screen, f"{int(data['slave']['temp'])}°", font_small, GREEN_COLOR, WIDTH * 0.25, temp_y)
    draw_text(screen, f"{int(data['master']['temp'])}°", font_small, GREEN_COLOR, WIDTH * 0.4, temp_y)

    temp_y -= 50
    pygame.draw.rect(screen, (200, 200, 200), (WIDTH * 0.5 + 10, temp_y - 22, WIDTH * 0.46, 44), width=2, border_radius=border_r)
    draw_text(screen, f"М/Б", font_small, (200, 200, 200), WIDTH * 0.6, temp_y)
    mos_color = get_battery_temp_color(int(data['bms_temp']['mosfet_temp']))
    draw_text(screen, f"{int(data['bms_temp']['mosfet_temp'])}°", font_small, mos_color, WIDTH * 0.75, temp_y)
    bal_color = get_battery_temp_color(int(data['bms_temp']['balance_temp']))
    draw_text(screen, f"{int(data['bms_temp']['balance_temp'])}°", font_small, bal_color, WIDTH * 0.9, temp_y)

    temp_y += 50
    pygame.draw.rect(screen, (200, 200, 200), (WIDTH * 0.5 + 10, temp_y - 22, WIDTH * 0.46, 44), width=2, border_radius=border_r)
    draw_text(screen, f"Б", font_small, (200, 200, 200), WIDTH * 0.6, temp_y)
    bat_temp_1 = get_battery_temp_color(int(data['bms_temp']['external_temp_0']))
    draw_text(screen, f"{int(data['bms_temp']['external_temp_0'])}°", font_small, bat_temp_1, WIDTH * 0.75, temp_y)
    bat_temp_2 = get_battery_temp_color(int(data['bms_temp']['external_temp_1']))
    draw_text(screen, f"{int(data['bms_temp']['external_temp_1'])}°", font_small, bat_temp_2, WIDTH * 0.9, temp_y)

    #ВОЛЬТАЖ

    # 4. Вольтаж батареи и заряд
    boostDown = 50
    v_y = 450
    # запоминаем вольтаж без нагрузки и рекуперации
    if int(summ_current) == 0:
      data['v_without_nagruzka'] = data['battery_voltage']

    voltage_down = (data['battery_voltage'] - data['v_without_nagruzka'])
    data['voltage_down'] = voltage_down
    voltage_down_color = GREEN_COLOR
    if voltage_down < -5:
      voltage_down_color = (255, 0, 0)
    elif voltage_down < -2:
      voltage_down_color = ORANGE_COLOR

    pygame.draw.rect(screen, (200, 200, 200), (15, v_y - 27, WIDTH - 30, 54), width=2, border_radius=border_r)
    draw_text(screen, f"{voltage_down:.1f}", font_medium, voltage_down_color, WIDTH * 0.1275, v_y)
    draw_text(screen, f"{data['battery_voltage']:.1f}", font_medium, (0, 100, 255), WIDTH * 0.38, v_y)
    #draw_text_left(screen, f"{data['v_without_nagruzka']:.1f}V", font_medium, (0, 100, 255), WIDTH * 0.5, v_y)

    #battery_text = font_medium.render(f"{data['battery_voltage']:.1f}V  {data['v_without_nagruzka']:.1f}V {int(data['battery_level'])}%", True, (0, 100, 255))
    #battery_rect = battery_text.get_rect(center=(WIDTH//2 - 40, 800 + boostDown))
    #screen.blit(battery_text, battery_rect)

    # Расчёт процента заряда батареи
    if int(summ_current) == 0:
      voltages = [v for v, _ in voltage_percent_table]
      percents = [p for _, p in voltage_percent_table]

      if data['battery_voltage'] >= voltages[0]:
        data['battery_level'] = 100
      elif data['battery_voltage'] <= voltages[-1]:
        data['battery_level'] = 0
      else:
        data['battery_level'] = 0
        for i in range(len(voltages) - 1):
          v_high, v_low = voltages[i], voltages[i + 1]
          p_high, p_low = percents[i], percents[i + 1]
          if v_high >= data['battery_voltage'] >= v_low:
            # Линейная интерполяция между двумя ближайшими точками
            ratio = (data['battery_voltage'] - v_low) / (v_high - v_low)
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
      
    if not PREV_VALS['bms_lost'] and BMS_LOST:
      add_speak_message("Связь с бмс потеряна")
    if PREV_VALS['bms_lost'] and not BMS_LOST:
      add_speak_message("Связь с бмс восстановлена")
    PREV_VALS['bms_lost'] = BMS_LOST


    # блокируем тач при движении
    if data['speed'] > 0:
      block_touch = True
    else:
      block_touch = False

    if trip_start_time is None and data['speed'] > 10:
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

    if int(data['speed']) == 0 or (not zamer_success and prev_speed > int(data['speed'])):
      start_time = None
      measured_time = None
      if zamer_success or int(data['speed']) == 0:
        ready = True
      measuring = False
      zamer_success = False
    prev_speed = int(data['speed'])

    if not zamer_success_prev and zamer_success:
      add_speak_message("Разгон " + f"{measured_time:.2f}".replace(".", " и ") + "... пиздец медленно")
    zamer_success_prev = zamer_success

    razg_boost = 260
    if measuring:
      current_elapsed = time.time() - start_time
      draw_text_center(screen, f"Разгон: {current_elapsed:.2f} сек", font_medium, (0, 0, 0), 600 + razg_boost)
    elif measured_time is not None:
      draw_text_center(screen, f"0-60: {measured_time:.2f} сек", font_medium, (0, 0, 0), 600 + razg_boost)
    else:
      draw_text_center(screen, "-", font_medium, (0, 0, 0), 600 + razg_boost)

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
      # Озвучиваем статистику поездки каждую минуту
      if PREV_VALS['trip_mins'] != minutes:
        PREV_VALS['trip_mins'] = minutes
        add_speak_message(f"Время в пути {minutes:02d} минут")
        if minutes == 15:
          add_speak_message(f"Опять еле едем из-за долбаебов на дороге")
        add_speak_message(f"Средняя скорость" + f" {data['trip_avg_speed']:.1f}".replace(".", " и ") + " километров в час")
        add_speak_message(f"Заряд {data['battery_level']} процентов")
        add_speak_message(f"Слабейший ряд {data['bad_cell_min_peak_index'] + 1}... минимальный заряд " + f"{data['bad_cell_min_peak']}".replace(".", " и ") + " вольт")
        
      trip_y += trip_y_new_line
      draw_text_right(screen, f"{minutes:02d}:{seconds:02d}", font_small, (0, 0, 0), WIDTH - 10, trip_y)


    # Отображение даты и времени
    now = datetime.datetime.now()
    weekdays = ['Пн', 'Вт', 'Ср', 'Чт', 'Пт', 'Сб', 'Вс']
    months = ['янв', 'фев', 'мар', 'апр', 'мая', 'июн', 'июл', 'авг', 'сен', 'окт', 'ноя', 'дек']
    trip_y -= trip_y_new_line * 2
    draw_text_left(screen, f"{weekdays[now.weekday()]}", font_small, (0, 0, 0), 10, trip_y)
    trip_y += trip_y_new_line
    draw_text_left(screen, f"{now.day} {months[now.month-1]}", font_small, (0, 0, 0), 10, trip_y)
    trip_y += trip_y_new_line
    draw_text_left(screen, f"{now.hour:02d}:{now.minute:02d}", font_small, (0, 0, 0), 10, trip_y)
    trip_end_datetime_str = f"{weekdays[now.weekday()]} {now.day} {months[now.month-1]} {now.hour:02d}:{now.minute:02d}"

    # Озвучиваем важную информацию
    #add_speak_message("1")

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

  #################### PAGE TRIP_STAT ###########################
  elif PAGE_NAME == "TRIP_STAT":
    y_trip_start = 80
    draw_text_center(screen, "Статистика поездки:", font_small, GRAY, y_trip_start)
    y_trip_start += 60
    y_trip_shift = 40
    draw_text_left(screen, "Приехал ", font_small, GRAY, 10, y_trip_start - 2)
    draw_text_right(screen, trip_end_datetime_str, font_small, (0, 0, 0), WIDTH - 20, y_trip_start)
    y_trip_start += y_trip_shift
    draw_text_left(screen, "Время в пути ", font_small, GRAY, 10, y_trip_start - 2)
    draw_text_right(screen, data['trip_time'], font_small, (0, 0, 0), WIDTH - 20, y_trip_start)
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
    draw_text_right(screen, f"{data_trip['best_time_0_60']:.2f} с", font_small, (0, 0, 0), WIDTH - 20, y_trip_start)
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

    sec_to_exit = 60#5
    if full_off:
      sec_to_exit = 60

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