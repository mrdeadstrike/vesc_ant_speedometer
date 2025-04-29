import datetime
import pygame
import threading
import serial
import struct
import math
import time


PACKET_INDEX_FOR_VESC = 47#4


# Данные контроллеров
data = {
  'speed': 0,
  'master': {'motor_current': 0, 'battery_current': 0, 'duty': 0, 'temp': 0},
  'slave': {'motor_current': 0, 'battery_current': 0, 'duty': 0, 'temp': 0},
  'battery_voltage': 0
}

# Параметры колеса
wheel_diameter_m = 0.28  # 280 мм = 0.28 м
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
              data['speed'] = int(speed_mps * 3.6)
              data['master']['motor_current'] = motor_current
              data['master']['battery_current'] = input_current
              data['master']['duty'] = duty_cycle
              data['master']['temp'] = mos_temp
              data['battery_voltage'] = volt
              data['battery_level'] = battery_level
              data['odometer'] = odometer

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

######## INTERFACE
import pygame
import math
import time

pygame.init()

WIDTH, HEIGHT = 600, 960
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption('VESC Speedometer')

font_large = pygame.font.SysFont('Arial', 125)
font_medium = pygame.font.SysFont('Arial', 36)
font_small = pygame.font.SysFont('Arial', 24)
clock = pygame.time.Clock()



data = {
  'speed': 0,
  'master': {'motor_current': 0, 'battery_current': 0, 'duty': 0, 'temp': 0},
  'slave': {'motor_current': 0, 'battery_current': 0, 'duty': 0, 'temp': 0},
  'battery_voltage': 0,
  'battery_level': 0,
  'odometer': 0
}

def draw_progress_bar(surface, x, y, width, height, value, max_value, color):
  pygame.draw.rect(surface, (50, 50, 50), (x, y, width, height), border_radius=5)
  fill_width = int(width * min(value / max_value, 1.0))
  if fill_width > 0:
    pygame.draw.rect(surface, color, (x, y, fill_width, height), border_radius=5)

def draw_speed_arc(surface, center, radius, speed, max_speed):
  pygame.draw.arc(surface, (100, 100, 100), (center[0]-radius, center[1]-radius, radius*2, radius*2),
                  0, math.pi, 10)
  end_angle = math.pi - (speed / max_speed) * math.pi
  if speed > 0:
    pygame.draw.arc(surface, (0, 200, 0), (center[0]-radius, center[1]-radius, radius*2, radius*2),
                    end_angle, math.pi, 10)
    
    # Маленький зелёный маркер на дуге
    marker_outer_x = center[0] + radius * math.cos(end_angle)
    marker_outer_y = center[1] - radius * math.sin(end_angle)
    marker_inner_x = center[0] + (radius - 70) * math.cos(end_angle)
    marker_inner_y = center[1] - (radius - 70) * math.sin(end_angle)
    pygame.draw.line(surface, (0, 200, 0), (marker_inner_x, marker_inner_y), (marker_outer_x, marker_outer_y), 8)

  # Отметки скорости
  for mark in [0, 20, 40, 60]:
    angle = math.pi + (mark / max_speed) * math.pi
    x_outer = center[0] + (radius + 5) * math.cos(angle)
    y_outer = center[1] + (radius + 5) * math.sin(angle)
    x_inner = center[0] + (radius - 15) * math.cos(angle)
    y_inner = center[1] + (radius - 15) * math.sin(angle)
    pygame.draw.line(surface, (200, 200, 200), (x_inner, y_inner), (x_outer, y_outer), 2)

    x = center[0] + radius * 1.2 * math.cos(angle)
    y = center[1] + radius * 1.2 * math.sin(angle)
    label = font_small.render(str(mark), True, (0, 0, 0))
    label_rect = label.get_rect(center=(x, y))
    surface.blit(label, label_rect)

def draw_text_center(surface, text, font, color, y):
  render = font.render(text, True, color)
  rect = render.get_rect(center=(WIDTH//2, y))
  surface.blit(render, rect)

def draw_text(surface, text, font, color, x, y):
  render = font.render(text, True, color)
  rect = render.get_rect(center=(x, y))
  surface.blit(render, rect)

def get_battery_color(level):
  if level < 25:
    return (255, 0, 0)
  elif level < 50:
    return (255, 165, 0)
  else:
    return (0, 255, 0)

# Переменные для замера разгона 0-40 км/ч
start_time = None
measured_time = None
ready = True
measuring = False
trip_start_time = None

running = True
while running:
  for event in pygame.event.get():
    if event.type == pygame.QUIT:
      running = False

  # Здесь подставляй реальные данные в переменную data

  screen.fill((254, 254, 254))

  # 1. Скорость полукруг
  draw_speed_arc(screen, (WIDTH//2, 200), 150, data['speed'], 60)
  draw_text_center(screen, f"{int(data['speed'])}", font_large, (0, 0, 0), 190)

  # 2. Показатели мастер и слейв
  y_offset = 360
  spacing_x = 150

  for idx, side in enumerate(['slave', 'master']):
    x = (WIDTH//2 - spacing_x) if side == 'master' else (WIDTH//2 + spacing_x)

    # Рисуем рамку вокруг каждого контроллера
    pygame.draw.rect(screen, (120, 120, 120), (x-60, y_offset-60, 120, 240), width=2, border_radius=10)

    draw_text(screen, f"{int(data[side]['motor_current'])}A", font_small, (255, 0, 0), x, y_offset - 40)
    draw_progress_bar(screen, x-40, y_offset - 10, 80, 10, data[side]['motor_current'], 100, (255, 0, 0))

    draw_text(screen, f"{int(data[side]['battery_current'])}A", font_small, (0, 0, 255), x, y_offset + 20)
    draw_progress_bar(screen, x-40, y_offset + 50, 80, 10, data[side]['battery_current'], 25, (0, 0, 255))

    draw_text(screen, f"{int(data[side]['duty'])}%", font_small, (0, 0, 0), x, y_offset + 80)
    draw_progress_bar(screen, x-40, y_offset + 110, 80, 10, data[side]['duty'], 100, (0, 0, 0))

    temp = font_small.render(f"{int(data[side]['temp'])}°C", True, (0, 255, 0))
    screen.blit(temp, (x-25, 500))

  # 3. Замер времени разгона 0-40 км/ч
  if ready and data['speed'] > 0:
    start_time = time.time()
    ready = False
    measuring = True

  if trip_start_time is None and data['speed'] > 10:
    trip_start_time = time.time()

  if measuring and data['speed'] >= 40:
    measured_time = time.time() - start_time
    measuring = False

  if data['speed'] == 0:
    start_time = None
    measured_time = None
    ready = True
    measuring = False

  if measuring:
    current_elapsed = time.time() - start_time
    draw_text_center(screen, f"Разгон: {current_elapsed:.2f} сек", font_medium, (0, 0, 0), 600)
  elif measured_time is not None:
    draw_text_center(screen, f"0-40: {measured_time:.2f} сек", font_medium, (0, 0, 0), 600)
  else:
    draw_text_center(screen, "Готов", font_medium, (0, 0, 0), 600)

  # 4. Вольтаж батареи и заряд
  battery_text = font_medium.render(f"{data['battery_voltage']:.1f}V  {int(data['battery_level'])}%", True, (0, 255, 255))
  battery_rect = battery_text.get_rect(center=(WIDTH//2 - 40, 800))
  screen.blit(battery_text, battery_rect)

  battery_color = get_battery_color(data['battery_level'])
  draw_progress_bar(screen, battery_rect.right + 10, 800 - 10, 100, 20, data['battery_level'], 100, battery_color)

  # 5. Одометр
  draw_text_center(screen, f"{data['odometer']} км", font_small, (200, 200, 0), 860)
  if trip_start_time is not None:
    trip_time = time.time() - trip_start_time
    minutes = int(trip_time // 60)
    seconds = int(trip_time % 60)
    draw_text(screen, f"{minutes:02d}:{seconds:02d}", font_small, (0, 0, 0), 400, 860)


  # Отображение даты и времени
  now = datetime.datetime.now()
  weekdays = ['Пн', 'Вт', 'Ср', 'Чт', 'Пт', 'Сб', 'Вс']
  months = ['янв', 'фев', 'мар', 'апр', 'май', 'июн', 'июл', 'авг', 'сен', 'окт', 'ноя', 'дек']
  date_text = font_small.render(f"{weekdays[now.weekday()]} {now.day} {months[now.month-1]}", True, (0, 0, 0))
  date_rect = date_text.get_rect(topleft=(20, 840 - 5))
  screen.blit(date_text, date_rect)

  time_text = font_small.render(f"{now.hour:02d}:{now.minute:02d}", True, (0, 0, 0))
  time_rect = time_text.get_rect(topleft=(20, 865 - 5))
  screen.blit(time_text, time_rect)

  # Кнопка выключения
  button_rect = pygame.Rect(20, 20, 60, 40)
  pygame.draw.rect(screen, (100, 0, 0), button_rect, border_radius=10)
  button_text = font_small.render("Выкл", True, (0, 0, 0))
  screen.blit(button_text, button_text.get_rect(center=button_rect.center))

  mouse = pygame.mouse.get_pos()
  click = pygame.mouse.get_pressed()
  if button_rect.collidepoint(mouse) and click[0]:
    import os
    pygame.quit()
    os.system('sudo shutdown now')

  pygame.display.flip()
  clock.tick(30)

pygame.quit()