import datetime
import pygame
import threading
import serial
import struct
import math
import time


PACKET_INDEX_FOR_VESC = 47#4


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
  'speed': 0,
  'master': {'motor_current': 0, 'battery_current': 0, 'duty': 0, 'temp': 0},
  'slave': {'motor_current': 0, 'battery_current': 0, 'duty': 0, 'temp': 0},
  'battery_voltage': 0,
  'battery_level': 0,
  'odometer': saved_odometer,
  'trip_odometer': 0.0,
  'trip_tick': 0,
  'trip_speed_sum': 0.0,
  'trip_avg_speed': 0.0,
  'trip_time': "00:00",
}


# Таблица для расчёта процента заряда батареи
voltage_percent_table = [
  (63.0, 100), (62.1, 95), (61.2, 90), (60.3, 85), (59.4, 80),
  (58.5, 75), (57.6, 70), (56.7, 65), (55.8, 60), (54.9, 55),
  (54.0, 50), (53.1, 45), (52.2, 40), (51.3, 35), (50.4, 30),
  (49.5, 25), (48.6, 20), (47.7, 15), (46.8, 10), (45.9, 5),
  (45.0, 0)
]

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
              data['speed'] = int(speed_mps * 3.6)
                
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

######## INTERFACE
import pygame
import math
import time

pygame.init()

WIDTH, HEIGHT = 600, 955
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption('VESC Speedometer')

font_large = pygame.font.SysFont('Arial', 150)
font_medium = pygame.font.SysFont('Arial', 50)
font_small = pygame.font.SysFont('Arial', 35)
clock = pygame.time.Clock()

setDebugValues = False
needSetValues = False


def draw_progress_bar(surface, x, y, width, height, value, max_value, color):
  pygame.draw.rect(surface, (200, 200, 200), (x, y, width, height), border_radius=10)
  fill_width = int(width * min(value / max_value, 1.0))
  if fill_width > 0:
    pygame.draw.rect(surface, color, (x, y, fill_width, height), border_radius=10)

def draw_speed_arc(surface, center, radius, speed, max_speed):
  pygame.draw.arc(surface, (200, 200, 200), (center[0]-radius, center[1]-radius, radius*2, radius*2),
                  -math.pi * 0.15, math.pi * 1.15, 15)
  end_angle = math.pi * 1.15 - (speed / max_speed) * math.pi * 1.3
  if speed > 0:
    pygame.draw.arc(surface, (0, 200, 0), (center[0]-radius, center[1]-radius, radius*2, radius*2),
                    end_angle, math.pi * 1.15, 15)
    
    # Маленький зелёный маркер на дуге
    marker_outer_x = center[0] + (radius - 1)* math.cos(end_angle)
    marker_outer_y = center[1] - (radius - 1) * math.sin(end_angle)
    marker_inner_x = center[0] + (radius - 50) * math.cos(end_angle)
    marker_inner_y = center[1] - (radius - 50) * math.sin(end_angle)
    pygame.draw.line(surface, (0, 200, 0), (marker_inner_x, marker_inner_y), (marker_outer_x, marker_outer_y), 10)

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
    pygame.draw.line(surface, (0, 200, 0), (marker_inner_x, marker_inner_y), (marker_outer_x, marker_outer_y), 10)

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
  rect = render.get_rect(center=(WIDTH//2, y))
  surface.blit(render, rect)

def draw_text(surface, text, font, color, x, y):
  render = font.render(text, True, color)
  rect = render.get_rect(center=(x, y))
  surface.blit(render, rect)

def draw_text_left(surface, text, font, color, x, y):
  render = font.render(text, True, color)
  rect = render.get_rect(topleft=(x, y))
  surface.blit(render, rect)

def get_battery_color(level):
  if level < 25:
    return (255, 0, 0)
  elif level < 50:
    return (255, 165, 0)
  else:
    return (0, 200, 0)
  
def SaveData():
  try:
    with open("mainData.txt", "w") as f:
      f.write(f"{data['odometer'] + data['trip_odometer']:.1f}")
  except Exception as e:
    print("Ошибка сохранения одометра:", e)

# Переменные для замера разгона 0-40 км/ч
start_time = None
measured_time = None
ready = True
measuring = False
trip_start_time = None
timer_power_off = None

full_off = False

PAGE_NAME = "SPEEDOMETER"

running = True
#FULL_SCREEN
screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)

while running:
  for event in pygame.event.get():
    if event.type == pygame.QUIT:
      running = False

  # Здесь подставляй реальные данные в переменную data

  screen.fill((254, 254, 254))

  if PAGE_NAME == "SPEEDOMETER":
    # 1. Скорость полукруг
    draw_speed_arc(screen, (WIDTH//2, 180), 150, data['speed'], 60)
    draw_text_center(screen, f"{int(data['speed'])}", font_large, (0, 0, 0), 180)

    # 2. Показатели контроллеров мастер и слейв
    y_offset = 360
    spacing_x = 190

    summ_current = data['slave']['motor_current'] + data['master']['motor_current']
    draw_arc(f"{int(summ_current)}A", screen, (WIDTH * 0.2, 370), 80, summ_current, 200, (255, 0, 0))
    summ_battery = data['slave']['battery_current'] + data['master']['battery_current']
    draw_arc(f"{int(summ_battery)}A", screen, (WIDTH * 0.5, 370), 80, summ_battery, 50, (0, 0, 255))
    average_duty = int((data['slave']['duty'] + data['master']['duty']) / 2)
    draw_arc(f"{int(average_duty)}%", screen, (WIDTH * 0.8, 370), 80, average_duty, 100, (0, 0, 0))

    for idx, side in enumerate(['slave', 'master']):
      x = (WIDTH//2 - spacing_x) if side == 'master' else (WIDTH//2 + spacing_x)

      # Рисуем рамку вокруг каждого контроллера
      #pygame.draw.rect(screen, (120, 120, 120), (x-70, y_offset-65, 140, 250), width=2, border_radius=20)

      #draw_text(screen, f"{int(data[side]['motor_current'])}A", font_small, (255, 0, 0), x, y_offset - 35)
      #draw_progress_bar(screen, x-50, y_offset - 10, 100, 10, data[side]['motor_current'], 100, (255, 0, 0))

      #draw_text(screen, f"{int(data[side]['battery_current'])}A", font_small, (0, 0, 255), x, y_offset + 25)
      #draw_progress_bar(screen, x-50, y_offset + 50, 100, 10, data[side]['battery_current'], 25, (0, 0, 255))

      #draw_text(screen, f"{int(data[side]['duty'])}%", font_small, (0, 0, 0), x, y_offset + 85)
      #draw_progress_bar(screen, x-50, y_offset + 110, 100, 10, data[side]['duty'], 100, (0, 0, 0))

      temp = font_small.render(f"{int(data[side]['temp'])}°C", True, (0, 200, 0))
      screen.blit(temp, (x-25, 450))

    # блокируем тач при движении
    if data['speed'] > 0:
      pass
    else:
      pass

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
    boostDown = 100
    battery_text = font_medium.render(f"{data['battery_voltage']:.1f}V  {int(data['battery_level'])}%", True, (0, 100, 255))
    battery_rect = battery_text.get_rect(center=(WIDTH//2 - 40, 800 + boostDown))
    screen.blit(battery_text, battery_rect)

    # Расчёт процента заряда батареи
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
    draw_progress_bar(screen, battery_rect.right + 10, 800 - 15 + boostDown, 120, 30, data['battery_level'], 100, battery_color)

    # 5. Одометр
    draw_text_center(screen, f"{(data['odometer'] + data['trip_odometer']):.1f} км", font_small, (170, 170, 0), 930)
    if trip_start_time is not None:
      # Расчёт дистанции и средней скорости поездки
      if 'trip_odometer' not in data:
        data['trip_odometer'] = 0.0
      elapsed_time = time.time() - trip_time_start
      data['trip_speed_sum'] += data['speed']
      data['trip_tick'] += 1
      data['trip_avg_speed'] = data['trip_speed_sum'] / data['trip_tick']
      data['trip_odometer'] = data['trip_avg_speed'] * (elapsed_time / 3600.0)

      # поездка
      trip_text_km = font_small.render(f"{data['trip_odometer']:.1f} км", True, (0, 0, 0))
      trip_text_speed = font_small.render(f"{data['trip_avg_speed']:.1f} км/ч", True, (0, 0, 0))
      trip_km_rect = trip_text_km.get_rect(topright=(WIDTH - 20, 840 + boostDown))
      trip_speed_rect = trip_text_speed.get_rect(topright=(WIDTH - 20, 875 + boostDown))

      screen.blit(trip_text_km, trip_km_rect)
      screen.blit(trip_text_speed, trip_speed_rect)

      trip_time = time.time() - trip_start_time
      minutes = int(trip_time // 60)
      seconds = int(trip_time % 60)
      draw_text(screen, f"{minutes:02d}:{seconds:02d}", font_small, (0, 0, 0), 540, 930 + boostDown)


    # Отображение даты и времени
    now = datetime.datetime.now()
    weekdays = ['Пн', 'Вт', 'Ср', 'Чт', 'Пт', 'Сб', 'Вс']
    months = ['янв', 'фев', 'мар', 'апр', 'май', 'июн', 'июл', 'авг', 'сен', 'окт', 'ноя', 'дек']
    date_week_text = font_small.render(f"{weekdays[now.weekday()]}", True, (0, 0, 0))
    date_week_rect = date_week_text.get_rect(topleft=(20, 840 + boostDown))
    screen.blit(date_week_text, date_week_rect)
    date_text = font_small.render(f"{now.day} {months[now.month-1]}", True, (0, 0, 0))
    date_rect = date_text.get_rect(topleft=(20, 875 + boostDown))
    screen.blit(date_text, date_rect)

    time_text = font_small.render(f"{now.hour:02d}:{now.minute:02d}", True, (0, 0, 0))
    time_rect = time_text.get_rect(topleft=(20, 930 - 20 + boostDown))
    screen.blit(time_text, time_rect)

    # Кнопка выключения системы
    button_rect = pygame.Rect(10, 10, 40, 40)
    pygame.draw.rect(screen, (255, 95, 87), button_rect, border_radius=25)
    button_text = font_small.render("", True, (0, 0, 0))
    screen.blit(button_text, button_text.get_rect(center=button_rect.center))

    mouse = pygame.mouse.get_pos()
    click = pygame.mouse.get_pressed()
    if button_rect.collidepoint(mouse) and click[0]:
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

    # Кнопка выключения программы
    button_rect = pygame.Rect(10, 70, 40, 40)
    pygame.draw.rect(screen, (255, 188, 46), button_rect, border_radius=25)
    button_text = font_small.render("", True, (0, 0, 0))
    screen.blit(button_text, button_text.get_rect(center=button_rect.center))

    mouse = pygame.mouse.get_pos()
    click = pygame.mouse.get_pressed()
    if button_rect.collidepoint(mouse) and click[0]:
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

    # Кнопка отладочного включения показателей
    button_rect = pygame.Rect(10, 130, 40, 40)
    pygame.draw.rect(screen, (40, 200, 64), button_rect, border_radius=25)
    button_text = font_small.render("", True, (0, 0, 0))
    screen.blit(button_text, button_text.get_rect(center=button_rect.center))

    mouse = pygame.mouse.get_pos()
    click = pygame.mouse.get_pressed()
    if button_rect.collidepoint(mouse) and click[0]:
      setDebugValues = not setDebugValues
      needSetValues = True

    if needSetValues:
      if setDebugValues:
        data['speed'] = 40
        data['battery_voltage'] = 58.3
        data['master']['motor_current'] = 100
        data['master']['battery_current'] = 25
        data['master']['duty'] = 100
      else:
        data['speed'] = 0
        data['battery_voltage'] = 0
        data['master']['motor_current'] = 0
        data['master']['battery_current'] = 0
        data['master']['duty'] = 0
      needSetValues = False

  elif PAGE_NAME == "TRIP_STAT":
    trip_text_km = font_small.render(f"{data['trip_odometer']:.1f} км", True, (0, 0, 0))
    trip_text_speed = font_small.render(f"{data['trip_avg_speed']:.1f} км/ч", True, (0, 0, 0))
    trip_km_rect = trip_text_km.get_rect(topright=(WIDTH - 20, 340))
    trip_speed_rect = trip_text_speed.get_rect(topright=(WIDTH - 20, 375))

    screen.blit(trip_text_km, trip_km_rect)
    screen.blit(trip_text_speed, trip_speed_rect)
    draw_text(screen, data['trip_time'], font_small, (0, 0, 0), WIDTH - 60, 430)

    draw_text_center(screen, "Статистика поездки:", font_small, (100, 100, 100), 280)
    draw_text_left(screen, "Пройденное расстояние ", font_small, (100, 100, 100), 10, 340 - 2)
    draw_text_left(screen, "Средняя скорость ", font_small, (100, 100, 100), 10, 375 - 2)
    draw_text_left(screen, "Время в пути ", font_small, (100, 100, 100), 10, 410 - 2)



    timer_off_t = f"До выключения: {15 - (time.time() - timer_power_off):.0f} сек"
    timer_off = font_small.render(timer_off_t, True, (0, 0, 0))
    draw_text(screen, timer_off_t, font_small, (0, 0, 0), WIDTH * 0.5, 530)

    if time.time() - timer_power_off > 15:
      SaveData()
      pygame.quit()
      if full_off:
        import os
        print("OFF")
        os.system('sudo shutdown now')


  pygame.display.flip()
  clock.tick(30)

  

pygame.quit()