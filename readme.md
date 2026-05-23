Спидометр с озвучкой и голосовыми командами для FarDriver/VESC и Smart BMS ANT

Скрипт для автозапуска на Raspbery PI 4:
[Desktop Entry]
Name=My Python Script
Comment=Запуск скрипта
Exec=python3 Documents/vesc_ant_speedometer/speedometer.py
Icon=utilities-terminal
Terminal=true
Type=Application
Encoding=UTF-8
Categories=Utility;


chmod +x ~/Desktop/MyScript.desktop

### Подключение FarDriver по Bluetooth
- Найдите MAC Bluetooth-адаптера FarDriver: `python3 scan_ble.py --timeout 20`. Обычно имя начинается с `YuanQu`, `FarDriver`, `CONTROLDM` или похоже на них.
- Запустите BLE UART мост:

```bash
BLE_DEVICE_MAC=AA:BB:CC:DD:EE:FF ./start_ble_bridge.sh
```

- Для двух FarDriver-контроллеров текущие MAC уже добавлены в `run.sh`:
  - master: `E0:00:AC:FB:00:23` -> `/tmp/fardriver-master-ble`
  - slave: `C0:05:EA:1E:00:45` -> `/tmp/fardriver-slave-ble`
- По умолчанию `speedometer.py` работает в режиме `CONTROLLER_TYPE=fardriver` и читает оба порта.
- Запуск двух контроллеров одной командой: `./run.sh`.
- Если нужен другой порт: `FARDRIVER_SERIAL_PORT=/dev/rfcomm0 CONTROLLER_TYPE=fardriver python3 speedometer.py`.
- Если скорость с FarDriver выглядит неверно, попробуйте альтернативный источник расчёта скорости: `FARDRIVER_SPEED_SOURCE=wheel python3 speedometer.py`.

Что читается с FarDriver:
- скорость из кадра `0x37` (`Speed / 1000`) или из `MeasureSpeed + AddrD0` при `FARDRIVER_SPEED_SOURCE=wheel`;
- напряжение из `0x37`/`AddrE8`/`AddrEE`;
- батарейный ток из `0x37` (`Curr / 50`) или `AddrE8` (`lineCurrent / 4`);
- фазный ток из `AddrEE` (`PhaseA/PhaseC`, берётся максимум);
- температура контроллера из `AddrD6`;
- температура мотора из `AddrF4`;
- RPM пока берётся из `AddrE2.MeasureSpeed`, потому что публичный реверс FarDriver описывает его именно как измеренную скорость/оборотный показатель.

Важно: VESC-команды ограничения скорости для lock/eco в режиме FarDriver не отправляются. Для ограничения скорости FarDriver нужен отдельный write-протокол параметров, его лучше включать только после проверки на твоей модели контроллера.

### Подключение VESC по Bluetooth
- Запускайте старый режим так: `CONTROLLER_TYPE=vesc VESC_SERIAL_PORT=/dev/rfcomm0 python3 speedometer.py`.
- После привязки VESC через `bluetoothctl` убедитесь, что появилось устройство `/dev/rfcommX` (`ls /dev/rfcomm*`).
- Скрипт автоматически попробует `rfcomm`, если USB-порт недоступен.

### BLE-мосты (если модули только BLE UART)
- Установите `ble-serial`: `pip install --user ble-serial`.
- Для FarDriver: `BLE_DEVICE_MAC=<MAC> ./start_ble_bridge.sh` (порт `/tmp/fardriver-ble`).
- Для BMS: `./start_ble_bms_bridge.sh` (порт `/tmp/bms-ble`, MAC `98:DE:1E:84:80:E6`).
- Пути по умолчанию уже зашиты в `speedometer.py` (`FARDRIVER_PORT_OVERRIDE`, `VESC_PORT_OVERRIDE`, `BMS_PORT_OVERRIDE`).

### Общий запуск
- Выполните `./run.sh`: скрипт стартует оба BLE-моста с задержками, затем запускает `speedometer.py`.
- Выход из Python-приложения автоматически завершает оба BLE-моста.
