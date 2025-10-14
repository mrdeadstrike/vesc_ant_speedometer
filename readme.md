Спидометр с озвучкой и голосовыми командами для контроллеров на базе VESC и Smart BMS ANT

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

### Подключение VESC по Bluetooth
- После привязки VESC через `bluetoothctl` убедитесь, что появилось устройство `/dev/rfcommX` (`ls /dev/rfcomm*`).
- Укажите порт (переменная окружения `VESC_SERIAL_PORT=/dev/rfcomm0` или измените `VESC_PORT_OVERRIDE` в `speedometer.py`).
- Скрипт автоматически попробует `rfcomm`, если USB-порт недоступен.

### BLE-мосты (если модули только BLE UART)
- Установите `ble-serial`: `pip install --user ble-serial`.
- Для VESC: `./start_ble_bridge.sh` (порт `/tmp/vesc-ble`, MAC `F5:00:47:10:37:D2`).
- Для BMS: `./start_ble_bms_bridge.sh` (порт `/tmp/bms-ble`, MAC `98:DE:1E:84:80:E6`).
- Пути по умолчанию уже зашиты в `speedometer.py` (`VESC_PORT_OVERRIDE`, `BMS_PORT_OVERRIDE`).

### Общий запуск
- Выполните `./run.sh`: скрипт стартует оба BLE-моста с задержками, затем запускает `speedometer.py`.
- Выход из Python-приложения автоматически завершает оба BLE-моста.
