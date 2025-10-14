Спидометр с озвучкой и голосовыми командами для контроллеров на базе VESC и Smart BMS ANT

Скрипт для автозапуска на Raspbery PI 4:
[Desktop Entry]
Name=My Python Script
Comment=Запуск скрипта
Exec=python3 Documents/vesc_ant_speedometer/white_vesc.py
Icon=utilities-terminal
Terminal=true
Type=Application
Encoding=UTF-8
Categories=Utility;


chmod +x ~/Desktop/MyScript.desktop

### Подключение VESC по Bluetooth
- После привязки VESC через `bluetoothctl` убедитесь, что появилось устройство `/dev/rfcommX` (`ls /dev/rfcomm*`).
- Укажите порт (переменная окружения `VESC_SERIAL_PORT=/dev/rfcomm0` или измените `VESC_PORT_OVERRIDE` в `white_vesc.py`).
- Скрипт автоматически попробует `rfcomm`, если USB-порт недоступен.
