# ESP32 Mirror Control

Firmware for the mirror actuator helper that talks to the Raspberry Pi speedometer over Bluetooth classic (SPP) and drives two servos through an ESP32 board. Build it with the Arduino IDE and pair the module with the Pi, then place the resulting binary on the ESP32.

## Features
- Two servo channels (left / right mirror) with configurable pins and limits.
- Bluetooth SPP service named `MirrorControl`; understands the same protocol as the Python dashboard.
- Text based commands (`SET`, `GET`, `PING`, `DELTA`) with numeric degree values.
- Sends back acknowledgements (`ANGLE`, `HELLO`, `PONG`) that the dashboard uses to show the actual position.

## Hardware defaults
- Left servo pin: `GPIO25`
- Right servo pin: `GPIO26`
- PWM period: 50 Hz (standard servo)
- Pulse range: 500–2400 µs (adjust if your servo needs a different range)
- Allowed angle range: 60°–150°
- Startup angle: 120° for both channels
- Optional fold button: `GPIO33` pulled to ground; short press toggles between folded and unfolded poses

All of these can be changed at the top of `mirror_control_esp32.ino`.

## Required Arduino libraries
- `ESP32Servo` (install from the Arduino Library Manager)

## Build and upload
1. Install the ESP32 board support package in the Arduino IDE.
2. Place this folder inside your sketches directory (or open the `.ino` directly).
3. Adjust the constants to match your wiring and mechanical limits.
4. Compile and upload to the ESP32.

## Bluetooth pairing
1. Power up the ESP32; it will advertise as `MirrorControl`.
2. Pair it with the Raspberry Pi (or whatever runs the dashboard).
3. Put the device MAC address into `mirror_config.json` (`bt_address` field) so the Python code can connect.

## Serial protocol
Commands are ASCII lines terminated by `\n`.

```
SET L 115        # Set left mirror to 115°
SET R 95         # Set right mirror to 95°
GET ALL          # Report both positions
GET L            # Report left position
PING             # Health check
DELTA L -5       # Optional: move left by -5 degrees
POSE FOLDED 120 120   # Update stored "folded" pose
POSE UNFOLDED 130 130 # Update stored "unfolded" pose
```

Responses follow the same format:

```
HELLO 120 120    # Sent once on connect
ANGLE L 115      # Current left position
ANGLE R 95       # Current right position
POSE FOLDED 120 120   # Acknowledgement with clamped pose values
PONG             # Reply to PING
ERR <reason>     # Error description
```

These messages match what the speedometer UI expects.
