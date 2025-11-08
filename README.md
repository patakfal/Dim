# Dim – ESP8266 Mains Dimming Controller

Dim is an ESP8266-based firmware that turns a D1 mini into a mains dimmer with
gesture control, EEPROM-backed brightness memory, and MQTT integration that
keeps Home Assistant (or any MQTT client) in sync with the physical load state.

## Hardware Overview

- **MCU:** Wemos D1 mini / ESP8266.
- **Outputs:** D7 drives the triac gate (active-low PWM up to 1 kHz).
- **Inputs:** D6 senses mains presence through an optocoupler.
- **Storage:** On-board EEPROM stores the last brightness so the light returns to
  the same intensity after a restart.

## Firmware Highlights

- Discrete 20-step brightness curve tuned for perceived linearity.
- Quick-toggle gestures: single tap starts an auto fade, double tap jumps to
  full brightness.
- MQTT state mirroring with cached brightness so Home Assistant stays accurate
  even when mains temporarily drop.
- Configurable minimum “on” level for MQTT OFF commands to keep the load safely lit.
- Doxygen-style documentation throughout `src/main.cpp` and `lib/DimNetwork`.

## Building & Flashing

1. Install [PlatformIO](https://platformio.org/) (VS Code extension or CLI).
2. Connect the D1 mini over USB.
3. From the project root, build and upload:

   ```bash
   platformio run --target upload
   ```

4. Monitor serial output (115200 baud) to verify Wi-Fi/MQTT status:

   ```bash
   platformio device monitor
   ```

The default PlatformIO environment targets the `d1_mini` board and includes the
required `PubSubClient` dependency (see `platformio.ini`).

## Configuration

| Setting | Location | Notes |
| --- | --- | --- |
| Wi-Fi SSID / Password | `src/main.cpp` (`wifiSsid`, `wifiPassword`) | Replace with your network credentials. |
| MQTT broker | `src/main.cpp` (`MQTT_BROKER_HOST` macro) | Set via `build_flags = -DMQTT_BROKER_HOST=\"your.host\"` or edit the default literal. |
| MQTT credentials | `src/main.cpp` (`mqttUsername`, `mqttPassword`) | Optional; leave empty for anonymous brokers. |
| Topics | `lib/DimNetwork/src/DimNetwork.cpp` | Default: `mydevice/light/...` (state/set/brightness/availability). |
| Sense polarity | `lib/DimNetwork/src/DimNetwork.h` (`sensePolarity`) | Flip to `SensePolarity::ActiveLow` if hardware inverts the optocoupler. |

After changing constants, rebuild and flash the firmware.

## MQTT Behavior

- Publishes retained state, brightness, and availability topics so Home
  Assistant can auto-discover the light.
- Accepts payloads on:
  - `mydevice/light/set` (`ON`, `OFF`)
  - `mydevice/light/brightness/set` (`0-100` or `0-100%`)
- OFF commands map to the minimum safe brightness rather than 0 to keep the
  triac biased; adjust `mqttMinBrightnessPercent` if needed.

## Repository Layout

```
├── src/                # Main firmware (`main.cpp`)
├── lib/DimNetwork/     # Wi-Fi + MQTT helper library
├── include/            # Project headers (with DimNetwork shim)
├── test/               # PlatformIO test scaffolding
├── platformio.ini      # Build configuration
└── README.md           # Project documentation
```

## Contributing

Pull requests are welcome! Focus areas include:

- Additional safety features (temperature sensing, brown-out detection).
- Expanded MQTT schema (effects, reporting raw PWM).
- Improved automated testing (unit tests under `test/`).

Open an issue if you encounter bugs or have ideas for new capabilities.
