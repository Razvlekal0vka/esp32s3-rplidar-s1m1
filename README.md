# ESP32-S3 + RPLIDAR S1M1

Firmware that talks Slamtec UART to an RPLIDAR S1M1 and streams one compact binary scan frame per revolution to a PC viewer.

Прошивка UART-протокола Slamtec для RPLIDAR S1M1: info/health, поток скана и бинарный кадр оборота на USB Serial.

Version / версия: see [`VERSION`](VERSION) (`0.1.0`). Changelog: [`CHANGELOG.md`](CHANGELOG.md).

---

## Русский

### Что это

PlatformIO (Arduino, плата `esp32-s3-devkitc-1`): ESP32-S3 читает лидар на **256000 8N1** и шлёт на ПК по USB Serial **115200**. После старта запрашиваются `GET_INFO` / `GET_HEALTH`, мотор ставится на 600 RPM, запускается typical scan (у S1 обычно DenseBoost ~9.2 кГц). Каждый оборот — бинарный кадр `AA 55 LD` (360 бинов) и строка `#rev`.

### Подключение

UART лидара — **3.3 V TTL, 256000 8N1**. Рабочие пины этого стенда:

| S1M1 | ESP32-S3 |
| --- | --- |
| TX | GPIO **16** (ESP RX) |
| RX | GPIO **15** (ESP TX) |
| GND | GND (общий) |
| 5V (ядро + мотор) | внешние **5 V ~1 A**, не с GPIO |

Мотор у S1 **встроенный**: PWM-пин адаптера не нужен, вращение задаётся командой `0xA8` (по умолчанию 600 RPM) и стартом скана.

Питание лидара — отдельный 5 V порядка 1 A. Общий GND с ESP обязателен.

### Сборка и прошивка

```bash
pio run -t upload
```

Монитор (необязательно): `pio device monitor` — 115200, DTR/RTS выключены в `platformio.ini`.

USB CDC on boot отключён (`ARDUINO_USB_CDC_ON_BOOT=0`): консоль идёт через UART-мост (часто CH343), не native USB. Если монитор пустой — другой USB-порт платы.

### Вьюер на ноутбуке

Каждый оборот ESP шлёт **737 байт**, little-endian:

| Смещение | Содержимое |
| --- | --- |
| 0–3 | magic `AA 55 4C 44` (`AA 55 LD`) |
| 4 | version = 1 |
| 5 | reserved |
| 6–7 | points (`uint16`) |
| 8–9 | valid (`uint16`) |
| 10–11 | RPM × 10 (`uint16`) |
| 12–13 | min_mm (`uint16`) |
| 14–15 | max_mm (`uint16`) |
| 16–735 | `dist[360]` `uint16` мм, `0` = нет попадания, бин = `floor(угол)` |
| 736 | XOR байтов `[4..735]` |

После кадра — текстовая строка `#rev` (RPM, точки, min/max).

```bash
python3 -m pip install --user -r tools/requirements.txt
python3 tools/lidar_viewer.py
```

Порт по умолчанию — первый `/dev/cu.usbmodem*`. DTR/RTS выключены, чтобы CH343 не сбросил ESP в загрузчик. Выход: `q`, Escape или закрытие окна.

Проверка протокола без окна:

```bash
python3 tools/lidar_viewer.py --frames 5
```

Опции: `--port /dev/cu.usbmodem…`, `--baud 115200`, `--max-range 8000`.

### Serial-команды

115200, перевод строки не важен. Символ в Serial Monitor:

| Клавиша | Действие |
| --- | --- |
| `i` | GET_INFO (модель / прошивка / серийник) |
| `h` | GET_HEALTH (`ok` / warning / protection-stop) |
| `s` | стандартный SCAN (~4.6 кГц) |
| `e` | typical EXPRESS (у S1 обычно DenseBoost ~9.2 кГц) |
| `x` | STOP |
| `r` | RESET |
| `v` | CSV всех точек (`угол,мм,качество`) |
| `m` | мотор 600 RPM |
| `?` | справка |

### Если нет связи

- TX/RX: **TX лидара → GPIO16**, **RX лидара → GPIO15**
- общий GND
- лидар на внешних 5 V ~1 A
- baud лидара только **256000**, не 115200 (115200 — только ПК ↔ ESP)

---

## English

### What this is

PlatformIO Arduino firmware (`esp32-s3-devkitc-1`) for ESP32-S3 + Slamtec RPLIDAR S1M1. The lidar UART is **256000 8N1**; the PC USB serial link is **115200**. On boot the firmware runs `GET_INFO` / `GET_HEALTH`, sets the motor to 600 RPM, and starts the typical scan (on S1 this is usually DenseBoost ~9.2 kHz). Each revolution is forwarded as a binary `AA 55 LD` frame (360 bins) plus a `#rev` text line.

### Wiring

Lidar UART is **3.3 V TTL, 256000 8N1**. Pins that work on this bench:

| S1M1 | ESP32-S3 |
| --- | --- |
| TX | GPIO **16** (ESP RX) |
| RX | GPIO **15** (ESP TX) |
| GND | GND (common) |
| 5V (core + motor) | external **5 V ~1 A**, not from GPIO |

The S1 motor is **internal**: no adapter PWM pin. Speed is set with command `0xA8` (default 600 RPM); rotation starts with the scan command.

Use a dedicated 5 V supply (~1 A) for the lidar. Share GND with the ESP.

### Build and flash

```bash
pio run -t upload
```

Optional serial monitor: `pio device monitor` — 115200, DTR/RTS off in `platformio.ini`.

USB CDC on boot is disabled (`ARDUINO_USB_CDC_ON_BOOT=0`): console goes through the UART bridge (often CH343), not native USB. Empty monitor usually means the other USB port on the board.

### PC viewer

Each revolution is **737 bytes**, little-endian:

| Offset | Payload |
| --- | --- |
| 0–3 | magic `AA 55 4C 44` (`AA 55 LD`) |
| 4 | version = 1 |
| 5 | reserved |
| 6–7 | points (`uint16`) |
| 8–9 | valid (`uint16`) |
| 10–11 | RPM × 10 (`uint16`) |
| 12–13 | min_mm (`uint16`) |
| 14–15 | max_mm (`uint16`) |
| 16–735 | `dist[360]` `uint16` mm, `0` = no hit, bin = `floor(angle)` |
| 736 | XOR of bytes `[4..735]` |

A `#rev` text line (RPM, points, min/max) follows the frame.

```bash
python3 -m pip install --user -r tools/requirements.txt
python3 tools/lidar_viewer.py
```

Default port is the first `/dev/cu.usbmodem*`. DTR/RTS are held low so CH343 does not reset the ESP into the bootloader. Quit with `q`, Escape, or closing the window.

Protocol smoke test (no GUI):

```bash
python3 tools/lidar_viewer.py --frames 5
```

Options: `--port /dev/cu.usbmodem…`, `--baud 115200`, `--max-range 8000`.

### Serial commands

115200; newline does not matter. Send a character in Serial Monitor:

| Key | Action |
| --- | --- |
| `i` | GET_INFO (model / firmware / serial) |
| `h` | GET_HEALTH (`ok` / warning / protection-stop) |
| `s` | standard SCAN (~4.6 kHz) |
| `e` | typical EXPRESS (S1 usually DenseBoost ~9.2 kHz) |
| `x` | STOP |
| `r` | RESET |
| `v` | CSV of all points (`angle,mm,quality`) |
| `m` | motor 600 RPM |
| `?` | help |

### No link

- TX/RX: **lidar TX → GPIO16**, **lidar RX → GPIO15**
- common GND
- lidar on external 5 V ~1 A
- lidar baud is **256000** only, not 115200 (115200 is PC ↔ ESP)
