# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Формат основан на [Keep a Changelog](https://keepachangelog.com/ru/1.1.0/),
проект следует [семантическому версионированию](https://semver.org/lang/ru/).

## [0.1.0] - 2026-09-16

### Added / Добавлено

- ESP32-S3 Arduino firmware (PlatformIO, `esp32-s3-devkitc-1`) for Slamtec RPLIDAR S1M1 over UART 256000 8N1.
  Прошивка ESP32-S3 (Arduino / PlatformIO, плата `esp32-s3-devkitc-1`) для RPLIDAR S1M1 по UART 256000 8N1.
- Working pinout: ESP RX = GPIO16 (lidar TX), ESP TX = GPIO15 (lidar RX); lidar powered from external 5 V ~1 A with common GND; S1 motor is internal (no PWM pin).
  Рабочие пины: ESP RX = GPIO16 (TX лидара), ESP TX = GPIO15 (RX лидара); питание лидара от внешнего 5 V ~1 A, общий GND; мотор S1 встроенный (PWM-пин не нужен).
- Slamtec UART commands: GET_INFO, GET_HEALTH, standard SCAN, typical EXPRESS/DenseBoost, STOP, RESET, motor 600 RPM.
  Команды протокола Slamtec: GET_INFO, GET_HEALTH, стандартный SCAN, typical EXPRESS/DenseBoost, STOP, RESET, мотор 600 RPM.
- Compact PC stream at 115200: binary revolution frame `AA 55 LD` with 360 distance bins (mm) plus a `#rev` text line.
  Компактный поток на ПК (115200): бинарный кадр оборота `AA 55 LD` с 360 бинами дистанций (мм) и текстовая строка `#rev`.
- Realtime viewer `python3 tools/lidar_viewer.py` (pyserial + matplotlib); `--frames` smoke test without GUI.
  Вьюер в реальном времени `python3 tools/lidar_viewer.py` (pyserial + matplotlib); дымовой тест `--frames` без окна.

### Changed / Изменено

- USB Serial uses UART (CDC-on-boot off, DTR/RTS disabled) so CH343 does not reset the board into the bootloader when the viewer opens the port.
  USB Serial идёт через UART (CDC-on-boot выключен, DTR/RTS сброшены), чтобы CH343 не уводил плату в загрузчик при открытии порта вьюером.
