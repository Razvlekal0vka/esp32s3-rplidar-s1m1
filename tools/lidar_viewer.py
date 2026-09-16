#!/usr/bin/env python3
"""Realtime 2D viewer for ESP32-S3 + RPLIDAR S1M1 binary scan frames.

RU: Протокол кадра оборота (little-endian), 737 байт.
EN: Revolution frame protocol (little-endian), 737 bytes.

  0-3   magic AA 55 4C 44
  4     version = 1
  5     reserved
  6-7   points (uint16)     — сырых точек за оборот / raw points this rev
  8-9   valid (uint16)      — точек с дистанцией / hits with range
  10-11 rpm * 10 (uint16)
  12-13 min_mm (uint16)
  14-15 max_mm (uint16)
  16-735 dist[360] uint16 mm, 0 = нет попадания (бин = floor(угол))
         0 = no hit (bin = floor(angle))
  736   XOR байт [4..735] / XOR of bytes [4..735]
"""

from __future__ import annotations

import argparse
import glob
import math
import struct
import sys
import threading
import time

import serial
from serial.serialutil import SerialException

MAGIC = b"\xAA\x55LD"
FRAME_LEN = 737
HEADER_FMT = "<BBHHHHH"  # version, reserved, pts, valid, rpm_x10, min, max
HEADER_SIZE = struct.calcsize(HEADER_FMT)


def find_port() -> str:
    # Сначала CDC usbmodem, затем CH343 usbserial.
    # Prefer CDC usbmodem, then CH343 usbserial adapters.
    preferred = sorted(glob.glob("/dev/cu.usbmodem*"))
    if preferred:
        return preferred[0]
    fallback = (
        sorted(glob.glob("/dev/cu.wchusbserial*"))
        + sorted(glob.glob("/dev/cu.usbserial*"))
        + sorted(glob.glob("/dev/cu.usb*"))
    )
    if not fallback:
        raise SystemExit("Не найден /dev/cu.usb* — проверьте USB CH343.")
    return fallback[0]


def open_serial(port: str, baud: int) -> serial.Serial:
    # DTR/RTS выкл.: иначе ESP32-S3 уходит в reset при открытии порта.
    # Keep DTR/RTS low: otherwise ESP32-S3 resets when the port is opened.
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.05
    ser.dsrdtr = False
    ser.rtscts = False
    ser.dtr = False
    ser.rts = False
    ser.open()
    ser.reset_input_buffer()
    ser.dtr = False
    ser.rts = False
    return ser


def checksum_ok(pkt: bytes) -> bool:
    cs = 0
    for b in pkt[4:-1]:
        cs ^= b
    return cs == pkt[-1]


def decode_frame(pkt: bytes) -> dict | None:
    if len(pkt) != FRAME_LEN or pkt[:4] != MAGIC or not checksum_ok(pkt):
        return None
    version, _reserved, points, valid, rpm_x10, min_mm, max_mm = struct.unpack_from(
        HEADER_FMT, pkt, 4
    )
    if version != 1:
        return None
    dists = struct.unpack_from("<" + "H" * 360, pkt, 16)
    return {
        "points": points,
        "valid": valid,
        "rpm": rpm_x10 / 10.0,
        "min_mm": min_mm,
        "max_mm": max_mm,
        "dist": dists,
    }


def extract_frames(buf: bytearray) -> list[dict]:
    frames: list[dict] = []
    while True:
        idx = buf.find(MAGIC)
        if idx < 0:
            if len(buf) > 3:
                del buf[:-3]  # хвост magic на границе чтения / keep split magic
            break
        if idx:
            del buf[:idx]
        if len(buf) < FRAME_LEN:
            break
        pkt = bytes(buf[:FRAME_LEN])
        frame = decode_frame(pkt)
        if frame is None:
            del buf[0]  # ложный magic — сдвиг на байт / false magic: skip 1 byte
            continue
        del buf[:FRAME_LEN]
        frames.append(frame)
    return frames


class ScanReader(threading.Thread):
    def __init__(self, ser: serial.Serial):
        super().__init__(daemon=True)
        self.ser = ser
        self.lock = threading.Lock()
        self.latest: dict | None = None
        self.frames = 0
        self.bad = 0
        self.running = True
        self._buf = bytearray()

    def run(self) -> None:
        while self.running:
            try:
                chunk = self.ser.read(2048)
            except SerialException:
                break
            if not chunk:
                continue
            self._buf.extend(chunk)
            parsed = extract_frames(self._buf)
            if not parsed:
                continue
            with self.lock:
                self.latest = parsed[-1]
                self.frames += len(parsed)

    def get_latest(self) -> dict | None:
        with self.lock:
            return self.latest


def polar_xy(dist: tuple[int, ...]) -> tuple[list[float], list[float]]:
    xs: list[float] = []
    ys: list[float] = []
    for deg, mm in enumerate(dist):
        if mm == 0:
            continue
        rad = math.radians(deg)
        # 0° вверх (вперёд), угол по часовой — как у RPLIDAR сверху.
        # 0° up (forward), clockwise — RPLIDAR top-down view.
        xs.append(mm * math.sin(rad))
        ys.append(mm * math.cos(rad))
    return xs, ys


def dump_frames(ser: serial.Serial, count: int, timeout: float) -> int:
    buf = bytearray()
    got = 0
    t0 = time.time()
    while got < count and (time.time() - t0) < timeout:
        chunk = ser.read(2048)
        if chunk:
            buf.extend(chunk)
            for frame in extract_frames(buf):
                got += 1
                bins = sum(1 for d in frame["dist"] if d > 0)
                print(
                    f"frame {got}: rpm={frame['rpm']:.1f} pts={frame['points']} "
                    f"valid={frame['valid']} bins={bins} "
                    f"min={frame['min_mm']} max={frame['max_mm']}"
                )
                if got >= count:
                    break
    return got


def run_viewer(reader: ScanReader, max_range_mm: float | None) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.patches import Circle

    plt.style.use("dark_background")
    fig, ax = plt.subplots(figsize=(8, 8))
    fig.canvas.manager.set_window_title("RPLIDAR S1M1")
    scat = ax.scatter([], [], s=8, c="#7CFF6B", linewidths=0, zorder=3)
    title = ax.set_title("ожидание кадра…")
    ax.set_aspect("equal")
    ax.set_xlabel("x, мм")
    ax.set_ylabel("y, мм")
    rings = [
        Circle((0, 0), r, fill=False, color="#446", lw=0.8, zorder=1)
        for r in (1000, 2000, 5000, 10000, 20000)
    ]
    for ring in rings:
        ax.add_patch(ring)
    ax.axhline(0, color="#335", lw=0.6)
    ax.axvline(0, color="#335", lw=0.6)
    ax.plot([0], [0], marker="+", color="#faa", markersize=10, zorder=4)

    def on_key(event) -> None:
        if event.key in ("q", "Q", "escape"):
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_frame):
        data = reader.get_latest()
        if data is None:
            return (scat,)
        xs, ys = polar_xy(data["dist"])
        scat.set_offsets(list(zip(xs, ys)) if xs else [(0.0, 0.0)])
        if not xs:
            scat.set_offsets([(0.0, 0.0)])
            scat.set_alpha(0.0)
        else:
            scat.set_alpha(1.0)
        span = max_range_mm
        if span is None:
            span = max(data["max_mm"] * 1.1, 2000.0)
        ax.set_xlim(-span, span)
        ax.set_ylim(-span, span)
        title.set_text(
            f"RPM {data['rpm']:.1f}   pts {data['points']}   "
            f"valid {data['valid']}   bins {len(xs)}   "
            f"min {data['min_mm']} mm   frames {reader.frames}"
        )
        return (scat, title)

    _ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    fig._lidar_ani = _ani  # держим ссылку, иначе GC убьёт анимацию / keep from GC
    print("Окно графика: q / Escape / закрытие окна — выход.")
    plt.show()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Realtime RPLIDAR viewer")
    p.add_argument("--port", help="Serial port (default: auto /dev/cu.usbmodem*)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument(
        "--frames",
        type=int,
        default=0,
        help="Parse N frames and exit (no GUI, smoke test)",
    )
    p.add_argument("--timeout", type=float, default=20.0, help="Smoke-test timeout, s")
    p.add_argument(
        "--max-range",
        type=float,
        default=None,
        help="Plot half-extent in mm (default: auto from scan)",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    port = args.port or find_port()
    print(f"port={port} baud={args.baud} dtr=0 rts=0")
    try:
        ser = open_serial(port, args.baud)
    except SerialException as exc:
        print(f"Не удалось открыть {port}: {exc}", file=sys.stderr)
        return 1

    try:
        if args.frames:
            n = dump_frames(ser, args.frames, args.timeout)
            print(f"parsed {n}/{args.frames} frames")
            return 0 if n >= args.frames else 2

        reader = ScanReader(ser)
        reader.start()
        run_viewer(reader, args.max_range)
        reader.running = False
        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
