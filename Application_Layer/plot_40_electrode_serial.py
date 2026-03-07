#!/usr/bin/env python3
"""Live plot of 40 EIT electrode measurements from serial output."""

import argparse
import sys
import time
from typing import Optional

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial
from serial import SerialException
from serial.tools import list_ports

NUM_CHANNELS = 40
DEFAULT_BAUD = 115200
DEFAULT_TIMEOUT = 1.0
PREFERRED_PORT_PREFIXES = (
    "/dev/cu.usbmodem",
    "/dev/tty.usbmodem",
    "/dev/cu.usbserial",
    "/dev/tty.usbserial",
)


def select_port(user_port: Optional[str]) -> str:
    if user_port:
        return user_port

    ports = [port.device for port in list_ports.comports()]
    for prefix in PREFERRED_PORT_PREFIXES:
        for port in ports:
            if port.startswith(prefix):
                return port

    if ports:
        return ports[0]

    raise RuntimeError("No serial ports found. Connect the device or pass --port explicitly.")


def open_serial(port: str, baud: int, timeout: float) -> serial.Serial:
    return serial.Serial(port, baud, timeout=timeout)


def parse_frame(raw: str) -> Optional[list[int]]:
    text = raw.strip()
    if not text:
        return None

    upper = text.upper()
    if upper == "TARE" or upper.startswith("TARE "):
        return None

    parts = text.replace(",", " ").split()
    if len(parts) < NUM_CHANNELS:
        return None

    try:
        values = [int(float(part)) for part in parts[:NUM_CHANNELS]]
    except ValueError:
        return None

    return values


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot the latest 40 electrode values from ESP serial output"
    )
    parser.add_argument("--port", default=None, help="Serial port. Auto-detected when omitted.")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate")
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT,
        help="Serial read timeout in seconds",
    )
    args = parser.parse_args()

    try:
        port = select_port(args.port)
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        sys.exit(1)

    print(f"Using serial port: {port}")

    fig, ax = plt.subplots(figsize=(14, 6))
    x_values = list(range(1, NUM_CHANNELS + 1))
    y_values = [0] * NUM_CHANNELS

    bars = ax.bar(x_values, y_values, color="#4c78a8", alpha=0.75)
    line, = ax.plot(x_values, y_values, color="#f58518", marker="o", linewidth=2)

    ax.set_title("Live 40-Channel EIT Measurements")
    ax.set_xlabel("Channel")
    ax.set_ylabel("Value")
    ax.set_xticks(x_values)
    ax.set_xlim(0.25, NUM_CHANNELS + 0.75)
    ax.grid(True, axis="y", linestyle=":", alpha=0.5)

    status_text = fig.text(0.01, 0.01, "Connecting...", ha="left", va="bottom")

    serial_state = {"handle": None}

    def set_status(message: str) -> None:
        status_text.set_text(message)

    def ensure_serial() -> Optional[serial.Serial]:
        if serial_state["handle"] is not None and serial_state["handle"].is_open:
            return serial_state["handle"]

        try:
            serial_state["handle"] = open_serial(port, args.baud, args.timeout)
            time.sleep(2)
            serial_state["handle"].reset_input_buffer()
            set_status(f"Connected to {port} @ {args.baud}")
            print(f"Connected to {port} @ {args.baud}")
            return serial_state["handle"]
        except SerialException as exc:
            set_status(f"Waiting for serial device: {exc}")
            return None

    def update(_frame_index: int):
        ser = ensure_serial()
        if ser is None:
            time.sleep(0.5)
            return (*bars, line)

        try:
            raw = ser.readline().decode("utf-8", errors="ignore")
        except SerialException as exc:
            set_status(f"Serial disconnected: {exc}")
            print(f"Serial disconnected: {exc}")
            try:
                ser.close()
            except SerialException:
                pass
            serial_state["handle"] = None
            return (*bars, line)

        frame_values = parse_frame(raw)
        if frame_values is None:
            return (*bars, line)

        max_value = max(frame_values) if frame_values else 1
        top = max(10, int(max_value * 1.15))
        ax.set_ylim(0, top)

        for bar, value in zip(bars, frame_values):
            bar.set_height(value)

        line.set_ydata(frame_values)
        set_status(
            f"Latest frame received | min={min(frame_values)} max={max(frame_values)} mean={sum(frame_values) / len(frame_values):.1f}"
        )
        return (*bars, line)

    animation = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    _ = animation

    try:
        plt.tight_layout(rect=(0, 0.04, 1, 1))
        plt.show()
    finally:
        if serial_state["handle"] is not None and serial_state["handle"].is_open:
            serial_state["handle"].close()
            print("Serial closed.")


if __name__ == "__main__":
    main()
