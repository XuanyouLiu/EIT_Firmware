#!/usr/bin/env python3
import argparse
import re
import time
from collections import deque

import matplotlib.pyplot as plt
import serial
from serial.tools import list_ports

ADC_PATTERN = re.compile(r"ADC:\s*(\d+)")


def select_port(user_port: str | None) -> str:
    if user_port:
        return user_port

    ports = [p.device for p in list_ports.comports()]

    for pattern in ("/dev/cu.usbmodem", "/dev/tty.usbmodem", "/dev/cu.usbserial", "/dev/tty.usbserial"):
        for port in ports:
            if port.startswith(pattern):
                return port

    if ports:
        return ports[0]

    raise RuntimeError("No serial ports found. Connect device or pass --port explicitly.")


def open_serial(port: str, baud: int) -> serial.Serial:
    return serial.Serial(port, baud, timeout=1)


def main() -> None:
    parser = argparse.ArgumentParser(description="Live plot ADC values from ESP serial output")
    parser.add_argument("--port", default=None, help="Serial port (example: /dev/tty.usbserial-0001)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--window", type=int, default=500, help="Number of recent samples to show")
    parser.add_argument("--headless", action="store_true", help="Print parsed ADC values without opening plot window")
    args = parser.parse_args()

    port = select_port(args.port)
    print(f"Using serial port: {port}")
    while True:
        try:
            ser = open_serial(port, args.baud)
            break
        except serial.SerialException as exc:
            print(f"Waiting for serial port ({exc})")
            time.sleep(0.5)

    x_vals = deque(maxlen=args.window)
    y_vals = deque(maxlen=args.window)
    sample_idx = 0

    if not args.headless:
        plt.ion()
        fig, ax = plt.subplots(figsize=(10, 4))
        line, = ax.plot([], [], lw=1)
        ax.set_title("ADC over Time")
        ax.set_xlabel("Sample")
        ax.set_ylabel("ADC Value")
        ax.grid(True)

    print("Listening for ADC lines...")

    try:
        while True:
            try:
                raw = ser.readline().decode("utf-8", errors="ignore").strip()
            except serial.SerialException:
                try:
                    ser.close()
                except Exception:
                    pass
                time.sleep(0.5)
                while True:
                    try:
                        ser = open_serial(port, args.baud)
                        break
                    except serial.SerialException:
                        time.sleep(0.5)
                continue

            if not raw:
                continue

            match = ADC_PATTERN.search(raw)
            if not match:
                continue

            value = int(match.group(1))
            x_vals.append(sample_idx)
            y_vals.append(value)
            sample_idx += 1

            if sample_idx % 100 == 0:
                print(f"samples={sample_idx} last={value}")

            if args.headless:
                print(f"ADC: {value}")
                continue

            line.set_data(x_vals, y_vals)
            ax.relim()
            ax.autoscale_view()
            plt.pause(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if not args.headless:
            plt.ioff()
            plt.show()


if __name__ == "__main__":
    main()
