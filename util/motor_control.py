#!/usr/bin/env python3
"""
Motor control utility for hoverboard controllers using the Remote UART Bus Protocol.
Sends speed commands in a loop and displays telemetry responses.
"""

import argparse
import serial
import struct
import time
import sys


def calc_crc(data: bytes) -> int:
    """Calculate CRC-16/CCITT (polynomial 0x1021, init 0)."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_speed_command(slave_id: int, speed: int, state: int = 0) -> bytes:
    """Build a Type 0 speed command packet."""
    # Pack: start(1) + type(1) + slave(1) + speed(2, little-endian) + state(1)
    data = struct.pack("<BBBHB", 0x2F, 0x00, slave_id, speed & 0xFFFF, state)
    crc = calc_crc(data)
    # CRC is big-endian
    return data + struct.pack(">H", crc)


def parse_response(data: bytes) -> dict | None:
    """Parse a SerialHover2Server response (15 bytes)."""
    if len(data) < 15:
        return None

    # Find start marker 0xABCD (little-endian: 0xCD 0xAB)
    start_idx = -1
    for i in range(len(data) - 14):
        if data[i] == 0xCD and data[i + 1] == 0xAB:
            start_idx = i
            break

    if start_idx < 0:
        return None

    packet = data[start_idx : start_idx + 15]

    # Verify CRC
    payload = packet[:13]
    expected_crc = struct.unpack(">H", packet[13:15])[0]
    actual_crc = calc_crc(payload)

    if expected_crc != actual_crc:
        return {"error": f"CRC mismatch: expected {expected_crc:04X}, got {actual_crc:04X}"}

    # Unpack response
    # cStart(2) + slave(1) + speed(2) + volt(2) + amp(2) + odom(4) + crc(2)
    _, slave, speed_raw, volt, amp, odom = struct.unpack("<HBhHhi", packet[:13])

    return {
        "slave": slave,
        "speed_rpm": speed_raw / 10.0,
        "voltage_v": volt / 1000.0,
        "current_a": amp / 100.0,
        "odometer": odom,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Send motor speed commands and display telemetry",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("port", help="Serial port (e.g., /dev/cu.usbserial-A50285BI)")
    parser.add_argument("-b", "--baud", type=int, default=19200, help="Baud rate")
    parser.add_argument("-s", "--slave", type=int, default=1, help="Slave ID (0-255)")
    parser.add_argument("-S", "--speed", type=int, default=300, help="Speed setpoint (-1000 to 1000)")
    parser.add_argument("-i", "--interval", type=float, default=0.4, help="Command interval in seconds")
    parser.add_argument("-t", "--timeout", type=float, default=0.1, help="Serial read timeout in seconds")
    parser.add_argument("--state", type=int, default=0, help="State flags (wState byte)")

    args = parser.parse_args()

    # Validate speed range
    if not -1000 <= args.speed <= 1000:
        print(f"Error: speed must be between -1000 and 1000", file=sys.stderr)
        sys.exit(1)

    print(f"Connecting to {args.port} at {args.baud} baud...")
    print(f"Slave ID: {args.slave}, Speed: {args.speed}, Interval: {args.interval}s")
    print("Press Ctrl+C to stop\n")

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=args.timeout,
        )
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}", file=sys.stderr)
        sys.exit(1)

    cmd = build_speed_command(args.slave, args.speed, args.state)
    print(f"Command bytes: {cmd.hex()}")
    print("-" * 60)

    try:
        count = 0
        while True:
            # Send command
            ser.write(cmd)
            count += 1

            # Read response (may need to read multiple times to get full packet)
            response_data = ser.read(32)  # Read more than needed to catch response

            if response_data:
                parsed = parse_response(response_data)
                if parsed:
                    if "error" in parsed:
                        print(f"[{count:4d}] Response error: {parsed['error']}")
                    else:
                        print(
                            f"[{count:4d}] "
                            f"Speed: {parsed['speed_rpm']:6.1f} RPM | "
                            f"Voltage: {parsed['voltage_v']:5.2f} V | "
                            f"Current: {parsed['current_a']:6.2f} A | "
                            f"Odometer: {parsed['odometer']}"
                        )
                else:
                    print(f"[{count:4d}] Raw: {response_data.hex()}")
            else:
                print(f"[{count:4d}] No response")

            time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\n\nStopping motor...")
        # Send stop command
        stop_cmd = build_speed_command(args.slave, 0, args.state)
        ser.write(stop_cmd)
        time.sleep(0.1)
        print("Motor stopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
