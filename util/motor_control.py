#!/usr/bin/env python3
"""
Motor control utility for hoverboard controllers using the Remote UART Bus Protocol.

This tool communicates with hoverboard motor controller boards running the
Hoverboard-Firmware-Hack-Gen2.x firmware. It sends speed commands and displays
telemetry responses (voltage, current, speed, odometer).

Protocol Reference:
    https://gitlab.com/ailife8881/Hoverboard-Firmware-Hack-Gen2.x-MM32

The protocol uses two packet types:
    - Command (host -> board): 8 bytes, starts with '/' (0x2F)
    - Response (board -> host): 15 bytes, starts with 0xABCD

All multi-byte values are little-endian unless otherwise noted.
"""

import argparse
import serial
import struct
import time
import sys


# =============================================================================
# CRC-16 Calculation
# =============================================================================

def calc_crc(data: bytes) -> int:
    """
    Calculate CRC-16/CCITT checksum for packet validation.

    This is a standard CRC-16 algorithm used to detect transmission errors.
    Both the host and the motor controller calculate this checksum independently
    and compare them to ensure data integrity.

    Algorithm details:
        - Polynomial: 0x1021 (CRC-CCITT)
        - Initial value: 0x0000
        - No final XOR
        - Process each byte MSB first

    How it works:
        1. For each byte in the data:
           a. XOR the byte into the high 8 bits of the CRC
           b. For each of the 8 bits:
              - If the MSB (most significant bit) is 1, shift left and XOR with polynomial
              - Otherwise, just shift left
        2. Return the final 16-bit CRC value

    Args:
        data: The bytes to calculate the checksum for (excludes the CRC field itself)

    Returns:
        16-bit CRC value (0x0000 to 0xFFFF)

    Example:
        >>> calc_crc(bytes([0x2F, 0x00, 0x01, 0x2C, 0x01, 0x00]))
        0x4629
    """
    crc = 0  # Initial CRC value

    for byte in data:
        # XOR the byte into the upper 8 bits of the CRC
        # The << 8 shifts the byte left by 8 bits so it aligns with the MSB of the 16-bit CRC
        crc ^= byte << 8

        # Process each of the 8 bits in the byte
        for _ in range(8):
            # Check if the most significant bit (bit 15) is set
            # 0x8000 in binary is: 1000 0000 0000 0000
            # Using & (bitwise AND) isolates just that bit
            if crc & 0x8000:
                # MSB is 1: shift left by 1 and XOR with the polynomial
                # The polynomial 0x1021 defines the CRC-CCITT standard
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                # MSB is 0: just shift left by 1
                crc = (crc << 1) & 0xFFFF
            # The & 0xFFFF ensures we keep only the lower 16 bits (prevents overflow)

    return crc


# =============================================================================
# Command Packet Building
# =============================================================================

def build_config_command(slave_id: int, drivemode: int = None, newslave: int = None) -> bytes:
    """
    Build a Type 2 (SerialServer2HoverConfig) configuration packet.

    This creates a packet that configures persistent settings on the motor controller.
    Each field has guard conditions in the firmware - invalid values are ignored.

    Packet Structure (15 bytes total):
        Offset  Size  Field       Description
        ------  ----  ----------  -----------
        0       1     cStart      Start marker, always 0x2F ('/')
        1       1     iDataType   Packet type: 2 = config
        2       1     iSlave      Target board ID (0-255)
        3       4     fBattFull   Battery full voltage (float, ignored if <=0 or >=60)
        7       4     fBattEmpty  Battery empty voltage (float, ignored if <=0 or >=60)
        11      1     iDriveMode  Drive mode (0-3, see below)
        12      1     iSlaveNew   New slave ID (-1 to keep, >=0 to change) - DISABLED in firmware
        13      2     checksum    CRC-16 of bytes 0-12

    Drive modes:
        0 = COM_VOLT   (commutation, voltage control)
        1 = COM_SPEED  (commutation, speed/RPM control)
        2 = SINE_VOLT  (sinusoidal, voltage control)
        3 = SINE_SPEED (sinusoidal, speed control)

    Args:
        slave_id: Target board address (0-255)
        drivemode: Drive mode 0-3, or None to leave unchanged
        newslave: New slave ID, or None to leave unchanged (NOTE: disabled in firmware!)

    Returns:
        15-byte config packet ready to send over serial
    """
    # Use out-of-range values for battery to skip those fields
    # Firmware only applies if: (value > 0) && (value < 60.0)
    fBattFull = 0.0   # Will be ignored
    fBattEmpty = 0.0  # Will be ignored

    # Drive mode: 255 (0xFF) is > SINE_SPEED (3), so it will be ignored if not specified
    iDriveMode = drivemode if drivemode is not None else 255

    # New slave ID: -1 means keep current (though this is disabled in firmware anyway)
    iSlaveNew = newslave if newslave is not None else -1

    # Pack the data: < = little-endian, B = uint8, f = float (4 bytes), b = int8
    data = struct.pack(
        "<BBBffBb",
        0x2F,           # cStart
        0x02,           # iDataType = 2 (config)
        slave_id,       # iSlave
        fBattFull,      # fBattFull (ignored)
        fBattEmpty,     # fBattEmpty (ignored)
        iDriveMode,     # iDriveMode
        iSlaveNew       # iSlaveNew
    )

    crc = calc_crc(data)
    return data + struct.pack("<H", crc)


def build_speed_command(slave_id: int, speed: int, state: int = 0) -> bytes:
    """
    Build a Type 0 (SerialServer2Hover) speed command packet.

    This creates an 8-byte packet that tells the motor controller to spin
    the motor at a specific speed. The controller will respond with telemetry.

    Packet Structure (8 bytes total):
        Offset  Size  Field       Description
        ------  ----  ----------  -----------
        0       1     cStart      Start marker, always 0x2F ('/')
        1       1     iDataType   Packet type: 0=speed, 1=master, 2=config
        2       1     iSlave      Target board ID (0-255)
        3       2     iSpeed      Speed setpoint, signed (-1000 to +1000)
        5       1     wState      LED/state flags (see below)
        6       2     checksum    CRC-16 of bytes 0-5

    wState bit flags:
        Bit 0 (0x01): Green LED
        Bit 1 (0x02): Orange LED (or Red+Green)
        Bit 2 (0x04): Red LED
        Bit 3 (0x08): Upper LED
        Bit 4 (0x10): Lower LED
        Bit 5 (0x20): Battery 3 LED
        Bit 6 (0x40): Disable motor
        Bit 7 (0x80): Shut off board

    Args:
        slave_id: Target board address (0-255). Use 1 for default single-board setup.
        speed: Motor speed from -1000 (full reverse) to +1000 (full forward).
               In DRIVEMODE=1 (COM_SPEED), this is the RPM setpoint.
               In DRIVEMODE=0 (COM_VOLT), this maps to PWM duty cycle.
        state: Optional LED/control flags (default 0 = no LEDs, motor enabled)

    Returns:
        8-byte command packet ready to send over serial

    Example:
        >>> cmd = build_speed_command(slave_id=1, speed=300)
        >>> cmd.hex()
        '2f00012c010029b6'
    """
    # struct.pack() converts Python values into raw bytes
    # Format string "<BBBHB" means:
    #   < = little-endian byte order (least significant byte first)
    #   B = unsigned char (1 byte, 0-255)
    #   B = unsigned char (1 byte)
    #   B = unsigned char (1 byte)
    #   H = unsigned short (2 bytes, 0-65535) - used for speed as signed via masking
    #   B = unsigned char (1 byte)
    #
    # Note: speed is int16 but we pack as uint16 with mask. The controller
    # interprets it as signed. For example:
    #   speed=300  -> 0x012C -> bytes [0x2C, 0x01] (little-endian)
    #   speed=-300 -> 0xFED4 -> bytes [0xD4, 0xFE] (two's complement)

    data = struct.pack(
        "<BBBHB",
        0x2F,           # cStart: '/' character marks packet start
        0x00,           # iDataType: 0 = speed command (SerialServer2Hover)
        slave_id,       # iSlave: target board ID
        speed & 0xFFFF, # iSpeed: mask to 16 bits (handles negative via two's complement)
        state           # wState: LED and control flags
    )

    # Calculate CRC over the 6 data bytes (everything except the CRC itself)
    crc = calc_crc(data)

    # Append CRC as little-endian (low byte first)
    # IMPORTANT: The firmware reads CRC with: (last_byte << 8) | second_to_last
    # This means it expects little-endian format in the packet.
    # Using "<H" packs the CRC as [low_byte, high_byte]
    return data + struct.pack("<H", crc)


# =============================================================================
# Response Packet Parsing
# =============================================================================

def parse_response(data: bytes) -> dict | None:
    """
    Parse a SerialHover2Server telemetry response from the motor controller.

    The controller sends this 15-byte packet after receiving a valid command.
    It contains real-time measurements from the board.

    Packet Structure (15 bytes total):
        Offset  Size  Field       Description
        ------  ----  ----------  -----------
        0       2     cStart      Start marker: 0xABCD (little-endian: bytes CD AB)
        2       1     iSlave      Board ID that sent this response
        3       2     iSpeed      Actual motor speed in 0.1 RPM units (divide by 10)
        5       2     iVolt       Battery voltage in centivolts (divide by 100 for volts)
        7       2     iAmp        DC bus current in 10mA units (divide by 100 for amps)
        9       4     iOdom       Odometer: cumulative hall sensor edge count
        13      2     checksum    CRC-16 of bytes 0-12

    Args:
        data: Raw bytes received from serial port (may contain extra bytes)

    Returns:
        Dictionary with parsed values, or None if no valid packet found.
        On CRC error, returns dict with 'error' key.

    Example successful response:
        {
            'slave': 1,
            'speed_rpm': 150.5,
            'voltage_v': 36.72,
            'current_a': 2.35,
            'odometer': 12345
        }
    """
    # Need at least 15 bytes for a complete response
    if len(data) < 15:
        return None

    # Search for the start marker 0xABCD
    # In little-endian byte order, 0xABCD appears as bytes [0xCD, 0xAB]
    # We scan through the buffer because there might be garbage bytes before the packet
    start_idx = -1
    for i in range(len(data) - 14):  # -14 because we need 15 bytes from start
        if data[i] == 0xCD and data[i + 1] == 0xAB:
            start_idx = i
            break

    if start_idx < 0:
        # No valid start marker found
        return None

    # Extract the 15-byte packet starting at the marker
    packet = data[start_idx : start_idx + 15]

    # Verify CRC to ensure packet wasn't corrupted in transit
    # CRC is calculated over first 13 bytes (everything except the 2-byte CRC)
    payload = packet[:13]

    # CRC is stored as little-endian in the packet
    # struct.unpack("<H", ...) reads 2 bytes as little-endian unsigned short
    received_crc = struct.unpack("<H", packet[13:15])[0]
    calculated_crc = calc_crc(payload)

    if received_crc != calculated_crc:
        return {
            "error": f"CRC mismatch: received 0x{received_crc:04X}, calculated 0x{calculated_crc:04X}"
        }

    # Unpack all fields from the packet
    # Format string "<HBhHhi" means:
    #   < = little-endian
    #   H = unsigned short (2 bytes) - cStart marker
    #   B = unsigned char (1 byte) - slave ID
    #   h = signed short (2 bytes) - speed (can be negative for reverse)
    #   H = unsigned short (2 bytes) - voltage (always positive)
    #   h = signed short (2 bytes) - current (can be negative for regen braking)
    #   i = signed int (4 bytes) - odometer (can go negative if run in reverse)
    _, slave, speed_raw, volt, amp, odom = struct.unpack("<HBhHhi", packet[:13])

    # Convert raw values to human-readable units
    return {
        "slave": slave,
        "speed_rpm": speed_raw / 10.0,   # Raw is in 0.1 RPM units
        "voltage_v": volt / 100.0,       # Raw is in centivolts (100 * V)
        "current_a": amp / 100.0,        # Raw is in 10mA units
        "odometer": odom,                # Raw hall edge count (no conversion)
    }


# =============================================================================
# Main Entry Point
# =============================================================================

def main():
    """
    Main function: parse arguments, connect to serial port, and run command loop.
    """
    parser = argparse.ArgumentParser(
        description="""
Motor control utility for hoverboard controllers.

Sends speed commands to a hoverboard motor controller board using the
Remote UART Bus Protocol and displays telemetry responses.

The board must be running Hoverboard-Firmware-Hack-Gen2.x firmware with
valid EEPROM configuration (run PinFinder first to configure pins).

Examples:
    %(prog)s /dev/ttyUSB0                    # Basic connection, default speed 300
    %(prog)s /dev/ttyUSB0 -S 500             # Set speed to 500
    %(prog)s /dev/ttyUSB0 -S -300            # Reverse at speed 300
    %(prog)s /dev/ttyUSB0 -s 2               # Talk to slave ID 2
    %(prog)s /dev/ttyUSB0 --state 0x40       # Send with motor disabled
    %(prog)s /dev/ttyUSB0 --drivemode 2      # Set SINE_VOLT mode (no motor spin)
    %(prog)s /dev/ttyUSB0 -s 2 --drivemode 3 # Set slave 2 to SINE_SPEED mode
        """,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "port",
        nargs="?",  # Makes it optional so we can show help when missing
        help="Serial port device (e.g., /dev/ttyUSB0, /dev/cu.usbserial-XXX, COM3)"
    )
    parser.add_argument(
        "-b", "--baud",
        type=int,
        default=19200,
        help="Baud rate (default: 19200, must match firmware BAUD setting)"
    )
    parser.add_argument(
        "-s", "--slave",
        type=int,
        default=1,
        help="Slave ID of target board (default: 1, range: 0-255)"
    )
    parser.add_argument(
        "-S", "--speed",
        type=int,
        default=300,
        help="Speed setpoint (default: 300, range: -1000 to +1000)"
    )
    parser.add_argument(
        "-i", "--interval",
        type=float,
        default=0.4,
        help="Time between commands in seconds (default: 0.4)"
    )
    parser.add_argument(
        "-t", "--timeout",
        type=float,
        default=0.1,
        help="Serial read timeout in seconds (default: 0.1)"
    )
    parser.add_argument(
        "--state",
        type=lambda x: int(x, 0),  # Allows hex input like 0x40
        default=0,
        help="wState flags byte (default: 0, supports hex like 0x40)"
    )
    parser.add_argument(
        "--drivemode",
        type=int,
        choices=[0, 1, 2, 3],
        help="Set drive mode and exit: 0=COM_VOLT, 1=COM_SPEED, 2=SINE_VOLT, 3=SINE_SPEED"
    )
    parser.add_argument(
        "--newslave",
        type=int,
        help="Set new slave ID (0-255) and exit. NOTE: Currently disabled in firmware!"
    )

    args = parser.parse_args()

    # If no port specified, print help and exit (same as -h)
    if args.port is None:
        parser.print_help()
        sys.exit(0)

    # Validate speed is within protocol limits
    # The motor controller clamps values but it's good to warn the user
    if not -1000 <= args.speed <= 1000:
        print(f"Error: speed must be between -1000 and 1000, got {args.speed}", file=sys.stderr)
        sys.exit(1)

    # Validate slave ID fits in 1 byte
    if not 0 <= args.slave <= 255:
        print(f"Error: slave ID must be between 0 and 255, got {args.slave}", file=sys.stderr)
        sys.exit(1)

    # Validate newslave if provided
    if args.newslave is not None and not 0 <= args.newslave <= 255:
        print(f"Error: new slave ID must be between 0 and 255, got {args.newslave}", file=sys.stderr)
        sys.exit(1)

    # Config mode: send config packet and exit
    if args.drivemode is not None or args.newslave is not None:
        drivemode_names = ["COM_VOLT", "COM_SPEED", "SINE_VOLT", "SINE_SPEED"]

        print(f"Connecting to {args.port} at {args.baud} baud...")
        print(f"Sending config packet to slave {args.slave}:")
        if args.drivemode is not None:
            print(f"  Drive mode: {args.drivemode} ({drivemode_names[args.drivemode]})")
        if args.newslave is not None:
            print(f"  New slave ID: {args.newslave} (WARNING: disabled in firmware!)")

        try:
            ser = serial.Serial(
                port=args.port,
                baudrate=args.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.5,  # Longer timeout for config
            )
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}", file=sys.stderr)
            sys.exit(1)

        cmd = build_config_command(args.slave, drivemode=args.drivemode, newslave=args.newslave)
        print(f"\nConfig packet ({len(cmd)} bytes): {cmd.hex()}")
        print("-" * 60)

        ser.write(cmd)
        time.sleep(0.2)  # Give firmware time to process and respond

        response_data = ser.read(32)
        if response_data:
            print(f"Raw response ({len(response_data)} bytes): {response_data.hex()}")
            parsed = parse_response(response_data)
            if parsed:
                if "error" in parsed:
                    print(f"Parse error: {parsed['error']}")
                else:
                    print(f"\nParsed response:")
                    print(f"  Slave ID:  {parsed['slave']}")
                    print(f"  Speed:     {parsed['speed_rpm']:.1f} RPM")
                    print(f"  Voltage:   {parsed['voltage_v']:.2f} V")
                    print(f"  Current:   {parsed['current_a']:.2f} A")
                    print(f"  Odometer:  {parsed['odometer']}")
            else:
                print("Could not parse response (no valid packet found)")
        else:
            print("No response received")

        ser.close()
        print("\nConfig sent. Settings saved to EEPROM.")
        sys.exit(0)

    print(f"Connecting to {args.port} at {args.baud} baud...")
    print(f"Slave ID: {args.slave}, Speed: {args.speed}, Interval: {args.interval}s")
    print("Press Ctrl+C to stop\n")

    # Open serial port connection
    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,   # 8 data bits (standard)
            parity=serial.PARITY_NONE,   # No parity bit
            stopbits=serial.STOPBITS_ONE, # 1 stop bit
            timeout=args.timeout,         # Read timeout
        )
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}", file=sys.stderr)
        sys.exit(1)

    # Ramping: start at 0 and move toward target speed by 10 each iteration
    current_speed = 0
    target_speed = args.speed
    ramp_step = 50

    print(f"Target speed: {target_speed}, ramping by {ramp_step} per message")
    print("-" * 60)

    # Main command/response loop
    try:
        count = 0
        while True:
            # Ramp toward target speed
            if current_speed < target_speed:
                current_speed = min(current_speed + ramp_step, target_speed)
            elif current_speed > target_speed:
                current_speed = max(current_speed - ramp_step, target_speed)

            # Build command with current (ramped) speed
            cmd = build_speed_command(args.slave, current_speed, args.state)

            # Send the speed command to the motor controller
            ser.write(cmd)
            count += 1

            # Read response from the controller
            # We read more bytes than needed (32) to ensure we capture the full
            # 15-byte response even if there's some buffered data
            response_data = ser.read(32)

            # Status indicator for ramping
            if current_speed == target_speed:
                ramp_status = "HOLD"
            else:
                ramp_status = "RAMP"

            if response_data:
                # Try to parse the response
                parsed = parse_response(response_data)
                if parsed:
                    if "error" in parsed:
                        # CRC mismatch or other parsing error
                        print(f"[{count:4d}] {ramp_status} cmd:{current_speed:5d} | Response error: {parsed['error']}")
                    else:
                        # Successfully parsed - display telemetry
                        print(
                            f"[{count:4d}] {ramp_status} cmd:{current_speed:5d} | "
                            f"Speed: {parsed['speed_rpm']:6.1f} RPM | "
                            f"Voltage: {parsed['voltage_v']:5.2f} V | "
                            f"Current: {parsed['current_a']:6.2f} A | "
                            f"Odometer: {parsed['odometer']}"
                        )
                else:
                    # Received data but couldn't find valid packet structure
                    # Show raw hex for debugging
                    print(f"[{count:4d}] {ramp_status} cmd:{current_speed:5d} | Raw: {response_data.hex()}")
            else:
                # No data received within timeout period
                print(f"[{count:4d}] {ramp_status} cmd:{current_speed:5d} | No response")

            # Wait before sending next command
            time.sleep(args.interval)

    except KeyboardInterrupt:
        # User pressed Ctrl+C - ramp down to stop
        print("\n\nRamping down to stop...")
        stop_ramp_step = 50  # Faster ramp for stopping
        while current_speed != 0:
            if current_speed > 0:
                current_speed = max(current_speed - stop_ramp_step, 0)
            else:
                current_speed = min(current_speed + stop_ramp_step, 0)
            cmd = build_speed_command(args.slave, current_speed, args.state)
            ser.write(cmd)
            print(f"  Ramping down: {current_speed}")
            time.sleep(0.05)
        # Send a few more zero commands to ensure motor stops
        stop_cmd = build_speed_command(args.slave, 0, args.state)
        for _ in range(3):
            ser.write(stop_cmd)
            time.sleep(0.05)
        print("Motor stopped.")
    finally:
        # Always close the serial port
        ser.close()


if __name__ == "__main__":
    main()
