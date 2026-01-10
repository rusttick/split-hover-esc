# ArduPilot Telemetry over WiFi

This document covers how to send telemetry data to ArduPilot from a Raspberry Pi Pico W,
the MAVLink data structures ArduPilot expects,
and all message types it can process or log.

## Overview

ArduPilot uses the **MAVLink** protocol for all telemetry communication.
MAVLink is a lightweight messaging protocol designed for drones and robotics.
It supports bidirectional communication,
meaning you can both send data TO ArduPilot and receive data FROM it.

## Architecture

```
┌─────────────────────┐         UDP/14550        ┌─────────────────────┐
│   Raspberry Pi      │ ◄──────────────────────► │     ArduPilot       │
│      Pico W         │         WiFi             │   (Rover firmware)  │
│                     │                          │                     │
│  Rust + Embassy     │                          │  Flight Controller  │
│  embassy-net UDP    │                          │  or SITL Simulator  │
└─────────────────────┘                          └─────────────────────┘
```

The Pico W connects to the same WiFi network as ArduPilot (or its ground station)
and sends/receives MAVLink packets over UDP port 14550.

---

## MAVLink Protocol Fundamentals

### Packet Structure

**MAVLink 2 Packet Format (280 bytes max):**
```
Byte    Field               Size    Description
───────────────────────────────────────────────────────
0       Magic               1       0xFD (MAVLink 2)
1       Payload Length      1       0-255 bytes
2       Incompat Flags      1       Flags that must be understood
3       Compat Flags        1       Flags that can be ignored
4       Sequence            1       Packet sequence number (0-255)
5       System ID           1       Sender system ID (vehicle=1, GCS=255)
6       Component ID        1       Sender component ID
7-9     Message ID          3       24-bit message ID (little-endian)
10+     Payload             0-255   Message data (trailing zeros truncated)
N       Checksum            2       CRC-16/MCRF4XX
N+2     Signature           13      Optional (for signed links)
```

**MAVLink 1 Packet Format (263 bytes max):**
```
Byte    Field               Size    Description
───────────────────────────────────────────────────────
0       Magic               1       0xFE (MAVLink 1)
1       Payload Length      1       Payload size
2       Sequence            1       Packet sequence number
3       System ID           1       Sender system ID
4       Component ID        1       Sender component ID
5       Message ID          1       8-bit message ID (0-255 only)
6+      Payload             N       Message data
N       Checksum            2       CRC-16/MCRF4XX
```

### System and Component IDs

| Entity | System ID | Component ID |
|--------|-----------|--------------|
| Vehicle (autopilot) | 1 | 1 |
| Ground Control Station | 255 | 190 |
| Pico W (companion) | 1 (same as vehicle) | 100 |
| External Sensor | 1 | varies |

### Checksum (CRC)

MAVLink uses CRC-16/MCRF4XX (same as ITU X.25). The CRC includes:
- All header bytes (except magic byte)
- All payload bytes
- A CRC_EXTRA byte (message-specific seed to verify sender/receiver compatibility)

**CRC_EXTRA values for common messages:**
```
HEARTBEAT (0):        50
STATUSTEXT (253):     83
GPS_INPUT (232):      151
DISTANCE_SENSOR (132): 85
BATTERY_STATUS (147): 154
RPM (226):            207
```

---

## Rust Implementation for RP2040

### Dependencies (Cargo.toml additions)

The project already has embassy-net support. For MAVLink, we'll implement
the protocol directly (no external crate needed for basic messages).

### MAVLink CRC Implementation

```rust
/// CRC-16/MCRF4XX (X.25) - same algorithm as in lib.rs but for MAVLink
pub fn mavlink_crc(data: &[u8], crc_extra: u8) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        let tmp = byte ^ (crc as u8);
        let tmp = tmp ^ (tmp << 4);
        crc = (crc >> 8) ^ ((tmp as u16) << 8) ^ ((tmp as u16) << 3) ^ ((tmp as u16) >> 4);
    }
    // Include CRC_EXTRA
    let tmp = crc_extra ^ (crc as u8);
    let tmp = tmp ^ (tmp << 4);
    crc = (crc >> 8) ^ ((tmp as u16) << 8) ^ ((tmp as u16) << 3) ^ ((tmp as u16) >> 4);
    crc
}
```

### MAVLink Message Builder

```rust
use heapless::Vec;

/// MAVLink message severity levels
#[repr(u8)]
pub enum MavSeverity {
    Emergency = 0,
    Alert = 1,
    Critical = 2,
    Error = 3,
    Warning = 4,
    Notice = 5,
    Info = 6,
    Debug = 7,
}

/// MAVLink packet builder for MAVLink 1
pub struct MavlinkBuilder {
    system_id: u8,
    component_id: u8,
    sequence: u8,
}

impl MavlinkBuilder {
    pub fn new(system_id: u8, component_id: u8) -> Self {
        Self {
            system_id,
            component_id,
            sequence: 0,
        }
    }

    fn next_seq(&mut self) -> u8 {
        let seq = self.sequence;
        self.sequence = self.sequence.wrapping_add(1);
        seq
    }

    /// Build HEARTBEAT message (ID: 0, CRC_EXTRA: 50)
    pub fn heartbeat(&mut self) -> Vec<u8, 17> {
        let mut buf: Vec<u8, 17> = Vec::new();
        buf.push(0xFE).ok();           // Magic (MAVLink 1)
        buf.push(9).ok();              // Payload length
        buf.push(self.next_seq()).ok(); // Sequence
        buf.push(self.system_id).ok();
        buf.push(self.component_id).ok();
        buf.push(0).ok();              // Message ID: HEARTBEAT

        // Payload (9 bytes)
        buf.push(0).ok();              // custom_mode (4 bytes)
        buf.push(0).ok();
        buf.push(0).ok();
        buf.push(0).ok();
        buf.push(18).ok();             // type: MAV_TYPE_ONBOARD_CONTROLLER
        buf.push(0).ok();              // autopilot: MAV_AUTOPILOT_INVALID
        buf.push(0).ok();              // base_mode
        buf.push(4).ok();              // system_status: MAV_STATE_ACTIVE
        buf.push(3).ok();              // mavlink_version

        // CRC (over bytes 1..15, then add CRC_EXTRA=50)
        let crc = mavlink_crc(&buf[1..15], 50);
        buf.push((crc & 0xFF) as u8).ok();
        buf.push((crc >> 8) as u8).ok();

        buf
    }

    /// Build STATUSTEXT message (ID: 253, CRC_EXTRA: 83)
    /// Text is truncated to 50 characters
    pub fn statustext(&mut self, severity: MavSeverity, text: &str) -> Vec<u8, 59> {
        let mut buf: Vec<u8, 59> = Vec::new();
        buf.push(0xFE).ok();           // Magic
        buf.push(51).ok();             // Payload length (1 + 50)
        buf.push(self.next_seq()).ok();
        buf.push(self.system_id).ok();
        buf.push(self.component_id).ok();
        buf.push(253).ok();            // Message ID: STATUSTEXT

        // Payload
        buf.push(severity as u8).ok(); // severity

        // Text (50 bytes, null-padded)
        let text_bytes = text.as_bytes();
        for i in 0..50 {
            if i < text_bytes.len() {
                buf.push(text_bytes[i]).ok();
            } else {
                buf.push(0).ok();
            }
        }

        // CRC
        let crc = mavlink_crc(&buf[1..57], 83);
        buf.push((crc & 0xFF) as u8).ok();
        buf.push((crc >> 8) as u8).ok();

        buf
    }

    /// Build RPM message (ID: 226, CRC_EXTRA: 207)
    pub fn rpm(&mut self, rpm1: f32, rpm2: f32) -> Vec<u8, 16> {
        let mut buf: Vec<u8, 16> = Vec::new();
        buf.push(0xFE).ok();
        buf.push(8).ok();              // Payload length
        buf.push(self.next_seq()).ok();
        buf.push(self.system_id).ok();
        buf.push(self.component_id).ok();
        buf.push(226).ok();            // Message ID: RPM

        // Payload (little-endian floats)
        for b in rpm1.to_le_bytes() { buf.push(b).ok(); }
        for b in rpm2.to_le_bytes() { buf.push(b).ok(); }

        // CRC
        let crc = mavlink_crc(&buf[1..14], 207);
        buf.push((crc & 0xFF) as u8).ok();
        buf.push((crc >> 8) as u8).ok();

        buf
    }

    /// Build BATTERY_STATUS message (ID: 147, CRC_EXTRA: 154)
    pub fn battery_status(
        &mut self,
        voltage_mv: u16,
        current_ca: i16,
        remaining_pct: i8,
    ) -> Vec<u8, 44> {
        let mut buf: Vec<u8, 44> = Vec::new();
        buf.push(0xFE).ok();
        buf.push(36).ok();             // Payload length
        buf.push(self.next_seq()).ok();
        buf.push(self.system_id).ok();
        buf.push(self.component_id).ok();
        buf.push(147).ok();            // Message ID: BATTERY_STATUS

        // Payload
        buf.push(0).ok();              // id
        buf.push(0).ok();              // battery_function
        buf.push(0).ok();              // type
        // temperature (i16, INT16_MAX = unknown)
        buf.push(0xFF).ok();
        buf.push(0x7F).ok();
        // voltages[10] (u16 each, UINT16_MAX = invalid)
        // First cell has our voltage
        buf.push((voltage_mv & 0xFF) as u8).ok();
        buf.push((voltage_mv >> 8) as u8).ok();
        // Remaining 9 cells = UINT16_MAX (invalid)
        for _ in 0..9 {
            buf.push(0xFF).ok();
            buf.push(0xFF).ok();
        }
        // current_battery (i16, centiamps)
        buf.push((current_ca & 0xFF) as u8).ok();
        buf.push(((current_ca >> 8) & 0xFF) as u8).ok();
        // current_consumed (i32, mAh, -1 = unknown)
        buf.push(0xFF).ok();
        buf.push(0xFF).ok();
        buf.push(0xFF).ok();
        buf.push(0xFF).ok();
        // energy_consumed (i32, hJ, -1 = unknown)
        buf.push(0xFF).ok();
        buf.push(0xFF).ok();
        buf.push(0xFF).ok();
        buf.push(0xFF).ok();
        // battery_remaining (i8, %)
        buf.push(remaining_pct as u8).ok();

        // CRC
        let crc = mavlink_crc(&buf[1..42], 154);
        buf.push((crc & 0xFF) as u8).ok();
        buf.push((crc >> 8) as u8).ok();

        buf
    }

    /// Build DISTANCE_SENSOR message (ID: 132, CRC_EXTRA: 85)
    pub fn distance_sensor(
        &mut self,
        time_boot_ms: u32,
        distance_cm: u16,
        orientation: u8,
    ) -> Vec<u8, 22> {
        let mut buf: Vec<u8, 22> = Vec::new();
        buf.push(0xFE).ok();
        buf.push(14).ok();             // Payload length
        buf.push(self.next_seq()).ok();
        buf.push(self.system_id).ok();
        buf.push(self.component_id).ok();
        buf.push(132).ok();            // Message ID: DISTANCE_SENSOR

        // Payload
        // time_boot_ms (u32)
        for b in time_boot_ms.to_le_bytes() { buf.push(b).ok(); }
        // min_distance (u16, cm)
        buf.push(2).ok();
        buf.push(0).ok();
        // max_distance (u16, cm)
        buf.push(0xA0).ok();  // 4000 cm = 40m
        buf.push(0x0F).ok();
        // current_distance (u16, cm)
        buf.push((distance_cm & 0xFF) as u8).ok();
        buf.push((distance_cm >> 8) as u8).ok();
        // type (u8) - MAV_DISTANCE_SENSOR_LASER = 0
        buf.push(0).ok();
        // id (u8)
        buf.push(0).ok();
        // orientation (u8)
        buf.push(orientation).ok();
        // covariance (u8, 255 = unknown)
        buf.push(255).ok();

        // CRC
        let crc = mavlink_crc(&buf[1..20], 85);
        buf.push((crc & 0xFF) as u8).ok();
        buf.push((crc >> 8) as u8).ok();

        buf
    }
}
```

### UDP Telemetry Task

```rust
use embassy_net::{udp::{UdpSocket, PacketMetadata}, Stack};
use embassy_time::{Duration, Timer};

/// MAVLink telemetry sender task
#[embassy_executor::task]
pub async fn mavlink_task(stack: Stack<'static>) {
    // Wait for network to be ready
    stack.wait_config_up().await;

    let mut rx_meta = [PacketMetadata::EMPTY; 4];
    let mut rx_buffer = [0u8; 280];
    let mut tx_meta = [PacketMetadata::EMPTY; 4];
    let mut tx_buffer = [0u8; 280];

    let mut socket = UdpSocket::new(
        stack,
        &mut rx_meta,
        &mut rx_buffer,
        &mut tx_meta,
        &mut tx_buffer,
    );

    // Bind to any port for sending
    socket.bind(0).unwrap();

    // ArduPilot address (configure for your network)
    let ardupilot_addr = embassy_net::IpEndpoint::new(
        embassy_net::IpAddress::v4(192, 168, 1, 100),  // ArduPilot IP
        14550,
    );

    let mut mav = MavlinkBuilder::new(1, 100);  // System 1, Component 100

    loop {
        // Send heartbeat
        let heartbeat = mav.heartbeat();
        socket.send_to(&heartbeat, ardupilot_addr).await.ok();

        // Send status message
        let status = mav.statustext(MavSeverity::Info, "Pico W operational");
        socket.send_to(&status, ardupilot_addr).await.ok();

        // Send battery status (example: 12.4V, 5.5A, 75%)
        let battery = mav.battery_status(12400, 550, 75);
        socket.send_to(&battery, ardupilot_addr).await.ok();

        // Send RPM (from hoverboard motors)
        let rpm = mav.rpm(1500.0, 1480.0);
        socket.send_to(&rpm, ardupilot_addr).await.ok();

        Timer::after(Duration::from_millis(100)).await;
    }
}
```

### Integration with Hoverboard Response

```rust
use split_hover_esc::protocol::Response;

impl MavlinkBuilder {
    /// Convert hoverboard response to MAVLink messages
    pub fn from_hoverboard_response(
        &mut self,
        response: &Response,
        time_ms: u32,
    ) -> (Vec<u8, 44>, Vec<u8, 16>) {
        // Battery status from hoverboard voltage/current
        let battery = self.battery_status(
            response.voltage_mv,
            response.current_ca,
            -1,  // Unknown remaining %
        );

        // RPM from speed (convert 0.1 RPM units to RPM)
        let rpm = self.rpm(
            response.speed as f32 / 10.0,
            0.0,  // Single motor, or use second response
        );

        (battery, rpm)
    }
}
```

---

## Sensor Orientation Reference

For DISTANCE_SENSOR orientation field:

```
Value   Name                    Direction
─────────────────────────────────────────────
0       ROTATION_NONE           Forward
1       ROTATION_YAW_45         45 deg right
2       ROTATION_YAW_90         Right
4       ROTATION_YAW_180        Backward
6       ROTATION_YAW_270        Left
24      ROTATION_PITCH_270      Down (altitude)
25      ROTATION_PITCH_90       Up
```

---

## Messages ArduPilot Can Receive

### STATUSTEXT (ID: 253)

Send custom debug/info/warning messages that ArduPilot logs.

| Field | Type | Description |
|-------|------|-------------|
| severity | u8 | 0=EMERGENCY to 7=DEBUG |
| text | char[50] | Message text (max 50 chars) |

### BATTERY_STATUS (ID: 147)

Send battery voltage/current from hoverboard ESC.

| Field | Type | Description |
|-------|------|-------------|
| id | u8 | Battery ID (0) |
| voltages[0] | u16 | Voltage in mV |
| current_battery | i16 | Current in cA (0.01A) |
| battery_remaining | i8 | Percentage (-1 = unknown) |

### RPM (ID: 226)

Send motor RPM from hoverboard.

| Field | Type | Description |
|-------|------|-------------|
| rpm1 | f32 | Left motor RPM |
| rpm2 | f32 | Right motor RPM |

### DISTANCE_SENSOR (ID: 132)

Send rangefinder/ultrasonic sensor data.

| Field | Type | Description |
|-------|------|-------------|
| time_boot_ms | u32 | Timestamp (ms) |
| min_distance | u16 | Min range (cm) |
| max_distance | u16 | Max range (cm) |
| current_distance | u16 | Reading (cm) |
| orientation | u8 | Sensor direction |

### GPS_INPUT (ID: 232)

Send external GPS position (requires `GPS1_TYPE = 14`).

| Field | Type | Description |
|-------|------|-------------|
| time_usec | u64 | Timestamp (us) |
| lat | i32 | Latitude (deg * 1E7) |
| lon | i32 | Longitude (deg * 1E7) |
| alt | f32 | Altitude AMSL (m) |
| fix_type | u8 | 0-1=none, 2=2D, 3=3D |
| satellites_visible | u8 | Sat count |

---

## Messages FROM ArduPilot

These can be received and parsed on the Pico W:

### HEARTBEAT (ID: 0)

Sent at 1Hz, confirms ArduPilot is alive.

### GLOBAL_POSITION_INT (ID: 33)

Fused GPS position with velocity.

### ATTITUDE (ID: 30)

Roll/pitch/yaw in radians.

### SYS_STATUS (ID: 1)

System health and battery info.

---

## Python Test Application

For testing MAVLink communication locally (same machine as ArduPilot SITL),
without WiFi in the loop.

### test_mavlink.py

```python
#!/usr/bin/env python3
"""
MAVLink test application for local development.
Runs on the same computer as ArduPilot SITL.

Usage:
    # Start ArduPilot SITL first:
    sim_vehicle.py -v Rover --console --map

    # Then run this test:
    python test_mavlink.py
"""
import time
import sys
from pymavlink import mavutil


class MavlinkTester:
    def __init__(self, connection_string='udpin:127.0.0.1:14550'):
        """
        Connect to ArduPilot.

        For SITL, use:
          - 'udpin:127.0.0.1:14550'  (listen for SITL broadcast)
          - 'udpout:127.0.0.1:14550' (send to SITL)
          - 'tcp:127.0.0.1:5760'     (TCP connection)
        """
        print(f"Connecting to {connection_string}...")
        self.conn = mavutil.mavlink_connection(
            connection_string,
            source_system=1,
            source_component=100  # Companion computer component
        )
        print("Waiting for heartbeat from ArduPilot...")
        self.conn.wait_heartbeat(timeout=30)
        print(f"Connected! System {self.conn.target_system}, "
              f"Component {self.conn.target_component}")

    def send_heartbeat(self):
        """Send companion computer heartbeat."""
        self.conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0,
            mavutil.mavlink.MAV_STATE_ACTIVE
        )

    def send_status(self, text, severity=None):
        """Send STATUSTEXT message (logged by ArduPilot)."""
        if severity is None:
            severity = mavutil.mavlink.MAV_SEVERITY_INFO
        self.conn.mav.statustext_send(severity, text[:50].encode())
        print(f"[TX] STATUSTEXT: {text}")

    def send_debug(self, text):
        self.send_status(text, mavutil.mavlink.MAV_SEVERITY_DEBUG)

    def send_info(self, text):
        self.send_status(text, mavutil.mavlink.MAV_SEVERITY_INFO)

    def send_warning(self, text):
        self.send_status(text, mavutil.mavlink.MAV_SEVERITY_WARNING)

    def send_error(self, text):
        self.send_status(text, mavutil.mavlink.MAV_SEVERITY_ERROR)

    def send_battery(self, voltage_mv, current_ca=-1, remaining=-1):
        """
        Send BATTERY_STATUS message.

        Args:
            voltage_mv: Total voltage in millivolts (e.g., 12400 for 12.4V)
            current_ca: Current in centiamps (e.g., 550 for 5.5A)
            remaining: Battery percentage (0-100, -1 = unknown)
        """
        voltages = [voltage_mv] + [65535] * 9  # First cell, rest invalid
        self.conn.mav.battery_status_send(
            id=0,
            battery_function=0,
            type=0,
            temperature=32767,  # Unknown
            voltages=voltages,
            current_battery=current_ca,
            current_consumed=-1,
            energy_consumed=-1,
            battery_remaining=remaining
        )
        print(f"[TX] BATTERY: {voltage_mv}mV, {current_ca}cA, {remaining}%")

    def send_rpm(self, rpm1, rpm2=0.0):
        """Send RPM message."""
        self.conn.mav.rpm_send(rpm1, rpm2)
        print(f"[TX] RPM: {rpm1:.1f}, {rpm2:.1f}")

    def send_distance(self, distance_cm, orientation=0, sensor_id=0):
        """
        Send DISTANCE_SENSOR message.

        Args:
            distance_cm: Distance in centimeters
            orientation: 0=forward, 24=down, 25=up
            sensor_id: Sensor instance ID
        """
        self.conn.mav.distance_sensor_send(
            time_boot_ms=int(time.time() * 1000) % (2**32),
            min_distance=2,
            max_distance=4000,
            current_distance=distance_cm,
            type=0,  # Laser
            id=sensor_id,
            orientation=orientation,
            covariance=255
        )
        print(f"[TX] DISTANCE: {distance_cm}cm (orient={orientation})")

    def receive_messages(self, timeout=0.1):
        """Receive and print any incoming messages."""
        while True:
            msg = self.conn.recv_match(blocking=False)
            if msg is None:
                break
            msg_type = msg.get_type()
            if msg_type == 'HEARTBEAT':
                print(f"[RX] HEARTBEAT from sys={msg.get_srcSystem()}")
            elif msg_type == 'STATUSTEXT':
                print(f"[RX] STATUSTEXT: {msg.text}")
            elif msg_type == 'SYS_STATUS':
                print(f"[RX] SYS_STATUS: battery={msg.voltage_battery}mV")
            elif msg_type == 'GLOBAL_POSITION_INT':
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                print(f"[RX] POSITION: {lat:.6f}, {lon:.6f}")


def simulate_hoverboard_telemetry(tester):
    """Simulate telemetry data similar to hoverboard ESC responses."""
    import math

    start_time = time.time()
    voltage = 36000  # 36V nominal

    while True:
        elapsed = time.time() - start_time

        # Simulate varying RPM
        rpm = 1000 + 500 * math.sin(elapsed * 0.5)

        # Simulate battery discharge
        voltage_now = voltage - int(elapsed * 10)  # 10mV/sec discharge

        # Simulate current based on RPM
        current = int(abs(rpm) * 0.3)  # 0.3 cA per RPM

        # Send telemetry
        tester.send_heartbeat()
        tester.send_battery(voltage_now, current)
        tester.send_rpm(rpm, rpm * 0.98)  # Slight difference between motors
        tester.send_distance(150 + int(50 * math.sin(elapsed)), orientation=0)

        # Occasional status messages
        if int(elapsed) % 10 == 0 and elapsed - int(elapsed) < 0.2:
            tester.send_info(f"Uptime: {int(elapsed)}s")

        # Check for incoming messages
        tester.receive_messages()

        time.sleep(0.1)


def main():
    # Default to UDP connection to SITL
    conn_string = 'udpin:127.0.0.1:14550'

    if len(sys.argv) > 1:
        conn_string = sys.argv[1]

    print("=" * 50)
    print("MAVLink Test Application")
    print("=" * 50)
    print()
    print("Connection modes:")
    print("  SITL default:    udpin:127.0.0.1:14550")
    print("  SITL TCP:        tcp:127.0.0.1:5760")
    print("  Hardware serial: /dev/ttyUSB0")
    print()

    tester = MavlinkTester(conn_string)

    print()
    print("Starting simulated telemetry loop...")
    print("Press Ctrl+C to stop")
    print()

    try:
        simulate_hoverboard_telemetry(tester)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
```

### Running with ArduPilot SITL

```bash
# Terminal 1: Start ArduPilot Rover SITL
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py -v Rover --console --map

# Terminal 2: Run the test application
cd ~/src/split-hover-esc
python test_mavlink.py

# Or with explicit connection string:
python test_mavlink.py tcp:127.0.0.1:5760
```

### Requirements

```bash
pip install pymavlink
```

---

## ArduPilot Logging Configuration

To ensure STATUSTEXT messages are logged:

```
LOG_BACKEND_TYPE = 1    # Log to SD card
LOG_DISARMED = 1        # Log before arming (for debugging)
```

### Extracting logs

```bash
# From telemetry log
mavlogdump.py --types=STATUSTEXT flight.tlog

# From dataflash log
mavlogdump.py --types=MSG 00000001.BIN
```

---

## Message ID Quick Reference

| ID | Message | Direction | Description |
|----|---------|-----------|-------------|
| 0 | HEARTBEAT | Both | System alive indicator |
| 1 | SYS_STATUS | FROM | System status |
| 24 | GPS_RAW_INT | FROM | Raw GPS data |
| 30 | ATTITUDE | FROM | Vehicle attitude |
| 33 | GLOBAL_POSITION_INT | FROM | Fused position |
| 74 | VFR_HUD | FROM | HUD data |
| 132 | DISTANCE_SENSOR | TO | Rangefinder data |
| 147 | BATTERY_STATUS | TO | Battery info |
| 226 | RPM | TO | RPM sensors |
| 232 | GPS_INPUT | TO | External GPS |
| 253 | STATUSTEXT | Both | Text messages |

---

## References

- [MAVLink Common Messages](https://mavlink.io/en/messages/common.html)
- [ArduPilot MAVLink Messages](https://mavlink.io/en/messages/ardupilotmega.html)
- [ArduPilot MAVLink Basics](https://ardupilot.org/dev/docs/mavlink-basics.html)
- [MAVLink Packet Serialization](https://mavlink.io/en/guide/serialization.html)
- [Debugging with send_text](https://ardupilot.org/dev/docs/debug-with-send-text.html)
- [Embassy-net UDP](https://docs.embassy.dev/embassy-net/)
