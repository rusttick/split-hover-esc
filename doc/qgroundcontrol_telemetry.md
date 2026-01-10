# QGroundControl Communication Protocol

This document describes the MAVLink communication between a custom rover and QGroundControl (QGC).

## Architecture

```
┌─────────────────────┐                           ┌─────────────────────┐
│   Raspberry Pi      │         UDP/14550         │   QGroundControl    │
│      Pico W         │ ─────────────────────────►│                     │
│                     │◄───────────────────────── │   (Your Computer)   │
│   Custom Rover      │         WiFi              │                     │
│   System ID: 1      │                           │   System ID: 255    │
│   Component ID: 1   │                           │   Component ID: 190 │
└─────────────────────┘                           └─────────────────────┘
```

## Network Configuration

| Parameter    | Value              | Notes                                  |
|--------------|--------------------|----------------------------------------|
| Protocol     | UDP                | Connectionless, fire-and-forget        |
| Default Port | 14550              | QGC listens on this port by default    |
| Direction    | Bidirectional      | Rover sends to QGC, QGC sends to rover |
| Auto-connect | Enabled by default | QGC connects when it sees a heartbeat  |

## Connection Sequence

```
    Rover                                    QGroundControl
      │                                            │
      │──────── HEARTBEAT (1 Hz) ─────────────────►│
      │                                            │ Vehicle detected
      │                                            │ Create Vehicle object
      │◄─────── HEARTBEAT (1 Hz) ──────────────────│
      │                                            │
      │◄─────── PARAM_REQUEST_LIST ────────────────│
      │                                            │
      │──────── PARAM_VALUE (for each param) ─────►│
      │         or ignore if no params             │
      │                                            │
      │◄─────── MISSION_REQUEST_LIST ──────────────│
      │                                            │
      │──────── MISSION_COUNT (count=0) ──────────►│
      │                                            │
      │                                            │ Vehicle ready
      │◄──────── Telemetry requests ───────────────│
      │                                            │
      │══════ Continuous telemetry exchange ══════►│
      │                                            │
```

## System Identification

### System ID Assignment

| Device                 | System ID | Notes                      |
|------------------------|-----------|----------------------------|
| Vehicle (rover)        | 1         | Default, configurable      |
| Ground Control Station | 255       | High ID to avoid conflicts |
| Companion Computer     | 1         | Same as vehicle            |

### Component ID Assignment

| Component                   | ID      | MAVLink Constant           |
|-----------------------------|---------|----------------------------|
| Autopilot / Main Controller | 1       | MAV_COMP_ID_AUTOPILOT1     |
| Ground Control Station      | 190     | MAV_COMP_ID_MISSIONPLANNER |
| Companion Computer          | 100-199 | Custom range               |
| Camera                      | 100     | MAV_COMP_ID_CAMERA         |
| Gimbal                      | 154     | MAV_COMP_ID_GIMBAL         |

---

## HEARTBEAT Message

The heartbeat is the foundation of MAVLink communication. It must be sent at 1 Hz.

### Message ID: 0

### Fields

| Field           | Type   | Description                         |
|-----------------|--------|-------------------------------------|
| type            | uint8  | Vehicle type (MAV_TYPE enum)        |
| autopilot       | uint8  | Autopilot type (MAV_AUTOPILOT enum) |
| base_mode       | uint8  | Mode flags (MAV_MODE_FLAG bitmask)  |
| custom_mode     | uint32 | Autopilot-specific mode             |
| system_status   | uint8  | System state (MAV_STATE enum)       |
| mavlink_version | uint8  | MAVLink version (auto-filled: 3)    |

### MAV_TYPE Values (Relevant)

| Value | Name                        | Description        |
|-------|-----------------------------|--------------------|
| 0     | MAV_TYPE_GENERIC            | Generic vehicle    |
| 10    | MAV_TYPE_GROUND_ROVER       | Ground rover       |
| 11    | MAV_TYPE_SURFACE_BOAT       | Surface vessel     |
| 18    | MAV_TYPE_ONBOARD_CONTROLLER | Companion computer |

### MAV_AUTOPILOT Values

| Value | Name                        | Use Case                |
|-------|-----------------------------|-------------------------|
| 0     | MAV_AUTOPILOT_GENERIC       | Custom autopilot        |
| 3     | MAV_AUTOPILOT_ARDUPILOTMEGA | ArduPilot               |
| 8     | MAV_AUTOPILOT_INVALID       | Non-autopilot component |
| 12    | MAV_AUTOPILOT_PX4           | PX4                     |

### MAV_STATE Values

| Value | Name                  | Description         |
|-------|-----------------------|---------------------|
| 0     | MAV_STATE_UNINIT      | Booting             |
| 1     | MAV_STATE_BOOT        | Calibrating         |
| 2     | MAV_STATE_CALIBRATING | Calibrating sensors |
| 3     | MAV_STATE_STANDBY     | Ready, not active   |
| 4     | MAV_STATE_ACTIVE      | Active / armed      |
| 5     | MAV_STATE_CRITICAL    | Critical failure    |
| 6     | MAV_STATE_EMERGENCY   | Emergency           |

### MAV_MODE_FLAG Values (Bitmask)

| Value | Name                 | Description                |
|-------|----------------------|----------------------------|
| 1     | CUSTOM_MODE_ENABLED  | Custom mode field is valid |
| 4     | AUTO_ENABLED         | Autonomous mode            |
| 8     | GUIDED_ENABLED       | Guided mode                |
| 16    | STABILIZE_ENABLED    | Stabilize mode             |
| 32    | HIL_ENABLED          | Hardware-in-the-loop       |
| 64    | MANUAL_INPUT_ENABLED | Manual control             |
| 128   | SAFETY_ARMED         | Vehicle is armed           |

### Recommended Heartbeat for Rover

| Field         | Value | Reason                                     |
|---------------|-------|--------------------------------------------|
| type          | 10    | MAV_TYPE_GROUND_ROVER                      |
| autopilot     | 0     | MAV_AUTOPILOT_GENERIC                      |
| base_mode     | 65    | CUSTOM_MODE_ENABLED + MANUAL_INPUT_ENABLED |
| custom_mode   | 0     | No custom modes defined                    |
| system_status | 4     | MAV_STATE_ACTIVE                           |

---

## SYS_STATUS Message

Reports system health and battery status.

### Message ID: 1

### Fields (Key)

| Field                           | Type   | Description                        |
|---------------------------------|--------|------------------------------------|
| onboard_control_sensors_present | uint32 | Bitmap of present sensors          |
| onboard_control_sensors_enabled | uint32 | Bitmap of enabled sensors          |
| onboard_control_sensors_health  | uint32 | Bitmap of healthy sensors          |
| load                            | uint16 | CPU load (0-1000 = 0-100%)         |
| voltage_battery                 | uint16 | Battery voltage (mV)               |
| current_battery                 | int16  | Battery current (cA, 10mA units)   |
| battery_remaining               | int8   | Remaining capacity (%, -1=unknown) |

### Sensor Flags (MAV_SYS_STATUS_SENSOR)

| Value | Name          |
|-------|---------------|
| 1     | 3D_GYRO       |
| 2     | 3D_ACCEL      |
| 4     | 3D_MAG        |
| 32    | GPS           |
| 8192  | MOTOR_OUTPUTS |
| 16384 | RC_RECEIVER   |
| 32768 | BATTERY       |

---

## STATUSTEXT Message

Send text messages to the GCS for logging and display.

### Message ID: 253

### Fields

| Field    | Type     | Description                 |
|----------|----------|-----------------------------|
| severity | uint8    | MAV_SEVERITY level          |
| text     | char[50] | Message text (max 50 chars) |

### MAV_SEVERITY Levels

| Value | Name      | QGC Display         |
|-------|-----------|---------------------|
| 0     | EMERGENCY | Red, critical alert |
| 1     | ALERT     | Red                 |
| 2     | CRITICAL  | Red                 |
| 3     | ERROR     | Red                 |
| 4     | WARNING   | Yellow              |
| 5     | NOTICE    | White               |
| 6     | INFO      | White               |
| 7     | DEBUG     | Gray (if enabled)   |

---

## Parameter Protocol

QGC requests all parameters on connection.

### Sequence Diagram

```
    Rover                              QGroundControl
      │                                      │
      │◄──── PARAM_REQUEST_LIST (76) ────────│
      │                                      │
      │───── PARAM_VALUE (22) ──────────────►│  param_index=0
      │───── PARAM_VALUE (22) ──────────────►│  param_index=1
      │───── PARAM_VALUE (22) ──────────────►│  param_index=2
      │      ...                             │
      │───── PARAM_VALUE (22) ──────────────►│  param_index=N-1
      │                                      │  (param_count=N in all)
```

### PARAM_VALUE Fields (ID: 22)

| Field       | Type     | Description                |
|-------------|----------|----------------------------|
| param_id    | char[16] | Parameter name             |
| param_value | float    | Value (as float)           |
| param_type  | uint8    | MAV_PARAM_TYPE             |
| param_count | uint16   | Total number of parameters |
| param_index | uint16   | Index of this parameter    |

### MAV_PARAM_TYPE

| Value | Name   | Size            |
|-------|--------|-----------------|
| 1     | UINT8  | 1 byte          |
| 2     | INT8   | 1 byte          |
| 3     | UINT16 | 2 bytes         |
| 4     | INT16  | 2 bytes         |
| 5     | UINT32 | 4 bytes         |
| 6     | INT32  | 4 bytes         |
| 9     | REAL32 | 4 bytes (float) |

### Handling No Parameters

If your rover has no parameters:
- Option 1: Ignore PARAM_REQUEST_LIST (QGC handles gracefully)
- Option 2: Send one PARAM_VALUE with param_count=0

---

## Mission Protocol

QGC requests the mission list on connection.

### Minimal Response (No Mission)

```
    Rover                              QGroundControl
      │                                      │
      │◄──── MISSION_REQUEST_LIST (43) ──────│
      │                                      │
      │───── MISSION_COUNT (44) ────────────►│  count=0
      │                                      │
```

### MISSION_COUNT Fields (ID: 44)

| Field            | Type   | Description             |
|------------------|--------|-------------------------|
| target_system    | uint8  | Target system ID        |
| target_component | uint8  | Target component ID     |
| count            | uint16 | Number of mission items |

---

## Command Protocol

Commands are sent via COMMAND_LONG and acknowledged with COMMAND_ACK.

### COMMAND_LONG (ID: 76)

| Field            | Type   | Description                     |
|------------------|--------|---------------------------------|
| target_system    | uint8  | Target system                   |
| target_component | uint8  | Target component                |
| command          | uint16 | MAV_CMD ID                      |
| confirmation     | uint8  | Retry count (0 = first attempt) |
| param1-7         | float  | Command parameters              |

### COMMAND_ACK (ID: 77)

| Field   | Type   | Description                |
|---------|--------|----------------------------|
| command | uint16 | Command being acknowledged |
| result  | uint8  | MAV_RESULT                 |

### MAV_RESULT Values

| Value | Name                 | Meaning               |
|-------|----------------------|-----------------------|
| 0     | ACCEPTED             | Command executed      |
| 1     | TEMPORARILY_REJECTED | Retry later           |
| 2     | DENIED               | Permanently rejected  |
| 3     | UNSUPPORTED          | Command not supported |
| 4     | FAILED               | Execution failed      |
| 5     | IN_PROGRESS          | Still executing       |

### Common Commands

| ID  | Name                                   | Description                    |
|-----|----------------------------------------|--------------------------------|
| 400 | MAV_CMD_COMPONENT_ARM_DISARM           | Arm/disarm motors              |
| 520 | MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES | Request version info           |
| 21  | MAV_CMD_NAV_LAND                       | Land (not applicable to rover) |

### Arm/Disarm Command

| Parameter | Arm | Disarm |
|-----------|-----|--------|
| param1    | 1.0 | 0.0    |
| param2    | 0.0 | 0.0    |
| param3-7  | 0.0 | 0.0    |

---

## Telemetry Messages (Rover → QGC)

### Recommended Message Set

| ID  | Message             | Rate      | Purpose               |
|-----|---------------------|-----------|-----------------------|
| 0   | HEARTBEAT           | 1 Hz      | Connection keep-alive |
| 1   | SYS_STATUS          | 1 Hz      | Battery, health       |
| 253 | STATUSTEXT          | As needed | Logging               |
| 33  | GLOBAL_POSITION_INT | 4 Hz      | GPS position          |
| 30  | ATTITUDE            | 10 Hz     | Orientation           |
| 74  | VFR_HUD             | 4 Hz      | Speed, heading        |
| 226 | RPM                 | 2 Hz      | Motor RPM             |

### GLOBAL_POSITION_INT (ID: 33)

| Field        | Type   | Description              |
|--------------|--------|--------------------------|
| time_boot_ms | uint32 | Timestamp (ms)           |
| lat          | int32  | Latitude (deg * 1E7)     |
| lon          | int32  | Longitude (deg * 1E7)    |
| alt          | int32  | Altitude AMSL (mm)       |
| relative_alt | int32  | Altitude above home (mm) |
| vx           | int16  | Ground speed X (cm/s)    |
| vy           | int16  | Ground speed Y (cm/s)    |
| vz           | int16  | Ground speed Z (cm/s)    |
| hdg          | uint16 | Heading (cdeg, 0-35999)  |

### ATTITUDE (ID: 30)

| Field        | Type   | Description        |
|--------------|--------|--------------------|
| time_boot_ms | uint32 | Timestamp (ms)     |
| roll         | float  | Roll (rad)         |
| pitch        | float  | Pitch (rad)        |
| yaw          | float  | Yaw (rad)          |
| rollspeed    | float  | Roll rate (rad/s)  |
| pitchspeed   | float  | Pitch rate (rad/s) |
| yawspeed     | float  | Yaw rate (rad/s)   |

### VFR_HUD (ID: 74)

| Field | Type | Description |
|-------|------|-------------|
| airspeed | float | Airspeed (m/s) |
| groundspeed | float | Ground speed (m/s) |
| heading | int16 | Heading (deg, 0-360) |
| throttle | uint16 | Throttle (%) |
| alt | float | Altitude (m) |
| climb | float | Climb rate (m/s) |

### RPM (ID: 226)

| Field | Type | Description |
|-------|------|-------------|
| rpm1 | float | RPM sensor 1 |
| rpm2 | float | RPM sensor 2 |

---

## CRC Calculation

MAVLink uses CRC-16/MCRF4XX with a message-specific seed (CRC_EXTRA).

### Algorithm

```
Initial value: 0xFFFF
Polynomial: 0x1021 (reflected)
Input: Header bytes (except magic) + Payload + CRC_EXTRA
Output: 16-bit CRC, little-endian
```

### CRC_EXTRA Values

| Message             | ID  | CRC_EXTRA |
|---------------------|-----|-----------|
| HEARTBEAT           | 0   | 50        |
| SYS_STATUS          | 1   | 124       |
| PARAM_VALUE         | 22  | 220       |
| ATTITUDE            | 30  | 39        |
| GLOBAL_POSITION_INT | 33  | 104       |
| MISSION_COUNT       | 44  | 221       |
| VFR_HUD             | 74  | 20        |
| COMMAND_LONG        | 76  | 152       |
| COMMAND_ACK         | 77  | 143       |
| STATUSTEXT          | 253 | 83        |
| RPM                 | 226 | 207       |

---

## Timeout and Retry Behavior

### Heartbeat Timeout

| Condition             | Action                           |
|-----------------------|----------------------------------|
| 4-5 missed heartbeats | Connection considered lost       |
| Reconnection          | Automatic when heartbeat resumes |

### Command Retry

| Condition                 | Action                 |
|---------------------------|------------------------|
| No ACK within 1-3 seconds | Retry command          |
| Max retries (typically 3) | Report failure to user |

---

## QGC Auto-Connect Behavior

```
┌─────────────────────────────────────────────────────────────────┐
│                     QGroundControl                              │
│                                                                 │
│  ┌─────────────┐                                                │
│  │ LinkManager │──► Listens on UDP 14550                        │
│  └──────┬──────┘                                                │
│         │                                                       │
│         ▼ Receives HEARTBEAT                                    │
│  ┌──────────────────┐                                           │
│  │ MAVLinkProtocol  │──► Parses MAVLink bytes                   │
│  └────────┬─────────┘                                           │
│           │                                                     │
│           ▼ HEARTBEAT with MAV_TYPE vehicle                     │
│  ┌────────────────────┐                                         │
│  │ MultiVehicleManager │──► Creates Vehicle object              │
│  └─────────┬──────────┘                                         │
│            │                                                    │
│            ▼                                                    │
│  ┌─────────────────┐  ┌───────────────-┐  ┌──────────────────-┐ │
│  │ ParameterLoader │  │ MissionManager │  │ VehicleComponents │ │
│  │ PARAM_REQUEST   │  │ MISSION_REQUEST│  │ Setup UI          │ │
│  └─────────────────┘  └──────────────-─┘  └──────────────────-┘ │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## Minimal Implementation Checklist

To connect a custom rover to QGroundControl:

1. **Send HEARTBEAT at 1 Hz**
   - type = 10 (MAV_TYPE_GROUND_ROVER)
   - autopilot = 0 (MAV_AUTOPILOT_GENERIC)
   - system_status = 4 (MAV_STATE_ACTIVE)
   - System ID = 1, Component ID = 1

2. **Handle PARAM_REQUEST_LIST**
   - Ignore it, or respond with param_count=0

3. **Handle MISSION_REQUEST_LIST**
   - Respond with MISSION_COUNT, count=0

4. **Send telemetry**
   - SYS_STATUS for battery
   - STATUSTEXT for logging

5. **Optional: Handle commands**
   - Respond to COMMAND_LONG with COMMAND_ACK

---

## References

- [MAVLink Heartbeat Protocol](https://mavlink.io/en/services/heartbeat.html)
- [MAVLink Parameter Protocol](https://mavlink.io/en/services/parameter.html)
- [MAVLink Command Protocol](https://mavlink.io/en/services/command.html)
- [MAVLink Message Definitions](https://mavlink.io/en/messages/common.html)
- [QGroundControl Communication Flow](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/communication_flow.html)
- [QGroundControl Vehicle Connection](https://docs.qgroundcontrol.com/Stable_V4.3/en/qgc-user-guide/troubleshooting/vehicle_connection.html)
