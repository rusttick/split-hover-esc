# Remote UART Bus Protocol

A serial protocol for controlling hoverboard motor controllers, designed by RoboDurden for the Hoverboard-Firmware-Hack-Gen2.x project.

## Overview

The Remote UART Bus protocol enables a master controller to communicate with one or more hoverboard motor controllers (slaves) over UART. Each slave has a unique ID, allowing multiple boards to share a single bus.

**Default Configuration:**
- Baud rate: 19200
- Data bits: 8
- Parity: None
- Stop bits: 1
- Slave ID: 1 (configurable, 0-255)
- Serial timeout: 1000ms (motor stops if no valid command received)

## Byte Order

- Multi-byte integer fields are **little-endian** (LSB first)
- CRC-16 is stored **big-endian** (MSB in last byte position)
- Floating-point fields use IEEE 754 single-precision (4 bytes, little-endian)

## Message Format

### Server → Hoverboard Messages

All server-to-hoverboard messages start with the ASCII character `'/'` (0x2F), followed by a data type identifier.

#### Type 0: Basic Speed Control (`SerialServer2Hover`)

| Offset | Size | Field     | Description                         |
|--------|------|-----------|-------------------------------------|
| 0      | 1    | cStart    | Start byte: `0x2F` (`'/'`)          |
| 1      | 1    | iDataType | Message type: `0x00`                |
| 2      | 1    | iSlave    | Target slave ID                     |
| 3      | 2    | iSpeed    | Speed setpoint (int16, signed)      |
| 5      | 1    | wState    | State flags (see State Flags below) |
| 6      | 2    | checksum  | CRC-16 (big-endian)                 |

**Total size: 8 bytes**

#### Type 1: Master Mode Control (`SerialServer2HoverMaster`)

Used for dual-motor hoverboard control with speed and steering.

| Offset | Size | Field       | Description                        |
|--------|------|-------------|------------------------------------|
| 0      | 1    | cStart      | Start byte: `0x2F` (`'/'`)         |
| 1      | 1    | iDataType   | Message type: `0x01`               |
| 2      | 1    | iSlave      | Target slave ID                    |
| 3      | 2    | iSpeed      | Speed setpoint (int16, signed)     |
| 5      | 2    | iSteer      | Steering value (int16, signed)     |
| 7      | 1    | wState      | State flags for this motor         |
| 8      | 1    | wStateSlave | State flags for paired slave motor |
| 9      | 2    | checksum    | CRC-16 (big-endian)                |

**Total size: 11 bytes**

#### Type 2: Configuration (`SerialServer2HoverConfig`)

| Offset | Size | Field      | Description                                       |
|--------|------|------------|---------------------------------------------------|
| 0      | 1    | cStart     | Start byte: `0x2F` (`'/'`)                        |
| 1      | 1    | iDataType  | Message type: `0x02`                              |
| 2      | 1    | iSlave     | Target slave ID                                   |
| 3      | 4    | fBattFull  | Full battery voltage (float, e.g. 42.0)           |
| 7      | 4    | fBattEmpty | Empty battery voltage (float, e.g. 27.0)          |
| 11     | 1    | iDriveMode | Drive mode (see Drive Modes below)                |
| 12     | 1    | iSlaveNew  | New slave ID (-1 = no change, currently disabled) |
| 13     | 2    | checksum   | CRC-16 (big-endian)                               |

**Total size: 15 bytes**

### Hoverboard → Server Response (`SerialHover2Server`)

Sent by the hoverboard after receiving a valid command addressed to its slave ID.

| Offset | Size | Field    | Description                                      |
|--------|------|----------|--------------------------------------------------|
| 0      | 2    | cStart   | Start frame: `0xABCD` (little-endian: 0xCD 0xAB) |
| 2      | 1    | iSlave   | Slave ID of responding board                     |
| 3      | 2    | iSpeed   | Current speed × 10 (int16, 0.1 km/h units)       |
| 5      | 2    | iVolt    | Battery voltage in mV (uint16)                   |
| 7      | 2    | iAmp     | DC current × 100 (int16, 0.01A units)            |
| 9      | 4    | iOdom    | Odometer (int32, hall sensor steps)              |
| 13     | 2    | checksum | CRC-16 (big-endian)                              |

**Total size: 15 bytes**

---

## Detailed Field Descriptions

### Command Fields

#### iSpeed (Speed Setpoint)

The speed command field behavior depends on the active drive mode:

**Voltage Control Modes (COM_VOLT=0, SINE_VOLT=2):**
- Interpreted as a direct PWM duty cycle command
- Range: -1000 to +1000
- Conversion: `pwm = iSpeed * PWM_RES / 1000`
- With default PWM_RES=8192: full range maps to ±8192 PWM counts
- Positive values = forward rotation, negative = reverse

**Speed Control Modes (COM_SPEED=1, SINE_SPEED=3):**
- Interpreted as an RPM setpoint for the PID controller
- The PID controller adjusts PWM to achieve the target speed
- When iSpeed=0, PWM is forced to 0 and PID integrator is reset
- PID parameters (default for COM_SPEED): Kp=50, Ki=100, Kd=200

**Code Reference:** `bldc.c:160-176` - speedupdate() function

#### iSteer (Steering Value)

**Current Implementation:** The iSteer field is received and parsed in Type 1 messages but is **not used** in the current firmware. Only iSpeed is extracted and applied.

**Intended Use:** For differential steering in dual-motor hoverboards, where iSteer would modify the speed of each wheel to achieve turning.

**Code Reference:** `remoteUartBus.c:225-228` - Type 1 message handling

#### iSlave (Slave ID)

- Each motor controller has a configured slave ID (default: 1)
- Messages are only processed if iSlave matches the board's SLAVE_ID
- Valid range: 0-255
- Multiple boards can share one UART bus using different IDs
- The slave ID is stored in EEPROM at pinstorage[38]

**Code Reference:** `remoteUartBus.c:210` - Slave ID check

#### wState (State Flags)

**Current Implementation:** The wState field is received and stored in a global variable but the **LED control logic is not implemented** in the current firmware version. The main.c LED handling uses battery level indicators instead.

**Intended Behavior (per protocol design):**

| Bit | Value | Name        | Intended Behavior                             |
|-----|-------|-------------|-----------------------------------------------|
| 0   | 0x01  | ledGreen    | Turn on green LED                             |
| 1   | 0x02  | ledOrange   | Turn on orange LED                            |
| 2   | 0x04  | ledRed      | Turn on red LED                               |
| 3   | 0x08  | ledUp       | Turn on up indicator LED                      |
| 4   | 0x10  | ledDown     | Turn on down indicator LED                    |
| 5   | 0x20  | Battery3Led | Enable 3-LED battery indicator mode           |
| 6   | 0x40  | Disable     | Disable motor PWM output                      |
| 7   | 0x80  | ShutOff     | Trigger board shutdown (if latch pin present) |

**Code Reference:** `remoteUartBus.c:220,227` - wState is stored but not acted upon

#### wStateSlave (Type 1 only)

State flags intended for a paired slave motor in dual-motor configurations. Like wState, this is parsed but not currently implemented.

### Configuration Fields (Type 2)

#### fBattFull (Full Battery Voltage)

- IEEE 754 single-precision float (4 bytes)
- Represents 100% charge voltage in volts (e.g., 42.0 for 10S Li-ion)
- Valid range: 0.0 < fBattFull < 60.0
- Stored internally as millivolts: `BAT_FULL = fBattFull * 1000`
- Default: 42.0V (42000mV)
- Used for battery percentage calculation and LED color thresholds

**Code Reference:** `remoteUartBus.c:233-236`

#### fBattEmpty (Empty Battery Voltage)

- IEEE 754 single-precision float (4 bytes)
- Represents 0% charge voltage in volts (e.g., 32.0 for 10S Li-ion)
- Valid range: 0.0 < fBattEmpty < 60.0
- Stored internally as millivolts: `BAT_EMPTY = fBattEmpty * 1000`
- Default: 32.0V (32000mV)
- **Low Battery Protection:** If battery drops below BAT_EMPTY for 10 consecutive readings while motor is stopped, the board enters permanent low-battery mode (motor disabled, buzzer warning)

**Code Reference:** `remoteUartBus.c:237-240`, `bldc.c:147-157`

#### iDriveMode (Drive Mode)

Selects the motor commutation and control strategy:

| Value | Name       | Description                                               |
|-------|------------|-----------------------------------------------------------|
| 0     | COM_VOLT   | Trapezoidal commutation with direct voltage (PWM) control |
| 1     | COM_SPEED  | Trapezoidal commutation with PID speed control (default)  |
| 2     | SINE_VOLT  | Sinusoidal commutation with direct voltage control        |
| 3     | SINE_SPEED | Sinusoidal commutation with PID speed control             |
| 4     | FOC_VOLT   | Field-oriented control, voltage mode (not via protocol)   |
| 5     | FOC_SPEED  | Field-oriented control, speed mode (not via protocol)     |
| 6     | FOC_TORQUE | Field-oriented control, torque mode (not via protocol)    |

**Note:** Only modes 0-3 can be set via the protocol (validated: `iDriveMode <= SINE_SPEED`). FOC modes 4-6 exist in firmware but cannot be enabled remotely.

**Side Effects:** Changing drive mode triggers:
1. PID parameter reinitialization (`PID_Init()`)
2. PID state reset (`PIDrst()`)
3. PWM output reconfiguration (`TIMOCInit()`)
4. EEPROM save of configuration

**Code Reference:** `remoteUartBus.c:241-247`, `bldc.c:193-216`

#### iSlaveNew (New Slave ID)

- Intended to allow remote slave ID reassignment
- **Currently Disabled:** The firmware contains `if (pData->iSlaveNew >= 0 && 0)` which always evaluates to false
- If enabled, would save new ID to EEPROM

**Code Reference:** `remoteUartBus.c:248-249`

---

### Response Fields

#### iSpeed (Current Speed)

- Calculated from hall sensor timing as motor RPM
- Transmitted as: `realspeed * 10`
- Formula: `realspeed = SPEEDX / HallTimeSum` where:
  - `SPEEDX = 60000000 / WINDINGS * 2` (default WINDINGS=30, so SPEEDX=4000000)
  - `HallTimeSum` = sum of last 6 hall sensor transition times in microseconds
- Sign indicates direction: positive = forward, negative = reverse
- Zeroed if no hall transitions for 250ms

**Units Clarification:** Despite the struct comment "100* km/h", the actual value is `RPM * 10`. To convert to km/h, the master must know the wheel diameter.

**Code Reference:** `hallhandle.c:101-106`, `remoteUartBus.c:152`

#### iVolt (Battery Voltage)

- 32-sample moving average of ADC battery voltage readings
- Units: millivolts (mV)
- Calculation: `vbat = VBAT_DIVIDER * ADC_value * Vcc * 100 / 4096`
  - Default VBAT_DIVIDER=31 (resistor divider ratio)
  - Vcc is measured via internal reference (nominally 3.3V)
- Example: 36500 = 36.5V

**Code Reference:** `interrupt.c:94`, `calculation.c:102-110`

#### iAmp (DC Bus Current)

- 64-sample moving average of ADC current sensor readings
- Units: centiamps (0.01A), signed
- Calculation: `itotal = ITOTAL_DIVIDER * (ADC_value - offset) * Vcc * 100 / 4096`
  - Default ITOTAL_DIVIDER=250 (based on current sense amplifier gain)
  - Offset calibrated at startup (128-sample average with motor off)
- Example: 250 = 2.50A, -150 = -1.50A (regenerative braking)

**Code Reference:** `interrupt.c:95-96`, `calculation.c:112-120`

#### iOdom (Odometer)

- Cumulative hall sensor step counter (int32, signed)
- Increments by 1 for each hall state transition in forward direction
- Decrements by 1 for reverse direction
- Does **not** reset on power cycle (but initialized to 0 on first boot)
- One electrical revolution = 6 hall steps
- One mechanical revolution = 6 × pole_pairs hall steps (typically 6 × 15 = 90 for hoverboard motors)

**Distance Calculation:** `distance_m = iOdom / (6 × pole_pairs) × wheel_circumference_m`

**Code Reference:** `hallhandle.c:117-121`

---

## State Flags (wState)

Bit field for LED control and motor state:

| Bit | Value | Name        | Description           |
|-----|-------|-------------|-----------------------|
| 0   | 0x01  | ledGreen    | Green LED on          |
| 1   | 0x02  | ledOrange   | Orange LED on         |
| 2   | 0x04  | ledRed      | Red LED on            |
| 3   | 0x08  | ledUp       | Up indicator LED on   |
| 4   | 0x10  | ledDown     | Down indicator LED on |
| 5   | 0x20  | Battery3Led | Battery 3-LED mode    |
| 6   | 0x40  | Disable     | Disable motor output  |
| 7   | 0x80  | ShutOff     | Shutdown the board    |

## Drive Modes (iDriveMode)

| Value | Name       | Description                        |
|-------|------------|------------------------------------|
| 0     | COM_VOLT   | Commutated voltage control         |
| 1     | COM_SPEED  | Commutated speed control (default) |
| 2     | SINE_VOLT  | Sinusoidal voltage control         |
| 3     | SINE_SPEED | Sinusoidal speed control           |
| 4     | FOC_VOLT   | Field-oriented voltage control     |
| 5     | FOC_SPEED  | Field-oriented speed control       |
| 6     | FOC_TORQUE | Field-oriented torque control      |

Note: Only modes 0-3 are accepted via the configuration message.

## CRC-16 Calculation

The protocol uses CRC-16/CCITT with polynomial `0x1021`, initial value `0`, and no final XOR.

The CRC is calculated over all bytes **except** the 2-byte checksum field.

### Algorithm (C)

```c
uint16_t CalcCRC(uint8_t *ptr, int count) {
    uint16_t crc = 0;
    uint8_t i;
    while (--count >= 0) {
        crc = crc ^ (uint16_t)*ptr++ << 8;
        i = 8;
        do {
            if (crc & 0x8000) {
                crc = crc << 1 ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        } while (--i);
    }
    return crc;
}
```

### Algorithm (JavaScript)

```javascript
function calculateCRC16(data) {
    let crc = 0;
    for (let byte of data) {
        crc ^= (byte << 8);
        for (let i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF;
            } else {
                crc = (crc << 1) & 0xFFFF;
            }
        }
    }
    return crc;
}
```

## Protocol Flow

1. **Master sends command**: A command packet (Type 0, 1, or 2) is sent to a specific slave ID
2. **Slave validates**: The slave checks:
   - Start byte is `'/'` (0x2F)
   - Message type is valid (0, 1, or 2)
   - Slave ID matches its configured ID
   - CRC is correct
3. **Slave processes**: If valid, the slave:
   - Updates speed/state/configuration as specified
   - Resets the serial timeout counter
4. **Slave responds**: Sends a `SerialHover2Server` response with current telemetry

## Timeout Behavior

If no valid command is received within the timeout period (default 1000ms), the motor speed setpoint is set to zero as a safety measure. The timeout is reset each time a valid command addressed to this slave is received.

**Code Reference:** `remoteUartBus.c:130-132`

## Low Battery Protection

The firmware implements automatic low-battery shutdown:

1. Battery voltage is continuously monitored against BAT_EMPTY threshold
2. If `vbat < BAT_EMPTY` for 10 consecutive readings AND motor is stopped (`speed==0` and `realspeed<100`), the board enters permanent low-battery mode
3. In low-battery mode:
   - Motor PWM is disabled
   - Buzzer sounds intermittently (700ms on/off)
   - Red LED flashes
4. Recovery requires power cycle with sufficient battery voltage

**Code Reference:** `bldc.c:147-157`, `main.c:197-203`

## Example Packets

### Type 0: Set speed to 100, green LED on, slave ID 1

```
Bytes: 2F 00 01 64 00 01 [CRC-H] [CRC-L]
       │  │  │  └──┴── iSpeed = 100 (0x0064 little-endian)
       │  │  └─ iSlave = 1
       │  └─ iDataType = 0
       └─ cStart = '/'
```

### Response: Speed 5.5 km/h, 36.5V, 2.5A, odometer 12345

```
Bytes: CD AB 01 37 00 71 8E FA 00 39 30 00 00 [CRC-H] [CRC-L]
       └──┴── cStart = 0xABCD (little-endian)
              │  └──┴── iSpeed = 55 (5.5 km/h × 10)
              └─ iSlave = 1
                       └──┴── iVolt = 36465 mV
                              └──┴── iAmp = 250 (2.5A × 100)
                                     └──────┴── iOdom = 12345
```

## Bus Configuration

Multiple hoverboard controllers can share the same UART bus by assigning unique slave IDs. The master addresses commands to specific slaves, and only the matching slave responds. This enables:

- Dual-wheel hoverboard control (2 slaves)
- Multi-board configurations
- Expandable motor control networks

## Default EEPROM Configuration

The firmware ships with these default values in pinstorage[]:

| Index | Name           | Default Value | Description                    |
|-------|----------------|---------------|--------------------------------|
| 33    | VBAT_DIVIDER   | 31            | Battery voltage divider ratio  |
| 34    | ITOTAL_DIVIDER | 250           | Current sense amplifier gain   |
| 36    | BAUD           | 19200         | UART baud rate                 |
| 37    | PWM_RES        | 8192          | PWM resolution (timer period)  |
| 38    | SLAVE_ID       | 1             | This board's slave address     |
| 39    | WINDINGS       | 30            | Motor pole pairs               |
| 41    | SOFT_ILIMIT    | 10            | Software current limit         |
| 44    | DRIVEMODE      | 1             | Default drive mode (COM_SPEED) |
| 45    | BAT_FULL       | 42000         | Full battery (mV)              |
| 46    | BAT_EMPTY      | 32000         | Empty battery (mV)             |
| 47    | SERIAL_TIMEOUT | 1000          | Command timeout (ms)           |

**Code Reference:** `main.c:61`

## References

- Server implementation: [remoteUartBus.c](https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x)
- Client example: [remoteuartbus-online](https://gitlab.com/ailife8881/remoteuartbus-online)
