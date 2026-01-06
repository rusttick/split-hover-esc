# PinFinder Firmware

An interactive serial-based configuration tool for hoverboard motor controller boards. PinFinder automatically detects hardware pin assignments and stores them in EEPROM for use by the main motor control firmware.

## Overview

Hoverboard controller boards from different manufacturers use different GPIO pin assignments for hall sensors, LEDs, buttons, and other peripherals. PinFinder solves this by:

1. Auto-detecting which GPIO pins are connected to which functions
2. Storing the detected configuration in EEPROM
3. Generating configuration arrays that can be compiled into custom firmware

## Entering Pin Finder Mode

### Fresh Flash (No EEPROM Configuration)

When the firmware is flashed to a board with no valid EEPROM configuration:

1. **Power on the board** - The firmware checks for a valid magic number (`0xDCAB`) at EEPROM position 32
2. **Self-hold detection** - If no valid config exists:
   - All GPIO pins are pulled HIGH for 1 second (to latch power on boards with self-hold circuits)
   - The firmware attempts to detect the self-hold/latch pin by monitoring for voltage drops
   - If a latch pin is found: board is classified as "Master"
   - If no latch pin found: board is classified as "Slave"
3. **UART detection** - The firmware scans UART TX/RX pin combinations by sending test patterns
4. **Wait for terminal** - Once UART is found, displays "Welcome to PinFinder, press enter to continue"

### Existing Configuration

If valid EEPROM configuration exists (magic number present):
- The firmware loads the stored pin assignments
- Jumps directly to the "press enter to continue" prompt

### Serial Terminal Requirements

Connect to the board via USB-to-serial adapter:
- **Baud rate:** 19200
- **Data bits:** 8
- **Parity:** None
- **Stop bits:** 1
- **Terminal:** Must support ANSI escape codes (PuTTY, minicom, screen, etc.)

The firmware sends an ANSI terminal query (`\x1b[5n`) to verify compatibility.

## Main Menu

After pressing Enter, the main menu displays:

```
  _   _  _____     _______ ____    _   _    _    ____ _  __ __     ______
 | | | |/ _ \ \   / / ____|  _ \  | | | |  / \  / ___| |/ / \ \   / /___ \
 | |_| | | | \ \ / /|  _| | |_) | | |_| | / _ \| |   | ' /   \ \ / /  __) |
 |  _  | |_| |\ V / | |___|  _ <  |  _  |/ ___ \ |___| . \    \ V /  / __/
 |_| |_|\___/  \_/  |_____|_| \_\ |_| |_/_/   \_\____|_|\_\    \_/  |_____|

Board type:Master  MCU Voltage:3.30V  Serial number:XXXXXXXXXXXXXXXXXXXX
Welcome to PinFinder, press number key to choose action.
  (0)-Auto detect all pins.
  (1)-Auto detect HALL sensor.
  (2)-Auto detect LED and buzzer.
  (3)-Auto detect battery voltage.
  (4)-Auto detect total current.
  (5)-Auto detect power button.
  (6)-Set Slave ID
  (7)-Modify configurations manually by command line.
  (8)-Test motor rotation.
  (9)-Power off.
>
```

## Detection Modes

### Option 0: Auto Detect All Pins

Runs through all detection steps sequentially:
1. Hall sensors → 2. LEDs/buzzer → 3. Battery voltage → 4. Button (master only) → 5. Slave ID

Recommended for initial board setup.

### Option 1: Hall Sensor Detection

**How it works:**
1. The motor is driven in open-loop sensorless mode at 50% PWM
2. The firmware monitors all unused GPIO pins for signals matching hall sensor patterns
3. Each commutation step has expected hall sensor states; pins that don't match are eliminated
4. After several rotations, only the correct hall pins remain

**User interaction:**
- Press `Y` to start motor rotation
- Press `I` to invert low-side drive signals (if motor doesn't spin or trips overcurrent)
- Press `Enter` to return to menu

**Output:**
```
HALLA:PB5
HALLB:PA0
HALLC:PA1
Detection sucessful
```

**Warnings:**
- If detected pins don't support timer input capture, a warning is displayed
- Increase input voltage to 42V if detection takes too long

### Option 2: LED and Buzzer Detection

**How it works:**
1. All unused GPIO pins are set HIGH (or LOW in alternate mode)
2. The selected pin blinks (toggles every 100ms)
3. User visually identifies which LED lights up or buzzer sounds

**Controls:**
| Key     | Action                                    |
|---------|-------------------------------------------|
| `W`     | Next pin                                  |
| `S`     | Previous pin                              |
| `R`     | Save as Red LED                           |
| `G`     | Save as Green LED                         |
| `B`     | Save as Blue/Orange LED                   |
| `U`     | Save as Upper LED                         |
| `L`     | Save as Lower LED                         |
| `Z`     | Save as Buzzer                            |
| `M`     | Toggle mode (all HIGH vs all LOW default) |
| `Enter` | Return to menu                            |

**Tips:**
- If an LED is always on regardless of pin selection, it may be active-low or broken
- Use mode toggle (`M`) to identify active-high vs active-low LEDs

### Option 3: Battery Voltage Detection

**How it works:**
1. Motor PWM is set to 100% with all low-side FETs closed (to prevent reading phase current)
2. All ADC-capable pins are scanned
3. Voltages are calculated using the internal voltage reference
4. Pins showing 20-45V (typical battery range) are displayed

**User interaction:**
- Measure actual battery voltage with a multimeter
- Press the number (0-9) corresponding to the closest reading
- Press `F` to toggle filtering (show all ADC readings vs filtered)
- Press `Enter` to return to menu

**Output:**
```
0->PA4:36.50V
1->PA5:0.00V
2->PA6:36.48V
================
```

### Option 4: Total Current Detection

**Status:** Not implemented in current firmware version.

### Option 5: Power Button Detection

**Master boards only.** Slave boards display "I'm a Slave board."

**How it works:**
1. All GPIO pins are monitored for state changes
2. User presses and releases the power button
3. The pin that transitions from LOW to HIGH is saved as the button pin

**User interaction:**
- Press and release the power button
- Press `E` if button press isn't detected (enables latch pin workaround)
- Press `Enter` to return to menu

### Option 6: Set Slave ID

Sets the board's address for the Remote UART Bus protocol.

**User interaction:**
- Type a number 0-255
- Press `Enter` to save
- Press `Backspace` to correct

Each board on a shared UART bus must have a unique slave ID.

### Option 7: Command Line Interface

Advanced configuration editor with direct access to all pinstorage values.

**Commands:**
| Command            | Description                  |
|--------------------|------------------------------|
| `r <addr>`         | Read value at address        |
| `w <addr> <value>` | Write value to address       |
| `l`                | List all stored values       |
| `g`                | Generate C array initializer |
| `e`                | Erase all EEPROM data        |
| `h`                | Display help                 |
| `i`                | Show version info            |
| `x`                | Exit to main menu            |

**Address names:**
Addresses can be specified by number (0-63) or name:

| Index | Name          | Description                   |
|-------|---------------|-------------------------------|
| 0     | halla         | Hall sensor A pin             |
| 1     | hallb         | Hall sensor B pin             |
| 2     | hallc         | Hall sensor C pin             |
| 3     | ledr          | Red LED pin                   |
| 4     | ledg          | Green LED pin                 |
| 5     | ledb          | Blue LED pin                  |
| 6     | ledu          | Upper LED pin                 |
| 7     | ledd          | Lower LED pin                 |
| 8     | buzzer        | Buzzer pin                    |
| 9     | button        | Power button pin              |
| 10    | latch         | Self-hold latch pin           |
| 12    | vbat          | Battery voltage ADC pin       |
| 13    | itotal        | Total current ADC pin         |
| 14    | tx            | UART TX pin                   |
| 15    | rx            | UART RX pin                   |
| 23    | ocp           | Overcurrent protection pin    |
| 24    | ocpref        | OCP reference pin             |
| 32    | magicnum      | Magic number (0xDCAB)         |
| 33    | vbatdivider   | Battery voltage divider ratio |
| 34    | itotaldivider | Current sense divider         |
| 36    | baud          | UART baud rate                |
| 37    | pwmres        | PWM resolution                |
| 38    | slaveid       | Slave ID for UART protocol    |
| 39    | windings      | Motor pole pairs              |
| 40    | invlowside    | Invert low-side drive         |
| 41    | softocp       | Software overcurrent limit    |
| 42    | hardocp       | Hardware overcurrent (AWDG)   |
| 44    | drivemode     | Drive mode (0-3)              |
| 45    | batfull       | Full battery voltage (mV)     |
| 46    | batempty      | Empty battery voltage (mV)    |
| 47    | timeout       | Serial timeout (ms)           |

**Generating firmware configuration:**
```
pinfinder~$ g
uint16_t pinstorage[64]={65535, 65535, 65535, ... };
```
Copy this line into the main firmware's `main.c` to compile with pre-configured pins.

### Option 8: Test Motor Rotation

Tests motor operation using detected hall sensors.

**Requirements:** Hall sensors must be detected first.

**Controls:**
| Key     | Action                        |
|---------|-------------------------------|
| `+`     | Increase speed (+100)         |
| `-`     | Decrease speed (-100)         |
| `Enter` | Stop motor and return to menu |

Speed range: -1000 to +1000 (negative = reverse)

### Option 9: Power Off

**Master boards only.** Releases the latch pin to power down the board.

## Saving Configuration

When exiting a detection mode with changes:
```
Changes were made to the configurations, do you want to save it permanently? Y/N
```

- Press `Y` to save to EEPROM
- Press `N` to discard changes

## GPIO Pin Naming

The firmware uses an internal pin numbering scheme:

| Index | Pin  | Index | Pin  | Index | Pin  |
|-------|------|-------|------|-------|------|
| 0     | PA0  | 13    | PB0  | 26    | PC13 |
| 1     | PA1  | 14    | PB1  | 27    | PC14 |
| 2     | PA2  | 15    | PB2  | 28    | PC15 |
| 3     | PA3  | 16    | PB3  | 29    | PD0  |
| 4     | PA4  | 17    | PB4  | 30    | PD1  |
| 5     | PA5  | 18    | PB5  | 31    | PD2  |
| 6     | PA6  | 19    | PB6  | 32    | PD3  |
| 7     | PA7  | 20    | PB7  | 33    | PD7  |
| 8     | PA11 | 21    | PB8  |       |      |
| 9     | PA12 | 22    | PB9  |       |      |
| 10    | PA13 | 23    | PB10 |       |      |
| 11    | PA14 | 24    | PB11 |       |      |
| 12    | PA15 | 25    | PB12 |       |      |

**Reserved pins:** PA13 and PA14 (SWD interface) cannot be used.

## EEPROM Storage Format

Configuration is stored as 64 × 16-bit values:

- **Positions 0-31:** GPIO pin assignments (value = pin index, 65535 = not set)
- **Position 32:** Magic number (`0xDCAB`) indicating valid configuration
- **Positions 33-47:** Numeric configuration values
- **Positions 48-63:** Reserved

## Differences from Main Firmware

The PinFinder firmware is a separate build with different behavior:

| Aspect        | PinFinder                 | Main Firmware             |
|---------------|---------------------------|---------------------------|
| Purpose       | Configuration/detection   | Motor control             |
| UART use      | Interactive terminal      | Remote UART Bus protocol  |
| Motor control | Test mode only            | Full speed/torque control |
| ADC           | Detection scanning        | Continuous monitoring     |
| Main loop     | Menu-driven state machine | Real-time control         |

### Protocol Differences (Type 2 Configuration Message)

The Type 2 configuration message structure differs between firmwares:

**PinFinder branch:**
```c
typedef struct {
   uint8_t cStart;        // '/'
   uint8_t iDataType;     // 2
   uint8_t iSlave;
   float   fBatteryVoltage;  // Single battery threshold
   uint8_t iDivemode;        // Note: typo in variable name
   uint16_t checksum;
} SerialServer2HoverConfig;  // 10 bytes total
```

**Main firmware:**
```c
typedef struct {
   uint8_t cStart;        // '/'
   uint8_t iDataType;     // 2
   uint8_t iSlave;
   float   fBattFull;     // Full battery voltage
   float   fBattEmpty;    // Empty battery voltage
   uint8_t iDriveMode;
   int8_t  iSlaveNew;     // New slave ID assignment
   uint16_t checksum;
} SerialServer2HoverConfig;  // 15 bytes total
```

The PinFinder Type 2 handler is a no-op (receives but ignores the message).

## Workflow Summary

1. **Flash PinFinder firmware** to board
2. **Connect serial terminal** at 19200 baud
3. **Press Enter** when prompted
4. **Run auto-detect** (option 0) or individual detections
5. **Save configuration** when prompted
6. **Use CLI** (option 7) to generate pinstorage array if needed
7. **Flash main firmware** - it will use the EEPROM configuration
8. Or: Copy generated array into main firmware source and recompile

## Troubleshooting

### Motor doesn't spin during hall detection
- Try pressing `I` to invert low-side drive signals
- Increase battery voltage to 42V
- Check motor phase connections

### UART not detected
- The firmware scans TX/RX combinations automatically
- Ensure serial adapter is connected to correct pins
- Try swapping TX/RX connections

### "Your serial terminal is not supported"
- Use a terminal that supports ANSI escape codes
- Recommended: PuTTY, minicom, screen
- Avoid: Arduino Serial Monitor

### Board powers off during detection
- The button pin may be misdetected as the latch pin
- Use option 5 and press `E` to enable the workaround

## Version Info

Current version: **PinFinder beta v173**

Changelog: Colored user input text and warning beep
