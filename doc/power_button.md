# Power Button Behavior in Hoverboard Firmware

This document describes how the MM32 hoverboard firmware handles power buttons and power latching on master and slave boards.

## Overview

Hoverboard driver boards come in two configurations:

- **Master board**: Has a power button and power latch circuit. Controls power-on/power-off for the entire hoverboard.
- **Slave board**: No power button or latch. Receives power from the master board and runs immediately when power is applied.

The firmware uses a pin configuration system (`pinstorage` array) to determine which type of board it's running on.

## Pin Configuration

The relevant pins are defined in `Inc/pinout.h`:

```c
#define BUTTONPIN pinstorage[9]   // Power button input
#define LATCHPIN pinstorage[10]   // Power latch output
```

The firmware checks if these pins are valid by comparing against `PINCOUNT`:
- If `BUTTONPIN < PINCOUNT` and `LATCHPIN < PINCOUNT`: Board has power button functionality (master)
- If either is `>= PINCOUNT` (typically `0xFFFF`): Board lacks power button (slave)

## Firmware Behavior

### Startup Sequence (`Src/main.c:120-144`)

```c
//latch on power
if(LATCHPIN<PINCOUNT){    //have latch
    DELAY_Ms(100);    //some board the micro controller can reset in time and turn back on
    digitalWrite(LATCHPIN, 1);
    while(digitalRead(BUTTONPIN)&&BUTTONPIN<PINCOUNT){    //wait while release button
        if(BUZZERPIN<PINCOUNT){    //have buzzer
            digitalWrite(BUZZERPIN, 1);
            DELAY_Ms(2);
            digitalWrite(BUZZERPIN, 0);
            DELAY_Ms(2);
        }else{
            __NOP();
            __NOP();
        }
        IWDG_ReloadCounter();
    }
}
```

On a **master board** with latch:
1. Wait 100ms to prevent immediate reset
2. Set `LATCHPIN` HIGH to latch power on
3. Wait for button release (with buzzer feedback if available)
4. Play power-on melody
5. Continue to main loop

On a **slave board** without latch:
1. Skip the latch code entirely
2. Continue directly to main loop
3. Motor runs as soon as valid UART commands are received

### Shutdown Sequence (`Src/main.c:212-231`)

```c
if(LATCHPIN<PINCOUNT&&BUTTONPIN<PINCOUNT){
    if(digitalRead(BUTTONPIN)){    //button press for shutdown
        TIM1->CCR1=0;    //shut down motor
        TIM1->CCR2=0;
        TIM1->CCR3=0;
        if(BUZZERPIN<PINCOUNT){
            for(int i=0;i<3;i++){    //power off melody
                digitalWrite(BUZZERPIN, 1);
                DELAY_Ms(150);
                digitalWrite(BUZZERPIN, 0);
                DELAY_Ms(150);
            }
        }
        while(digitalRead(BUTTONPIN)) {    //wait for release
            __NOP();
            IWDG_ReloadCounter();
        }
        digitalWrite(LATCHPIN, 0);    //last line to ever be executed
        while(1);//incase the hardware failed...
    }
}
```

On a **master board**:
1. Continuously checks button state in main loop
2. When pressed: stops motor PWM, plays shutdown melody
3. Waits for button release
4. Sets `LATCHPIN` LOW to cut power
5. Enters infinite loop as failsafe

On a **slave board**:
1. This code block is never executed (condition fails)
2. Slave only stops when power is cut by master or UART timeout

### GPIO Initialization (`Src/initialize.c:37-42`)

```c
if(LATCHPIN<PINCOUNT){
    pinMode(LATCHPIN, OUTPUT);
}
if(BUTTONPIN<PINCOUNT){
    pinMode(BUTTONPIN, INPUT);
}
```

Pins are only configured if they are valid, preventing accidental GPIO conflicts on slave boards.

## Hardware Power Latch Circuit

Master boards have a **hardware power latch circuit** that controls whether the MCU receives power:

```
Battery ──► Power Latch Circuit ──► MCU Power
                    │
                    ├── Button Input (momentary)
                    └── Latch Enable (from MCU GPIO)
```

**How it works:**
1. When battery is connected, the MCU has **no power** (latch is open)
2. Pressing the button temporarily closes the circuit, powering the MCU
3. The MCU must quickly set `LATCHPIN` HIGH to keep the latch closed
4. Releasing the button: if latch is set, power stays on; if not, power cuts

**Critical implication:** The firmware can only run AFTER the MCU has power. On a master board with a hardware latch, simply changing firmware configuration (`pinstorage[9]` and `pinstorage[10]` to `0xFFFF`) will cause the firmware to skip setting the latch, and **power will be cut as soon as the button is released**.

## Recommendations for Running Without Power Button

To have the firmware run immediately when power is applied, you must address both hardware and firmware:

### Step 1: Hardware Modification (Required for Master Boards)

Master boards with a hardware power latch **require physical modification** to bypass the latch circuit:

1. **Identify the latch circuit**: Look for a MOSFET or transistor near the power input that is controlled by the `LATCHPIN` GPIO
2. **Bypass options**:
   - **Bridge the latch MOSFET**: Connect drain to source (or collector to emitter) so power flows directly
   - **Tie the gate/base HIGH**: Force the switching element to stay on
   - **Cut the latch trace**: Remove the MCU's control and wire the enable signal directly to the power rail
3. **Optional - remove button pull**: If the button has a pull-up/down that affects startup, you may need to adjust it

**Warning**: Hardware modifications require understanding of the specific board's power circuit. Incorrect modifications could damage the board or create safety hazards.

### Step 2: Firmware Configuration (After Hardware Bypass)

Once hardware provides always-on power, configure the firmware to skip power button logic:

**Option A: Configure via EEPROM**

Set `BUTTONPIN` and `LATCHPIN` to invalid values:
```
pinstorage[9]  = 0xFFFF  (BUTTONPIN)
pinstorage[10] = 0xFFFF  (LATCHPIN)
```

This can be done via:
- The PinFinder firmware's autodetect (if it detects no button/latch)
- Serial configuration commands (SerialServer2HoverConfig protocol)

**Option B: Modify Default pinstorage Array**

In `Src/main.c:61`, change the default `pinstorage` array:

```c
// Before (master board with button at pin 9, latch at pin 10):
uint16_t pinstorage[64]={..., 9, 10, ...};

// After (no button/latch):
uint16_t pinstorage[64]={..., 0xFFFF, 0xFFFF, ...};
```

The relevant indices are:
- Index 9: BUTTONPIN
- Index 10: LATCHPIN

Set `EEPROMEN` to 0 to use compiled defaults instead of EEPROM:
```c
#define EEPROMEN 0
```

### Slave Boards (No Modification Needed)

Slave boards typically:
- Have no hardware power latch (power comes directly from master or external supply)
- Already have `BUTTONPIN` and `LATCHPIN` set to `0xFFFF`
- Run immediately when power is applied

If your board is already a slave configuration, no changes are needed.

## Master/Slave Communication

In a typical hoverboard:
- **Master board** handles: power button, power latch, and may forward commands to slave
- **Slave board** receives: UART commands from master or external controller

Both boards use the RemoteUartBus protocol. The slave responds to messages matching its `SLAVE_ID`.

## UART Timeout Behavior

When no valid UART commands are received within `SERIAL_TIMEOUT` milliseconds (`pinstorage[47]`), the motor speed is set to 0:

```c
// Src/remoteUartBus.c:127-129
if (millis - iTimeLastRx > SERIAL_TIMEOUT){
    speed = 0;
}
```

This is a safety feature independent of the power button mechanism.

## Summary Table

| Feature            | Master Board                     | Slave Board          |
|--------------------|----------------------------------|----------------------|
| BUTTONPIN          | Valid pin (< 33)                 | 0xFFFF               |
| LATCHPIN           | Valid pin (< 33)                 | 0xFFFF               |
| Power-on behavior  | Button press required, latch set | Immediate on power   |
| Power-off behavior | Button press cuts power          | Power cut externally |
| Motor timeout      | Yes (UART timeout)               | Yes (UART timeout)   |

## File References

- Pin definitions: `Inc/pinout.h:25-26`
- Default configuration: `Src/main.c:61`
- GPIO initialization: `Src/initialize.c:37-42`
- Startup latch: `Src/main.c:120-144`
- Shutdown handling: `Src/main.c:212-231`
- EEPROM restore: `Src/sim_eeprom.c:239-254`
