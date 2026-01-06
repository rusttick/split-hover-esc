# Connect Razor board



# Flash and Configure Razor Board

## Power button

 - Connect a momentary single pole button across the 2 pins of the red JST connector


## Motor

 - motor phase bullet connectors left to right: [Blue, Yellow, Green]
 - hall 6-pin JST: [Red, Blue, Yellow, Green, Black, White]


## ST-LINK V2

 - Solder on 5 header pins for connection to ST-Link V2
 - Board [DIO, CLK, 5V, G] to ST-LINK [SWDIO, SWCLK, 3.3V, GND]
 - Board [RST] must connect to GND because the generic ST-link v2 doesn't have reset connected correctly for this MCU
 - I added a JST connector and toggle switch between board Reset and board ground for convenience


## pyocd

install and configure pyocd


```bash
pip install pyocd
pyocd pack --update
pyocd pack install mm32spin05pf
pyocd list --targets | grep -i mm32
pyocd list  # should show STM32 STLink
```


## Binary Files

identify the binary files to flash:

```bash
util/HoverboardOutputMM32SPIN05_pinfinder_5_3_24.hex
util/HoverboardOutputMM32SPIN05_main_5_1_24.hex
```


## 1. Leave power off

Motor connections don't matter for flashing, but should be connected on pin finder boot.


## 2. Connect ST-Link V2

Connect to header, JST ground connector, toggle switch ON to connect RST to Ground


## 3. Erase first:

Run pyocd to erase the chip.

```bash
pyocd erase -t mm32spin05pf --chip
# immediately toggle switch RST floating
```


## 4. Load pin finder

Run pyocd to load the pin finder firmware.
ensure --no-reset to give timme to make all connections for first boot

```bash
pyocd load --no-reset --target mm32spin05pf util/HoverboardOutputMM32SPIN05_pinfinder_5_3_24.hex
```


## 5. Disconnect the ST-LINK V2

Remove it from the header pins and remove the JST connector that connected RST to Ground.


## 6. Connect Serial

 - the blue JST connector contains the serial interface pins
 - set the jumper on the FTDI to the 3.3v position
 - pin mapping: board [G, T2, R2] to FTDI [SGND, RX, TX] ... TX/RX cross


## 7. First Boot

The first thing pin finder does is detect the power button on first boot.

 - press and release the power button and wait 5 seconds.


## 8. Second Boot

When pin finder boots for the second time, it will try to find the Serial TX and RX pins.

 - press and release the power button
 - quickly short tx and rx together for about 1 second.
   I made a JST connector for this
 - at this point, I disconnect power and FTDI in preparation for a repeatable way to connect to pin finder

## 9. Connect serial

 - plug in FTDI
 - plug in 24v 1a power
 - run screen and immediately press and release the power button

```bash
screen /dev/cu.usbserial-A5069RR4 19200
```

 - you should see a banner and dots printing .......
 - hit enter to start the pin finder interface


## 10. Pin Finder Configuration Values (pinstorage[64])

GPIO Pin Assignments (indices 0-31)

  | Index | Name     | Description                               | Setting   |
  |       |          |                                           |           |
  |-------|----------|-------------------------------------------|-----------|
  | 0     | halla    | Hall sensor A input                       | 26 (PC13) |
  | 1     | hallb    | Hall sensor B input                       | 27 (PC14) |
  | 2     | hallc    | Hall sensor C input                       | 28 (PC15) |
  | 3     | ledr     | Red LED output                            | 31 (PD2)  |
  | 4     | ledg     | Green LED output                          | 9 (PA12)  |
  | 5     | ledb     | Blue LED output (orange on some boards)   |           |
  | 6     | ledu     | Upper/auxiliary LED output                |           |
  | 7     | ledd     | Lower/auxiliary LED output                |           |
  | 8     | buzzer   | Buzzer/beeper output                      | 22 (PB9)  |
  | 9     | button   | Power button input                        | 18 (PB5)  |
  | 10    | latch    | Self-hold/latch pin (keeps board powered) | 15 (PB2)  |
  | 11    | charge   | Charger detection input                   |           |
  | 12    | vbat     | Battery voltage ADC input                 | 14 (PB1)  |
  | 13    | itotal   | Total current sense ADC input             |           |
  | 14    | tx       | UART TX output                            | 19 (PB6)  |
  | 15    | rx       | UART RX input                             | 17(PB4)   |
  | 16    | iphasea  | Phase A current sense (reserved)          |           |
  | 17    | iphaseb  | Phase B current sense (reserved)          |           |
  | 18-22 | reserved | Reserved for future use                   |           |
  | 23    | ocp      | Over-current protection input             |           |
  | 24    | ocpref   | OCP reference voltage output              |           |
  | 25    | irl      | IR sensor left (reserved)                 |           |
  | 26    | irr      | IR sensor right (reserved)                |           |


System Configuration (indices 32-47)

Values store numeric settings, not pin indices.

  | Index | Name          | Default | Description                                       | Setting             |
  |-------|---------------|---------|---------------------------------------------------|---------------------|
  | 32    | magicnum      | 0xDCAB  | EEPROM validation magic number                    |                     |
  | 33    | vbatdivider   | 31      | Battery voltage divider ratio                     |                     |
  | 34    | itotaldivider | 250     | Current sense divider/scaling                     |                     |
  | 35    | reserved      | -       | (iphase divider, unused)                          |                     |
  | 36    | baud          | 19200   | UART baud rate                                    |                     |
  | 37    | pwmres        | 8192    | PWM resolution (higher = lower freq)              |                     |
  | 38    | slaveid       | 1       | Board address for multi-board comms (0-255)       |                     |
  | 39    | windings      | 30      | Motor winding count for speed calc                |                     |
  | 40    | invlowside    | 0       | Invert low-side gate drive (0=normal, 1=inverted) |                     |
  | 41    | softocp       | 10      | Software over-current limit threshold             |                     |
  | 42    | hardocp       | 300     | Hardware AWDG (analog watchdog) limit             |                     |
  | 43    | reserved      | -       | (UART mode, unused)                               | 1(probably default) |
  | 44    | drivemode     | 1       | Motor drive mode selection                        | XXX                 |
  | 45    | batfull       | 42000   | Battery full voltage (mV)                         |                     |
  | 46    | batempty      | 32000   | Battery empty voltage (mV)                        |                     |
  | 47    | timeout       | 1000    | Serial timeout (ms)                               |                     |
  | 48-63 | reserved      | 0       | Reserved for future use                           |                     |


## 12. test and save pinfinder settings




## 13. then flash main:

```bash
pyocd load -t mm32spin05pf util/HoverboardOutputMM32SPIN05_main_5_1_24.hex
```

## 14. and test with util/motor_control.py  ????






