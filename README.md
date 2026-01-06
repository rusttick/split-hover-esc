# Driver Hoverboard Motors

Drive hoverboard motors using new driver boards from aliexpress.

 - Buy MM32SPIN05 driver pairs on aliexpress
 - download binary of https://gitlab.com/ailife8881/Hoverboard-Firmware-Hack-Gen2.x-MM32 from discord
 - flash like  https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x
 - control like https://github.com/reed-dan/hoverboard_hack_esp32_manualspeed



## TODO
- [x] pinfinder needs higher voltage to find hall pins
- [x] 48v didn't work... swap out hall cables and double check pinout and try again
      with correct pinout, 24v worked!
- [x] find a suitable led to connect: 3 pin 12, R, G
- [ ] complete pin finder and flash main
- [ ] motor_control.py spins the motor


- [ ] speed1.rs spins the motor
- [ ] flash and run pinfinder on the slave. motor_control.py on slave. speed1.rs on slave
- [ ] speed1.rs over rs485 bridge at 19200
- [ ] get 2 boards with motors all talking over rs485 taking speed commands from rp2040
- [ ] get 4 boards with motors all talking over rs485 taking speed commands from rp2040



- [ ] disable master power button and deal with latch????
- [ ] xt30 power connectors
- [ ] test RC
- [ ] determine if wifi control will be adequate or if the RC protocol is required.
- [ ] decide on running voltage and mosfet cooling requirements



## RS485 serial bus
 - serial to rs485 converters running at the default 19200 serial bus speed
 - daisy chain wire topology required
 - short RO jumper to enable 120ohm termination at both ends of the chain
 - probably a separate chain to each sub-assembly
 - contains: https://www.ti.com/lit/ds/symlink/iso7741.pdf
 - and MAX485CSA+  https://www.analog.com/media/en/technical-documentation/data-sheets/max1487-max491.pdf
 - there should not be any protocol changes required. the rs485 chips have auto flow control







## Flash rp2040

how to flash this project onto rp2040 for controlling many hoverboard motors



## Hardware Connect Razor board:

 - motor phases
 - hall leads and the white wire??
 - red power button on master... what to do on slave????
 - blue serial interface to FTDI USB to serial interface with 3.3v jumper
   board [G, T2, R2] to FTDI [SGND, RX, TX] ... TX/RX cross
 - Solder on the header pins for connection to ST-Link V2:
   Board [DIO, CLK, 5V, G] to ST [SWDIO, SWCLK, 3.3V, GND]
   Board [RST] must connect to GND because the generic ST-link v2 doesn't have reset connected correctly for this MCU
 - I added a JST connector and toggle switch between board Reset and board Ground.


## Flash and Configure Razor Board

### 0. Plug in the st-link v2 (not connected to mcu) and Install pyocd with support for mm32spin05pf

```bash
pip install pyocd
pyocd pack --update
pyocd pack install mm32spin05pf
pyocd list --targets | grep -i mm32
pyocd list  # should show STM32 STLink
```

### 1. Leave power off. Motor connections don't matter for flashing, but should be connected on pin finder boot.
### 2. Connect ST-Link V2 and Reset to Ground JST connector
### 3. Toggle switch to connect the board RST pin to ground
### 4. get ready to quickly toggle the RST switch...
### 5. Erase first:

```bash
pyocd erase -t mm32spin05pf --chip
```

### 6. Then flash pin finder without running the code:

```bash
pyocd load --no-reset --target mm32spin05pf util/HoverboardOutputMM32SPIN05_pinfinder_5_3_24.hex
```

### 7. Disconnect the ST Link. leave the FTDI serial DISconnected as well.

### 8. `First boot` power button detection: Press and release the power button and wait 5 to 10 seconds

### 9. `Second boot` and Serial pin assignment: press and release the power button and then short tx and rx together for about 1 to 2 seconds ???

### 10. Connect serial and run pinfinder:

```bash
screen /dev/cu.usbserial-A5069RR4 19200
```

### 11. Pin Finder Configuration Values (pinstorage[64])

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


### 12. test and save pinfinder settings




### 13. then flash main:

```bash
pyocd load -t mm32spin05pf util/HoverboardOutputMM32SPIN05_main_5_1_24.hex
```

### 14. and test with util/motor_control.py  ????

????




## Notes to my future self in chronological order

* you wanted to make an RC lawnmower
* you wanted cheap motors with enough torque to move the mower up steep hills
* youtube thinks there are many unwanted hoverboards floating around
* you bought a bunch of hoverboards and ripped them apart
* you made piles of hoverboard connectors, boards, and motors
* you assumed you would want to buy new speed controllers for each motor
* you tried 6 different, cheap, speed controllers
* the only one that had a chance of working well was the $25 BLD-510B
* you liked using modbus RTU to talk to the BLD-510B...
  but then gave up when you could not figure out how to actually
  throttle the motor by setting speed values with modbus RTU
* if this all fails, go back and feed PWM and direction to the BLD-510B
  and save the modbus RTU for initial configuration only
* you still had piles of free boards sitting around, so you decided
  to start trying hoverboard hack firmware
* you bought a bunch of example MCU boards, serial interfaces, stlink interfaces
  and all the other junk you would need to try to put new firmware
  on hoverboard pcb
* you found https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x
  and started working through the process of identifying and flashing boards
* you burned some boards and stlink v2 (maybe.. they might still be good, not sure)
* you then decided to order some known-good hoverboard pcb from aliexpress
* now you had new firmware on one hoverboard pcb, but not yet control from another mcu
* Now you can't remember why, but for some reason you abandoned the serial control interface
  and decided to try rebuilding the whole MM32 project in rust.
* then after a month or so, you decided that wouldn't work
* so now you're back working on the serial interface to the pre-built binary





