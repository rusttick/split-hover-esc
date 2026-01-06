# Driver Hoverboard Motors

Drive hoverboard motors using new driver boards from aliexpress.

 - Buy MM32SPIN05 driver pairs on aliexpress
 - download binary of https://gitlab.com/ailife8881/Hoverboard-Firmware-Hack-Gen2.x-MM32 from discord
 - flash like  https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x
 - control like https://github.com/reed-dan/hoverboard_hack_esp32_manualspeed



## TODO
- [ ] pinfinder needs higher voltage to find hall pins
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

0. Plug in the st-link v2 (not connected to mcu) and Install pyocd with support for mm32spin05pf

```bash
pip install pyocd
pyocd pack --update
pyocd pack install mm32spin05pf
pyocd list --targets | grep -i mm32
pyocd list  # should show STM32 STLink
```

1. Leave power off. Motor connections don't matter for flashing, but should be connected on pin finder boot.
2. Connect ST-Link V2 and Reset to Ground JST connector
3. Toggle switch to connect the board RST pin to ground
4. get ready to quickly toggle the RST switch...
5. Erase first:

```bash
pyocd erase -t mm32spin05pf --chip
```

6. Then flash pin finder without running the code:

```bash
pyocd load --no-reset --target mm32spin05pf util/HoverboardOutputMM32SPIN05_pinfinder_5_3_24.hex
```

7. Disconnect the ST Link. leave the FTDI serial disconnected as well.

8. `First boot` power button detection: Press and release the power button and wait 5 to 10 seconds

9. `Second boot` and Serial pin assignment: press and release the power button and then short tx and rx together for about 1 to 2 seconds ???

10. Serial connect and run pinfinder:

```bash
screen /dev/cu.usbserial-A5069RR4 19200
```

Configure pinfinder with these settings?????

- ???




9. then flash main:

```bash
pyocd load -t mm32spin05pf util/HoverboardOutputMM32SPIN05_main_5_1_24.hex
```

9. and test with util/motor_control.py  ????













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





