# Driver Hoverboard Motors

Drive hoverboard motors using new driver boards from aliexpress.

 - Buy MM32SPIN05 driver pairs on aliexpress
 - use pre-built binary from https://gitlab.com/ailife8881/Hoverboard-Firmware-Hack-Gen2.x-MM32
   HoverboardOutputMM32SPIN05.hex downloaded from discord posted on 5/3/2024
 - flash like  https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x
 - control like https://github.com/reed-dan/hoverboard_hack_esp32_manualspeed
 - Insert modbus between for more robust serial communication

# TODO

- [ ] download firmware
- [ ] try to remember and document the flash setup and flash procedures

- [ ] try to use the serial interface for that board to control motor
- [ ] implement the serial motor control interface in rust on rp2040

# Flash rp2040

how to flash this project onto rp2040 for controlling many hoverboard motors


# Flash MM32SPIN05

how to download and flash the pre-built binary onto an MM32SPIN05







# Notes to my future self in chronological order

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





