# Flash Raspberry Pi Pico W

![pico w pinout](pico-w-pinout.png)


## Todo
- [ ] wire 2 motors and controllers with rs485 converters
- [ ] enable the 120ohm resistors on rs485 converters at both ends (slave board and rp2040) by shorting jumper pads on the rs485 board
- [ ] flash a program that sends 0 speed only and reads response
- [ ] flash ramping speed program
- [ ] add RC receiver to print debug messages only
- [ ] have RC receiver set speed
- [ ] add voltage supervisor KA75330 and a power filter capacitor and diode on Vsys
      https://www.youtube.com/watch?v=BqwUNOHiZ9k
      https://www.adafruit.com/product/3428
- [ ] pick battery, container, connectors, and board for rp2040, RC receiver, rs485, etc...
- [ ] solder and mount




## Make a Pico Debug Probe:

Get a pre-built debug probe binary:

```bash
wget https://github.com/raspberrypi/debugprobe/releases/latest/download/debugprobe_on_pico.uf2
```

Install the debug probe firmware per usual:

 - hold down bootsel and plug in the usb cable
 - drag and drop the .uf2 file


Wire the debug probe:

```
  | Probe Pico | Target Pico W | Function    |
  |------------|---------------|-------------|
  | GP2        | SWCLK         | Debug clock |
  | GP3        | SWDIO         | Debug data  |
  | GND        | GND           | Ground      |
  | Vsys       | Vsys          | Power

```

Then test the debug probe:

```bash
probe-rs list
```

And then flash as usual:

```bash
cargo run
```


## Check

Type-check without building:

```bash
cargo check              # check src/main.rs
cargo check --bin NAME   # check specific binary
```


## Build

Build firmware (output is ELF file):

```bash
cargo build              # debug build of src/main.rs
cargo build --bin NAME   # build specific binary
```


## Run

Connect the Pico W via the USB debug probe and:

```bash
cargo run              # flash src/main.rs
cargo run --bin NAME   # flash specific binary
```


## Monitor

Install cargo-monitor:

```bash
ln -sf $(pwd)/util/cargo-monitor ~/.cargo/bin/cargo-monitor
```

Monitor serial output only (reattach to running device):

```bash
cargo monitor              # monitor src/main.rs
cargo monitor --bin NAME   # monitor specific binary
```
