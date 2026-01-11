# RadioLink R8EF Receiver - Rust Libraries for RP2040

The RadioLink R8EF is an 8-channel receiver supporting both PPM and S.BUS output protocols.
This document covers compatible Rust libraries for receiving and parsing these protocols on RP2040.

## Protocol Overview

### S.BUS Protocol
- **Baud rate**: 100,000 bps
- **Frame format**: 8E2 (8 data bits, even parity, 2 stop bits)
- **Signal**: Inverted UART (logic levels are inverted)
- **Frame structure**: 25 bytes total
  - Header: `0x0F`
  - Data: 22 bytes containing 16 channels (11 bits each)
  - Flags: 1 byte (failsafe, frame lost, digital channels 17-18)
  - Footer: `0x00`
- **Channel values**: 0-2047 (11-bit resolution)
- **Update rate**: ~14ms per frame (~71 Hz)

### PPM Protocol
- **Signal**: Pulse Position Modulation
- **Channels**: Encoded as gaps between pulses
- **Frame sync**: Long gap (>2.5ms typically) marks frame boundary
- **Channel values**: Encoded as pulse-to-pulse timing (~1000-2000us per channel)
- **Single wire**: All channels multiplexed on one signal

---

## S.BUS Libraries

### 1. sbus-rs (Recommended)

**Repository**: [AeroRust/sbus-rs](https://github.com/AeroRust/sbus-rs)
**Crate**: [sbus-rs on crates.io](https://crates.io/crates/sbus-rs)
**Documentation**: [docs.rs/sbus-rs](https://docs.rs/sbus-rs/latest/sbus_rs/)

A pure Rust, `no_std` compatible S.BUS parser from the AeroRust aerospace organization.

**Features**:
- `no_std` compatible for embedded systems
- Both blocking and async I/O support (mutually exclusive features)
- Streaming parser for incremental data processing
- Zero-copy parsing for efficient memory usage
- `defmt` feature for embedded logging

**Installation**:
```toml
# Blocking (default)
sbus-rs = "0.1.3"

# Async (for Embassy)
sbus-rs = { version = "0.1.3", default-features = false, features = ["async"] }

# With defmt logging
sbus-rs = { version = "0.1.3", features = ["defmt"] }
```

**Key Types**:
- `SbusParser<R>`: Main parser for reading from I/O
- `StreamingParser`: Incremental byte-by-byte parsing (ideal for UART)
- `SbusPacket`: Decoded frame with channel data and flags
- `Flags`: Status indicators (failsafe, frame_lost, d1, d2)

**Usage Example** (streaming):
```rust
use sbus_rs::StreamingParser;

let mut parser = StreamingParser::new();

// Called from UART interrupt or async read
fn on_byte_received(parser: &mut StreamingParser, byte: u8) {
    if let Some(packet) = parser.push_byte(byte) {
        // packet.channels[0..15] contains channel values (0-2047)
        // packet.flags.failsafe indicates receiver failsafe
        let throttle = packet.channels[0];
        let steering = packet.channels[1];
    }
}
```

### 2. parse_sbus

**Crate**: [parse_sbus on crates.io](https://crates.io/crates/parse_sbus)
**Documentation**: [docs.rs/parse_sbus](https://docs.rs/parse_sbus/latest/parse_sbus/)

A simpler S.BUS packet parser.

**Version**: 0.1.1
**License**: MIT

**Features**:
- Parses complete 25-byte S.BUS packets
- Exposes flag byte bits (failsafe, frame_lost, digital channels)

**Key Types**:
- `SbusPacket`: Parsed packet container
- `ParsingError` / `ErrorKind`: Error handling

**Note**: Less actively maintained than sbus-rs and fewer features.

### 3. sbus-parser

**Crate**: [sbus-parser on crates.io](https://crates.io/crates/sbus-parser)

Tagged as `#no-std` and `#embedded`. Uses byte-swap parsing approach.

**Note**: Limited documentation available.

---

## PPM Libraries

### 1. ppm_decode (Recommended)

**Repository**: [tstellanova/ppm_decode](https://github.com/tstellanova/ppm_decode)
**Crate**: [ppm_decode on crates.io](https://crates.io/crates/ppm_decode)
**Documentation**: [docs.rs/ppm_decode](https://docs.rs/ppm_decode/latest/ppm_decode/)

A `no_std` PPM decoder for embedded Rust.

**Version**: 0.1.3
**License**: BSD-3-Clause

**Features**:
- `no_std` compatible
- Does not require specific interrupt handling strategy
- Handles clock overflow
- Configurable timing parameters

**Key Types**:
- `PpmParser`: Main decoder
- `PpmFrame`: Container for channel values
- `ParserConfig`: Timing configuration
- `Microseconds` / `PpmTime`: Timing types

**Constants**:
- `MIN_CHAN_VAL` / `MAX_CHAN_VAL`: Channel value bounds
- `MID_CHAN_VAL`: Midpoint (typically 1500)
- `MIN_PPM_CHANNELS` / `MAX_PPM_CHANNELS`: Channel count limits
- `MIN_SYNC_WIDTH`: Frame sync gap threshold

**Usage Pattern**:
The library expects you to provide pulse timing from input capture or edge interrupts. The parser extracts frames from the timing data.

```rust
use ppm_decode::{PpmParser, ParserConfig};

let config = ParserConfig::default();
let mut parser = PpmParser::new(config);

// Called on each pulse edge (from timer capture or GPIO interrupt)
fn on_pulse_edge(parser: &mut PpmParser, timestamp_us: u32) {
    if let Some(frame) = parser.handle_pulse(timestamp_us) {
        // frame contains channel values
        let ch1 = frame.channels[0]; // Typically 1000-2000us
    }
}
```

**Hardware Test Reference**: [test_ppm_decode](https://github.com/tstellanova/test_ppm_decode) - Example on STM32F4 with microsecond timer capture.

---

## RP2040-Specific Considerations

### S.BUS Signal Inversion

S.BUS uses inverted UART signaling. On RP2040 with Embassy, you have two options:

#### Option A: Embassy UART with invert_rx (Simplest)

Embassy-rp's UART config supports signal inversion directly:

```rust
use embassy_rp::uart::{Config, Uart, Parity, StopBits, DataBits};

let mut config = Config::default();
config.baudrate = 100_000;
config.data_bits = DataBits::DataBits8;
config.parity = Parity::ParityEven;
config.stop_bits = StopBits::STOP2;
config.invert_rx = true;  // Enable RX signal inversion for S.BUS

let uart = Uart::new(p.UART1, p.PIN_4, p.PIN_5, Irqs, p.DMA_CH0, p.DMA_CH1, config);
```

#### Option B: PIO-based Inverted UART

For more control, implement an inverted UART using PIO state machines. The `embassy-rp` PIO module provides the necessary abstractions.

#### Option C: External Hardware Inverter

Use a transistor-based inverter circuit between the R8EF and the RP2040 UART RX pin.

### PPM Input Capture

PPM requires precise timing measurement of pulse edges. Options for RP2040:

#### Option A: PIO State Machine (Recommended)

Use PIO to measure pulse timing with cycle-accurate precision. PIO can:
- Wait for pin edges
- Count cycles between edges
- Push timing values to FIFO
- Trigger DMA transfers

See [embassy_rp::pio](https://docs.embassy.dev/embassy-rp/git/rp2040/pio/index.html) for the PIO API.

#### Option B: GPIO Interrupt + Timer

Use GPIO edge interrupts and read a timer value on each edge:

```rust
use embassy_rp::gpio::{Input, Pull, Level};
use embassy_time::Instant;

let ppm_pin = Input::new(p.PIN_X, Pull::Down);

loop {
    ppm_pin.wait_for_rising_edge().await;
    let timestamp = Instant::now();
    // Feed timestamp to ppm_decode parser
}
```

#### Option C: PWM Counter

RP2040's PWM can count input transitions, useful for frequency measurement but less ideal for PPM pulse timing.

---

## Related Crates in This Project

The current project already uses:
- `embassy-rp` (v0.9.0) - RP2040 HAL with PIO and UART support
- `embassy-executor` - Async runtime
- `heapless` - No-std collections
- `defmt` - Logging

These are compatible with the recommended libraries above.

---

## Recommendations

### For S.BUS (Recommended Protocol)

**Use `sbus-rs` with Embassy async UART**:

1. S.BUS is more robust than PPM (digital with CRC-like checking)
2. `embassy-rp` UART natively supports `invert_rx` - no hardware mods needed
3. `sbus-rs` has async support and `defmt` integration
4. StreamingParser handles partial frames gracefully

**Suggested Configuration**:
```toml
[dependencies]
sbus-rs = { version = "0.1.3", default-features = false, features = ["async", "defmt"] }
```

### For PPM

**Use `ppm_decode` with PIO-based timing**:

1. PPM is simpler but requires precise timing
2. Implement a PIO program to capture pulse edges and push timestamps to FIFO
3. Feed timestamps to `ppm_decode::PpmParser`

**Suggested Configuration**:
```toml
[dependencies]
ppm_decode = "0.1.3"
```

### Overall Recommendation: S.BUS

**S.BUS is strongly recommended** over PPM for this project because:

1. **No external hardware needed**: Embassy-rp's `invert_rx` handles signal inversion in software
2. **Higher reliability**: Digital protocol with error detection
3. **More channels**: 16 channels vs 8 for PPM
4. **Better library support**: `sbus-rs` is actively maintained with async support
5. **Simpler integration**: Standard UART peripheral vs custom PIO/timer code

### Implementation Priority

1. **Start with S.BUS + sbus-rs** - simplest path to working receiver input
2. **Add PPM as fallback** (optional) - if S.BUS integration has issues
3. **Consider PIO for both** - if you need custom timing behavior

---

## References

- [sbus-rs GitHub](https://github.com/AeroRust/sbus-rs)
- [ppm_decode GitHub](https://github.com/tstellanova/ppm_decode)
- [Embassy-rp UART Documentation](https://docs.embassy.dev/embassy-rp/git/rp2040/uart/index.html)
- [Embassy-rp PIO Documentation](https://docs.embassy.dev/embassy-rp/git/rp2040/pio/index.html)
- [RP2040 PIO Input Capture (C example)](https://people.ece.cornell.edu/land/courses/ece4760/RP2040/C_SDK_PIO_control/Input_capture/index_pio_control.html)
- [RadioLink R8EF Manual](https://www.radiolink.com/r8ef)
