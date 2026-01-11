//! S.BUS Receiver using PIO
//!
//! Receives S.BUS protocol from RadioLink R8EF receiver on GP9 using PIO.
//! S.BUS uses inverted UART at 100000 baud, 8E2 format.
//!
//! Hardware:
//! - GP9: S.BUS input from R8EF receiver

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{
    self, Common, Config, FifoJoin, Instance, PioPin, ShiftDirection, StateMachine,
};
use embassy_rp::pio::program::{
    Assembler, InSource, JmpCondition, Program, SetDestination, WaitSource,
};
use fixed::types::U24F8;
use panic_probe as _;
use rtt_target::rtt_init_defmt;
use sbus_rs::SbusPacket;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

/// S.BUS baud rate
const SBUS_BAUD: u32 = 100_000;

/// System clock frequency (default for RP2040)
const SYS_CLK: u32 = 125_000_000;

/// Build the PIO program for S.BUS RX (inverted UART 8E2)
///
/// S.BUS uses:
/// - Inverted signaling (idle LOW, start bit HIGH on wire)
/// - 100000 baud
/// - 8 data bits
/// - Even parity (1 bit) - we skip this
/// - 2 stop bits - we skip these
///
/// We handle inverted signal directly (no GPIO inversion):
/// - Idle LOW, start bit HIGH
/// - Wait for HIGH, sample inverted data, invert in software
fn build_sbus_rx_program() -> Program<32> {
    let mut a = Assembler::<32>::new();

    // Labels for jumps
    let mut wrap_target = a.label();
    let mut bitloop = a.label();

    // Program for inverted UART (S.BUS):
    // 1. Wait for start bit (HIGH on inverted signal)
    // 2. Delay to center of first data bit
    // 3. Sample 8 data bits, 8 cycles apart
    // 4. Skip parity + 2 stop bits
    // 5. Push byte (will be inverted, fix in software)

    a.bind(&mut wrap_target);

    // Wait for start bit (pin goes HIGH - inverted start bit)
    // Don't wait for LOW first - that can cause us to skip bytes
    a.wait(1, WaitSource::PIN, 0, false);

    // Set X to 7 (will count down 8 times for 8 bits)
    // Delay to center of first data bit: 1.5 bit times = 12 cycles
    // set takes 1 cycle + 11 delay = 12 cycles
    a.set_with_delay(SetDestination::X, 7, 11);

    // Bitloop: sample 8 data bits
    a.bind(&mut bitloop);
    // Sample input pin into ISR (shift right)
    a.r#in(InSource::PINS, 1);
    // Decrement X, jump if not zero, with 6 cycle delay
    // Total loop: 1 (in) + 1 (jmp) + 6 (delay) = 8 cycles per bit
    a.jmp_with_delay(JmpCondition::XDecNonZero, &mut bitloop, 6);

    // After 8 data bits, skip parity (1 bit) + 2 stop bits = 3 bits = 24 cycles
    // Tuning: 32 was too slow (22 bytes/frame), 20 was too fast. Try 26.
    a.nop_with_delay(7); // 8 cycles
    a.nop_with_delay(7); // 8 cycles
    a.nop_with_delay(7); // 8 cycles
    a.nop_with_delay(1); // 2 cycles = 26 total

    // Push the 8-bit byte to RX FIFO
    a.push(false, true); // block=true to wait if FIFO full

    // Wrap back to start
    let mut wrap_source = a.label();
    a.bind(&mut wrap_source);

    a.assemble_with_wrap(wrap_source, wrap_target)
}

/// Configure a state machine for S.BUS reception
fn configure_sbus_rx<'d, PIO: Instance, const SM: usize>(
    common: &mut Common<'d, PIO>,
    sm: &mut StateMachine<'d, PIO, SM>,
    pin: embassy_rp::Peri<'d, impl PioPin>,
) {
    // Load the program
    let program = build_sbus_rx_program();
    let installed = common.load_program(&program);

    let mut cfg = Config::default();
    cfg.use_program(&installed, &[]);

    // Configure pin for PIO input
    let mut pin = common.make_pio_pin(pin);
    pin.set_input_sync_bypass(true);
    sm.set_pin_dirs(pio::Direction::In, &[&pin]);
    cfg.set_in_pins(&[&pin]);
    // JMP PIN uses the same pin (for potential framing error detection)
    cfg.set_jmp_pin(&pin);

    // Configure shift register
    // Shift right (LSB first, matching UART), autopush disabled
    cfg.shift_in.direction = ShiftDirection::Right;
    cfg.shift_in.auto_fill = false;
    cfg.shift_in.threshold = 8;

    // Use RX FIFO only (TX not needed)
    cfg.fifo_join = FifoJoin::RxOnly;

    // Clock divisor for 100000 baud with 8 cycles per bit
    // div = system_clock / (baud * cycles_per_bit)
    // div = 125_000_000 / (100_000 * 8) = 156.25
    let div = SYS_CLK as f32 / (SBUS_BAUD as f32 * 8.0);
    cfg.clock_divider = U24F8::from_num(div);

    // Apply config and enable
    sm.set_config(&cfg);
    sm.set_enable(true);
}

/// Parse and display an S.BUS packet
fn display_sbus_packet(packet: &SbusPacket, frame_count: u32) {
    // Only log every 50th frame
    if !frame_count.is_multiple_of(50) {
        return;
    }

    // All channels on one line
    info!(
        "#{} ch[{} {} {} {} {} {} {} {}] fs={}",
        frame_count,
        packet.channels[0], packet.channels[1], packet.channels[2], packet.channels[3],
        packet.channels[4], packet.channels[5], packet.channels[6], packet.channels[7],
        packet.flags.failsafe
    );
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize RTT for debug output
    rtt_init_defmt!(rtt_target::ChannelMode::BlockIfFull, 65536);

    info!("=== S.BUS PIO Receiver ===");
    info!("Pin: GP9 | Baud: {} | Format: 8E2 (SW invert)", SBUS_BAUD);

    // Initialize peripherals
    let p = embassy_rp::init(Default::default());

    // Initialize PIO
    let pio::Pio {
        mut common,
        mut sm0,
        ..
    } = pio::Pio::new(p.PIO0, Irqs);

    // Configure state machine for S.BUS reception on GP9
    configure_sbus_rx(&mut common, &mut sm0, p.PIN_9);

    info!("S.BUS receiver started, waiting for data...");

    // Create streaming parser
    let mut parser = sbus_rs::StreamingParser::new();
    let mut frame_count: u32 = 0;

    // Main loop: read bytes from PIO and feed to parser
    let mut byte_count: u32 = 0;
    loop {
        // Wait for and read a byte from PIO RX FIFO
        let word = sm0.rx().wait_pull().await;
        byte_count += 1;

        // Data is in upper 8 bits (shift right fills from MSB)
        // Also inverted (S.BUS), XOR with 0xFF to uninvert
        let byte = ((word >> 24) as u8) ^ 0xFF;

        // Log bytes - first 60, then periodically
        if byte_count <= 60 || byte_count.is_multiple_of(500) {
            info!("PIO {}: 0x{:02X}", byte_count, byte);
        }

        // Feed to streaming parser
        match parser.push_byte(byte) {
            Ok(Some(packet)) => {
                frame_count += 1;
                display_sbus_packet(&packet, frame_count);
            }
            Ok(None) => {}
            Err(_) => {
                if byte_count <= 100 {
                    info!("Parse err at byte {}", byte_count);
                }
            }
        }
    }
}
