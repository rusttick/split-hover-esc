//! Skid Steer Simulator using S.BUS
//!
//! Simulates a remote control skid steer based on S.BUS input.
//!
//! Channel mapping:
//! - ch2: Right track control
//! - ch3: Left track control
//! - ch5: Engine enable (switch)
//! - ch6: Engine start (momentary)
//! - ch7: Steering enable (switch)
//!
//! Hardware:
//! - GP9: S.BUS input from R8EF receiver (UART1 RX)

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::UART1;
use embassy_rp::uart::{self, Config, DataBits, Parity, StopBits, Uart};
use panic_probe as _;
use rtt_target::rtt_init_defmt;
use sbus_rs::SbusPacket;

bind_interrupts!(struct Irqs {
    UART1_IRQ => uart::InterruptHandler<UART1>;
});

/// Convert channel value (200-1800) to speed percentage (-100 to +100)
/// 200-999: reverse (-100% to 0%)
/// 1000-1800: forward (0% to +100%)
fn channel_to_speed(value: u16) -> i8 {
    if value < 200 {
        -100
    } else if value <= 999 {
        // Reverse: 200 = -100%, 999 = 0%
        // Map 200-999 to -100-0
        let range = 999 - 200; // 799
        let offset = value - 200;
        let percent = (offset as i32 * 100) / range as i32;
        (percent - 100) as i8
    } else if value <= 1800 {
        // Forward: 1000 = 0%, 1800 = +100%
        let range = 1800 - 1000; // 800
        let offset = value - 1000;
        let percent = (offset as i32 * 100) / range as i32;
        percent as i8
    } else {
        100
    }
}

/// Check if switch is enabled (>1400)
fn switch_enabled(value: u16) -> bool {
    value > 1400
}

/// Process and display skid steer state
fn process_packet(packet: &SbusPacket, frame_count: u32) {
    // Only log every 50th frame
    if !frame_count.is_multiple_of(50) {
        return;
    }

    // Channel assignments (0-indexed)
    let engine_enabled = switch_enabled(packet.channels[4]); // ch5
    let engine_start = switch_enabled(packet.channels[5]);   // ch6
    let steering_enabled = switch_enabled(packet.channels[6]); // ch7

    // Speed is 0 if steering disabled
    let (left_speed, right_speed) = if steering_enabled {
        (channel_to_speed(packet.channels[2]), channel_to_speed(packet.channels[1])) // ch3, ch2
    } else {
        (0, 0)
    };

    // Format speed with sign
    let left_sign = if left_speed >= 0 { '+' } else { '-' };
    let right_sign = if right_speed >= 0 { '+' } else { '-' };
    let left_abs = left_speed.unsigned_abs();
    let right_abs = right_speed.unsigned_abs();

    // Build status strings
    let engine_str = if engine_enabled { "Enabled" } else { "Disabled" };
    let steering_str = if steering_enabled { "Enabled" } else { "Disabled" };

    if engine_enabled && engine_start {
        info!(
            "L={}{:02}% R={}{:02}% Engine {} Steering {} Engine Starting",
            left_sign, left_abs, right_sign, right_abs, engine_str, steering_str
        );
    } else {
        info!(
            "L={}{:02}% R={}{:02}% Engine {} Steering {}",
            left_sign, left_abs, right_sign, right_abs, engine_str, steering_str
        );
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    rtt_init_defmt!(rtt_target::ChannelMode::BlockIfFull, 65536);

    info!("=== Skid Steer Simulator ===");

    let p = embassy_rp::init(Default::default());

    // Configure UART for S.BUS: 100000 baud, 8E2
    let mut config = Config::default();
    config.baudrate = 100_000;
    config.data_bits = DataBits::DataBits8;
    config.parity = Parity::ParityEven;
    config.stop_bits = StopBits::STOP2;

    // Create UART1 with RX on GP9 (TX on GP8, not used)
    let mut uart = Uart::new(
        p.UART1, p.PIN_8, p.PIN_9,
        Irqs, p.DMA_CH0, p.DMA_CH1, config,
    );

    // Set GPIO input inversion for RX pin (GP9)
    let io_bank0 = embassy_rp::pac::IO_BANK0;
    io_bank0.gpio(9).ctrl().modify(|w| {
        w.set_inover(embassy_rp::pac::io::vals::Inover::INVERT);
    });

    info!("UART1 on GP9, waiting for S.BUS...");

    let mut parser = sbus_rs::StreamingParser::new();
    let mut frame_count: u32 = 0;
    let mut buf = [0u8; 1];

    loop {
        match uart.read(&mut buf).await {
            Ok(()) => {
                match parser.push_byte(buf[0]) {
                    Ok(Some(packet)) => {
                        frame_count += 1;
                        process_packet(&packet, frame_count);
                    }
                    Ok(None) => {}
                    Err(_) => {}
                }
            }
            Err(_) => {}
        }
    }
}
