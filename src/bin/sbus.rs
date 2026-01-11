//! S.BUS Receiver using UART
//!
//! Receives S.BUS protocol from RadioLink R8EF receiver on GP5 using UART1.
//! S.BUS uses inverted UART at 100000 baud, 8E2 format.
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

/// Parse and display an S.BUS packet
fn display_sbus_packet(packet: &SbusPacket, frame_count: u32) {
    // Only log every 10th frame (~140ms at 14ms/frame)
    if !frame_count.is_multiple_of(50) {
        return;
    }

    // All channels on one line
    info!(
        "#{} ch[{} {} {} {} {} {} {} {}] fs={}",
        frame_count,
        packet.channels[0],
        packet.channels[1],
        packet.channels[2],
        packet.channels[3],
        packet.channels[4],
        packet.channels[5],
        packet.channels[6],
        packet.channels[7],
        packet.flags.failsafe
    );
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize RTT for debug output
    rtt_init_defmt!(rtt_target::ChannelMode::BlockIfFull, 65536);

    info!("=== S.BUS UART Receiver ===");

    // Initialize peripherals
    let p = embassy_rp::init(Default::default());

    // Configure UART for S.BUS: 100000 baud, 8E2
    let mut config = Config::default();
    config.baudrate = 100_000;
    config.data_bits = DataBits::DataBits8;
    config.parity = Parity::ParityEven;
    config.stop_bits = StopBits::STOP2;

    // Create UART1 with RX on GP9 (TX on GP8, not used)
    let mut uart = Uart::new(
        p.UART1, p.PIN_8, // TX (not used)
        p.PIN_9, // RX (S.BUS input)
        Irqs, p.DMA_CH0, p.DMA_CH1, config,
    );

    // Set GPIO input inversion for RX pin (GP9)
    // S.BUS is inverted, so we invert at GPIO level so UART sees normal signaling
    let io_bank0 = embassy_rp::pac::IO_BANK0;
    io_bank0.gpio(9).ctrl().modify(|w| {
        w.set_inover(embassy_rp::pac::io::vals::Inover::INVERT);
    });

    info!("UART1 configured: 100000 baud, 8E2, RX on GP9 (inverted)");
    info!("Waiting for S.BUS data...");

    // Create streaming parser
    let mut parser = sbus_rs::StreamingParser::new();
    let mut frame_count: u32 = 0;
    let mut byte_count: u32 = 0;
    let mut buf = [0u8; 1];

    // Main loop: read bytes from UART and feed to parser
    loop {
        match uart.read(&mut buf).await {
            Ok(()) => {
                let byte = buf[0];
                byte_count += 1;

                // Feed to streaming parser
                match parser.push_byte(byte) {
                    Ok(Some(packet)) => {
                        frame_count += 1;
                        display_sbus_packet(&packet, frame_count);
                    }
                    Ok(None) => {
                        // No complete packet yet, continue
                    }
                    Err(_e) => {
                        info!("Parse error at byte {}, resyncing...", byte_count);
                    }
                }
            }
            Err(e) => {
                info!("UART read error: {:?}", e);
            }
        }
    }
}
