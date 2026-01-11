//! Skid Steer Right Track Controller
//!
//! Receives S.BUS input and controls right track motor speed.
//! Combines S.BUS receiver input with PIO UART motor output.
//!
//! Channel mapping:
//! - ch2 (index 1): Right track control (200-1800 -> -100% to +100%)
//! - ch7 (index 6): Steering enable switch (must be enabled for motor to move)
//!
//! Hardware:
//! - GP9: S.BUS input from R8EF receiver (UART1 RX, inverted)
//! - GP4: Motor controller TX (PIO UART)
//! - GP5: Motor controller RX (PIO UART)
//! - Baud: S.BUS 100000 8E2, Motor 19200 8N1

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{self, InterruptHandler as PioInterruptHandler};
use embassy_rp::pio_programs::uart::{PioUartRx, PioUartRxProgram, PioUartTx, PioUartTxProgram};
use embassy_rp::uart::{Config as UartConfig, DataBits, Parity, StopBits, Uart};
use embassy_time::{Duration, with_timeout};
use embedded_io_async::{Read, Write};
use panic_probe as _;
use rtt_target::rtt_init_defmt;
use sbus_rs::SbusPacket;

use split_hover_esc::protocol::{self, Response};
use split_hover_esc::Irqs as UartIrqs;

bind_interrupts!(struct PioIrqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

/// Motor UART baud rate
const MOTOR_BAUD: u32 = 19200;

/// Timeout for motor response
const RESPONSE_TIMEOUT_MS: u64 = 50;

/// Motor slave IDs (both motors on right track)
const SLAVE_ID_1: u8 = 1;
const SLAVE_ID_2: u8 = 2;

/// Maximum motor speed value (±1000 = ±100%)
const MAX_MOTOR_SPEED: i16 = 1000;

/// Convert S.BUS channel value (200-1800) to motor speed (-1000 to +1000)
/// Center (1000) = 0, with deadband around center
fn channel_to_motor_speed(value: u16) -> i16 {
    const CENTER: i16 = 1000;
    const DEADBAND: i16 = 50; // ±50 around center = no movement
    const MIN_INPUT: i16 = 200;
    const MAX_INPUT: i16 = 1800;

    let value = value as i16;

    // Apply deadband around center
    if value > CENTER - DEADBAND && value < CENTER + DEADBAND {
        return 0;
    }

    if value <= CENTER - DEADBAND {
        // Reverse: 200 = -1000, (CENTER - DEADBAND) = 0
        let range = (CENTER - DEADBAND) - MIN_INPUT;
        let offset = value - MIN_INPUT;
        let speed = (offset as i32 * MAX_MOTOR_SPEED as i32) / range as i32 - MAX_MOTOR_SPEED as i32;
        speed.clamp(-MAX_MOTOR_SPEED as i32, 0) as i16
    } else {
        // Forward: (CENTER + DEADBAND) = 0, 1800 = +1000
        let range = MAX_INPUT - (CENTER + DEADBAND);
        let offset = value - (CENTER + DEADBAND);
        let speed = (offset as i32 * MAX_MOTOR_SPEED as i32) / range as i32;
        speed.clamp(0, MAX_MOTOR_SPEED as i32) as i16
    }
}

/// Check if switch is enabled (>1400)
fn switch_enabled(value: u16) -> bool {
    value > 1400
}

/// Send speed command to motor controller
async fn send_speed<TX: Write>(uart_tx: &mut TX, slave_id: u8, speed: i16, state: u8) {
    let msg = protocol::build_speed_msg(slave_id, speed, state);
    let _ = uart_tx.write_all(&msg).await;
}

/// Try to receive motor response with timeout
async fn receive_response<RX: Read>(uart_rx: &mut RX) -> Option<Response> {
    let mut buf = [0u8; protocol::RESPONSE_SIZE];

    let result = with_timeout(Duration::from_millis(RESPONSE_TIMEOUT_MS), async {
        uart_rx.read_exact(&mut buf).await
    })
    .await;

    match result {
        Ok(Ok(())) => protocol::parse_response(&buf),
        _ => None,
    }
}

/// Process S.BUS packet and return desired motor speed
fn process_sbus(packet: &SbusPacket) -> i16 {
    // ch7 (index 6) = steering enable
    let steering_enabled = switch_enabled(packet.channels[6]);

    if steering_enabled && !packet.flags.failsafe {
        // ch2 (index 1) = right track
        channel_to_motor_speed(packet.channels[1])
    } else {
        0 // Safety: no movement if steering disabled or failsafe
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    rtt_init_defmt!(rtt_target::ChannelMode::BlockIfFull, 65536);

    info!("=== Skid Steer Right Track Controller ===");
    info!("S.BUS on GP9 | Motor on GP4/GP5 | Slaves {},{}", SLAVE_ID_1, SLAVE_ID_2);

    let p = embassy_rp::init(Default::default());

    // ========== S.BUS Input (UART1 on GP9) ==========
    let mut sbus_config = UartConfig::default();
    sbus_config.baudrate = 100_000;
    sbus_config.data_bits = DataBits::DataBits8;
    sbus_config.parity = Parity::ParityEven;
    sbus_config.stop_bits = StopBits::STOP2;

    let mut sbus_uart = Uart::new(
        p.UART1,
        p.PIN_8, // TX (not used)
        p.PIN_9, // RX (S.BUS input)
        UartIrqs,
        p.DMA_CH0,
        p.DMA_CH1,
        sbus_config,
    );

    // Invert GP9 for S.BUS (inverted UART)
    let io_bank0 = embassy_rp::pac::IO_BANK0;
    io_bank0.gpio(9).ctrl().modify(|w| {
        w.set_inover(embassy_rp::pac::io::vals::Inover::INVERT);
    });

    // ========== Motor Output (PIO UART on GP4/GP5) ==========
    let pio::Pio {
        mut common,
        sm0,
        sm1,
        ..
    } = pio::Pio::new(p.PIO0, PioIrqs);

    let tx_program = PioUartTxProgram::new(&mut common);
    let rx_program = PioUartRxProgram::new(&mut common);

    let mut motor_tx = PioUartTx::new(MOTOR_BAUD, &mut common, sm0, p.PIN_4, &tx_program);
    let mut motor_rx = PioUartRx::new(MOTOR_BAUD, &mut common, sm1, p.PIN_5, &rx_program);

    // LED for status
    let mut led = Output::new(p.PIN_25, Level::Low);

    info!("Initialized, waiting for S.BUS...");

    let mut sbus_parser = sbus_rs::StreamingParser::new();
    let mut sbus_buf = [0u8; 1];
    let mut frame_count: u32 = 0;
    let mut last_speed: i16 = 0;

    loop {
        // Read S.BUS byte
        if let Ok(()) = sbus_uart.read(&mut sbus_buf).await {
            // Feed to parser
            if let Ok(Some(packet)) = sbus_parser.push_byte(sbus_buf[0]) {
                frame_count += 1;
                led.set_high();

                // Get desired speed from S.BUS
                let speed = process_sbus(&packet);

                // Send motor command to both slaves
                send_speed(&mut motor_tx, SLAVE_ID_1, speed, 0).await;
                let resp1 = receive_response(&mut motor_rx).await;

                send_speed(&mut motor_tx, SLAVE_ID_2, speed, 0).await;
                let resp2 = receive_response(&mut motor_rx).await;

                // Log periodically or on speed change
                if frame_count % 50 == 0 || speed != last_speed {
                    let resp1_spd = resp1.map(|r| r.speed).unwrap_or(-9999);
                    let resp2_spd = resp2.map(|r| r.speed).unwrap_or(-9999);
                    info!(
                        "#{} ch2={} -> speed={} resp1={} resp2={}",
                        frame_count, packet.channels[1], speed, resp1_spd, resp2_spd
                    );
                    last_speed = speed;
                }

                led.set_low();
            }
        }
    }
}
