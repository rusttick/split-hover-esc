//! Remote UART Bus Speed Test (Dual Slave) using PIO UART
//!
//! Same as speed1.rs but uses PIO-based UART instead of hardware UART.
//! Tests the hoverboard motor controller using the Remote UART Bus protocol.
//! Sends the same speed commands to both slave 1 and slave 2.
//! Ramps speed from 0 to max forward, then cycles between max forward and
//! max backward continuously.
//!
//! Hardware:
//! - PIO UART TX: GP4 (connect to MAX485 DI)
//! - PIO UART RX: GP5 (connect to MAX485 RO)
//! - Baud: 19200 8N1

#![no_std]
#![no_main]

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{self, InterruptHandler};
use embassy_rp::pio_programs::uart::{PioUartRx, PioUartRxProgram, PioUartTx, PioUartTxProgram};
use embassy_time::{Duration, Timer, with_timeout};
use embedded_io_async::{Read, Write};
use panic_probe as _;
use rtt_target::rtt_init_defmt;

use split_hover_esc::protocol::{self, Response};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

/// UART baud rate (must match hoverboard firmware)
const BAUD_RATE: u32 = 19200;

/// Maximum speed value (±500 = ±50%)
const MAX_SPEED: i16 = 500;

/// Speed increment per step (5% = 50 units)
const SPEED_STEP: i16 = 50;

/// Time between speed changes (400ms, matching motor_control.py)
const STEP_DELAY_MS: u64 = 400;

/// Timeout for waiting for response
const RESPONSE_TIMEOUT_MS: u64 = 100;

/// Send a speed command and print what we're sending
async fn send_speed_command<TX: Write>(uart_tx: &mut TX, slave_id: u8, speed: i16, state: u8) {
    let msg = protocol::build_speed_msg(slave_id, speed, state);

    info!(
        "TX: slave={} speed={} state=0x{:02X}",
        slave_id, speed, state
    );
    info!(
        "    raw: {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}",
        msg[0], msg[1], msg[2], msg[3], msg[4], msg[5], msg[6], msg[7]
    );

    if let Err(_) = uart_tx.write_all(&msg).await {
        error!("PIO UART write error");
    }
}

/// Try to receive and parse a response, with timeout
async fn receive_response<RX: Read>(uart_rx: &mut RX) -> Option<Response> {
    let mut buf = [0u8; protocol::RESPONSE_SIZE];

    // Read with timeout
    let result = with_timeout(Duration::from_millis(RESPONSE_TIMEOUT_MS), async {
        uart_rx.read_exact(&mut buf).await
    })
    .await;

    match result {
        Ok(Ok(())) => {
            info!(
                "RX raw: {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}",
                buf[0],
                buf[1],
                buf[2],
                buf[3],
                buf[4],
                buf[5],
                buf[6],
                buf[7],
                buf[8],
                buf[9],
                buf[10],
                buf[11],
                buf[12],
                buf[13],
                buf[14]
            );

            match protocol::parse_response(&buf) {
                Some(resp) => {
                    info!(
                        "RX: slave={} speed={} voltage={}mV current={}cA odom={}",
                        resp.slave_id, resp.speed, resp.voltage_mv, resp.current_ca, resp.odometer
                    );
                    Some(resp)
                }
                None => {
                    warn!("RX: invalid response (bad start frame or CRC)");
                    None
                }
            }
        }
        Ok(Err(_)) => {
            warn!("PIO UART read error");
            None
        }
        Err(_) => {
            warn!("RX: timeout (no response within {}ms)", RESPONSE_TIMEOUT_MS);
            None
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize RTT with BlockIfFull mode and 64KB buffer
    rtt_init_defmt!(rtt_target::ChannelMode::BlockIfFull, 65536);

    info!("=== Remote UART Bus Speed Test (PIO UART, Dual Slave) ===");
    info!(
        "Baud: {} | Slaves: 1, 2 | Max speed: +/-{}",
        BAUD_RATE, MAX_SPEED
    );

    // Initialize peripherals
    let p = embassy_rp::init(Default::default());

    // Initialize PIO0
    let pio::Pio {
        mut common,
        sm0,
        sm1,
        ..
    } = pio::Pio::new(p.PIO0, Irqs);

    // Load the UART TX and RX programs into PIO instruction memory
    let tx_program = PioUartTxProgram::new(&mut common);
    let rx_program = PioUartRxProgram::new(&mut common);

    // Create PIO UART TX on GP4 (state machine 0)
    let mut uart_tx = PioUartTx::new(BAUD_RATE, &mut common, sm0, p.PIN_4, &tx_program);

    // Create PIO UART RX on GP5 (state machine 1)
    let mut uart_rx = PioUartRx::new(BAUD_RATE, &mut common, sm1, p.PIN_5, &rx_program);

    info!("PIO UART initialized: TX=GP4 (SM0), RX=GP5 (SM1)");

    // Optional: LED on GP25 for visual feedback (Pico onboard LED)
    let mut led = Output::new(p.PIN_25, Level::Low);

    info!("Starting speed ramp test...");
    info!("Phase 1: Ramp from 0 to {}% forward", MAX_SPEED / 10);

    // State flags (none set for basic operation)
    let state: u8 = 0;

    // Phase 1: Ramp from 0 to max forward
    let mut speed: i16 = 0;
    while speed <= MAX_SPEED {
        led.set_high();

        // Send to slave 1
        send_speed_command(&mut uart_tx, 1, speed, state).await;
        let _resp1 = receive_response(&mut uart_rx).await;

        // Send to slave 2
        send_speed_command(&mut uart_tx, 2, speed, state).await;
        let _resp2 = receive_response(&mut uart_rx).await;

        led.set_low();

        Timer::after(Duration::from_millis(STEP_DELAY_MS)).await;
        speed += SPEED_STEP;
    }

    info!("Phase 2: Continuous cycle {} <-> -{}", MAX_SPEED, MAX_SPEED);

    // Phase 2: Continuous cycling between +max and -max
    let mut direction: i16 = -1; // Start going backward
    speed = MAX_SPEED;

    loop {
        // Move speed in current direction
        speed += direction * SPEED_STEP;

        // Reverse direction at limits
        if speed <= -MAX_SPEED {
            speed = -MAX_SPEED;
            direction = 1;
            info!("--- Reversing: now going FORWARD ---");
        } else if speed >= MAX_SPEED {
            speed = MAX_SPEED;
            direction = -1;
            info!("--- Reversing: now going BACKWARD ---");
        }

        led.set_high();

        // Send to slave 1
        send_speed_command(&mut uart_tx, 1, speed, state).await;
        let _resp1 = receive_response(&mut uart_rx).await;

        // Send to slave 2
        send_speed_command(&mut uart_tx, 2, speed, state).await;
        let _resp2 = receive_response(&mut uart_rx).await;

        led.set_low();

        Timer::after(Duration::from_millis(STEP_DELAY_MS)).await;
    }
}
