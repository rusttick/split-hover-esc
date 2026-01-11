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

use defmt::{info, warn};
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
const MAX_SPEED: i16 = 600;

/// Speed increment per step (5% = 50 units)
const SPEED_STEP: i16 = 50;

/// Time between speed changes (400ms, matching motor_control.py)
const STEP_DELAY_MS: u64 = 500;

/// Timeout for waiting for response
const RESPONSE_TIMEOUT_MS: u64 = 100;

/// Send a speed command (silent)
async fn send_speed_command<TX: Write>(uart_tx: &mut TX, slave_id: u8, speed: i16, state: u8) {
    let msg = protocol::build_speed_msg(slave_id, speed, state);
    let _ = uart_tx.write_all(&msg).await;
}

/// Try to receive and parse a response, with timeout (silent)
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

/// Send to both slaves and print one summary line
async fn send_to_both_and_report<TX: Write, RX: Read>(
    uart_tx: &mut TX,
    uart_rx: &mut RX,
    speed: i16,
    state: u8,
) {
    // Send to slave 1
    send_speed_command(uart_tx, 1, speed, state).await;
    let resp1 = receive_response(uart_rx).await;

    // Send to slave 2
    send_speed_command(uart_tx, 2, speed, state).await;
    let resp2 = receive_response(uart_rx).await;

    // Convert response speed to RPM (response_speed / 10 = RPM for 30P motor)
    let rpm1: i16;
    let rpm2: i16;
    let mut timeout_warning = false;

    match resp1 {
        Some(r) => rpm1 = r.speed / 10,
        None => {
            rpm1 = -999;
            timeout_warning = true;
        }
    }

    match resp2 {
        Some(r) => rpm2 = r.speed / 10,
        None => {
            rpm2 = -999;
            timeout_warning = true;
        }
    }

    // Get voltage/current from slave 1
    let (voltage, current) = match resp1 {
        Some(r) => (r.voltage_mv, r.current_ca),
        None => (0, 0),
    };

    if timeout_warning {
        warn!(
            "target={} rpm1={} rpm2={} V={}mV I={}cA (TIMEOUT)",
            speed, rpm1, rpm2, voltage, current
        );
    } else {
        info!(
            "target={} rpm1={} rpm2={} V={}mV I={}cA",
            speed, rpm1, rpm2, voltage, current
        );
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
        send_to_both_and_report(&mut uart_tx, &mut uart_rx, speed, state).await;
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
        send_to_both_and_report(&mut uart_tx, &mut uart_rx, speed, state).await;
        led.set_low();

        Timer::after(Duration::from_millis(STEP_DELAY_MS)).await;
    }
}
