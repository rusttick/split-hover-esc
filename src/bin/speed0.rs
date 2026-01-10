//! Dual Slave Zero Speed Test (Pico W version)
//!
//! Sends speed=0 commands alternately to slave ID 1 and slave ID 2,
//! logging responses from each. Useful for verifying communication
//! with both motor controller boards.
//!
//! Hardware:
//! - UART1 TX: GP4 (connect to MAX485 DI)
//! - UART1 RX: GP5 (connect to MAX485 RO)
//! - Baud: 19200 8N1
//!
//! Note: Pico W onboard LED requires WiFi subsystem, so no LED feedback.

#![no_std]
#![no_main]

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_rp::uart::{Config as UartConfig, Uart};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use panic_probe as _;
use rtt_target::rtt_init_defmt;
use split_hover_esc::Irqs;
use split_hover_esc::protocol::{self, Response};

/// UART baud rate (must match hoverboard firmware)
const BAUD_RATE: u32 = 19200;

/// Timeout for waiting for response
const RESPONSE_TIMEOUT_MS: u64 = 50;

/// Target cycle rate (cycles per second)
const TARGET_CPS: u64 = 30;

/// Target cycle duration in microseconds (1_000_000 / 30 = 33333us)
const CYCLE_DURATION_US: u64 = 1_000_000 / TARGET_CPS;

/// Send a speed command
async fn send_speed_command(
    uart: &mut Uart<'_, embassy_rp::uart::Async>,
    slave_id: u8,
    speed: i16,
    state: u8,
) {
    let msg = protocol::build_speed_msg(slave_id, speed, state);

    if uart.write(&msg).await.is_err() {
        // if let Err(_) = uart.write(&msg).await {
        error!("UART write error");
    }
}

/// Try to receive and parse a response, with timeout
async fn receive_response(
    uart: &mut Uart<'_, embassy_rp::uart::Async>,
    slave_id: u8,
) -> Option<Response> {
    let mut buf = [0u8; protocol::RESPONSE_SIZE];

    // Read with timeout
    let result = with_timeout(
        Duration::from_millis(RESPONSE_TIMEOUT_MS),
        uart.read(&mut buf),
    )
    .await;

    match result {
        Ok(Ok(())) => match protocol::parse_response(&buf) {
            Some(resp) => Some(resp),
            None => {
                warn!("s{}: bad CRC", slave_id);
                None
            }
        },
        Ok(Err(_)) => {
            warn!("s{}: err", slave_id);
            None
        }
        Err(_) => {
            warn!("s{}: timeout", slave_id);
            None
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize RTT with BlockIfFull mode and 64KB buffer
    // With minimal logging (~1 msg/sec), buffer won't fill - blocking mode is more stable
    rtt_init_defmt!(rtt_target::ChannelMode::BlockIfFull, 65536);

    info!("=== Dual Slave Zero Speed Test ===");
    info!("Baud: {} | Slaves: 1, 2 | Speed: 0", BAUD_RATE);

    // Initialize peripherals
    let p = embassy_rp::init(Default::default());

    // Configure UART1 on GP4 (TX) and GP5 (RX) for MAX485 RS-485 transceiver
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = BAUD_RATE;

    let mut uart = Uart::new(
        p.UART1,
        p.PIN_4, // TX -> MAX485 DI
        p.PIN_5, // RX <- MAX485 RO
        Irqs,
        p.DMA_CH0,
        p.DMA_CH1,
        uart_config,
    );

    info!("Starting dual slave test loop...");

    // State flags (none set for basic operation)
    let state: u8 = 0;
    let speed: i16 = 0;

    let mut cycle: u32 = 0;
    let mut last_report = Instant::now();
    let mut cycles_since_report: u32 = 0;
    let mut total_s1_us: u64 = 0;
    let mut total_s2_us: u64 = 0;

    loop {
        let cycle_start = Instant::now();
        cycle += 1;
        cycles_since_report += 1;

        // Send to slave 1 and measure response time
        let s1_start = Instant::now();
        send_speed_command(&mut uart, 1, speed, state).await;
        let _resp1 = receive_response(&mut uart, 1).await;
        total_s1_us += s1_start.elapsed().as_micros();

        // Send to slave 2 and measure response time
        let s2_start = Instant::now();
        send_speed_command(&mut uart, 2, speed, state).await;
        let _resp2 = receive_response(&mut uart, 2).await;
        total_s2_us += s2_start.elapsed().as_micros();

        // Report cycles per second every ~1 second
        let elapsed = last_report.elapsed();
        if elapsed.as_millis() >= 1000 {
            let cps = (cycles_since_report as u64 * 1000) / elapsed.as_millis();
            let avg_s1_ms = total_s1_us / cycles_since_report as u64 / 1000;
            let avg_s2_ms = total_s2_us / cycles_since_report as u64 / 1000;
            info!(
                "Cycle {}: {} cps, s1={}ms s2={}ms",
                cycle, cps, avg_s1_ms, avg_s2_ms
            );
            last_report = Instant::now();
            cycles_since_report = 0;
            total_s1_us = 0;
            total_s2_us = 0;
        }

        // Fixed timing: sleep for remaining time to hit target cycle rate
        let cycle_elapsed_us = cycle_start.elapsed().as_micros();
        if cycle_elapsed_us < CYCLE_DURATION_US {
            Timer::after(Duration::from_micros(CYCLE_DURATION_US - cycle_elapsed_us)).await;
        }
    }
}
