//! Dual Slave Zero Speed Test using PIO UART
//!
//! Same as speed0.rs but uses PIO-based UART instead of hardware UART.
//! This frees up the hardware UARTs for other purposes (e.g., SBUS).
//!
//! Hardware:
//! - PIO UART TX: GP4 (connect to MAX485 DI)
//! - PIO UART RX: GP5 (connect to MAX485 RO)
//! - Baud: 19200 8N1
//!
//! Uses embassy-rp's built-in PIO UART programs (proven/tested).
//! Each simplex UART uses 1 PIO state machine.
//! RP2040 has 2 PIO blocks × 4 SMs = 8 simplex UARTs possible.

#![no_std]
#![no_main]

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{self, InterruptHandler};
use embassy_rp::pio_programs::uart::{PioUartRx, PioUartRxProgram, PioUartTx, PioUartTxProgram};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_io_async::{Read, Write};
use panic_probe as _;
use rtt_target::rtt_init_defmt;
use split_hover_esc::protocol::{self, Response};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

/// UART baud rate (must match hoverboard firmware)
const BAUD_RATE: u32 = 19200;

/// Timeout for waiting for response
const RESPONSE_TIMEOUT_MS: u64 = 50;

/// Target cycle rate (cycles per second)
const TARGET_CPS: u64 = 30;

/// Target cycle duration in microseconds (1_000_000 / 30 = 33333us)
const CYCLE_DURATION_US: u64 = 1_000_000 / TARGET_CPS;

/// Send a speed command via PIO UART
async fn send_speed_command<TX: Write>(uart_tx: &mut TX, slave_id: u8, speed: i16, state: u8) {
    let msg = protocol::build_speed_msg(slave_id, speed, state);

    if uart_tx.write_all(&msg).await.is_err() {
        error!("PIO UART write error");
    }
}

/// Try to receive and parse a response, with timeout
async fn receive_response<RX: Read>(uart_rx: &mut RX, slave_id: u8) -> Option<Response> {
    let mut buf = [0u8; protocol::RESPONSE_SIZE];

    // Read with timeout
    let result = with_timeout(Duration::from_millis(RESPONSE_TIMEOUT_MS), async {
        uart_rx.read_exact(&mut buf).await
    })
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
            warn!("s{}: read err", slave_id);
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
    rtt_init_defmt!(rtt_target::ChannelMode::BlockIfFull, 65536);

    info!("=== Dual Slave Zero Speed Test (PIO UART) ===");
    info!("Baud: {} | Slaves: 1, 2 | Speed: 0", BAUD_RATE);

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
        send_speed_command(&mut uart_tx, 1, speed, state).await;
        let _resp1 = receive_response(&mut uart_rx, 1).await;
        total_s1_us += s1_start.elapsed().as_micros();

        // Send to slave 2 and measure response time
        let s2_start = Instant::now();
        send_speed_command(&mut uart_tx, 2, speed, state).await;
        let _resp2 = receive_response(&mut uart_rx, 2).await;
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
