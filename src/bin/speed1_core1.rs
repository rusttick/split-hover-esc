//! Remote UART Bus Speed Test (Dual Slave, Core 1)
//!
//! Tests the hoverboard motor controller using the Remote UART Bus protocol.
//! Sends the same speed commands to both slave 1 and slave 2.
//! Ramps speed from 0 to max forward, then cycles between max forward and
//! max backward continuously.
//!
//! Multicore architecture:
//! - Core 0: Idle (available for future use)
//! - Core 1: UART speed control loop
//!
//! Hardware:
//! - UART1 TX: GP4 (connect to MAX485 DI)
//! - UART1 RX: GP5 (connect to MAX485 RO)
//! - Baud: 19200 8N1

#![no_std]
#![no_main]

use core::ptr::addr_of_mut;

use defmt::{error, info, warn};
use embassy_executor::Executor;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, PIN_25, PIN_4, PIN_5, UART1, WATCHDOG};
use embassy_rp::uart::{Config as UartConfig, Uart};
use embassy_rp::watchdog::Watchdog;
use embassy_rp::Peri;
use embassy_time::{Duration, Timer, with_timeout};
use panic_probe as _;
use rtt_target::rtt_init_defmt;
use static_cell::StaticCell;

use split_hover_esc::Irqs;
use split_hover_esc::protocol::{self, Response};

/// UART baud rate (must match hoverboard firmware)
const BAUD_RATE: u32 = 19200;

/// Maximum speed value (±800 = ±80%)
const MAX_SPEED: i16 = 500;

/// Speed increment per step (5% = 50 units)
const SPEED_STEP: i16 = 50;

/// Time between speed changes (400ms, matching motor_control.py)
const STEP_DELAY_MS: u64 = 400;

/// Timeout for waiting for response
const RESPONSE_TIMEOUT_MS: u64 = 100;

/// Watchdog timeout (must be longer than one loop iteration)
const WATCHDOG_TIMEOUT_MS: u64 = 2000;

// Multicore statics
static mut CORE1_STACK: Stack<65536> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

/// Send a speed command and print what we're sending
async fn send_speed_command(
    uart: &mut Uart<'_, embassy_rp::uart::Async>,
    slave_id: u8,
    speed: i16,
    state: u8,
) {
    let msg = protocol::build_speed_msg(slave_id, speed, state);

    info!(
        "TX: slave={} speed={} state=0x{:02X}",
        slave_id, speed, state
    );
    info!(
        "    raw: {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}",
        msg[0], msg[1], msg[2], msg[3], msg[4], msg[5], msg[6], msg[7]
    );

    if let Err(e) = uart.write(&msg).await {
        error!("UART write error: {:?}", e);
    }
}

/// Try to receive and parse a response, with timeout
async fn receive_response(uart: &mut Uart<'_, embassy_rp::uart::Async>) -> Option<Response> {
    let mut buf = [0u8; protocol::RESPONSE_SIZE];

    // Read with timeout
    let result = with_timeout(
        Duration::from_millis(RESPONSE_TIMEOUT_MS),
        uart.read(&mut buf),
    )
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
        Ok(Err(e)) => {
            warn!("UART read error: {:?}", e);
            None
        }
        Err(_) => {
            warn!("RX: timeout (no response within {}ms)", RESPONSE_TIMEOUT_MS);
            None
        }
    }
}

/// Speed control task - runs on Core 1
#[embassy_executor::task]
async fn speed_task(
    uart1: Peri<'static, UART1>,
    pin_4: Peri<'static, PIN_4>,
    pin_5: Peri<'static, PIN_5>,
    dma_ch0: Peri<'static, DMA_CH0>,
    dma_ch1: Peri<'static, DMA_CH1>,
    pin_25: Peri<'static, PIN_25>,
    watchdog: Peri<'static, WATCHDOG>,
) -> ! {
    info!("Core 1: speed_task starting");

    // Initialize and start watchdog timer
    let mut watchdog = Watchdog::new(watchdog);
    watchdog.start(Duration::from_millis(WATCHDOG_TIMEOUT_MS));
    info!("Watchdog started with {}ms timeout", WATCHDOG_TIMEOUT_MS);

    // Configure UART1 on GP4 (TX) and GP5 (RX) for MAX485 RS-485 transceiver
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = BAUD_RATE;

    let mut uart = Uart::new(
        uart1,
        pin_4,  // TX -> MAX485 DI
        pin_5,  // RX <- MAX485 RO
        Irqs,
        dma_ch0,
        dma_ch1,
        uart_config,
    );

    // LED on GP25 for visual feedback (Pico onboard LED)
    let mut led = Output::new(pin_25, Level::Low);

    info!("Starting speed ramp test...");
    info!("Phase 1: Ramp from 0 to {}% forward", MAX_SPEED / 10);

    // State flags (none set for basic operation)
    let state: u8 = 0;

    // Phase 1: Ramp from 0 to max forward
    let mut speed: i16 = 0;
    while speed <= MAX_SPEED {
        watchdog.feed();
        led.set_high();

        // Send to slave 1
        send_speed_command(&mut uart, 1, speed, state).await;
        let _resp1 = receive_response(&mut uart).await;

        // Send to slave 2
        send_speed_command(&mut uart, 2, speed, state).await;
        let _resp2 = receive_response(&mut uart).await;

        led.set_low();

        Timer::after(Duration::from_millis(STEP_DELAY_MS)).await;
        speed += SPEED_STEP;
    }

    info!("Phase 2: Continuous cycle {} <-> -{}", MAX_SPEED, MAX_SPEED);

    // Phase 2: Continuous cycling between +max and -max
    let mut direction: i16 = -1; // Start going backward
    speed = MAX_SPEED;

    loop {
        watchdog.feed();

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
        send_speed_command(&mut uart, 1, speed, state).await;
        let _resp1 = receive_response(&mut uart).await;

        // Send to slave 2
        send_speed_command(&mut uart, 2, speed, state).await;
        let _resp2 = receive_response(&mut uart).await;

        led.set_low();

        Timer::after(Duration::from_millis(STEP_DELAY_MS)).await;
    }
}

/// Core 0 idle task
#[embassy_executor::task]
async fn core0_idle_task() -> ! {
    info!("Core 0: idle task running");
    loop {
        Timer::after_secs(60).await;
    }
}

/// Entry point - sets up multicore execution
#[cortex_m_rt::entry]
fn main() -> ! {
    // Initialize RTT with BlockIfFull mode and 64KB buffer
    rtt_init_defmt!(rtt_target::ChannelMode::BlockIfFull, 65536);

    info!("=== Remote UART Bus Speed Test (Dual Slave, Core 1) ===");
    info!(
        "Baud: {} | Slaves: 1, 2 | Max speed: +/-{}",
        BAUD_RATE, MAX_SPEED
    );

    // Initialize peripherals
    let p = embassy_rp::init(Default::default());

    // ========== Core 1 peripherals (speed control) ==========
    let uart1 = p.UART1;
    let pin_4 = p.PIN_4;
    let pin_5 = p.PIN_5;
    let dma_ch0 = p.DMA_CH0;
    let dma_ch1 = p.DMA_CH1;
    let pin_25 = p.PIN_25;
    let watchdog = p.WATCHDOG;

    // ========== Spawn Core 1 (speed task) ==========
    info!("spawning Core 1 for speed task");
    spawn_core1(
        p.CORE1,
        unsafe { &mut *addr_of_mut!(CORE1_STACK) },
        move || {
            info!("Core 1: executor starting");
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| {
                spawner
                    .spawn(speed_task(uart1, pin_4, pin_5, dma_ch0, dma_ch1, pin_25, watchdog))
                    .unwrap();
            });
        },
    );

    // ========== Run Core 0 executor ==========
    info!("Core 0: executor starting");
    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.spawn(core0_idle_task()).unwrap();
    });
}
