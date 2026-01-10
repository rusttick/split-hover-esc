//! WiFi LED Blink Test
//!
//! Blinks the onboard LED on Pico W, which is connected to the CYW43 WiFi chip
//! and not directly accessible via GPIO.
//!
//! The onboard LED is controlled via the CYW43 chip's GPIO 0.

#![no_std]
#![no_main]

use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};
use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_time::{Duration, Timer};
use panic_probe as _;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("=== WiFi LED Blink Test ===");

    let p = embassy_rp::init(Default::default());

    // CYW43 firmware
    let fw = include_bytes!("../../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../cyw43-firmware/43439A0_clm.bin");

    // CYW43 uses PIO0 for SPI communication
    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio0 = Pio::new(p.PIO0, Irqs);

    let spi = PioSpi::new(
        &mut pio0.common,
        pio0.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio0.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    // Initialize CYW43
    static CYW43_STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = CYW43_STATE.init(cyw43::State::new());

    let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;

    spawner.spawn(cyw43_task(runner)).unwrap();

    control.init(clm).await;

    info!("CYW43 initialized, starting LED blink loop");

    // Blink the onboard LED (CYW43 GPIO 0)
    loop {
        info!("LED ON");
        control.gpio_set(0, true).await;
        Timer::after(Duration::from_millis(500)).await;

        info!("LED OFF");
        control.gpio_set(0, false).await;
        Timer::after(Duration::from_millis(500)).await;
    }
}
