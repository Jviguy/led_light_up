//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
use defmt::*;
use defmt_rtt as _;

use core::convert::Infallible;
use core::sync::atomic::{AtomicU32, Ordering};

use embedded_hal::digital::{InputPin, OutputPin};
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
// TODO replace with this stuff once its back:
/*
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    multicore::{Multicore, Stack},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
*/

use rp2040_hal::{
    clocks::{init_clocks_and_plls, Clock},
    multicore::{Multicore, Stack},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

// TODO: once rp-pico updates remember to get rid of this as well.

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

#[rp2040_hal::entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = rp2040_hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    //
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead.
    // One way to do that is by using [embassy](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/wifi_blinky.rs)
    //
    // If you have a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here. Don't forget adding an appropriate resistor
    // in series with the LED.
    // we are on a pico W so we have an external small blue led connected to gpio15.
    let mut led_pin = pins.gpio15.into_push_pull_output();

    let mut click_pin = pins.gpio18.into_pull_up_input();

    let mut led_enabled = false;
    // start with it off.
    led_pin.set_low().unwrap();

    let clk_pin = pins.gpio16.into_pull_up_input();
    let dt_pin = pins.gpio17.into_pull_up_input();

    let mut psm = pac.PSM;
    let mut ppb = pac.PPB;
    let mut sio_fifo = sio.fifo;

    let mut mc = Multicore::new(&mut psm, &mut ppb, &mut sio_fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

    let _ = core1.spawn(CORE1_STACK.take().unwrap(), move || {
        core1_task(clk_pin, dt_pin) // Pass the pins to the task
    });

    loop {
        if click_pin.is_low().unwrap() {
            led_enabled = !led_enabled;

            if led_enabled {
                led_pin.set_high().unwrap();
                info!("Click! LED ON");
            } else {
                led_pin.set_low().unwrap();
                info!("Click! LED OFF");
            }

            while click_pin.is_low().unwrap() {}
            delay.delay_ms(50); // Wait 50ms for the click to settle
        }

        if led_enabled {
            // Read the delay value that Core 1 is setting
            let delay_val = SHARED_DELAY.load(Ordering::Relaxed);

            info!("Blink! Delay: {}ms", delay_val);
            led_pin.set_high().unwrap();
            delay.delay_ms(delay_val);
            led_pin.set_low().unwrap();
            delay.delay_ms(delay_val);
        }
    }
}

// CORE 1 STUFF:
static SHARED_DELAY: AtomicU32 = AtomicU32::new(500);

static CORE1_STACK: Stack<4096> = Stack::new();

fn core1_task<C, D>(mut clk_pin: C, mut dt_pin: D) -> !
where
    C: InputPin<Error = Infallible>,
    D: InputPin<Error = Infallible>,
{
    let core = unsafe { pac::CorePeripherals::steal() };
    let sys_freq = 125_000_000; // Default system clock freq
    let mut delay = cortex_m::delay::Delay::new(core.SYST, sys_freq);

    let mut last_clk_state = clk_pin.is_high().unwrap();

    loop {
        let clk_state = clk_pin.is_high().unwrap();

        if clk_state != last_clk_state && clk_pin.is_low().unwrap() {
            // Read the delay from shared memory
            let mut current_delay = SHARED_DELAY.load(Ordering::Relaxed);

            if dt_pin.is_high().unwrap() != clk_state {
                // Add 50ms, but don't go over 2000
                current_delay = (current_delay + 50).min(2000);
                info!("Core 1: Clockwise! Delay: {}", current_delay);
            } else {
                // Subtract 50ms, but don't go under 50
                current_delay = (current_delay - 50).max(50);
                info!("Core 1: Counter-Clockwise! Delay: {}", current_delay);
            }

            // Write the new delay back to shared memory
            SHARED_DELAY.store(current_delay, Ordering::Relaxed);
        }

        last_clk_state = clk_state;

        // A tiny delay to "debounce" the encoder and save power
        delay.delay_ms(1);
    }
}
