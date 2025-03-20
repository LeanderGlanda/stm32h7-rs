#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use stm32h7xx_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    
    // RCC and clocks
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.freeze(vos, &dp.SYSCFG);

    // Get reset controller for GPIOB
    let gpio_b = dp.GPIOB.split(ccdr.peripheral.GPIOB);

    // Configure PB0 as output
    let mut led = gpio_b.pb0.into_push_pull_output();

    loop {
        led.set_high();
        cortex_m::asm::delay(8_000_000);
        led.set_low();
        cortex_m::asm::delay(8_000_000);
    }
}
