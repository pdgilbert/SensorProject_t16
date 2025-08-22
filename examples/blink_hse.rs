//! Blink  onboard LED on PC13.  Using high speed external (hse) oscillator.   TIMING TOO QUICK
//! Compare crate rust-integration-testing  examples/misc/blink_impl.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

pub trait LED: OutputPin {
    // default methods
    fn on(&mut self) -> () {
        self.set_low().unwrap()
    }

    fn off(&mut self) -> () {
        self.set_high().unwrap()
    }

    fn blink(&mut self, time: u32, delay: &mut impl DelayNs) -> () {
        self.on();
        delay.delay_ms(time);
        self.off()
    }

}



use stm32f4xx_hal::{
    gpio::{gpioc::PC13, Output, PushPull},
    pac::{CorePeripherals, Peripherals},
    rcc::Config,
    prelude::*,
};

impl LED for PC13<Output<PushPull>>{}

#[cfg(feature = "v020_2025-05")]
fn setup() -> (impl LED, impl DelayNs) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();

    let gpioc = dp.GPIOC.split(&mut rcc);
    let delay = cp.SYST.delay(&mut rcc.clocks);

    //rcc.cfgr.use_hse(25.MHz()).freeze();
    rcc.freeze(Config::hse(25.MHz()));

   // rcc.freeze(
   //     Config::hsi()
   //        .hclk(48.MHz())
   //         .sysclk(48.MHz())
   //         .pclk1(24.MHz())
   //         .pclk2(24.MHz()),
   // );

    // return tuple  (led, delay)
    (
        gpioc.pc13.into_push_pull_output(), // led on pc13 with on/off
        delay,
    )
}



#[entry]
fn main() -> ! {
    let (mut led, mut delay) = setup();

    led.blink(1000, &mut delay); // blink  to indicate setup complete.

    led.off();    
    delay.delay_ms(5000);  //off for  5 s

    led.on();     
    delay.delay_ms(20000);   // on for 20s to check delay timer

    led.off();    
    delay.delay_ms(5000);  //off for  5 s

    let on: u32  = 10; // on for 10 ms
    let off: u32 = 990; //off for remainder of second

    // blink LED and sleep
    loop {
        led.blink(on, &mut delay);
        delay.delay_ms(off);
    }
}
