//! Blink  onboard LED on PC13.  Using high speed external (hse) oscillator.
//! Compare crate rust-integration-testing  examples/misc/blink_impl.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;
//use cortex_m_semihosting::hprintln;

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
    pac::{Peripherals},
    rcc::Config,
    prelude::*,
    time::Hertz,
};

impl LED for PC13<Output<PushPull>>{}

#[cfg(feature = "stm32f411")]
fn setup() -> (impl LED, impl DelayNs) {
    let dp = Peripherals::take().unwrap();

    //let mut rcc = dp.RCC.constrain();  constrain` is just a `freeze` with default config. (since changes circa Oct 2025) 

    // Config creates a structure for passing to freeze, but does not itself set the clocks. 
    //let config = Config::hse(25.MHz());
    // or
    let config = Config::hse(25.MHz()).hclk(100.MHz()).sysclk(100.MHz()).pclk1(50.MHz()).pclk2(50.MHz());
    // `pclk1` can't be bigger than `sysclk`

    // RCC configuration is done here. Freeze before using `rcc`. 
    let mut rcc = dp.RCC.freeze(config);

    //hprintln!("clocks() {:?}", rcc.clocks);

    // `Clocks` structure lives inside `Rcc`. Clone is possible: `rcc.clocks.clone()`
    //assert_eq!(rcc.clocks.hse(),     Hertz::MHz(25));
    assert_eq!(rcc.clocks.sysclk(),  Hertz::MHz(100)); 
    assert_eq!(rcc.clocks.hclk(),    Hertz::MHz(100));
    assert_eq!(rcc.clocks.pclk1(),   Hertz::MHz(50));
    assert_eq!(rcc.clocks.pclk2(),   Hertz::MHz(50));

    // Delay constructor requires Rcc to be already configured. 
    //let delay = dp.TIM2.delay_us(&mut rcc);              //Tick at 1Mhz
    //let delay = dp.TIM2.delay::<25000000>(&mut rcc);     //Tick at 25Mhz
    //let delay = dp.TIM2.delay::<50000000>(&mut rcc);     //Tick at 50Mhz
    let mut delay = dp.TIM2.delay::<100000000>(&mut rcc);     //Tick at 100Mhz
    // panic let delay = dp.TIM2.delay_ms(&mut rcc);       //Tick at 1Khz

    let gpioc = dp.GPIOC.split(&mut rcc);
    
    // these both work here
    delay.delay(20.millis());
    delay.delay_ms(20);

    // return tuple  (led, delay)
    (
        gpioc.pc13.into_push_pull_output(), // led on pc13 with on/off
        delay,
    )
}



#[entry]
fn main() -> ! {
    let (mut led, mut delay) = setup();

    led.blink(1000, &mut delay); // blink 1s to indicate setup complete.

    led.off();    
    delay.delay_ms(2000);  //off for  2s
    //delay.delay(2000.millis());  //off for 2s  trait not recognized

    led.on();     
    delay.delay_ms(20000);   // on for 20s to check delay timer

    led.off();    
    delay.delay_ms(3000);  //off for  3s

    // blink once per second
    let on: u32  = 10; // on for 10 ms
    let off: u32 = 990; //off for remainder of second

    // blink LED and sleep
    loop {
        led.blink(on, &mut delay);
        delay.delay_ms(off);
    }
}
