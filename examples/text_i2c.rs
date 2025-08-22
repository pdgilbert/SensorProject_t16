//! Using crate ssd1306 to print with i2c on a generic ssd1306 based OLED display.
//! Compare crate rust-integration-testing  examples/misc/text_i2c.
//!
//! Print "Hello world!" then "Hello rust!". Uses the `embedded_graphics` crate to draw.
//! Wiring pin connections for scl and sda to display as in the setup sections below.
//! Tested on generic (cheap) ssd1306 OLED 0.91" 128x32 and 0.96" 128x64 displays.
//! Note that the DisplaySize setting needs to be adjusted for 128x64 or 128x32 display

#![no_std]
#![no_main]

use cortex_m_rt::entry;
//use cortex_m_rt::{entry, exception, ExceptionFrame};
use cortex_m_semihosting::hprintln;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

use panic_halt as _;

use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use stm32f4xx_hal::{
    i2c::{I2c},
    pac::{Peripherals, I2C1},
    rcc::Config,
    prelude::*,
};

#[cfg(feature = "v020_2025-05")]
fn setup() -> I2c<I2C1> {
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let gpiob = dp.GPIOB.split(&mut rcc);

    // can have (scl, sda) using I2C1  on (PB8  _af4, PB9 _af4) or on  (PB6 _af4, PB7 _af4)
    //     or   (scl, sda) using I2C2  on (PB10 _af4, PB3 _af9)

    // I2C2 is wired to ads bus
    //let scl = gpiob.pb10.into_alternate().set_open_drain(); // scl on PB10
    //let sda = gpiob.pb3.into_alternate().set_open_drain();  // sda on PB3
    //let i2c = I2c::new(dp.I2C2, (scl, sda), 400.kHz(), &mut rcc);

    //v020_2025-05 has I2C1 wiring error:  (scl, sda) on (pb8, pb6) UFQFPN48 pins (45, 42)
    //  Should not be pb6, should be pb7 or pb9 on pins 43 or 46
    let scl = gpiob.pb8.into_alternate().set_open_drain(); 
    let sda = gpiob.pb9.into_alternate().set_open_drain();   
    let i2c = I2c::new(dp.I2C1, (scl, sda), 400.kHz(), &mut rcc);

    rcc.freeze(Config::default() );

    i2c
}


#[entry]
fn main() -> ! {
    hprintln!("text_i2c example");
    let i2c = setup();
    hprintln!("done setup()");

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    hprintln!("start init()");
    match display.init() {
          Ok(_v) => {hprintln!("done init()")},
          Err(e) => {hprintln!("init() error {:?}", e);
                     panic!("{:#?}", e)
                    },
    };

    hprintln!("done interface");

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    hprintln!("done Hello world!");

    Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    display.flush().unwrap();

    hprintln!("enter loop {}");
    loop {}
}

//#[exception]
//fn HardFault(ef: &ExceptionFrame) -> ! {  // requires unsafe as of cortex-m-rt = "0.7.0"
//    panic!("{:#?}", ef);
//}
