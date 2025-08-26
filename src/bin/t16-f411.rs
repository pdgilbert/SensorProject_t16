//! Measure temperature on sixteen temperature sensors (NTC 3950 10k thermistors probes) 
//! using  4-channel adc's on I2C2 and crate ads1x1x. 
//! Display using SSD1306 on I2C1.
//! Transmit with LoRa on SPI.

//  based on crate rust-integration-testing  examples/projects/temperature-display_no-rtic.rs

//! Status: Compiles and runs, August 22, 2025. 
//!         Prototype PCB hardware needs adjustments. (I2C1 not properly wired.)

//!  To Do:
//!    - improve calculation of sensor mv to degree C.

//! https://www.ametherm.com/thermistor/ntc-thermistor-beta

// set this with
// MONITOR_ID="whatever" cargo build ...
// MONITOR_ID="whatever"  cargo  run --no-default-features --target thumbv7em-none-eabihf --features v020_2025-05 --bin t16-f411
// or  cargo:rustc-env=MONITOR_ID="whatever"
//const MONITOR_IDU : &[u8] = option_env!("MONITOR_ID").expect("Txxx").as_bytes();

///////////////////// 

#![deny(unsafe_code)]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

const MONITOR_ID: &str = option_env!("MONITOR_ID").expect("Txxx");
const MONITOR_IDU : &[u8] = MONITOR_ID.as_bytes();

const MODULE_CODE:  &str = "t16-f411"; 
const READ_INTERVAL:  u32 = 600;  // used as seconds 
const BLINK_DURATION: u32 = 1;    // used as seconds  but  ms would be better
const S_FMT:       usize  = 12;
const MESSAGE_LEN: usize  = 16 * S_FMT;  

//////////////////// 

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

//use cortex_m_semihosting::{debug, hprintln};
//use cortex_m_semihosting::{hprintln};
//use cortex_m::asm; // for delay
use cortex_m_rt::entry;

use core::fmt::Write;

use stm32f4xx_hal::{
    pac::{Peripherals, I2C1, SPI1, TIM5},
    rcc::{RccExt},
    spi::{Spi},
    i2c::I2c,   //this is a type vs  embedded_hal::i2c::I2c trait
    gpio::{Output, PushPull, GpioExt, Input},
    gpio::{Pin}, 
    gpio::{gpioa::{PA1, PA4, }},
    gpio::{gpiob::{PB4, PB5, }},  
    gpio::{gpioc::{PC13}},
    prelude::*,
    block,
    timer::{Delay as halDelay},
    timer::{TimerExt},
};

use embedded_hal::{spi::{Mode, Phase, Polarity},
                   delay::DelayNs,
                   digital::OutputPin,
};


//////////////////////  ssd display  

// See https://docs.rs/embedded-graphics/0.7.1/embedded_graphics/mono_font/index.html re fonts

use embedded_graphics::{
    mono_font::{iso_8859_1::FONT_8X13 as FONT, MonoTextStyleBuilder}, // need iso for degree symbol
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

//common display sizes are 128x64 and 128x32
type  DisplaySizeType = ssd1306::prelude::DisplaySize128x64;
const DISPLAYSIZE: DisplaySizeType = DisplaySize128x64;
type  DisplayType = Ssd1306<I2CInterface<I2c<I2C1>>, DisplaySizeType, BufferedGraphicsMode<DisplaySizeType>>;

use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};


/////////////////////  lora

use t16_f411::lora::Base;
use t16_f411::lora::{CONFIG_RADIO};

use radio_sx127x::{
    Transmit,  // trait needs to be in scope to find  methods start_transmit and check_transmit.
    //Error as sx127xError, // Error name conflict with hals
    prelude::*, // prelude has Sx127x,
    //device::regs::Register, // for config examination on debugging
    // read_register, get_mode, get_signal_bandwidth, get_coding_rate_4, get_spreading_factor,

};


/////////////////////  for 4 adc sharing bus

use core::cell::RefCell;
use embedded_hal_bus::i2c;  // has RefCellDevice;

//   //////////////////////////////////////////////////////////////////////

trait LED: OutputPin {  // see The Rust Programming Language, section 19, Using Supertraits...
    // depending on board wiring, on may be set_high or set_low, with off also reversed
    // A default of set_low() for on is defined here, but implementation should deal with a difference
    fn on(&mut self) -> () {
        self.set_low().unwrap()
    }
    fn off(&mut self) -> () {
        self.set_high().unwrap()
    }

}

impl LED for  PC13<Output<PushPull>> {}    

struct SpiExt {  cs:    PA4<Output<PushPull>>, 
                 busy:  PB4<Input<>>, 
                 ready: PB5<Input<>>, 
                 reset: PA1<Output<PushPull>>
}

const MODE: Mode = Mode {
    //  SPI mode for radio
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};


/////////////////////   adc
//use ads1x1x::{Ads1x1x, ic::Ads1115, ic::Resolution16Bit, channel, FullScaleRange, TargetAddr};
use ads1x1x::{Ads1x1x, channel, FullScaleRange, TargetAddr};

// The units of measurement are  [32767..-32768] for 16-bit devices (and  [2047..-2048] for 12-bit devices)
// but only half the range is used for single-ended measurements. (Precision could be improved by using
// one input set at GND as one side of a differential measurement.)
// set FullScaleRange to measure expected max voltage.

// SCALE is measured units per mV, so the adc measurement is divided by SCALE to get mV.
// Calibrated to get mV depends on FullScaleRange.
// adc.set_full_scale_range(FullScaleRange::Within4_096V) sets adc max to 4.096v.
// Check by connecting J2 +pin to 3.3v on blackpill and J2 -pin NC.
// (Also J1 +pin to GND on blackpill to check 0, but this will cause power draw through 10k).
// (blackpill pin measures  3200mv by meter, so measurement/SCALE should give 3200mv.)

const  SCALE: i64 = 8 ;  // = 32767 / 4096

    // The FullScaleRange needs to be small if measuring diff across low value shunt resistors for current
    //   but would be up to 5v when measuring usb power.
    // +- 6.144v , 4.096v, 2.048v, 1.024v, 0.512v, 0.256v


/////////////////////////// functions ////////////////////////////////////////

    fn show_display(
        t: [i64; 16],
        //disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
        disp: &mut Option<DisplayType>,
    ) -> ()
    {    
     if disp.is_some() { // show_message does nothing if disp is None. 
       let mut line: heapless::String<128> = heapless::String::new(); // \n to separate lines
           
       // Consider handling error in next. If line is too short then attempt to write it crashes
       write!(line,   "1:{:3}.{:1}  {:3}.{:1}°C",  t[0]/10,t[0].abs() %10,   t[1]/10,t[1].abs() %10).unwrap();
       write!(line, "\n3:{:3}.{:1}  {:3}.{:1}°C",  t[2]/10,t[2].abs() %10,   t[3]/10,t[3].abs() %10,).unwrap();
       write!(line, "\n5:{:3}.{:1}  {:3}.{:1}°C",  t[4]/10,t[4].abs() %10,   t[5]/10,t[5].abs() %10,).unwrap();
       write!(line, "\n7:{:3}.{:1}  {:3}.{:1}°C",  t[6]/10,t[6].abs() %10,   t[7]/10,t[7].abs() %10,).unwrap();
       write!(line, "\n9:{:3}.{:1}  {:3}.{:1}°C",  t[8]/10,t[8].abs() %10,   t[9]/10,t[9].abs() %10,).unwrap();
 //      delay.delay_ms(3000u32);
       line.clear();
       write!(line, "\n7:{:3}.{:1}  {:3}.{:1}°C",  t[6]/10,t[6].abs() %10,   t[7]/10,t[7].abs() %10,).unwrap();
       write!(line, "\n9:{:3}.{:1}  {:3}.{:1}°C",  t[8]/10,t[8].abs() %10,   t[9]/10,t[9].abs() %10,).unwrap();
       write!(line, "\n11:{:3}.{:1} {:3}.{:1}°C",  t[10]/10,t[10].abs() %10, t[11]/10,t[11].abs() %10,).unwrap();
       write!(line, "\n13:{:3}.{:1} {:3}.{:1}°C",  t[12]/10,t[12].abs() %10, t[13]/10,t[13].abs() %10,).unwrap();
       write!(line, "\n15:{:3}.{:1} {:3}.{:1}°C",  t[14]/10,t[14].abs() %10, t[15]/10,t[15].abs() %10,).unwrap();

   // CHECK SIGN IS CORRECT FOR -0.3 C
   // NEED TO DISPLAY THE REST
      // hprintln!(" t {:?} = 10 * degrees", t);

      show_message(&line, disp);
       ()
     };
    }



    fn show_message(
        text: &str,   //text_style: MonoTextStyle<BinaryColor>,
        //disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
        disp: &mut Option<DisplayType>,
    ) -> ()
    {
       if disp.is_some() { // show_message does nothing if disp is None. 
           //let mut disp = *disp;   
           // workaround. build here because text_style cannot be shared
           let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();
        
           disp.as_mut().expect("REASON").clear_buffer();
           Text::with_baseline( &text, Point::new(0, 0), text_style, Baseline::Top)
                   .draw(disp.as_mut().expect("REASON"))
                   .unwrap();

           disp.as_mut().expect("REASON").flush().unwrap();
       };
       ()
    }

    fn form_temp(
            t: [i64; 16],
           ) -> heapless::Vec<u8, MESSAGE_LEN> {

        let mut line: heapless::Vec<u8, MESSAGE_LEN> = heapless::Vec::new(); 
        let mut temp: heapless::Vec<u8, S_FMT> = heapless::Vec::new(); 

        // Consider handling error in next. If line is too short then attempt to write it crashes
        
        for i in 0..MONITOR_IDU.len() { line.push(MONITOR_IDU[i]).unwrap()};
        line.push(b'<').unwrap();
        
        // t is long enough for 16 sensors - J1 to J16 on a module
        for i in 0..t.len() {
                temp.clear();
                //hprintln!(" J{}:{:3}.{:1}",      i+1, t[i]/10, t[i].abs() %10); // t[0] is for J1
                write!(temp,  " J{}:{:3}.{:1}",  i+1, t[i]/10, t[i].abs() %10).unwrap(); // must not exceed S_FMT
                //hprintln!("temp {:?}  temp.len {}", temp, temp.len());
                for j in 0..temp.len() {line.push(temp[j]).unwrap()};
        };

        line.push(b'>').unwrap();
         
        line
     }


    //type LoraType = Sx127x<Base<Spi<SPI1>, Pin<'A', 4, Output>, Pin<'B', 4>, Pin<'B', 5>, Pin<'A', 0, Output>, halDelay<TIM5, 1000000>>>;
    type LoraType = Sx127x<Base<Spi<SPI1>, Pin<'A', 4, Output>, Pin<'B', 4>, Pin<'B', 5>, Pin<'A', 1, Output>, halDelay<TIM5, 1000000>>>;

    fn send(
            lora: &mut LoraType,
            m:  heapless::Vec<u8, MESSAGE_LEN>,
            disp: &mut Option<DisplayType>,
           ) -> () {
        
        match lora.start_transmit(&m) {
            Ok(_b)   => {//show_message("start_transmit ok", disp);
                         //hprintln!("lora.start ok")
                        } 
            Err(_e)  => {//show_message("start_transmit error", disp);
                         //hprintln!("Error in lora.start_transmit()")
                        }
        };

        lora.delay_ms(10); // treated as seconds. Without some delay next returns bad. (interrupt may also be an option)

        match lora.check_transmit() {
            Ok(b)   => {if b {//show_message("TX good", disp);
                              //hprintln!("TX good"); 
                             }
                        else {show_message("TX bad", disp);
                              //hprintln!("TX bad")
                             }
                       }
            Err(_e) => {//show_message("check_transmit Fail", disp);
                        //hprintln!("check_transmit() Error. Should return True or False."))
                       }
        };
        lora.delay_ms(3);

      show_message(&MONITOR_ID, disp);
       ()
    }

//   //////////////////////////////////////////////////////////////////

#[entry]
fn main() -> ! {

    //hprintln!("t16-f411");

    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);

    //v020_2025-05 has wiring error:  (scl, sda) on (pb8, pb6) UFQFPN48 pins (45, 42)
    let scl = gpiob.pb8.into_alternate_open_drain(); 
    let sda = gpiob.pb9.into_alternate_open_drain(); // not pb6, should be pb7 or pb9 on pins 43 or 46
    let i2c1 = I2c::new(dp.I2C1, (scl, sda), 400.kHz(), &mut rcc);

    let scl = gpiob.pb10.into_alternate_open_drain();
    let sda = gpiob.pb3.into_alternate_open_drain();
    let i2c2 = I2c::new(dp.I2C2, (scl, sda), 400.kHz(), &mut rcc);

    let mut led = gpioc.pc13.into_push_pull_output();
    led.off();

    let spi = Spi::new(
        dp.SPI1,
        (
            Some(gpioa.pa5.into_alternate()), // sck  
            Some(gpioa.pa6.into_alternate()), // miso 
            Some(gpioa.pa7.into_alternate()), // mosi 
        ),
        MODE, 8.MHz(), &mut rcc,
    );
    
    let spiext = SpiExt {
         cs:    gpioa.pa4.into_push_pull_output(), //CsPin         
         busy:  gpiob.pb4.into_floating_input(),   //BusyPin  DI00 
         ready: gpiob.pb5.into_floating_input(),   //ReadyPin DI01 
         reset: gpioa.pa1.into_push_pull_output(), //ResetPin   
         };   


    let mut delay = dp.TIM5.delay(&mut rcc);

    led.off();
    delay.delay_ms(2000); // treated as ms   
    led.on();
    delay.delay_ms(BLINK_DURATION); // treated as ms
    led.off();

    /////////////////////   ssd

    let interface = I2CDisplayInterface::new(i2c1); // ssd on I2C1 with default address 0x3C

    let mut z = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0).into_buffered_graphics_mode();

    //hprintln!("display initializing.");

    // disp is Optional because ssd screen may not be plugged in. It is set None if init fails.
    // If it is None then messages displays are skipped.
    // It will be None for  v020_2025-05 because  I2C1 for ssd is improperly wired.

    #[cfg(feature = "v020_2025-05")]
    let mut disp: Option<DisplayType> = None;

    #[cfg(not(feature = "v020_2025-05"))]
    let mut disp: Option<DisplayType> = match z.init() {
        Ok(_d)  => {Some(z)
                   },
        Err(e)  => {hprintln!("display set None. {:?}", e);
                    None
                   }
    };
    
    show_message(MODULE_CODE, &mut disp);
    delay.delay_ms(2000); // treated as ms
    //hprintln!("display init done.");

     
    /////////////////////   adc

    // ADS11x5 chips allows four different I2C addresses using one address pin ADDR. 
    // Connect ADDR pin to GND for 0x48(1001000) , to VCC for 0x49. to SDA for 0x4A, and to SCL for 0x4B.

    let i2c_ref_cell = RefCell::new(i2c2); //  adc on I2C2

    let mut adc_a = Ads1x1x::new_ads1115(i2c::RefCellDevice::new(&i2c_ref_cell),  TargetAddr::Gnd);
    let mut adc_b = Ads1x1x::new_ads1115(i2c::RefCellDevice::new(&i2c_ref_cell),  TargetAddr::Vdd);
    let mut adc_c = Ads1x1x::new_ads1115(i2c::RefCellDevice::new(&i2c_ref_cell),  TargetAddr::Sda);
    let mut adc_d = Ads1x1x::new_ads1115(i2c::RefCellDevice::new(&i2c_ref_cell),  TargetAddr::Scl);

    //hprintln!("adc initialized.");
    show_message("adc initialized.", &mut disp);

    // set FullScaleRange to measure expected max voltage.
    // This is very small if measuring diff across low value shunt resistors for current
    //   but would be up to 5v when measuring usb power.
    // +- 6.144v , 4.096v, 2.048v, 1.024v, 0.512v, 0.256v

    // wiring errors such as I2C1 on PB8-9 vs I2C2 on PB10-3 show up here as Err(I2C(ARBITRATION)) in Result
    match adc_a.set_full_scale_range(FullScaleRange::Within4_096V) {  
        Ok(())  =>  (),
        Err(_e)  =>  {show_message("range error.", &mut disp);
                     delay.delay_ms(2000u32);
                     //hprintln!("Error {:?} in adc_a.set_full_scale_range(). Check i2c is on proper pins.", e); 
                     //panic!("panic")
                    },
    };

    match adc_b.set_full_scale_range(FullScaleRange::Within4_096V) {
        Ok(())  =>  (),
        Err(_e)  =>  {show_message("range error.", &mut disp);
                     delay.delay_ms(2000u32);
                     //hprintln!("Error {:?} in adc_b.set_full_scale_range(). Check i2c is on proper pins.", e); 
                     //panic!("panic")
                    },
    };

    match adc_c.set_full_scale_range(FullScaleRange::Within4_096V) {
        Ok(())  =>  (),
        Err(_e)  =>  {show_message("range error.", &mut disp);
                     delay.delay_ms(2000u32);
                     //hprintln!("Error {:?} in adc_c.set_full_scale_range(). Check i2c is on proper pins.", e); 
                     //panic!("panic")
                    },
    };

    match adc_d.set_full_scale_range(FullScaleRange::Within4_096V) {
        Ok(())  =>  (),
        Err(_e)  =>  {show_message("range error.", &mut disp);
                     delay.delay_ms(2000u32);
                     //hprintln!("Error {:?} in adc_d.set_full_scale_range(). Check i2c is on proper pins.", e); 
                     //panic!("panic")
                    },
    };
    //hprintln!("set_full_scale_range done.");

    /////////////////////   lora

    // cs is named nss on many radio_sx127x module boards
    let z = Sx127x::spi(spi, spiext.cs,  spiext.busy, spiext.ready, spiext.reset, delay, 
                   &CONFIG_RADIO ); 

    let mut lora =  match z {
        Ok(lr)  => {show_message("lora setup ok", &mut disp);
                    //hprintln!("lora setup completed.");
                    lr
                   } 
        Err(e)  => {show_message("lora setup Error", &mut disp);
                    //hprintln!("Error in lora setup. {:?}", e);
                    //asm::bkpt();
                    panic!("{:?}", e) 
                   }
    };
 

    //delay consumed by lora. It is available in lora BUT treats arg as seconds not ms!!
    lora.delay_ms(1);  // arg is being treated as seconds

    /////////////////////   loop

   //COMPARE temperature-display_4jst REGARDING CALCULATION HERE
   //  REALLY DO BETTER APROX.
   // very crude linear aproximation mv to degrees C using 
   // based on 
   // t = a + v/b , v in mV, b inverse slope
   let a = 72i64;    //  72 deg
   let b = -34i64;   //  -34 mv/degree   
   //hprintln!("a {:?}  b {:?}   SCALE {:?}", a,b, SCALE);

   loop { 
      // blink
      led.on(); 
      lora.delay_ms(BLINK_DURATION);  // arg is being treated as seconds
      led.off();
      
      // note the range can be switched if needed, (and switched back) eg.
      //adc_a.set_full_scale_range(FullScaleRange::Within0_256V).unwrap();

      let values_a = [
          block!(adc_a.read(channel::SingleA0)).unwrap_or(-40) as i64 / SCALE,
          block!(adc_a.read(channel::SingleA1)).unwrap_or(-40) as i64 / SCALE,
          block!(adc_a.read(channel::SingleA2)).unwrap_or(-40) as i64 / SCALE,
          block!(adc_a.read(channel::SingleA3)).unwrap_or(-40) as i64 / SCALE,
      ];

      let values_b = [
          block!(adc_b.read(channel::SingleA0)).unwrap_or(-40) as i64 / SCALE,
          block!(adc_b.read(channel::SingleA1)).unwrap_or(-40) as i64 / SCALE,
          block!(adc_b.read(channel::SingleA2)).unwrap_or(-40) as i64 / SCALE,
          block!(adc_b.read(channel::SingleA3)).unwrap_or(-40) as i64 / SCALE,
      ];

      let values_c = [
          block!(adc_c.read(channel::SingleA0)).unwrap_or(-40) as i64 / SCALE,
          block!(adc_c.read(channel::SingleA1)).unwrap_or(-40) as i64 / SCALE,
          block!(adc_c.read(channel::SingleA2)).unwrap_or(-40) as i64 / SCALE,
          block!(adc_c.read(channel::SingleA3)).unwrap_or(-40) as i64 / SCALE,
      ];

      let values_d = [
          block!(adc_d.read(channel::SingleA0)).unwrap_or(-40) as i64 / SCALE,
          block!(adc_d.read(channel::SingleA1)).unwrap_or(-40) as i64 / SCALE,
          block!(adc_d.read(channel::SingleA2)).unwrap_or(-40) as i64 / SCALE,
          block!(adc_d.read(channel::SingleA3)).unwrap_or(-40) as i64 / SCALE,
      ];

      // this is ugly
      let mut mv:[i64; 16]= [-100; 16];
      for i in 0..4  {mv[   i] = values_a[i]};
      for i in 0..4  {mv[ 4+i] = values_b[i]};
      for i in 0..4  {mv[ 8+i] = values_c[i]};
      for i in 0..4  {mv[12+i]=  values_d[i]};

      //If the mv value is over 3000 (temperature < about -19.0 C ) then the thermistor is probably missing.
      
      //hprintln!(" mv{:?} = values_a mv ", values_a);
      //hprintln!(" mv{:?} =      mv     ", mv);

      //show_display(mv, &mut disp);

 //     for i in 0..mv.len() { mv[i] = v[i] as i64 / SCALE};  
 //     //hprintln!(" mv {:?}", mv);

      // t in tenths of a degrees C, so it is an int  but t[0]/10, t[0].abs() %10 give a degree with one decimal.
      let mut t:[i64; 16] = [-100; 16] ;

      //for i in 0..t.len() { t[i] = 10 * (a + mv[i] / b) }; // loses the decimal rounding division
      for i in 0..t.len() { t[i] =  10 * a  + (10 * mv[i]) / b };
 
      //hprintln!(" t {:?} = 10 * degrees", t);
      
      let message = form_temp(t);
      //hprintln!("message {:?}", message);

      show_display(t, &mut disp);
      send(&mut lora, message, &mut disp);

      //delay.delay_ms(READ_INTERVAL);// treated as ms
      lora.delay_ms(READ_INTERVAL);  // treated as seconds

   }
}
