//!  Transmit a simple message with LoRa using crate radio_sx127x (on SPI).
//!
//!  Compare crate rust-integration-testing  examples/radio/lora_spi_send.
//!
//!  Status: tested ...
//!     Debugging with hprintln requires probe connect from computer.
//!     Code with  hprintln does not work on battery power. Uncomment hprintln if needed.
//!
//!     lora.delay_ms(5) CONVERTS 5 TO SECONDS ON  STM32F411 BUT MILLISECONDS ON  STM32G474
//!
//!  See FREQUENCY in src/lora.rs to set the channel.
//!  Before running, check  FREQUENCY to be sure you have a channel setting appropriate for
//!  your country, hardware and any testing sender/receiver on the other end of the communication.
//!

#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

// semihosting for debugging  requires connection to computer and openocd. 
// Comment out next and hprintln statements to run on battery.
//use cortex_m_semihosting::*;

//use cortex_m_semihosting::{debug, hprintln};
//use cortex_m_semihosting::{hprintln};

use embedded_hal::{spi::{Mode, Phase, Polarity},
                   delay::DelayNs,
                   digital::OutputPin,
};


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



//use radio::Transmit;  // trait needs to be in scope to find  methods start_transmit and check_transmit.
//use radio_sx127x::Transmit;  // trait needs to be in scope to find  methods start_transmit and check_transmit.

use radio_sx127x::{
    //Error as sx127xError, // Error name conflict with hals
    prelude::*, // prelude has Sx127x,
};

// for config examination on debugging
//use radio_sx127x::{
//    device::regs::Register,
//   // read_register, get_mode, get_signal_bandwidth, get_coding_rate_4, get_spreading_factor,  
//};


//   //////////////////////////////////////////////////////////////////////

/////////////////////  lora

//use t16_f411::lora::Base;
use t16_f411::lora::{CONFIG_RADIO};

use radio_sx127x::{
    Transmit,  // trait needs to be in scope to find  methods start_transmit and check_transmit.
    //Error as sx127xError, // Error name conflict with hals
    //prelude::*, // prelude has Sx127x,
    //device::regs::Register, // for config examination on debugging
    // read_register, get_mode, get_signal_bandwidth, get_coding_rate_4, get_spreading_factor,

};


//   //////////////////////////////////////////////////////////////////////

use stm32f4xx_hal::{
    gpio::{PC13, PA1, PA4, PB4, PB5, Input, Output, PushPull},
    pac::{Peripherals, SPI1,},
    rcc::{RccExt},
    rcc::Config,
    spi::{Spi},
    prelude::*,
};

impl LED for PC13<Output<PushPull>>{}

pub type SpiType =  Spi<SPI1>;

//pub struct SpiExt { pub cs:    Pin<'A', 4, Output>, 
//                    pub busy:  Pin<'B', 4>, 
//                    pub ready: Pin<'B', 5>, 
//                    pub reset: Pin<'A', 1, Output>
//}
// these should just be in SpiExt, but radio Sx127x still wants them separately
pub type Cs    = PA4<Output<PushPull>>;
pub type Busy  = PB4<Input<>>;
pub type Ready = PB5<Input<>>;
pub type Reset = PA1<Output<PushPull>>;


// compare trait stm32f4xx_hal::spi::SpiExt,
pub struct SpiExt { pub cs:    Cs, 
                    pub busy:  Busy, 
                    pub ready: Ready, 
                    pub reset: Reset
}

const MODE: Mode = Mode {
    //  SPI mode for radio
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};


#[cfg(feature = "stm32f411")]
fn setup() -> (impl LED, SpiType, SpiExt, impl DelayNs) {
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);

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

    let delay = dp.TIM5.delay::<1000000_u32>(&mut rcc);

    rcc.freeze(Config::default());

    // return tuple
    (
        gpioc.pc13.into_push_pull_output(), // led on pc13 with on/off
        spi,
        spiext,
        delay,
    )
}


//   //////////////////////////////////////////////////////////////////////

#[entry]
fn main() -> ! {
    let (mut led, spi, spiext, mut delay) = setup(); 
    led.off();

    //hprintln!("start delay.delay_ms(5)").unwrap();
    delay.delay_ms(5);
    //hprintln!(" end  delay.delay_ms(5)").unwrap();

    // cs should be called nss
    let lora = Sx127x::spi(spi, spiext.cs,  spiext.busy, spiext.ready, spiext.reset, delay, 
                       &CONFIG_RADIO ); 

    let mut lora =  match lora {
            Ok(lr)  => { //hprintln!("lora setup completed.").unwrap();
                         lr
                       } 
            Err(e) =>  { //hprintln!("Error in lora setup. {:?}", e).unwrap();
                         panic!("{:?}", e)
                       }
    };
 
    //let mut lora = lora.unwrap();
 
    //delay is available in lora

    
   
    // print out configuration (for debugging)
//    hprintln!("frequency          {:?}", lora.get_frequency());

 //   use radio_sx127x::device::regs::Register;
 //
//    let v = lora.lora_get_config();
//    hprintln!("configuration {:?}", v).unwrap();
// 
//    hprintln!("channel      {}", lora.get_channel()).unwrap();
// 
//    hprintln!("mode             {}",    lora.get_mode()).unwrap();
//    hprintln!("mode             {}",    lora.read_register(Register::RegOpMode.addr())).unwrap();
//    hprintln!("bandwidth        {:?}",  lora.get_signal_bandwidth()).unwrap();
//    hprintln!("coding_rate      {:?}",  lora.get_coding_rate_4()).unwrap();
//    hprintln!("spreading_factor {:?}",  lora.get_spreading_factor()).unwrap();
//    hprintln!("invert_iq        {:?}",  lora.get_invert_iq()).unwrap();
//    hprintln!("tx_power         {:?}",  lora.get_tx_power()).unwrap();

    // transmit something

    //let buffer = &[0xaa, 0xbb, 0xcc];

    let message = b"Hello, LoRa!";

    //let mut buffer = [0;100];      //Nov 2020 limit data.len() < 255 in radio_sx127x  .start_transmit
    //for (i,c) in message.chars().enumerate() {
    //	buffer[i] = c as u8;
    //	}

    loop {
        match lora.start_transmit(message) {
            Ok(_b)   => { //hprintln!("start_transmit").unwrap()
                        } 
            Err(_e)  => { //hprintln!("Error in lora.start_transmit()").unwrap()
                        }
        };
        //hprintln!("start_transmit done").unwrap();

        lora.delay_ms(1); // without some delay next returns bad. (interrupt may also be an option)

        match lora.check_transmit() {
            Ok(b)   => {if b {//hprintln!("TX good").unwrap()
                             } 
                        else {//hprintln!("TX bad").unwrap()
                             }
                       }
            Err(_e) => {
                        //hprintln!("Error in lora.check_transmit(). Should return True or False.").unwrap()
                       }
        };
        //hprintln!("check_transmit done").unwrap();
        led.on();     
        lora.delay_ms(1);   // on for 1s
        led.off();    

        //lora.delay_ms(5000);
        lora.delay_ms(5);
        //hprintln!("re-loop").unwrap();
    }
}
