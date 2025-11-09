STATUS: Software compiles and runs, August 22, 2025. Prototype PCB hardware needs adjustments.

##  Contents

See the auto-generated menu in the github README display (above right).

## Summary

This crate has code for a hardware design as in repository
https://github.com/pdgilbert/SensorProject_t16-pcb.
It measures temperature from sixteen 10K NTC 3950 sensors using an adc. 
The measurements are transmitted by LoRa and optionally displayed on an SSD1306.
(Hardware is not supporting this yet.)

The code here is based on code in 
https://github.com/pdgilbert/rust-integration-testing/tree/main/examples/projects.
Relative to that repository, this crate is much simplified by targetting specific hardware
versions, which have a specific MCU and peripheral setup. 
The intention is that code here should be stable and remain working for the hardware.
The code in repository `rust-integration-testing` is intended for testing new versions of 
crates and hals, and as a result is sometimes broken.

The main code is in ```src/bin```.
Examples in this crate are intended for testing the hardware and do not (yet) demonstate the use of the crate.

## Building

```
MONITOR_ID="whatever"  cargo build --no-default-features  --target thumbv7em-none-eabihf  --features stm32f411 --bin t16-f411   [ --release ]
```
The ssd is disabled if  `--feature `v020_2025-05` is added. 
(The prototype hardware version `v0.2.0 2025-05` has a mis-wired I2C1 for the ssd.)

MONITOR_ID is optional. If not supplied "Txxx" will be used. 
The hal `stm32f4xx_hal` is used and set as a dependency in Cargo.toml.

## Loading

If `openocd`, `gdb`, `.cargo/config` with needed runners, and an appropriate probe are 
in place then in one window run

```
openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg
```
Adjust interface for your programming dongle.

In another window do
```
MONITOR_ID="whatever"  cargo  run --target thumbv7em-none-eabihf --features stm32f411 --bin t16-f411  [ --release]
```
The `--release` will be needed if code is too big for memory.

## Testing

Test with examples in `examples/` directory ( blink_default`,`blink_hsi`, `blink_hse`,
 `text_i2c`, `lora_spi_send` ). For example
```
cargo  run --target thumbv7em-none-eabihf --features stm32f411  --example blink_default
```
Some of the tests are to check subsets of the functionality needed for the 
main program src/bin/t16-f411.rs. Other test are to check pcb functionality
that is not yet used in the main program. Many of these tests will also run
on a `blackpill` with an `stm32f411` MCU. See comments in the test files for
addition information.

## License

Licensed under either of

 * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
   http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or
   http://opensource.org/licenses/MIT)

at your option.

## Contributing

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.
