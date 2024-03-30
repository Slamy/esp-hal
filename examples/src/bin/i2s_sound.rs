//! This shows how to transmit data continously via I2S.
//!
//! Pins used:
//! BCLK    GPIO2
//! WS      GPIO4
//! DOUT    GPIO5
//!
//! Without an additional I2S sink device you can inspect the MCLK, BCLK, WS
//!  andDOUT with a logic analyzer.
//!
//! You can also connect e.g. a PCM510x to hear an annoying loud sine tone (full
//! scale), so turn down the volume before running this example.
//!
//! Wiring is like this:
//!
//! | Pin   | Connected to    |
//! |-------|-----------------|
//! | BCK   | GPIO0           |
//! | DIN   | GPIO2           |
//! | LRCK  | GPIO1           |
//! | SCK   | Gnd             |
//! | GND   | Gnd             |
//! | VIN   | +3V3            |
//! | FLT   | Gnd             |
//! | FMT   | Gnd             |
//! | DEMP  | Gnd             |
//! | XSMT  | +3V3            |

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    analog::dac::{DAC1, DAC2},
    clock::ClockControl,
    dma::{Dma, DmaPriority},
    dma_buffers,
    gpio::IO,
    i2s::{DataFormat, I2s, I2sWriteDma, Standard},
    peripherals::Peripherals,
    prelude::*,
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            let dac1_pin = io.pins.gpio25.into_analog();
            let dac2_pin = io.pins.gpio26.into_analog();
        } else if #[cfg(feature = "esp32s2")] {
            let dac1_pin = io.pins.gpio17.into_analog();
            let dac2_pin = io.pins.gpio18.into_analog();
        }
    }
    // Create DAC instances
    let mut dac1 = DAC1::new(peripherals.DAC1, dac1_pin);
    let mut dac2 = DAC2::new(peripherals.DAC2, dac2_pin);

    let dma = Dma::new(peripherals.DMA);
    #[cfg(any(feature = "esp32", feature = "esp32s2"))]
    let dma_channel = dma.i2s0channel;
    #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
    let dma_channel = dma.channel0;

    let (tx_buffer, mut tx_descriptors, _, mut rx_descriptors) = dma_buffers!(32000, 0);

    let i2s = I2s::new(
        peripherals.I2S0,
        Standard::Philips,
        DataFormat::Data16Channel16,
        44100.Hz(),
        dma_channel.configure(
            false,
            &mut tx_descriptors,
            &mut rx_descriptors,
            DmaPriority::Priority0,
        ),
        &clocks,
    );

    let mut i2s_tx = i2s.i2s_tx.build();

    // construct a stereo sine signal with one channel having a sine wave
    // and the other one a cosine wave.
    // Connect to an oscilloscope in XY mode and you see a circle!
    let mut SINE: [u16; 64] = [0; 64];
    for i in 0..SINE.len() / 2 {
        let phi = (i as f32 / 32.0) * core::f32::consts::PI * 2.0;
        SINE[i * 2 + 0] = (0x8000 as f32 + libm::sinf(phi) * 0x7f00 as f32) as u16;
        SINE[i * 2 + 1] = (0x8000 as f32 + libm::cosf(phi) * 0x7f00 as f32) as u16;
    }
    println!("{:?}", SINE);
    let data =
        unsafe { core::slice::from_raw_parts(&SINE as *const _ as *const u8, SINE.len() * 2) };

    let mut idx = 0;
    for i in 0..usize::min(data.len(), tx_buffer.len()) {
        tx_buffer[i] = data[idx];

        idx += 1;

        if idx >= data.len() {
            idx = 0;
        }
    }

    let mut filler = [0u8; 10000];
    let mut transfer = i2s_tx.write_dma_circular(&tx_buffer).unwrap();

    loop {
        let avail = transfer.available();
        if avail > 0 {
            let avail = usize::min(10000, avail);
            for bidx in 0..avail {
                filler[bidx] = data[idx];
                idx += 1;

                if idx >= data.len() {
                    idx = 0;
                }
            }
            transfer.push(&filler[0..avail]).unwrap();
        }
    }
}
