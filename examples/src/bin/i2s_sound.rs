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

use core::cell::RefCell;

use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    analog::dac::{DAC1, DAC2},
    clock::ClockControl,
    delay::Delay,
    dma::{Dma, DmaPriority},
    dma_buffers,
    gpio::IO,
    i2s::{DataFormat, I2s, I2sWriteDma, I2sWriteDmaTransfer, Standard},
    interrupt::{self, Priority},
    peripherals::{Interrupt, Peripherals},
    prelude::*,
};
use esp_println::println;

//static BUTTON: Mutex<RefCell<Option<I2sWriteDmaTransfer>>> = Mutex::new(RefCell::new(None));

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
    let delay = Delay::new(&clocks);

    // Create DAC instances
    let _dac1 = DAC1::new(peripherals.DAC1, dac1_pin);
    let _dac2 = DAC2::new(peripherals.DAC2, dac2_pin);

    let dma = Dma::new(peripherals.DMA);
    #[cfg(any(feature = "esp32", feature = "esp32s2"))]
    let dma_channel = dma.i2s0channel;
    #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
    let dma_channel = dma.channel0;

    let (tx_buffer, mut tx_descriptors, _, mut rx_descriptors) = dma_buffers!(128, 0);

    let i2s = I2s::new(
        peripherals.I2S0,
        Standard::DAC,
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

    interrupt::enable(Interrupt::I2S0, Priority::Priority1).unwrap();

    let mut i2s_tx = i2s.i2s_tx.build();

    // construct a stereo sine signal with one channel having a sine wave
    // and the other one a cosine wave.
    // Connect to an oscilloscope in XY mode and you see a circle!
    for i in 0..tx_buffer.len() / 4 {
        let phi = (i as f32 / 32.0) * core::f32::consts::PI * 2.0;
        tx_buffer[i * 4 + 1] = (0x80 as f32 + libm::sinf(phi) * 0x7c as f32) as u8;
        tx_buffer[i * 4 + 3] = (0x80 as f32 + libm::cosf(phi) * 0x7c as f32) as u8;
        tx_buffer[i * 4 + 0] = 0;
        tx_buffer[i * 4 + 2] = 0;

        // tx_buffer[i * 4 + 1] = if ((i & 1) == 0) { 0xf0 } else { 0x00 };
        // tx_buffer[i * 4 + 3] = if ((i & 1) == 1) { 0xf0 } else { 0x00 };
        // tx_buffer[i * 4 + 0] = 0;
        // tx_buffer[i * 4 + 2] = 0;
    }
    /*
    tx_buffer[0 * 4 + 1] = 0xff;
    tx_buffer[1 * 4 + 1] = 0x00;
    tx_buffer[2 * 4 + 1] = 0x80;
    tx_buffer[3 * 4 + 1] = 0x80;
    */
    println!("{:?}", tx_buffer);

    let mut transfer = i2s_tx.write_dma(&tx_buffer).unwrap();
    transfer.clear_int();
    loop {
        //println!("X");
        delay.delay_micros(50);

        //transfer.wait().unwrap();
    }
}

#[interrupt]
fn I2S0() {
    // interrupt_handler();
    esp_println::println!("I2S0 Interrupt");

    //let mut device = RTC.borrow_ref_mut(cs);
    //let device = rtc.as_mut().unwrap();
}
