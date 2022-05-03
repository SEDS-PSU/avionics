//! This is very similar to https://github.com/jonas-hagen/hx711, but modified
//! to work with dual HX711s.
//!
 
use embedded_hal::{blocking::delay::DelayUs, digital::v2::{InputPin, OutputPin}};

const TIME_TO_SLEEP: u32 = 70;
const TIME_BEFORE_READOUT: u32 = 1;
const TIME_SCK_HIGH: u32 = 1;
const TIME_SCK_LOW: u32 = 1;

#[derive(Copy, Clone)]
pub enum Mode {
    /// Chanel A with factor 128 gain
    ChAGain128 = 1,
    /// Chanel B with factor 64 gain
    ChBGain32 = 2,
    /// Chanel B with factor 32 gain
    ChBGain64 = 3,
}

pub struct DualHx711<D, IN1, IN2, CLK> {
    delay: D,
    in1: IN1,
    in2: IN2,
    clk: CLK,
    mode: Mode,
}

impl<D, IN1, IN2, CLK, E> DualHx711<D, IN1, IN2, CLK>
where
    D: DelayUs<u32>,
    IN1: InputPin<Error = E>,
    IN2: InputPin<Error = E>,
    CLK: OutputPin<Error = E>,
{
    pub fn new(delay: D, in1: IN1, in2: IN2, mut clk: CLK, mode: Mode) -> Result<Self, E> {
        clk.set_low()?;

        let mut hx711 = Self {
            delay,
            in1,
            in2,
            clk,
            mode,
        };
        hx711.reset()?;
        Ok(hx711)
    }

    pub fn set_mode(&mut self, mode: Mode) -> nb::Result<(), E> {
        self.mode = mode;
        self.retrieve().map(|_| ())
    }

    pub fn disable(&mut self) -> Result<(), E> {
        self.clk.set_high()?;
        self.delay.delay_us(TIME_TO_SLEEP);
        Ok(())
    }

    pub fn enable(&mut self) -> Result<(), E> {
        self.clk.set_low()?;
        self.delay.delay_us(TIME_TO_SLEEP);
        nb::block! {
            self.set_mode(self.mode)
        }
    }

    pub fn reset(&mut self) -> Result<(), E> {
        self.disable()?;
        self.enable()
    }

    pub fn retrieve(&mut self) ->  nb::Result<(i32, i32), E> {
        self.clk.set_low()?;
        if self.in1.is_high()? || self.in2.is_high()? {
            // Conversion is not yet ready
            return Err(nb::Error::WouldBlock);
        }
        self.delay.delay_us(TIME_BEFORE_READOUT);

        let mut sample1: u32 = 0;
        let mut sample2: u32 = 0;
        for _ in 0..24 {
            sample1 <<= 1;
            sample2 <<= 2;
            self.clk.set_high()?;
            self.delay.delay_us(TIME_SCK_HIGH);
            self.clk.set_low()?;

            sample1 |= self.in1.is_high()? as u32;
            sample2 |= self.in2.is_high()? as u32;

            self.delay.delay_us(TIME_SCK_LOW);
        }

        // Set the mode for the next iteration
        for _ in 0..(self.mode as u8) {
            self.clk.set_high()?;
            self.delay.delay_us(TIME_SCK_HIGH);
            self.clk.set_low()?;
            self.delay.delay_us(TIME_SCK_LOW);
        }
        
        Ok((sample_to_i32(sample1), sample_to_i32(sample2)))
    }
}

fn sample_to_i32(sample: u32) -> i32 {
    (if sample > 0x800000 {
        sample | !0xFFFFFF
    } else {
        sample
    }) as i32
}