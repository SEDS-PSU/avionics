#![cfg_attr(not(test), no_std)]

use core::convert::Infallible;
use core::slice;
use embedded_hal::{
    digital::v2::{OutputPin, InputPin},
    blocking::{
        delay::DelayMs,
        spi::{Transfer, Write}
    },
};

mod consts;
use self::consts::*;

struct CsGuard<'a, CS: OutputPin<Error = Infallible>>(&'a mut CS);

impl<'a, CS: OutputPin<Error = Infallible>> CsGuard<'a, CS> {
    fn new(cs: &'a mut CS) -> Self {
        let _ = cs.set_low();
        Self(cs)
    }
}

impl<'a, CS: OutputPin<Error = Infallible>> Drop for CsGuard<'a, CS> {
    fn drop(&mut self) {
        let _ = self.0.set_high();
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error<E> {
    InvalidDevice,
    InvalidDeviceState,
    SpiError(E),
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Gain {
    X1 = 0b000,
    X2 = 0b001,
    X4 = 0b010,
    X8 = 0b011,
    X16 = 0b100,
    X32 = 0b101,
    X64 = 0b110,
    X128 = 0b111,
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Channel {
    AIN0 = 0b0000,
    AIN1 = 0b0001,
    AIN2 = 0b0010,
    AIN3 = 0b0011,
    AIN4 = 0b0100,
    AIN5 = 0b0101,
    AIN6 = 0b0110,
    AIN7 = 0b0111,
    AIN8 = 0b1000,
    AIN9 = 0b1001,
    AIN10 = 0b1010,
    AIN11 = 0b1011,
    AINCOM = 0b1100,
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Self::SpiError(e)
    }
}

pub struct ADS114S08<SPI, CS, RST, DRDY, DELAY> {
    spi: SPI,
    cs: CS,
    reset: RST,
    ready: DRDY,
    delay: DELAY,
}

impl<SPI, CS, RST, DRDY, DELAY, E> ADS114S08<SPI, CS, RST, DRDY, DELAY>
where
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
    CS: OutputPin<Error = Infallible>,
    RST: OutputPin<Error = Infallible>,
    DRDY: InputPin<Error = Infallible>,
    DELAY: DelayMs<u8>,
{
    /// Create a new ADS114S08 instance.
    ///
    /// This sets the device into continuous conversion mode.
    pub fn new(spi: SPI, cs: CS, reset: RST, ready: DRDY, delay: DELAY) -> Result<Self, Error<E>> {
        let mut this = Self {
            spi,
            cs,
            reset,
            ready,
            delay,
        };
        
        this.init()?;
        
        Ok(this)
    }

    pub fn read(&mut self, low: Channel, high: Channel, gain: Gain) -> Result<i16, Error<E>> {
        while let Some(false) = self.ready.is_low().ok() {}
        let mut guard = CsGuard::new(&mut self.cs);

        // Set the channels that are read from.
        let inpmux = (low as u8) | ((high as u8) << 4);
        let gain = gain as u8 | 0b01 << 4;

        Self::write_regs(&mut self.spi, &mut guard, Reg::INPMUX, [inpmux, gain])?;

        // Get the reading from the device
        self.spi.write(&[cmd::RDATA])?;
        let mut data = [0; 2];
        self.spi.transfer(&mut data)?;

        Ok(i16::from_le_bytes(data))
    }

    fn init(&mut self) -> Result<(), Error<E>> {
        let mut guard = CsGuard::new(&mut self.cs);

        // Reset the device
        self.spi.write(&[cmd::RESET])?;
        self.delay.delay_ms(1);

        // Check if the RDY bit is 0.
        let [status] = Self::read_regs(&mut self.spi, &mut guard, Reg::STATUS)?;
        if status & (1 << 6) != 0 {
            return Err(Error::InvalidDevice);
        }

        // Clear the FL_POR flag
        Self::write_regs(&mut self.spi, &mut guard, Reg::STATUS, [0])?;
        
        self.spi.write(&[cmd::START])?;

        Ok(())
    }

    fn read_regs<const N: usize>(spi: &mut SPI, _: &mut CsGuard<CS>, start_reg: Reg) -> Result<[u8; N], Error<E>> {
        assert!(N <= 0b1_1111);

        spi.write(&cmd::rreg(start_reg as u8, N as u8))?;
        let mut output = [0; N];
        spi.transfer(&mut output)?;

        Ok(output)
    }

    fn write_regs<const N: usize>(spi: &mut SPI,  _: &mut CsGuard<CS>, start_reg: Reg, data: [u8; N]) -> Result<(), Error<E>> {
        assert!(N <= 0b1_1111);

        spi.write(&cmd::wreg(start_reg as u8, N as u8))?;
        spi.write(&data)?;

        Ok(())
    }
}
