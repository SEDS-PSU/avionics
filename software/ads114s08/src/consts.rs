#![allow(dead_code)]

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Reg {
    ID = 0x00,
    STATUS = 0x01,
    INPMUX = 0x02,
    PGA = 0x03,
    DATARATE = 0x04,
    REF = 0x05,
    IDACMAG = 0x06,
    IDACMUX = 0x07,
    VBIAS = 0x08,
    SYS = 0x09,
    OFCAL0 = 0x0A,
    OFCAL1 = 0x0B,
    OFCAL2 = 0x0C,
    FSCAL0 = 0x0D,
    FSCAL1 = 0x0E,
    FSCAL2 = 0x0F,
    GPIODAT = 0x10,
    GPIOCON = 0x11,
}

pub mod cmd {
    // Control Commands
    /// No Operation
    pub const NOP: u8 = 0x0;
    /// Wake-up from power-down mode
    pub const WAKEUP: u8 = 0x02;
    /// Enter power-down mode
    pub const POWERDOWN: u8 = 0x04;
    /// Reset the device
    pub const RESET: u8 = 0x06;
    /// Start conversions
    pub const START: u8 = 0x08;
    /// Stop conversions
    pub const STOP: u8 = 0x0A;

    // Calibration Commands
    /// System offset calibration
    pub const SYOCAL: u8 = 0x16;
    /// System gain calibration
    pub const SYGCAL: u8 = 0x17;
    /// Self offset calibration
    pub const SFOCAL: u8 = 0x19;

    // Data Read Command
    /// Read data
    pub const RDATA: u8 = 0x12;

    // Register Read and Write Commands
    /// Read `num_regs` registers starting at address `start_addr`.
    pub fn rreg(start_addr: u8, num_regs: u8) -> [u8; 2] {
        [0b0010_0000u8 | (start_addr & 0b1_1111), 0b000_0000u8 | (num_regs & 0b1_1111)]
    }

    /// Write `num_regs` registers starting at address `start_addr`.
    pub fn wreg(start_addr: u8, num_regs: u8) -> [u8; 2] {
        [0b0100_0000u8 | (start_addr & 0b1_1111), 0b000_0000u8 | (num_regs & 0b1_1111)]
    }
}
