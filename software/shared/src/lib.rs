#![cfg_attr(not(test), no_std)] // Disable the standard library when not testing.

mod valve;
pub mod pi_output;
pub mod pi_sensor;

pub use valve::{Valves, PackedValves, TwoWay, ThreeWay};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Id {
    RaspiOutputStatus = 0b00_000,
    RaspiPressure1 = 0b00_001,
    RaspiPressure2 = 0b00_010,
    RaspiFlowAndLoad = 0b00_011,
    OutputBoard = 0b01_000,
    SensorBoard = 0b10_000,
}
