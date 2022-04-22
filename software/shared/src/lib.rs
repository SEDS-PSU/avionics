#![cfg_attr(not(test), no_std)] // Disable the standard library when not testing.

mod valve;
pub mod pi_output;
pub mod pi_sensor;

pub use valve::{Valves, PackedValves, TwoWay, ThreeWay};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Id {
    Raspi = 0,
    OutputBoard = 1,
    SensorBoard = 2,
}
