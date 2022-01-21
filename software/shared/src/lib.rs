#![cfg_attr(not(test), no_std)] // Disable the standard library when not testing.

mod valve;
pub mod pi_output;
pub mod pi_sensor;

pub use valve::{Valve, ValveStates, ValveState};

pub const RASPI_ID: u16 = 0;
pub const OUTPUT_BOARD_ID: u16 = 1;
pub const SENSOR_BOARD_ID: u16 = 2;
