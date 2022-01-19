#![cfg_attr(not(test), no_std)] // Disable the standard library when not testing.

mod valve;
pub mod pi_output;
pub mod pi_sensor;

pub use valve::{Valve, ValvesStates, ValveState};
