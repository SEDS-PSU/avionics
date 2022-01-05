#![cfg_attr(not(test), no_std)] // Disable the standard library when not testing.

mod valve;

pub use valve::{Valve, Valves, ValveState};

#[repr(u8)]
pub enum Op {
    /// Set all valve states.
    Set(Valves),
    // Wait a number of milliseconds before executing the next operation.
    Wait(u16),
}

const _: () = assert!(core::mem::size_of::<Op>() == 4);

pub type Response = Result<(), Error>;

#[repr(u8)]
pub enum Error {
    /// An unknown error occurred.
    Unknown,
}
