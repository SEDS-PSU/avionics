//! Shared definitions and code for Pi <-> Output Board communication.

use core::{num::NonZeroU16, mem};

use crate::{ValvesStates, ValveState};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Request {
    /// The output board should respond with status.
    GetStatus,
    /// The output board should set all valves to the given state immediately.
    SetValvesImmediately(ValvesStates),
    /// The following frames contain one or two commands ([`Command`]) each,
    /// adding up to `length` commands.
    BeginSequence {
        length: u8
    },
    /// The output board should reset itself.
    Reset,
}

const _: () = assert!(mem::size_of::<Request>() == 4);

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Wait {
    WaitMs(NonZeroU16),
    Forever,
}

const _: () = assert!(mem::size_of::<Wait>() == 2);

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Command {
    SetValves {
        states: ValveState,
        wait: Wait,
    },
    Ignite {
        timeout: u16,
    }
}

const _: () = assert!(mem::size_of::<Command>() == 4);

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(C)]
pub struct Status {
    pub states: ValvesStates,
    pub state: State,
    pub error: OutputBoardError,
}

const _: () = assert!(mem::size_of::<Status>() == 4);

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(transparent)]
pub struct State(u8);

impl State {
    pub const fn new() -> Self {
        Self(0)
    }

    pub const fn set_armed(self) -> Self {
        Self(self.0 | 0b1)
    }

    pub const fn set_ignited(self) -> Self {
        Self(self.0 | 0b10)
    }

    pub const fn is_armed(self) -> bool {
        self.0 & 0b1 != 0
    }

    pub const fn is_ignited(self) -> bool {
        self.0 & 0b10 != 0
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum OutputBoardError {
    Unknown,
}
