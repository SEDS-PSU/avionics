//! Shared definitions and code for Pi <-> Output Board communication.

use core::{num::NonZeroU16, mem};

use crate::PackedValves;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Request {
    /// The output board should respond with status.
    GetStatus,
    /// The output board should set all valves to the given state immediately.
    SetValvesImmediately(PackedValves),
    /// The following `length` frames each contain one command ([`Command`]) each.
    BeginSequence {
        length: u8
    },
    /// The output board should reset itself.
    Reset,
}

const _: () = assert!(mem::size_of::<Request>() == 4);

impl Request {
    pub fn as_bytes(self) -> [u8; 4] {
        unsafe { mem::transmute(self) }
    }

    pub fn from_bytes(bytes: [u8; 4]) -> Self {
        unsafe { mem::transmute(bytes) }
    }
}

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
        states: PackedValves,
        wait: Wait,
    },
    Ignite {
        /// The between ignition and the next command being executed.
        /// This is in milliseconds.
        delay: u16,
    }
}

const _: () = assert!(mem::size_of::<Command>() == 6);

impl Command {
    pub fn as_bytes(self) -> [u8; 6] {
        unsafe { mem::transmute(self) }
    }

    pub fn from_bytes(bytes: [u8; 6]) -> Self {
        unsafe { mem::transmute(bytes) }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(C)]
pub struct Status {
    pub states: Option<PackedValves>,
    pub state: State,
    pub error: Option<OutputBoardError>,
}

const _: () = assert!(mem::size_of::<Status>() == 6);

impl Status {
    pub fn as_bytes(self) -> [u8; 6] {
        unsafe { mem::transmute(self) }
    }

    pub fn from_bytes(bytes: [u8; 6]) -> Self {
        unsafe { mem::transmute(bytes) }
    }
}

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
