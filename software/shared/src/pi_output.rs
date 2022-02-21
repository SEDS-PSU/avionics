//! Shared definitions and code for Pi <-> Output Board communication.

use core::num::NonZeroU16;

use serde::{Serialize, Deserialize};

use crate::PackedValves;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Deserialize, Serialize, postcard::MaxSize)]
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

const _: () = assert!(<Request as postcard::MaxSize>::POSTCARD_MAX_SIZE == 3);

#[derive(Copy, Clone, Debug, PartialEq, Eq, Deserialize, Serialize, postcard::MaxSize)]
#[serde(untagged)]
pub enum Wait {
    WaitMs(NonZeroU16),
    Forever,
}

const _: () = assert!(<Wait as postcard::MaxSize>::POSTCARD_MAX_SIZE == 3);

#[derive(Copy, Clone, Debug, PartialEq, Eq, Deserialize, Serialize, postcard::MaxSize)]
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

const _: () = assert!(<Command as postcard::MaxSize>::POSTCARD_MAX_SIZE == 6);

#[derive(Copy, Clone, Debug, PartialEq, Eq, Deserialize, Serialize, postcard::MaxSize)]
pub struct Status {
    pub states: Option<PackedValves>,
    pub state: State,
    pub error: Option<OutputBoardError>,
}

const _: () = assert!(<Status as postcard::MaxSize>::POSTCARD_MAX_SIZE == 6);

#[derive(Copy, Clone, Debug, PartialEq, Eq, Deserialize, Serialize, postcard::MaxSize)]
pub struct State(u8);

const _: () = assert!(<State as postcard::MaxSize>::POSTCARD_MAX_SIZE == 1);

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

#[derive(Copy, Clone, Debug, PartialEq, Eq, Deserialize, Serialize, postcard::MaxSize)]
pub enum OutputBoardError {
    Unknown,
}

const _: () = assert!(<OutputBoardError as postcard::MaxSize>::POSTCARD_MAX_SIZE == 1);
