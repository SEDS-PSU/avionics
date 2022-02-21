use core::mem;

use serde::{Serialize, Deserialize};

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Valves {
    pub fc_fp: TwoWay,
    pub fc_op: TwoWay,
    pub fo_p: TwoWay,
    pub pv_f: ThreeWay,
    pub fo_fp: TwoWay,
    pub fc_p: TwoWay,
    pub pv_o: ThreeWay,
    pub fo_op: TwoWay,
    pub fc1_o: TwoWay,
    pub fc2_o: TwoWay,
}

impl Default for Valves {
    fn default() -> Self {
        Valves {
            fc_fp: TwoWay::Closed,
            fc_op: TwoWay::Closed,
            fo_p: TwoWay::Closed,
            pv_f: ThreeWay::NitrogenPathway,
            fo_fp: TwoWay::Closed,
            fc_p: TwoWay::Closed,
            pv_o: ThreeWay::NitrogenPathway,
            fo_op: TwoWay::Closed,
            fc1_o: TwoWay::Closed,
            fc2_o: TwoWay::Closed,
        }
    }
}

impl From<Valves> for PackedValves {
    fn from(v: Valves) -> Self {
        PackedValves(
            (v.fc_fp as u16) << 0
            | (v.fc_op as u16) << 1
            | (v.fo_p as u16) << 2 
            | (v.pv_f as u16) << 3
            | (v.fo_fp as u16) << 4
            | (v.fc_p as u16) << 5
            | (v.pv_o as u16) << 6
            | (v.fo_op as u16) << 7
            | (v.fc1_o as u16) << 8
            | (v.fc2_o as u16) << 9,
        )
    }
}

impl From<PackedValves> for Valves {
    fn from(v: PackedValves) -> Self {
        fn a(t: u16) -> TwoWay {
            unsafe { mem::transmute(t as u8 & 0b1) }
        }

        fn b(t: u16) -> ThreeWay {
            unsafe { mem::transmute(t as u8 & 0b1) }
        }

        Self {
            fc_fp: a(v.0 >> 0),
            fc_op: a(v.0 >> 1),
            fo_p: a(v.0 >> 2),
            pv_f: b(v.0 >> 3),
            fo_fp: a(v.0 >> 4),
            fc_p: a(v.0 >> 5),
            pv_o: b(v.0 >> 6),
            fo_op: a(v.0 >> 7),
            fc1_o: a(v.0 >> 8),
            fc2_o: a(v.0 >> 9),
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Deserialize, Serialize, postcard::MaxSize)]
#[repr(transparent)]
pub struct PackedValves(u16);

const _: () = assert!(<PackedValves as postcard::MaxSize>::POSTCARD_MAX_SIZE == 2);

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum TwoWay {
    /// The valve is closed.
    Closed,
    /// The valve is open.
    Open,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ThreeWay {
    /// The valve is routed from the nitrogen purge tank into the chamber.
    NitrogenPathway,
    /// The valve is routed from the fuel or oxidizer tank into the chamber.
    FuelOxidizer,
}
