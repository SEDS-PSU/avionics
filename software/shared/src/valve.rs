use core::{mem, fmt};

use serde::{Serialize, Deserialize};

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize, defmt::Format)]
pub struct Valves {
    pub fo_fp: TwoWay,
    pub fc_fp: TwoWay,
    pub fc_p: TwoWay,
    pub fc1_f: TwoWay,
    pub fo_p1: TwoWay,
    pub fo2_o: TwoWay,
    pub fc4_o: TwoWay,
    pub fc3_o: TwoWay,
    pub pv_f: ThreeWay,
    pub pv_o: ThreeWay,
}

impl Default for Valves {
    fn default() -> Self {
        Valves {
            fo_fp: TwoWay::Closed,
            fc_fp: TwoWay::Closed,
            fc_p: TwoWay::Closed,
            fc1_f: TwoWay::Closed,
            fo_p1: TwoWay::Closed,
            fo2_o: TwoWay::Closed,
            fc4_o: TwoWay::Closed,
            fc3_o: TwoWay::Closed,
            pv_f: ThreeWay::NitrogenPathway,
            pv_o: ThreeWay::NitrogenPathway,
        }
    }
}

impl From<Valves> for PackedValves {
    fn from(v: Valves) -> Self {
        PackedValves(
            (v.fo_fp as u16) << 0
            | (v.fc_fp as u16) << 1
            | (v.fc_p as u16) << 2
            | (v.fc1_f as u16) << 3
            | (v.fo_p1 as u16) << 4
            | (v.fo2_o as u16) << 5
            | (v.fc4_o as u16) << 6
            | (v.fc3_o as u16) << 7
            | (v.pv_f as u16) << 8
            | (v.pv_o as u16) << 9,
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
            fo_fp: a(v.0 >> 1),
            fc_p: a(v.0 >> 2),
            fc1_f: a(v.0 >> 3),
            fo_p1: a(v.0 >> 4),
            fo2_o: a(v.0 >> 5),
            fc4_o: a(v.0 >> 6),
            fc3_o: a(v.0 >> 7),
            pv_f: b(v.0 >> 8),
            pv_o: b(v.0 >> 9),
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Deserialize, Serialize, postcard::MaxSize)]
#[repr(transparent)]
pub struct PackedValves(u16);

impl fmt::Debug for PackedValves {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let v = Valves::from(*self);
        write!(f, "{:?}", v)
    }
}

impl defmt::Format for PackedValves {
    fn format(&self, f: defmt::Formatter<'_>) {
        let v = Valves::from(*self);
        defmt::write!(f, "{:?}", v)
    }
}

const _: () = assert!(<PackedValves as postcard::MaxSize>::POSTCARD_MAX_SIZE == 2);

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize, defmt::Format)]
pub enum TwoWay {
    /// The valve is closed.
    Closed,
    /// The valve is open.
    Open,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize, defmt::Format)]
pub enum ThreeWay {
    /// The valve is routed from the nitrogen purge tank into the chamber.
    NitrogenPathway,
    /// The valve is routed from the fuel or oxidizer tank into the chamber.
    FuelOxidizer,
}
