use core::mem;


#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(transparent)]
pub struct ValvesStates(u16);

impl ValvesStates {
    pub const fn all_open() -> Self {
        ValvesStates(0)
    }

    pub const fn all_closed() -> Self {
        ValvesStates(!0)
    }

    pub fn open(&mut self, valve: Valve) {
        self.0 |= 1 << (valve as u16);
    }

    pub fn close(&mut self, valve: Valve) {
        self.0 &= !(1 << (valve as u16));
    }

    pub fn valves(&self) -> impl Iterator<Item = (Valve, ValveState)> {
        let x = self.0;
        (0..Valve::num()).map(move |idx| {
            let state = if x & (1 << idx) != 0 {
                ValveState::Open
            } else {
                ValveState::Closed
            };
            // SAFETY: `Valve::num()` must always return the number
            // of variants in `Valve`. Ideally, we'd use `mem::variants`,
            // but that's still unstable.
            (unsafe { mem::transmute(idx) }, state)
        })
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Valve {
    FcO,
    // etc
}

impl Valve {
    pub const fn num() -> u8 {
        1
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ValveState {
    /// The valve is closed.
    Closed,
    /// The valve is open.
    Open,
}
