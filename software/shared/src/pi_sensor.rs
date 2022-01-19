//! Shared definitions and code for Pi <-> Sensor Board communication.

use core::mem;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Request {
    /// The sensor board should respond with 6 frames, each containing
    /// four sensor readings.
    GetSensorData,
    /// The sensor board should reset itself.
    Reset,
}

const _: () = assert!(mem::size_of::<Request>() == 1);

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(transparent)]
pub struct SensorReading(u16);

impl SensorReading {
    pub const fn new(data: u16) -> Self {
        SensorReading(data ^ (1 << 15)) // lop off the MSB
    }

    pub const fn new_error(error: SensorError) -> Self {
        SensorReading(error as u16 | (1 << 15))
    }

    pub const fn unpack(self) -> Result<u16, SensorError> {
        if self.0 & (1 << 15) == 0 {
            Ok(self.0)
        } else {
            Err(unsafe { mem::transmute(self.0 as u8)})
        }
    }
}

#[repr(u8)]
pub enum SensorError {
    Unknown,
}

pub mod response {
    use super::*;

    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    #[repr(C)]
    pub struct Frame0 {
        pub tc0: SensorReading,
        pub tc1: SensorReading,
        pub tc2: SensorReading,
        pub tc3: SensorReading,
    }

    const _: () = assert!(mem::size_of::<Frame0>() == 8);

    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    #[repr(C)]
    pub struct Frame1 {
        pub tc4: SensorReading,
        pub tc5: SensorReading,
        pub tc6: SensorReading,
        pub tc7: SensorReading,
    }

    const _: () = assert!(mem::size_of::<Frame1>() == 8);

    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    #[repr(C)]
    pub struct Frame2 {
        pub tc8: SensorReading,
        pub tc9: SensorReading,
        pub tc10: SensorReading,
        pub tc11: SensorReading,
    }

    const _: () = assert!(mem::size_of::<Frame2>() == 8);

    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    #[repr(C)]
    pub struct Frame3 {
        pub tc12: SensorReading,
        pub flow0: SensorReading,
        pub flow1: SensorReading,
        pub d: SensorReading,
    }

    const _: () = assert!(mem::size_of::<Frame3>() == 8);

    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    #[repr(C)]
    pub struct Frame4 {
        pub some_sensor: SensorReading,
        pub b: SensorReading,
        pub c: SensorReading,
        pub d: SensorReading,
    }

    const _: () = assert!(mem::size_of::<Frame4>() == 8);

    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    #[repr(C)]
    pub struct Frame5 {
        pub some_sensor: SensorReading,
        pub b: SensorReading,
        pub c: SensorReading,
        pub d: SensorReading,
    }

    const _: () = assert!(mem::size_of::<Frame5>() == 8);
}
