//! Shared definitions and code for Pi <-> Sensor Board communication.

use core::mem;

use serde::{Serialize, Deserialize};

#[derive(Copy, Clone, Debug, PartialEq, Eq, Deserialize, Serialize, postcard::MaxSize)]
pub enum Request {
    /// The sensor board should respond with 6 frames.
    /// The first contains the general status of the sensor board.
    /// The following 5 frames each contain four sensor readings.
    GetSensorData,
    /// The sensor board should reset itself.
    Reset,
}

const _: () = assert!(<Request as postcard::MaxSize>::POSTCARD_MAX_SIZE == 1);

#[derive(Copy, Clone, Debug, PartialEq, Eq, Deserialize, Serialize, postcard::MaxSize)]
pub struct ResponseHeader {
    pub doing_good: bool,
}

const _: () = assert!(<ResponseHeader as postcard::MaxSize>::POSTCARD_MAX_SIZE == 1);

#[derive(Copy, Clone, Debug, PartialEq, Eq, Deserialize, Serialize, postcard::MaxSize)]
pub struct SensorReading(u16);

const _: () = assert!(<SensorReading as postcard::MaxSize>::POSTCARD_MAX_SIZE == 2);

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

impl Default for SensorReading {
    fn default() -> Self {
        Self::new_error(SensorError::NoData)
    }
}

pub enum SensorError {
    Unknown,
    NoData,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, Deserialize, Serialize, postcard::MaxSize)]
pub struct SensorReadings(pub SensorReading, pub SensorReading, pub SensorReading, pub SensorReading);

const _: () = assert!(<SensorReadings as postcard::MaxSize>::POSTCARD_MAX_SIZE == 8);

/// On sensor board, call the [`AllSensors::into_iter`] function to get an iterator over the
/// the frames to send to the Pi.
/// 
/// On the Pi, call [`AllSensors::from_bytes`] to turn the raw data into [`AllSensors`].
#[derive(Clone, Default, Deserialize, Serialize, postcard::MaxSize)]
pub struct AllSensors {
    // Thermocouples 
    pub tc1_e: SensorReading,
    pub tc2_e: SensorReading,
    pub tc1_f: SensorReading,
    pub tc2_f: SensorReading,
    pub tc1_o: SensorReading,
    pub tc5_o: SensorReading,
    /// Unused?
    pub therm7: SensorReading,

    // Flow Meters
    pub fm_f: SensorReading,
    pub fm_o: SensorReading,

    // Load Cells
    pub load1: SensorReading,
    pub load2: SensorReading,

    // Pressure Transducers
    pub pt1_f: SensorReading,
    pub pt2_f: SensorReading,
    pub pt1_e: SensorReading,
    pub pt2_e: SensorReading,
    pub pt1_o: SensorReading,
    pub pt2_o: SensorReading,
    pub pt4_o: SensorReading,
    pub pt1_p: SensorReading,
    pub pt2_p: SensorReading,
}

const _: () = assert!(<AllSensors as postcard::MaxSize>::POSTCARD_MAX_SIZE == 40);
