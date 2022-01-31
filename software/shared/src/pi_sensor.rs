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
        if data & (1 << 15) != 0 {
            Self::new_error(SensorError::OutOfRange)
        } else {
            Self(data ^ (1 << 15)) // lop off the MSB
        }
    }

    pub const fn new_error(error: SensorError) -> Self {
        SensorReading(error as u16 | (1 << 15))
    }

    pub const fn unpack(self) -> Result<u16, SensorError> {
        if self.0 & (1 << 15) == 0 {
            Ok(self.0)
        } else {
            assert!(self.0 as u8 <= 1);
            Err(unsafe { mem::transmute(self.0 as u8)})
        }
    }
}

impl Default for SensorReading {
    fn default() -> Self {
        Self::new_error(SensorError::NoData)
    }
}



#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, Deserialize, Serialize, postcard::MaxSize)]
pub struct Temperature(SensorReading);

const _: () = assert!(<Temperature as postcard::MaxSize>::POSTCARD_MAX_SIZE == 2);

impl Temperature {
    /// Receives a temperature with a LSB that represents 0.25 degrees C.
    pub const fn new(temp_quarter_degree: i16) -> Self {
        Temperature(SensorReading::new(temp_quarter_degree as u16 >> 1))
    }

    pub const fn new_error(error: SensorError) -> Self {
        Temperature(SensorReading::new_error(error))
    }

    /// Returns the temperature with a resolution of 0.5 degrees C.
    pub fn unpack(self) -> Result<f32, SensorError> {
        self.0.unpack().map(|temp_quarter_degree| {
            (temp_quarter_degree << 1) as i16 as f32 / 2.0
        })
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, Deserialize, Serialize, postcard::MaxSize)]
pub struct Force(SensorReading);

const _: () = assert!(<Force as postcard::MaxSize>::POSTCARD_MAX_SIZE == 2);

impl Force {
    pub const fn new(newtons: u16) -> Self {
        Self(SensorReading::new(newtons))
    }

    pub const fn new_error(error: SensorError) -> Self {
        Self(SensorReading::new_error(error))
    }

    /// Return the force in Newtons.
    pub fn unpack(self) -> Result<u16, SensorError> {
        self.0.unpack()
    }
}


pub enum SensorError {
    Unknown = 0,
    NoData = 1,
    OutOfRange = 2,
}

#[derive(Clone, Default, Deserialize, Serialize, postcard::MaxSize)]
pub struct AllSensors {
    // Thermocouples 
    pub tc1_e: Temperature,
    pub tc2_e: Temperature,
    pub tc1_f: Temperature,
    pub tc2_f: Temperature,
    pub tc1_o: Temperature,
    pub tc5_o: Temperature,
    /// Unused?
    pub therm7: Temperature,

    // Flow Meters
    pub fm_f: SensorReading,
    pub fm_o: SensorReading,

    // Load Cells
    pub load1: Force,
    pub load2: Force,

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
