//! Shared definitions and code for Pi <-> Sensor Board communication.

use core::{mem, fmt};

use serde::{Serialize, Deserialize};

#[derive(Copy, Clone, Debug, PartialEq, Eq, Deserialize, Serialize, postcard::MaxSize)]
pub enum Request {
    // /// The sensor board should respond with 6 frames.
    // /// The first contains the general status of the sensor board.
    // /// The following 5 frames each contain four sensor readings.
    // GetSensorData,
    /// Start sending data from the sensor board to the pi.
    StartSensing,
    /// The sensor board should reset itself.
    Reset,
}

const _: () = assert!(<Request as postcard::MaxSize>::POSTCARD_MAX_SIZE == 1);

#[derive(Copy, Clone, Debug, PartialEq, Eq, Deserialize, Serialize, postcard::MaxSize)]
pub struct ResponseHeader {
    pub doing_good: bool,
    /// The battery voltage in millivolts.
    pub battery_millivolts: Option<u16>,
}

const _: () = assert!(<ResponseHeader as postcard::MaxSize>::POSTCARD_MAX_SIZE == 4);

#[derive(Debug)]
pub enum SensorError {
    Unknown = 0,
    NoData = 1,
    OutOfRange = 2,
}

#[derive(Copy, Clone, PartialEq, Eq, Deserialize, Serialize, postcard::MaxSize)]
pub struct SensorReading(u16);

const _: () = assert!(<SensorReading as postcard::MaxSize>::POSTCARD_MAX_SIZE == 2);

impl SensorReading {
    pub const fn new(data: u16) -> Self {
        if data & (1 << 15) != 0 {
            Self::new_error(SensorError::OutOfRange)
        } else {
            Self(data) // lop off the MSB
        }
    }

    pub const fn new_error(error: SensorError) -> Self {
        SensorReading(error as u16 | (1 << 15))
    }

    pub fn unpack(self) -> Result<u16, SensorError> {
        if self.0 & (1 << 15) == 0 {
            Ok(self.0)
        } else {
            assert!(self.0 as u8 <= 2);
            Err(unsafe { mem::transmute(self.0 as u8)})
        }
    }
}

impl fmt::Debug for SensorReading {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self.unpack() {
            Ok(value) => write!(f, "{}", value),
            Err(error) => write!(f, "{:?}", error),
        }
    }
}

impl Default for SensorReading {
    fn default() -> Self {
        Self::new_error(SensorError::NoData)
    }
}



#[derive(Copy, Clone, PartialEq, Eq, Default, Deserialize, Serialize, postcard::MaxSize)]
pub struct Temperature(SensorReading);

const _: () = assert!(<Temperature as postcard::MaxSize>::POSTCARD_MAX_SIZE == 2);

impl Temperature {
    /// Receives a temperature in degrees Celsius.
    pub const fn new(degrees: i16) -> Self {
        Temperature(SensorReading::new((degrees * 2) as u16 >> 1))
    }

    pub const fn new_error(error: SensorError) -> Self {
        Temperature(SensorReading::new_error(error))
    }

    /// Returns the temperature in degrees celsius.
    pub fn unpack(self) -> Result<i16, SensorError> {
        self.0.unpack().map(|data| {
            (data << 1) as i16 / 2
        })
    }
}

impl fmt::Debug for Temperature {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self.unpack() {
            Ok(value) => write!(f, "{}°C", value),
            Err(error) => write!(f, "{:?}", error),
        }
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

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, Deserialize, Serialize, postcard::MaxSize)]
pub struct Pressure(SensorReading);

const _: () = assert!(<Force as postcard::MaxSize>::POSTCARD_MAX_SIZE == 2);

impl Pressure {
    /// Recieves the pressure in PSIG.
    pub const fn new(psig: u16) -> Self {
        Self(SensorReading::new(psig))
    }

    pub const fn new_error(error: SensorError) -> Self {
        Self(SensorReading::new_error(error))
    }

    /// Return the pressure in PSIG.
    pub fn unpack(self) -> Result<u16, SensorError> {
        self.0.unpack()
    }
}

#[derive(Deserialize, Serialize, postcard::MaxSize)]
pub struct Pressure1 {
    pub pt1_f: Pressure,
    pub pt2_f: Pressure,
    pub pt1_e: Pressure,
    pub pt2_o: Pressure,
}

const _: () = assert!(<Pressure1 as postcard::MaxSize>::POSTCARD_MAX_SIZE == 8);

#[derive(Deserialize, Serialize, postcard::MaxSize)]
pub struct Pressure2 {
    pub pt3_o: Pressure,
    pub pt4_o: Pressure,
    pub pt2_p: Pressure,
}

const _: () = assert!(<Pressure2 as postcard::MaxSize>::POSTCARD_MAX_SIZE == 6);

#[derive(Deserialize, Serialize, postcard::MaxSize)]
pub struct FlowAndLoad {
    pub fm_f: SensorReading,
    pub fm_o: SensorReading,
    pub load1: Force,
    pub load2: Force,
}

const _: () = assert!(<FlowAndLoad as postcard::MaxSize>::POSTCARD_MAX_SIZE == 8);

#[derive(Deserialize, Serialize, postcard::MaxSize)]
pub struct Thermo1 {
    pub tc1_e: Temperature,
    pub tc1_f: Temperature,
    pub tc2_f: Temperature,
    pub tc1_o: Temperature,
}

const _: () = assert!(<Thermo1 as postcard::MaxSize>::POSTCARD_MAX_SIZE == 8);

#[derive(Deserialize, Serialize, postcard::MaxSize)]
pub struct Thermo2 {
    pub tc5_o: Temperature,
}

const _: () = assert!(<Thermo2 as postcard::MaxSize>::POSTCARD_MAX_SIZE == 2);

// #[derive(Clone, Default, Deserialize, Serialize, postcard::MaxSize, Debug)]
// pub struct AllSensors {
//     // Thermocouples 
//     pub tc1_e: Temperature,
//     /// Unused
//     pub thermo2: Temperature,
//     pub tc1_f: Temperature,
//     pub tc2_f: Temperature,
//     pub tc1_o: Temperature,
//     pub tc5_o: Temperature,
//     /// Unused?
//     pub therm7: Temperature,

//     // Flow Meters
//     pub fm_f: SensorReading,
//     pub fm_o: SensorReading,

//     // Load Cells
//     pub load1: Force,
//     pub load2: Force,

//     // Pressure Transducers
//     pub pt1_f: Pressure,
//     pub pt2_f: Pressure,
//     pub pt1_e: Pressure,
//     pub pt2_o: Pressure,
//     /// Unused
//     pub pres5: Pressure,
//     pub pt3_o: Pressure,
//     /// Unused
//     pub pres7: Pressure,
//     pub pt4_o: Pressure,
//     pub pt2_p: Pressure,
// }

// const _: () = assert!(<AllSensors as postcard::MaxSize>::POSTCARD_MAX_SIZE == 40);
