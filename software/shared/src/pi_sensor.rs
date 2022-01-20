//! Shared definitions and code for Pi <-> Sensor Board communication.

use core::mem;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Request {
    /// The sensor board should respond with 6 frames.
    /// The first contains the general status of the sensor board.
    /// The following 5 frames each contain four sensor readings.
    GetSensorData,
    /// The sensor board should reset itself.
    Reset,
}

const _: () = assert!(mem::size_of::<Request>() == 1);

impl Request {
    pub fn as_bytes(self) -> [u8; 1] {
        unsafe { mem::transmute(self) }
    }

    pub fn from_bytes(bytes: [u8; 1]) -> Self {
        unsafe { mem::transmute(bytes) }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(C)]
pub struct ResponseHeader {
    pub doing_good: bool,
}

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

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(C)]
pub struct SensorReadings(SensorReading, SensorReading, SensorReading, SensorReading);

const _: () = assert!(mem::size_of::<SensorReadings>() == 8);

impl SensorReadings {
    pub fn as_bytes(self) -> [u8; 8] {
        unsafe { mem::transmute(self) }
    }

    pub fn from_bytes(bytes: [u8; 8]) -> Self {
        unsafe { mem::transmute(bytes) }
    }
}

/// On sensor board, call the [`AllSensors::into_iter`] function to get an iterator over the
/// the frames to send to the Pi.
/// 
/// On the Pi, call [`AllSensors::from_bytes`] to turn the raw data into [`AllSensors`].
#[repr(C)]
pub struct AllSensors {
    // Thermocouples 
    pub therm1: SensorReading,
    pub therm2: SensorReading,
    pub therm3: SensorReading,
    pub therm4: SensorReading,
    pub therm5: SensorReading,
    pub therm6: SensorReading,
    pub therm7: SensorReading,

    // Flow Meters
    pub flow1: SensorReading,
    pub flow2: SensorReading,

    // Load Cells
    pub load1: SensorReading,
    pub load2: SensorReading,

    // Pressure Transducers
    pub pressure1: SensorReading,
    pub pressure2: SensorReading,
    pub pressure3: SensorReading,
    pub pressure4: SensorReading,
    pub pressure5: SensorReading,
    pub pressure6: SensorReading,
    pub pressure7: SensorReading,
    pub pressure8: SensorReading,
    pub pressure9: SensorReading,
}

const _: () = assert!(mem::size_of::<AllSensors>() == 40);

impl AllSensors {
    pub fn from_bytes(bytes: &[u8; 40]) -> &AllSensors {
        unsafe { &*(bytes as *const _ as *const _) }
    }

    pub fn into_iter(&self) -> impl Iterator<Item = SensorReadings> {
        [
            SensorReadings(self.therm1, self.therm2, self.therm3, self.therm4),
            SensorReadings(self.therm5, self.therm6, self.therm7, self.flow1),
            SensorReadings(self.flow2, self.load1, self.load2, self.pressure1),
            SensorReadings(self.pressure2, self.pressure3, self.pressure4, self.pressure5),
            SensorReadings(self.pressure6, self.pressure7, self.pressure8, self.pressure9),
        ].into_iter()
    }
}
