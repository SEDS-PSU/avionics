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
pub struct SensorReadings(pub SensorReading, pub SensorReading, pub SensorReading, pub SensorReading);

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

const _: () = assert!(mem::size_of::<AllSensors>() == 40);

impl AllSensors {
    pub fn from_bytes(bytes: &[u8; 40]) -> &AllSensors {
        unsafe { &*(bytes as *const _ as *const _) }
    }

    pub fn into_iter(&self) -> [SensorReadings; 5] {
        [
            SensorReadings(self.tc1_e, self.tc2_e, self.tc1_f, self.tc2_f),
            SensorReadings(self.tc1_o, self.tc5_o, self.therm7, self.fm_f),
            SensorReadings(self.fm_o, self.load1, self.load2, self.pt1_f),
            SensorReadings(self.pt2_f, self.pt1_e, self.pt2_e, self.pt1_o),
            SensorReadings(self.pt2_o, self.pt4_o, self.pt1_p, self.pt2_p),
        ]
    }
}
