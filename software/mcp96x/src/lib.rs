#![no_std]

// mod k_type;

use core::{slice, fmt};

use embedded_hal::blocking::i2c;

pub enum ThermocoupleType {
    K = 0b000,
    J = 0b001,
    T = 0b010,
    N = 0b011,
    S = 0b100,
    E = 0b101,
    B = 0b110,
    R = 0b111,
}

pub enum FilterCoefficient {
    Off = 0,
    Minimum = 1,
    N2 = 2,
    N3 = 3,
    Mid = 4,
    N5 = 5,
    N6 = 6,
    Maximum = 7,
}

#[derive(Debug)]
pub struct Fault {
    pub open_circuit: bool,
    pub short_circuit: bool,
}

#[derive(Debug)]
pub enum Error<E: fmt::Debug> {
    I2C(E),
    InvalidDevice,
}

// impl<E: fmt::Debug> fmt::Debug for Error<E> {
//     fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
//         match self {
//             Self::I2C(_) => write!(f, "I2C error"),
//             Self::InvalidDevice => write!(f, "InvalidDevice"),
//         }
//     }
// }

impl<E: fmt::Debug> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::I2C(error)
    }
}

mod reg {
    // pub const HOT_JUNCTION: u8 = 0x00;
    pub const JUNCTION_DELTA: u8 = 0x01;
    // pub const COLD_JUNCTION: u8 = 0x02;
    // pub const RAW_ADC_DATA: u8 = 0x03;
    pub const STATUS: u8 = 0x04;
    pub const SENSOR_CONFIG: u8 = 0x05;
    pub const DEVICE_CONFIG: u8 = 0x06;
    pub const DEVICE_ID: u8 = 0x20;
}

pub struct MCP96X<I2C: i2c::WriteRead<u8> + i2c::Write<u8>, const ADDRESS: u8> {
    i2c: I2C,
}

impl<E: fmt::Debug, I2C: i2c::WriteRead<u8, Error = E> + i2c::Write<u8, Error = E>, const ADDRESS: u8>
    MCP96X<I2C, ADDRESS>
{
    pub fn new(
        i2c: I2C,
        ty: ThermocoupleType,
        filter: FilterCoefficient,
    ) -> Result<Self, Error<E>> {
        let mut this = Self { i2c };

        // First read doesn't work for some reason?
        let _ = this.device_id_revision();

        let (id, _) = this.device_id_revision()?;

        if !(id == 0b01000000 || id == 0b01000001) {
            return Err(Error::InvalidDevice);
        }

        this.set_default_device_config()?;
        this.configure_sensor(ty, filter)?;

        Ok(this)
    }

    /// Read this cold-junction compensated and error-corrected thermocouple temperature
    /// in degrees Celsius.
    pub fn read(&mut self) -> Result<i16, Error<E>> {
        // Read the temperature.
        let mut buf = [0; 2];
        self.i2c
            .write_read(ADDRESS, &[reg::JUNCTION_DELTA], &mut buf)?;

        defmt::info!("junction read: {:#?}", buf);

        // let temp = i16::from_be_bytes(buf);
        let temp = ((buf[1] << 8) as u16 | buf[0] as u16) as i16 / 16;

        // Reset the update flag.
        let mut status = 0;
        self.i2c
            .write_read(ADDRESS, &[reg::STATUS], slice::from_mut(&mut status))?;
        status &= !(1 << 6); // clear the update flag
        self.i2c.write(ADDRESS, &[reg::STATUS, status])?;

        Ok(temp)
    }

    // /// Read this cold-junction compensated and error-corrected thermocouple temperature
    // /// in degrees Celsius.
    // pub fn read_raw_and_convert(&mut self) -> Result<i16, Error<E>> {
    //     let mut buf = [0; 3];
    //     self.i2c
    //         .write_read(ADDRESS, &[reg::RAW_ADC_DATA], &mut buf)?;
        
    //     let raw = (buf[0] as u32) << 16 | (buf[1] as u32) << 8 | (buf[2] as u32);

    //     // The thermocouples are flipped on the PCB.
    //     let half_microvolts = -(if raw >> 23 != 0 {
    //         raw | 0xFF_00_00_00
    //     } else {
    //         raw
    //     } as i32);

    //     let millivolts = fixed::FixedI32::from_num(half_microvolts) / 500;
    //     let hot_junction = k_type::t(millivolts);

    //     // compensate for reference temperature.
    //     let mut buf = [0; 2];
    //     self.i2c
    //         .write_read(ADDRESS, &[reg::COLD_JUNCTION], &mut buf)?;
    //     let reference_temp = (((buf[0] as u16) << 8 | (buf[1] as u16)) as i16) / 16;

    //     let real_temp = hot_junction + reference_temp;

    //     Ok(real_temp)
    // }

    pub fn has_fault(&mut self) -> Result<Fault, Error<E>> {
        let mut status = 0;
        self.i2c.write_read(ADDRESS, &[reg::STATUS], slice::from_mut(&mut status))?;

        Ok(Fault {
            open_circuit: status & (1 << 4) != 0,
            short_circuit: status & (1 << 5) != 0,
        })
    }

    /// Check if the junction temperature has been updated since the last read.
    pub fn has_updated(&mut self) -> Result<bool, Error<E>> {
        let mut status = 0;
        self.i2c
            .write_read(ADDRESS, &[reg::STATUS], slice::from_mut(&mut status))?;

        Ok(status & (1 << 6) != 0)
    }

    pub fn configure_sensor(
        &mut self,
        ty: ThermocoupleType,
        filter: FilterCoefficient,
    ) -> Result<(), Error<E>> {
        let conf = (ty as u8) << 4 | (filter as u8);

        self.i2c.write(ADDRESS, &[reg::SENSOR_CONFIG, conf])?;
        Ok(())
    }

    pub fn device_revision(&mut self) -> Result<u8, Error<E>> {
        self.device_id_revision().map(|(_, rev)| rev)
    }

    // pub fn enabled(&mut self) -> Result<bool, Error<E>> {

    // }

    fn device_id_revision(&mut self) -> Result<(u8, u8), Error<E>> {
        let mut buf = [0u8; 2];
        self.i2c.write_read(ADDRESS, &[reg::DEVICE_ID], &mut buf)?;

        Ok((buf[0], buf[1]))
    }

    fn set_default_device_config(&mut self) -> Result<(), Error<E>> {
        let conf = 0b0_00_000_00; // 0.25 degree temperature resolution, 14-bit ADC resolution, 1 sample for burst mode, and normal power on.

        self.i2c.write(ADDRESS, &[reg::DEVICE_CONFIG, conf])?;

        let mut out_conf = 0;
        self.i2c.write_read(ADDRESS, &[reg::DEVICE_CONFIG], slice::from_mut(&mut out_conf))?;
        assert_eq!(conf, out_conf);

        Ok(())
    }
}
