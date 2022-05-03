use std::{rc::Rc, time::Duration, io::{Error, ErrorKind}};

use anyhow::{Result, Context};
use shared::{Id, pi_sensor, pi_output};
use socketcan::{CanSocket, CanFrame};

pub enum SensorDataKind {
    Pressure1(pi_sensor::Pressure1),
    Pressure2(pi_sensor::Pressure2),
    Flow(pi_sensor::Flow),
    Thermo1(pi_sensor::Thermo1),
    Thermo2(pi_sensor::Thermo2),
    Load(pi_sensor::Load),
}

pub enum CanMessage {
    OutputStatus(pi_output::Status),
    SensorData(SensorDataKind),
}


#[derive(Clone)]
pub struct CanBus {
    socket: Rc<CanSocket>,
}

impl CanBus {
    pub fn setup() -> Result<Self> {
        let socket = CanSocket::open("can0")?;
        socket.set_read_timeout(Duration::from_micros(3_000))?;
        socket.set_write_timeout(Duration::from_micros(1_0000))?;

        Ok(Self {
            socket: Rc::new(socket),
        })
    }

    pub fn clear(&self) -> Result<()> {
        self.socket.set_nonblocking(true)?;
        while let Ok(_) = self.socket.read_frame() {}
        self.socket.set_nonblocking(false)?;

        Ok(())
    }

    pub fn send(&self, to: Id, item: impl serde::Serialize) -> Result<()> {
        let data  = postcard::to_stdvec(&item)?;
    
        let frame = CanFrame::new(
            to as u32,
            &data,
            false,
            false,
        )?;
    
        self.socket.write_frame_insist(&frame)
            .context("failed to write to CAN socket")?;
    
        Ok(())
    }

    pub fn receive_msg(&self) -> Result<Option<CanMessage>> {
        self.socket.set_nonblocking(true)?;
        
        let res = match self.socket.read_frame() {
            Ok(frame) => {
                let data = frame.data();
                let msg = match frame.id() {
                    0b00_000 => {
                        // Id::RaspiOutputStatus
                        let status = postcard::from_bytes(&data)?;
                        CanMessage::OutputStatus(status)
                    }
                    0b00_001 => {
                        // Id::RaspiPressure1
                        let pressure1 = postcard::from_bytes(&data)?;
                        CanMessage::SensorData(SensorDataKind::Pressure1(pressure1))
                    }
                    0b00_010 => {
                        // Id::RaspiPressure2
                        let pressure2 = postcard::from_bytes(&data)?;
                        CanMessage::SensorData(SensorDataKind::Pressure2(pressure2))
                    }
                    0b00_011 => {
                        // Id::RaspiFlow
                        let flow = postcard::from_bytes(&data)?;
                        CanMessage::SensorData(SensorDataKind::Flow(flow))
                    }
                    0b00_100 => {
                        // Id::RaspiThermo1
                        let thermo1 = postcard::from_bytes(&data)?;
                        CanMessage::SensorData(SensorDataKind::Thermo1(thermo1))
                    }
                    0b00_101 => {
                        // Id::RaspiThermo2
                        let thermo2 = postcard::from_bytes(&data)?;
                        CanMessage::SensorData(SensorDataKind::Thermo2(thermo2))
                    }
                    0b00_110 => {
                        // Id::RaspiLoad
                        let load = postcard::from_bytes(&data)?;
                        CanMessage::SensorData(SensorDataKind::Load(load))
                    }
                    _ => Err(Error::new(ErrorKind::Other, format!("unexpected CAN ID: {:0b}", frame.id())))?,
                };
                Ok(Some(msg))
            },
            Err(ref e) if e.kind() == ErrorKind::WouldBlock => Ok(None),
            Err(e) => Err(e)?,
        };
        
        self.socket.set_nonblocking(false)?;
        res
    }
    
    pub fn receive<T: serde::de::DeserializeOwned>(&self) -> Result<T> {
        let frame = self.socket.read_frame()
            .context("failed to read CAN frame")?;
        let data = postcard::from_bytes(&frame.data())?;
    
        Ok(data)
    }
}
