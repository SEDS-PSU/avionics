use shared::{Id, pi_sensor::{Request, ResponseHeader, AllSensors}};

use crate::{Result, can::CanBus};

pub struct SensorBoard {
    can_bus: CanBus,
}

impl SensorBoard {
    pub fn connect(can_bus: CanBus) -> Result<Self> {
        let this = Self {
            can_bus,
        };

        if let Err(e) = this.reset() {
            eprintln!("failed to connect to sensor board");
            return Err(e);
        }

        Ok(this)
    }

    pub fn reset(&self) -> Result<()> {
        self.can_bus.send(Id::SensorBoard, Request::Reset)?;
        Ok(())
    }

    pub fn get_sensor_data(&self) -> Result<AllSensors> {
        self.can_bus.send(Id::SensorBoard, Request::GetSensorData)?;

        let _header: ResponseHeader = self.can_bus.receive()?;

        let buf = self.can_bus.recieve_buf::<{8 * 5}>()?;
        let sensors = postcard::from_bytes(&buf)?;

        Ok(sensors)
    }
}