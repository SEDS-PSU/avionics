use std::{thread, time::Duration};

use shared::{Id, pi_sensor::Request};

use crate::can::CanBus;
use anyhow::{Result, Context};

pub struct SensorBoard {
    can_bus: CanBus,
}

impl SensorBoard {
    pub fn connect(can_bus: CanBus) -> Result<Self> {
        let this = Self {
            can_bus,
        };

        this.reset().context("failed to connect to sensor board")?;
        
        thread::sleep(Duration::from_millis(100));

        Ok(this)
    }

    pub fn reset(&self) -> Result<()> {
        self.can_bus.send(Id::SensorBoard, Request::Reset)?;
        Ok(())
    }

    pub fn start_sensing(&self) -> Result<()> {
        self.can_bus.send(Id::SensorBoard, Request::StartSensing)?;
        Ok(())
    }

    // pub fn get_sensor_data(&self) -> Result<AllSensors> {
    //     self.can_bus.send(Id::SensorBoard, Request::GetSensorData)?;

    //     let buf = self.can_bus.recieve_buf::<{8 * 5}>()?;
    //     let sensors = postcard::from_bytes(&buf)?;

    //     Ok(sensors)
    // }
}