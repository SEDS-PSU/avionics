use std::{rc::Rc, time::Duration};

use crate::Result;
use shared::Id;
use socketcan::{CanSocket, CanFrame};


#[derive(Clone)]
pub struct CanBus {
    socket: Rc<CanSocket>,
}

impl CanBus {
    pub fn setup() -> Result<Self> {
        let socket = CanSocket::open("can0")?;
        socket.set_read_timeout(Duration::from_micros(1_000))?;
        socket.set_write_timeout(Duration::from_micros(1_0000))?;

        Ok(Self {
            socket: Rc::new(socket),
        })
    }

    pub fn send(&self, to: Id, item: impl serde::Serialize) -> Result<()> {
        let data  = postcard::to_stdvec(&item)?;
    
        let frame = CanFrame::new(
            to as u32,
            &data,
            false,
            false,
        )?;
    
        self.socket.write_frame(&frame)?;
    
        Ok(())
    }
    
    pub fn receive<T: serde::de::DeserializeOwned>(&self) -> Result<T> {
        let frame = self.socket.read_frame()?;
        let data = postcard::from_bytes(&frame.data())?;
    
        Ok(data)
    }
}
