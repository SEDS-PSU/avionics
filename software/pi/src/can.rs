use std::{rc::Rc, time::Duration};

use anyhow::Result;
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
    
        self.socket.write_frame_insist(&frame)?;
    
        Ok(())
    }
    
    pub fn receive<T: serde::de::DeserializeOwned>(&self) -> Result<T> {
        let frame = self.socket.read_frame()?;
        let data = postcard::from_bytes(&frame.data())?;
    
        Ok(data)
    }

    pub fn recieve_buf<const N: usize>(&self) -> Result<[u8; N]> {
        let mut buf = [0u8; N];
        let mut idx = 0;
        
        while idx < buf.len() {
            let frame = self.socket.read_frame()?;
            let data = frame.data();
            buf[idx..][..data.len()].copy_from_slice(data);
            idx += data.len();
        }

        Ok(buf)
    }
}
