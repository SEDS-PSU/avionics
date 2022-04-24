use shared::{pi_output::{Status, Request, Command}, Id};

use crate::{can::CanBus, Result};

pub struct OutputBoard {
    can_bus: CanBus,
}

impl OutputBoard {
    pub fn connect(can_bus: CanBus) -> Result<Self> {
        let this = Self {
            can_bus,
        };

        if let Err(e) = this.reset() {
            eprintln!("failed to connect to output board");
            return Err(e);
        }

        Ok(this)
    }

    pub fn reset(&self) -> Result<()> {
        self.can_bus.send(Id::OutputBoard, Request::Reset)?;
        Ok(())
    }

    pub fn get_status(&self) -> Result<Status> {
        self.can_bus.send(Id::OutputBoard, Request::GetStatus)?;
        self.can_bus.receive()
    }

    pub fn execute_commands(&self, cmds: &[Command]) -> Result<()> {
        assert!(cmds.len() <= 256);

        self.can_bus.send(Id::OutputBoard, Request::BeginSequence { length: (cmds.len() as u8).try_into().unwrap() })?;
        for cmd in cmds {
            self.can_bus.send(Id::OutputBoard, cmd)?;
        }

        Ok(())
    }
}