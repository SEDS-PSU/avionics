// mod ground_station;

mod can;
mod boards;

use std::{error::Error, time::Duration};

use shared::pi_output::{Command, Igniter};
// use ground_station::GroundStation;
use thread_priority::{ThreadPriority, ThreadSchedulePolicy, RealtimeThreadSchedulePolicy};

use crate::{can::CanBus, boards::OutputBoard};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

fn main() -> Result<()> {
    // let ground_station = GroundStation::connect(soft_cpu)?;

    thread_priority::set_thread_priority_and_policy(
        0 as _, // current therad
        ThreadPriority::Deadline {
            runtime: Duration::from_millis(7),
            deadline: Duration::from_millis(9),
            period: Duration::from_millis(10),
        },
        ThreadSchedulePolicy::Realtime(RealtimeThreadSchedulePolicy::Deadline),
    ).unwrap();

    let can_bus = CanBus::setup()?;
    let output_board = OutputBoard::connect(can_bus)?;

    let status = output_board.get_status()?;

    println!("output board status: {:#?}", status);

    output_board.execute_commands(&[
        Command::Igniter(Igniter::Activate),
        Command::Wait(2_000.try_into().unwrap()),
        Command::Igniter(Igniter::Deactivate),
    ])?;

    // loop {
    //     // TODO: Do stuff with CAN bus, etc

    //     // Yield until the next period.
    //     thread::yield_now();
    // }

    Ok(())
}
