mod ground_station;
mod can;
mod boards;

use std::{time::Duration, fs::OpenOptions, io::Write, thread};

use anyhow::{Result, Context, Ok};
use shared::pi_output::{Command, Igniter};
// use ground_station::GroundStation;
use thread_priority::{ThreadPriority, ThreadSchedulePolicy, RealtimeThreadSchedulePolicy};

use crate::{can::CanBus, boards::{OutputBoard, SensorBoard}};

fn run() -> Result<()> {
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
    thread::sleep(Duration::from_millis(200));

    can_bus.clear().context("failed to clear CAN buffer")?;

    let output_board = OutputBoard::connect(can_bus.clone())?;
    let sensor_board = SensorBoard::connect(can_bus)?;

    let status = output_board.get_status()?;

    println!("output board status: {:#?}", status);

    // output_board.execute_commands(&[
    //     Command::Igniter(Igniter::Activate),
    //     Command::Wait(2_000.try_into().unwrap()),
    //     Command::Igniter(Igniter::Deactivate),
    // ])?;

    // loop {
    //     // TODO: Do stuff with CAN bus, etc

    //     // Yield until the next period.
    //     thread::yield_now();
    // }

    loop {
        thread::sleep(Duration::from_millis(1000));

        let sensor_data = sensor_board.get_sensor_data()?;
        println!("{:#?}", sensor_data);
    }
}

fn write_crash_log(msg: &str) -> Result<()> {
    let mut f = OpenOptions::new()
        .append(true)
        .create(true)
        .open("crashlog.txt")?;

    let s = format!("{}\n\n{:?}\n", "-".repeat(40), msg);
    f.write_all(s.as_bytes())?;
    
    Ok(())
}

fn main() -> Result<()> {
    loop {
        println!("Starting...");
        if let Err(e) = run() {
            eprintln!("{:?}", e);
            if let Err(e) = write_crash_log(&format!("{:?}", e)) {
                eprintln!("failed to write crash log: {}", e);
            }
            return Ok(())
        } else {
            return Ok(())
        }
    }
}
