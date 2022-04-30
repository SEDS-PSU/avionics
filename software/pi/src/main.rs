mod ground_station;
mod can;
mod boards;

use std::{time::Duration, fs::OpenOptions, io::Write, thread};

use anyhow::{Result, Context, Ok};
use ground_station::GroundStation;
use thread_priority::{ThreadPriority, ThreadSchedulePolicy, RealtimeThreadSchedulePolicy};

use crate::{can::CanBus, boards::{OutputBoard, SensorBoard}};

fn run() -> Result<()> {
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

    let ground_station = GroundStation::connect()?;

    loop {
        let sensor_data = sensor_board.get_sensor_data()?;
        ground_station.send_sensor_data(sensor_data);

        if let Some(cmds) = ground_station.read_new_commands() {
            output_board.execute_commands(&cmds)?;
        }

        // Yield until the next period.
        thread::yield_now();
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
