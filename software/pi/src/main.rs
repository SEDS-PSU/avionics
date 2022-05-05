mod ground_station;
mod can;
mod boards;

use std::{time::{Duration, Instant}, fs::OpenOptions, io::Write, thread};

use anyhow::{Result, Context};
use ground_station::GroundStation;
use thread_priority::{ThreadPriority, ThreadSchedulePolicy, RealtimeThreadSchedulePolicy};

use crate::{can::{CanBus, CanMessage}, boards::{OutputBoard, SensorBoard}};

fn run() -> Result<()> {
    let can_bus = CanBus::setup()?;
    thread::sleep(Duration::from_millis(200));

    can_bus.clear().context("failed to clear CAN buffer")?;

    let ground_station = GroundStation::connect()?;
    let output_board = OutputBoard::connect(can_bus.clone())?;
    let sensor_board = SensorBoard::connect(can_bus.clone())?;

    println!("output board status: {:#?}", output_board.get_status()?);

    thread_priority::set_thread_priority_and_policy(
        0 as _, // current therad
        ThreadPriority::Deadline {
            runtime: Duration::from_millis(7),
            deadline: Duration::from_millis(9),
            period: Duration::from_millis(10),
        },
        ThreadSchedulePolicy::Realtime(RealtimeThreadSchedulePolicy::Deadline),
    ).unwrap();

    sensor_board.start_sensing().context("failed to start sensing")?;

    let mut last_sensor_data = Instant::now();
    let mut previous_keep_alive = Instant::now();

    loop {
        if previous_keep_alive.elapsed() > Duration::from_millis(500) {
            sensor_board.start_sensing().context("failed to resume sensing")?;
            previous_keep_alive = Instant::now();
        }

        loop {
            if let Some(cmds) = ground_station.read_new_commands() {
                println!("recieved new commands from ground station");
                output_board.execute_commands(&cmds)?;
            }
            
            if let Some(msg) = can_bus.receive_msg()? {
                last_sensor_data = Instant::now();
                match msg {
                    CanMessage::OutputStatus(_status) => {},
                    CanMessage::SensorData(data) => {
                        ground_station.send_sensor_data(data, last_sensor_data);
                    }
                }
            } else {
                break
            }
        }

        let elapsed = last_sensor_data.elapsed();

        if elapsed > Duration::from_millis(20) {
            println!("haven't received sensor data in {:?}", elapsed);
        }

        // match sensor_board.get_sensor_data() {
        //     Ok(sensor_data) => {
        //         ground_station.send_sensor_data(sensor_data);
        //     }
        //     Err(e) => {
        //         eprintln!("failed to read sensor data, attempting to reset the board and clear the buffers: {:?}", e);
        //         let _ = sensor_board.reset();
        //         let _ = can_bus.clear();
        //     }
        // }

        // if let Some(cmds) = ground_station.read_new_commands() {
        //     output_board.execute_commands(&cmds)?;
        // } else {
        //     if let Err(e) = output_board.get_status() {
        //         eprintln!("failed to read output board status: {:?}", e);
        //     }
        // }

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
