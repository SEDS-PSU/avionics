// mod ground_station;

use std::{error::Error, time::Duration, thread};

// use ground_station::GroundStation;
use socketcan::{CanSocket, CanFrame};
use shared::{pi_sensor, pi_output};
use thread_priority::{ThreadPriority, ThreadSchedulePolicy, RealtimeThreadSchedulePolicy};

fn main() -> Result<(), Box<dyn Error>> {
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

    let can = CanSocket::open("can0")?;
    can.set_read_timeout(Duration::from_micros(10_000))?;
    can.set_write_timeout(Duration::from_micros(10_0000))?;

    // let (header, data) = request_sensor_data(&can).unwrap();
    // println!("{:#?}", header);
    // println!("{:#?}", data);

    let status = request_output_status(&can)?;
    println!("{:#?}", status);

    loop {
        // TODO: Do stuff with CAN bus, etc

        // Yield until the next period.
        thread::yield_now();
    }
}

fn output_request_to_frame(
    request: pi_output::Request,
) -> Result<CanFrame, Box<dyn Error>> {
    let data  = postcard::to_stdvec(&request)?;

    let frame = CanFrame::new(
        shared::OUTPUT_BOARD_ID as _,
        &data,
        false,
        false,
    )?;

    Ok(frame)
}

fn request_output_status(
    can: &CanSocket,
) -> Result<pi_output::Status, Box<dyn Error>> {
    let frame = output_request_to_frame(pi_output::Request::GetStatus)?;

    can.write_frame(&frame)?;

    let status_frame = can.read_frame()?;
    let status: pi_output::Status = postcard::from_bytes(&status_frame.data())?;

    Ok(status)
}

fn sensor_request_to_frame(request: pi_sensor::Request) -> Result<CanFrame, Box<dyn Error>> {
    let data = postcard::to_stdvec(&request)?;

    let frame = CanFrame::new(
        shared::SENSOR_BOARD_ID as _,
        &data,
        false,
        false,
    )?;

    Ok(frame)
}

fn request_sensor_data(
    can: &CanSocket,
) -> Result<(pi_sensor::ResponseHeader, pi_sensor::AllSensors), Box<dyn Error>> {
    let frame = sensor_request_to_frame(pi_sensor::Request::GetSensorData)?;

    can.write_frame(&frame)?;

    let header_frame = can.read_frame()?;
    let header: pi_sensor::ResponseHeader = postcard::from_bytes(header_frame.data())?;

    let mut v = vec![];
    for _ in 0..5 {
        let frame = can.read_frame()?;
        v.extend_from_slice(frame.data());
    }

    let data: pi_sensor::AllSensors = postcard::from_bytes(&v)?;

    Ok((header, data))
}

// fn main() -> Result<(), Box<dyn Error>> {
//     let can = CanSocket::open("can0")?;

//     let frame = sensor_request_to_frame(pi_sensor::Request::GetSensorData)?;
    
//     can.write_frame_insist(&frame)?;

//     let header_frame = can.read_frame()?;
//     let header: pi_sensor::ResponseHeader = postcard::from_bytes(header_frame.data())?;

//     println!("{:#?}", header);

//     let mut v = vec![];
//     for _ in 0..5 {
//         let frame = can.read_frame()?;
//         v.extend_from_slice(frame.data());
//     }

//     let sensor_data: pi_sensor::AllSensors = postcard::from_bytes(&v)?;

//     println!("{:#?}", sensor_data);

//     Ok(())
// }
