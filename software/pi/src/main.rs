// mod ground_station;

use std::{error::Error, time::Duration, thread};

// use ground_station::GroundStation;
use socketcan::{CanSocket, CanFrame};
use shared::pi_sensor;
use thread_priority::ThreadPriority;

fn main() -> Result<(), Box<dyn Error>> {
    let core_ids = core_affinity::get_core_ids().expect("failed to register CPU cores");
    let (rt_cpu, _soft_cpu) = if let &[rt_cpu, soft_cpu] = core_ids.as_slice() {
        (rt_cpu, soft_cpu)
    } else {
        panic!("not enough CPU cores?")
    };

    // let ground_station = GroundStation::connect(soft_cpu)?;

    core_affinity::set_for_current(rt_cpu);
    thread_priority::set_current_thread_priority(ThreadPriority::Deadline {
        runtime: Duration::from_millis(7),
        deadline: Duration::from_millis(9),
        period: Duration::from_millis(10),
    }).unwrap();

    let can = CanSocket::open("can0")?;

    let (header, data) = request_sensor_data(&can, pi_sensor::Request::GetSensorData).unwrap();
        println!("{:#?}", header);
        println!("{:#?}", data);

    loop {
        // TODO: Do stuff with CAN bus, etc

        // Yield until the next period.
        thread::yield_now();
    }
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
    request: pi_sensor::Request,
) -> Result<(pi_sensor::ResponseHeader, pi_sensor::AllSensors), Box<dyn Error>> {
    let frame = sensor_request_to_frame(request)?;

    can.write_frame_insist(&frame)?;

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