
use std::error::Error;

use socketcan::{CanSocket, CanFrame};
use shared::pi_sensor;

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

fn main() -> Result<(), Box<dyn Error>> {
    let can = CanSocket::open("can0")?;

    let frame = sensor_request_to_frame(pi_sensor::Request::GetSensorData)?;
    
    can.write_frame_insist(&frame)?;

    let header_frame = can.read_frame()?;
    let header: pi_sensor::ResponseHeader = postcard::from_bytes(header_frame.data())?;

    println!("{:#?}", header);

    let mut v = vec![];
    for _ in 0..5 {
        let frame = can.read_frame()?;
        v.extend_from_slice(frame.data());
    }

    let sensor_data: pi_sensor::AllSensors = postcard::from_bytes(&v)?;

    println!("{:#?}", sensor_data);

    Ok(())
}
