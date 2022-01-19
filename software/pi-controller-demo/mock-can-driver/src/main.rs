// extern crate rtnetlink;
extern crate socketcan;

// use rtnetlink::LinkAddRequest;
use std::{process::{Command, ExitStatus}, thread::sleep, time::{Duration, SystemTime}};
use socketcan::{CANSocket, CANFrame};

fn exit_success(result: ExitStatus) -> Result<(), ExitStatus>
{
    if result.code().is_some() && result.code().unwrap() == 0
    {
        return Ok(())
    }
    else
    {
        return Err(result)
    }
}

fn main()
{
    exit_success(Command::new("modprobe").arg("vcan").status().unwrap()).expect("ERROR: Failed to load vcan kernel module");


    // let (connection, handle, _) = rtnetlink::new_connection().unwrap();
    // let msg = handle.link().add().message_mut();
    // msg.header.interface_family =

    let link_add_result = Command::new("ip").args(["link", "add", "dev", "vcan0", "type", "vcan"]).output().unwrap();
    // let link_add_result = exit_success(Command::new("ip").args(["link", "add", "dev", "vcan0", "type", "vcan"]).status().unwrap());

    if !link_add_result.status.success()
    {
        let parsed_output = std::str::from_utf8(&link_add_result.stderr);

        if parsed_output.is_err() || parsed_output.unwrap().find("File exists").is_none()
        {
            panic!("ERROR: Failed to create virtual CAN interface");
        }
    }

    match exit_success(Command::new("ip").args(["link", "set", "up", "vcan0"]).status().unwrap())
    {
        Ok(_) => {
            let socket: CANSocket = CANSocket::open("vcan0").expect("Failed to open CAN socket.");
            let start_time = SystemTime::now();
            loop {
                // TODO: Send CAN messages for testing
                let elapsed_time: Duration = start_time.elapsed().unwrap();

                let mock_sensor_3_val = f64::sin(elapsed_time.as_secs_f64());
                socket.write_frame_insist(
                    &CANFrame::new((5 << 8) + 3, &mock_sensor_3_val.to_be_bytes(), false, false).unwrap()
                ).expect("Error sending CAN frame");
                println!("Wrote mock value: {}", mock_sensor_3_val);

                sleep(Duration::from_millis(300));
            }
        },
        Err(_) => {
            eprintln!("ERROR: Failed to bring virtual CAN interface up");
        }
    }
}