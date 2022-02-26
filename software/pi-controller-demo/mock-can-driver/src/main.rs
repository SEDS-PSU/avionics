// extern crate rtnetlink;
extern crate socketcan;
extern crate shared;
extern crate postcard;

// use rtnetlink::LinkAddRequest;
use std::{process::{Command, ExitStatus}, thread::sleep, time::{Duration, SystemTime}};
use socketcan::{CANSocket, CANFrame, CANFilter, EFF_MASK};
use shared::{RASPI_ID, OUTPUT_BOARD_ID, pi_output};

const SUBID_MASK: u32 = EFF_MASK & !0xff;

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
            std::thread::spawn(run_mock_output_board);
            run_mock_sensor_board();
        },
        Err(_) => {
            eprintln!("ERROR: Failed to bring virtual CAN interface up");
        }
    }
}

fn run_mock_sensor_board()
{
    let socket: CANSocket = CANSocket::open("vcan0").expect("Failed to open CAN socket.");
    let start_time = SystemTime::now();
    loop
    {
        // TODO: Send CAN messages for testing
        let elapsed_time: Duration = start_time.elapsed().unwrap();

        let mock_sensor_3_val = f64::sin(elapsed_time.as_secs_f64());
        socket.write_frame_insist(
            &CANFrame::new((u32::from(RASPI_ID) << 8) + 3, &mock_sensor_3_val.to_be_bytes(), false, false).unwrap()
        ).expect("Error sending CAN frame");
        println!("Wrote mock value: {}", mock_sensor_3_val);

        sleep(Duration::from_millis(300));
    }
}

fn run_mock_output_board()
{
    let socket: CANSocket = CANSocket::open("vcan0").expect("Failed to open CAN socket.");
    let filter = CANFilter::new(u32::from(OUTPUT_BOARD_ID) << 8, SUBID_MASK).expect("Failed to construct CAN filter.");
    socket.set_filter(&[filter]).expect("Failed to set CAN filter.");

    let mut remaining_sequence_frames: u8 = 0;

    loop
    {
        let frame = socket.read_frame().unwrap();

        if remaining_sequence_frames == 0
        {
            let parsed_command: pi_output::Request = postcard::from_bytes(frame.data()).unwrap();

            match parsed_command
            {
                pi_output::Request::GetStatus => {
                    // TODO
                },
                pi_output::Request::BeginSequence { length } => {
                    remaining_sequence_frames = length;
                    println!("DEBUG: Expecting valve sequence with {} frames.", remaining_sequence_frames);
                },
                pi_output::Request::Reset => {
                    // TODO
                },
                pi_output::Request::SetValvesImmediately(valves) => {

                }
            }
        }
        else
        {
            remaining_sequence_frames -= 1;
            let parsed_command: pi_output::Command = postcard::from_bytes(frame.data()).unwrap();

            println!("Received valve sequence command: {:?}", parsed_command);
        }
    }
}