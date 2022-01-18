extern crate libc;
extern crate socketcan;
extern crate websocket;
extern crate serde;
extern crate serde_json;
// extern crate warp;
// extern crate tokio;

use libc::{sched_yield, c_int, strerror};
use socketcan::{CANSocket, CANFrame, CANFilter, EFF_MASK};
use websocket::{Message, server::sync::Server, client::sync::Client, sender::Writer, receiver::Reader};
use websocket::message::OwnedMessage::{Text, Close};
use serde::{Serialize, Deserialize, Serializer};

// use warp::Filter;
// use futures_util::{FutureExt, StreamExt};
use std::{clone::Clone, time::{SystemTime, Duration}, ffi::CStr, thread, sync::{Arc, Mutex}, collections::LinkedList, convert::TryFrom};

extern
{
    fn set_deadline_scheduling(sched_runtime: u64, sched_deadline: u64, sched_period: u64) -> c_int;
    fn set_ioprio_highest() -> c_int;
}

enum ValveSequences
{
    TEST_SEQ_1 = 0,
    TEST_SEQ_2 = 1
}

// trait WebSocketMessage<DataType>: Serialize
// {
//     fn getMessageType(self) -> WebSocketMessageType;
//     fn getData(self) -> DataType;

//     fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
//     where
//         S: Serializer
//     {
//         serializer.serialize_none()
//     }
// }

#[derive(Serialize, Deserialize, Clone, Copy)]
struct SensorData
{
    engine_thermocouple: f64
}

#[derive(Serialize, Deserialize)]
#[serde(tag = "msg_type")]
enum OutgoingMessage
{
    SENSOR_DATA(SensorDataMessagePayload),
    FLIGHT_ABORTED(AbortCause)
}

#[derive(Serialize, Deserialize)]
struct SensorDataMessagePayload
{
    data: SensorData
}

#[derive(Serialize, Deserialize)]
#[serde(tag = "msg_type")]
enum IncomingMessage
{
    COMMAND(IncomingCommand)
}

#[derive(Serialize, Deserialize)]
#[serde(tag = "cmd_name")]
enum IncomingCommand
{
    ABORT(AbortCommandMessage)
}

#[derive(Serialize, Deserialize)]
struct AbortCommandMessage
{
    reason: String
}

#[derive(Serialize, Deserialize)]
#[serde(tag = "cause")]
enum AbortCause
{
    GROUND_TRIGGERED(GroundTriggeredAbortInfo),
    SENSOR_OUT_OF_RANGE(SensorOutOfRangeAbortInfo)
}

#[derive(Serialize, Deserialize)]
struct GroundTriggeredAbortInfo
{
    reason: String
}

#[derive(Serialize, Deserialize)]
struct SensorOutOfRangeAbortInfo
{
    sensor_num: u8,
    // measurement_timestamp: 
    value: f64
}

const DEV_ID: u32 = 5 << 8;
const SUBID_MASK: u32 = EFF_MASK & !0xff;

const UPDATE_INTERVAL: Duration = Duration::from_millis(10);
fn valve_periodic_update(currentSequenceLockRef: Arc<Mutex<ValveSequences>>, requestedSequenceLockRef: Arc<Mutex<ValveSequences>>,
    sensorDataRef: Arc<Mutex<SensorData>>)
{
    let UPDATE_INTERVAL_NANOS: u64 = u64::try_from(UPDATE_INTERVAL.as_nanos()).unwrap();

    unsafe
    {
        // TODO: Double-check these values
        set_deadline_scheduling(UPDATE_INTERVAL_NANOS, UPDATE_INTERVAL_NANOS, UPDATE_INTERVAL_NANOS);
    }

    loop
    {
        {
            // Valve sequencing state machine

            let mut current_sequence = currentSequenceLockRef.lock().unwrap();

            let new_sequence = match *current_sequence
            {
                ValveSequences::TEST_SEQ_1 => {
                    ValveSequences::TEST_SEQ_1
                },
                ValveSequences::TEST_SEQ_2 => {
                    ValveSequences::TEST_SEQ_2
                }
            }
        }
    }
}

// impl WebSocketMessage<SensorData> for SensorData
// {
//     fn getMessageType(self) -> WebSocketMessageType { WebSocketMessageType::TELEMETRY }
//     fn getData(self) -> SensorData { self }
// }

fn main() {
    let mut active_connections_mutex: Arc<Mutex<Vec<Writer<std::net::TcpStream>>>> = Arc::new(Mutex::from(Vec::new()));
    let sensor_data_mutex: Arc<Mutex<SensorData>> = Arc::new(Mutex::from(SensorData { engine_thermocouple: 0.0 }));

    let sensor_data_ref = Arc::clone(&sensor_data_mutex);
    let ws_thread_connections_mutex = Arc::clone(&active_connections_mutex);
    let websocket_thread_handle = thread::spawn(move || {
        let test_srv = Server::bind("0.0.0.0:8000").expect("Failed to bind server to local port");

        let update_sender_ref = Arc::clone(&ws_thread_connections_mutex);
        let update_sender_thread = thread::spawn(move || send_periodic_updates(update_sender_ref, sensor_data_ref));

        for srv_connection in test_srv
        {
            let mut client_connection = srv_connection.expect("Failed to accept connection")
                .accept().expect("Failed to perform acceptance handshake");
            client_connection.send_message(&Message::text("You will receive periodic updates."))
                .expect("Failed to send message");

            let mut active_connections = ws_thread_connections_mutex.lock().unwrap();
            let (rx, tx) = client_connection.split().unwrap();
            active_connections.push(tx);
            let receiver_conn_ref = Arc::clone(&ws_thread_connections_mutex);
            thread::spawn(move || receive_commands(rx, receiver_conn_ref));
        }

        update_sender_thread.join().unwrap();
    });

    // unsafe
    // {
    //     let result = set_deadline_scheduling(500_000, 1_000_000, 1_000_000);
    //     if(result != 0)
    //     {
    //         println!("ERROR: Could not set deadline scheduling: {}", CStr::from_ptr(strerror(result)).to_str().expect("Could not convert error message"));
    //         panic!();
    //     }
    // }

    unsafe
    {
        let result = set_ioprio_highest();
        if(result != 0)
        {
            println!("ERROR: Could not set I/O priority: {}", CStr::from_ptr(strerror(result)).to_str().expect("Could not convert error message"));
            panic!();
        }
    }

    let socket: CANSocket = CANSocket::open("vcan0").expect("Failed to open CAN socket.");
    let filter = CANFilter::new(DEV_ID, SUBID_MASK).expect("Failed to construct CAN filter.");
    socket.set_filter(&[filter]).expect("Failed to set CAN filter.");


    loop
    {
        let frame: CANFrame = socket.read_frame().expect("Failed to read a CAN frame");
        println!("{}", format!("Received CAN frame: {:X}", frame));

        let sensor_num: u8 = u8::try_from(frame.id() % (1 << 8)).unwrap();
        let mut owned_frame_data: [u8; 8] = [0; 8];
        owned_frame_data.clone_from_slice(frame.data());
        let sensor_data_double: f64 = f64::from_be_bytes(owned_frame_data);
        println!("Frame data: Sensor {} value is {}.", sensor_num, sensor_data_double);
        
        let mut sensor_data = sensor_data_mutex.lock().unwrap();
        match sensor_num
        {
            3 => { 
                sensor_data.engine_thermocouple = sensor_data_double;
                if sensor_data.engine_thermocouple > 0.99
                {
                    eprintln!("FATAL: Out of range reading for engine thermocouple: {}", sensor_data.engine_thermocouple);
                    // TODO: Use proper abort handler
                    let abort_msg = OutgoingMessage::FLIGHT_ABORTED(AbortCause::SENSOR_OUT_OF_RANGE(
                        SensorOutOfRangeAbortInfo { sensor_num: 3, value: sensor_data.engine_thermocouple }));
                    send_broadcast(&mut active_connections_mutex, &Message::text(serde_json::to_string(&abort_msg).unwrap()));
                    std::process::exit(1);
                }
            }
            other => {
                eprintln!("WARNING: Invalid sensor number {}", other);
                continue;
            }
        }
        // let last_time = SystemTime::now();
        // /* sleep_ms(10); */ yield_now();
        // let time_elapsed = last_time.elapsed();

        // println!("Yielded for {} us", time_elapsed.expect("The system time changed.").as_micros());
    }

    websocket_thread_handle.join().unwrap();
}

fn receive_commands(mut ws_input: Reader<std::net::TcpStream>, mut active_connections_mutex: Arc<Mutex<Vec<Writer<std::net::TcpStream>>>>)
{
    loop
    {
        match ws_input.recv_message()
        {
            Ok(msg) => {
                match msg
                {
                    Text(text_data) => {
                        match serde_json::from_str::<IncomingMessage>(&text_data)
                        {
                            Ok(parsed_msg) => {
                                handle_message(parsed_msg, &mut active_connections_mutex);
                            },
                            Err(info) => {
                                println!("WARNING: Could not parse incoming message: {:?}", info);
                            }
                        }
                    },
                    Close(_) => {
                        println!("INFO: Connection was closed");
                        break;
                    },
                    other => {
                        eprintln!("WARNING: Ignoring message of unknown type: {:?}", other);
                    }
                }
            },
            Err(info) => {
                println!("INFO: Stopped receiving input due to error: {:?}", info);
                break;
            }
        }
    }
}

fn handle_message(message: IncomingMessage, active_connections_mutex: &mut Arc<Mutex<Vec<Writer<std::net::TcpStream>>>>)
{
    match message
    {
        IncomingMessage::COMMAND(cmd) => {
            match cmd
            {
                IncomingCommand::ABORT(abort_msg) => {
                    // TODO: Implement a proper abort handler here
                    eprintln!("FATAL: Abort was triggered from ground station for the following reason: {}", abort_msg.reason);
                    let abort_out_msg = OutgoingMessage::FLIGHT_ABORTED(AbortCause::GROUND_TRIGGERED(GroundTriggeredAbortInfo { reason: abort_msg.reason }));
                    send_broadcast(active_connections_mutex, &Message::text(serde_json::to_string(&abort_out_msg).unwrap()));
                    std::process::exit(1);
                }
            }
        }
    }
}

fn send_broadcast(active_connections_mutex: &mut Arc<Mutex<Vec<Writer<std::net::TcpStream>>>>, message: &Message)
{
    // TODO: Find a more efficient and less clunky way to store and remove connections
    // (HashSet/BTreeSet doesn't work because connections aren't comparable or
    // hashable, and the interface for constant-time edits to linked lists isn't
    // stable yet).
    let mut to_remove: Vec<usize> = Vec::new();
    let mut active_connections = active_connections_mutex.lock().unwrap();
    for this_connection in 0..(&*active_connections).len()
    {
        let send_result = active_connections[this_connection].send_message(message);
        
        if send_result.is_err()
        {
            println!("INFO: Will remove connection {} due to error: {:?}", this_connection, send_result.unwrap_err());
            to_remove.push(this_connection);
        }
    }

    for i in to_remove
    {
        active_connections.remove(i);
    }
}

fn send_periodic_updates(mut active_connections_mutex: Arc<Mutex<Vec<Writer<std::net::TcpStream>>>>,
                         sensor_data_mutex: Arc<Mutex<SensorData>>)
{
    unsafe
    {
        let result = set_deadline_scheduling(500_000_000, 1_000_000_000, 1_000_000_000);
        if(result != 0)
        {
            println!("ERROR: Could not set deadline scheduling: {}", CStr::from_ptr(strerror(result)).to_str().expect("Could not convert error message"));
            panic!();
        }
    }

    loop
    {
        {
            let sensor_data = sensor_data_mutex.lock().unwrap();

            let msg_obj = OutgoingMessage::SENSOR_DATA(SensorDataMessagePayload { data: *sensor_data });
            let message = Message::text(serde_json::to_string(&msg_obj).unwrap());
            send_broadcast(&mut active_connections_mutex, &message);
        }

        unsafe
        {
            sched_yield();
        }
    }
}

// #[tokio::main]
// async fn websocket_server()
// {
//     let routes = warp::path("/").and(warp::ws()).map(|new_conn: warp::ws::Ws| {
//         return new_conn.on_upgrade(create_conn_task);
//     });

//     warp::serve(routes).run(([127, 0, 0, 1], 3030)).await;
// }

// async fn create_conn_task(conn: warp::ws::WebSocket)
// {
//     tokio::task::spawn(async move { handle_connection(conn) });
// }

// async fn handle_connection(conn: warp::ws::WebSocket)
// {
//     let (tx, rx) = conn.split();
    
// }