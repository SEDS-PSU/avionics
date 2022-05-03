use std::{thread::{self, JoinHandle}, time::{Instant, Duration}, sync::Arc};
use futures_util::{StreamExt, try_join, SinkExt, Sink, Stream};
use shared::{pi_sensor, pi_output};
use tokio::{net::{TcpListener, TcpStream}, task, runtime};
use tungstenite::Message;
use anyhow::{Result, Context};
use serde::{Serialize, Deserialize};
use atomicring::AtomicRingBuffer;

use crate::can::SensorDataKind;

#[derive(Debug, Deserialize, Serialize)]
enum TwoWayState {
    #[serde(rename = "closed")]
    Closed,
    #[serde(rename = "open")]
    Open,
}

#[derive(Debug, Deserialize, Serialize)]
enum ThreeWayState {
    #[serde(rename = "nitrogen")]
    NitrogenPathway,
    #[serde(rename = "fueloxidizer")]
    FuelOxidizer,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct Valves {
    #[serde(rename = "FO_FP")]
    fo_fp: TwoWayState,
    #[serde(rename = "FC_FP")]
    fc_fp: TwoWayState,
    #[serde(rename = "FC_P")]
    fc_p: TwoWayState,
    #[serde(rename = "FC1_F")]
    fc1_f: TwoWayState,
    #[serde(rename = "FO_P1")]
    fo_p1: TwoWayState,
    #[serde(rename = "FO2_O")]
    fo2_o: TwoWayState,
    #[serde(rename = "FC4_O")]
    fc4_o: TwoWayState,
    #[serde(rename = "FC3_O")]
    fc3_o: TwoWayState,
    #[serde(rename = "PV_F")]
    pv_f: ThreeWayState,
    #[serde(rename = "PV_O")]
    pv_o: ThreeWayState,
}

#[derive(Debug, Deserialize, Serialize)]
enum Command {
    SetValves(Valves),
    HardAbort,
    SoftAbort,
    Ignite,
}

#[derive(Debug, Default, Deserialize, Serialize)]
struct SensorData {
    #[serde(rename = "TIMESTAMP")]
    timestamp: u32,
    #[serde(rename = "PT1_F")]
    pt1_f: Option<f32>,
    #[serde(rename = "PT2_F")]
    pt2_f: Option<f32>,
    #[serde(rename = "PT2_O")]
    pt2_o: Option<f32>,
    #[serde(rename = "PT3_O")]
    pt3_o: Option<f32>,
    #[serde(rename = "PT4_O")]
    pt4_o: Option<f32>,
    #[serde(rename = "PT2_P")]
    pt2_p: Option<f32>,
    #[serde(rename = "PT1_E")]
    pt1_e: Option<f32>,
    #[serde(rename = "TC1_F")]
    tc1_f: Option<f32>,
    #[serde(rename = "TC2_F")]
    tc2_f: Option<f32>,
    #[serde(rename = "TC1_O")]
    tc1_o: Option<f32>,
    #[serde(rename = "TC5_O")]
    tc5_o: Option<f32>,
    #[serde(rename = "TC1_E")]
    tc1_e: Option<f32>,
    #[serde(rename = "FM_F")]
    fm_f: Option<f32>,
    #[serde(rename = "FM_O")]
    fm_o: Option<f32>,
    #[serde(rename = "ThrustLoadCell")]
    thrust_load_cell: Option<f32>,
    #[serde(rename = "NitrousLoadCell")]
    nitrous_load_cell: Option<f32>,
}

pub struct GroundStation {
    _handle: JoinHandle<()>,
    sensor_to_gs: Arc<AtomicRingBuffer<(SensorDataKind, Instant)>>,
    gs_to_output: Arc<AtomicRingBuffer<Vec<pi_output::Command>>>,
}

impl GroundStation {
    /// Ideally, the ground station would be hosting the server and the test-stand/rocket
    /// would connect to that as a client (over UDP). However, the test-stand hosting the server itself
    /// is easier given current constraints.
    pub fn connect() -> Result<GroundStation> {
        let sensor_to_gs = Arc::new(AtomicRingBuffer::with_capacity(16));
        let gs_to_output = Arc::new(AtomicRingBuffer::with_capacity(1));

        let a = Arc::clone(&sensor_to_gs);
        let b = Arc::clone(&gs_to_output);

        let handle = thread::spawn(move || {
            run_ground_station(a, b).unwrap();
        });

        Ok(Self {
            _handle: handle,
            sensor_to_gs,
            gs_to_output,
        })
    }

    pub fn send_sensor_data(&self, data: SensorDataKind, ts: Instant) {
        self.sensor_to_gs.push_overwrite((data, ts));
    }

    pub fn read_new_commands(&self) -> Option<Vec<pi_output::Command>> {
        self.gs_to_output.try_pop()
    }
}

struct ConnSide {
    stream: TcpStream,
    sensor_to_gs: Arc<AtomicRingBuffer<(SensorDataKind, Instant)>>,
    gs_to_output: Arc<AtomicRingBuffer<Vec<pi_output::Command>>>,
    start_time: Instant,
}

fn run_ground_station(
    sensor_to_gs: Arc<AtomicRingBuffer<(SensorDataKind, Instant)>>,
    gs_to_output: Arc<AtomicRingBuffer<Vec<pi_output::Command>>>
) -> Result<()> {
    let rt = runtime::Builder::new_current_thread()
        .enable_io()
        .enable_time()
        .build()?;

    let local = task::LocalSet::new();

    let start_time = Instant::now();

    local.block_on(&rt, async {
        let server = TcpListener::bind("0.0.0.0:8080").await?;
        println!("listening");
    
        for (stream, addr) in server.accept().await {
            println!("accepted connection from {}", addr);

            let conn_side = ConnSide {
                stream,
                sensor_to_gs: Arc::clone(&sensor_to_gs),
                gs_to_output: Arc::clone(&gs_to_output),
                start_time,
            };
            
            if let Err(e) = accept_connection(conn_side).await {
                eprintln!("disconnected from ground station: {}", e);
            }
        }

        Ok(())
    })
}

async fn outgoing(
    sensor_to_gs: Arc<AtomicRingBuffer<(SensorDataKind, Instant)>>,
    mut writer: impl Sink<Message, Error = tungstenite::Error> + Unpin,
    start_time: Instant,
) -> Result<()> {
    let mut most_recent_update_time = start_time;

    let mut interval = tokio::time::interval(Duration::from_millis(20));

    loop {
        interval.tick().await;

        let mut sensor_data = SensorData::default();
        let mut update_count = 0;
        while let Some((sensor_update, ts)) = sensor_to_gs.try_pop() {
            most_recent_update_time = ts;
            match sensor_update {
                SensorDataKind::Pressure1(data) => {
                    let pi_sensor::Pressure1 { pt1_f, pt2_f, pt1_e, pt2_o } = data;
                    sensor_data.pt1_f = pt1_f.unpack().map(|i| i as f32).ok();
                    sensor_data.pt2_f = pt2_f.unpack().map(|i| i as f32).ok();
                    sensor_data.pt1_e = pt1_e.unpack().map(|i| i as f32).ok();
                    sensor_data.pt2_o = pt2_o.unpack().map(|i| i as f32).ok();
                }
                SensorDataKind::Pressure2(data) => {
                    let pi_sensor::Pressure2 { pt3_o, pt4_o, pt2_p  } = data;
                    sensor_data.pt3_o = pt3_o.unpack().map(|i| i as f32).ok();
                    sensor_data.pt4_o = pt4_o.unpack().map(|i| i as f32).ok();
                    sensor_data.pt2_p = pt2_p.unpack().map(|i| i as f32).ok();
                }
                SensorDataKind::FlowAndLoad(data) => {
                    let pi_sensor::FlowAndLoad { fm_f, fm_o, load1, load2 } = data;
                    sensor_data.fm_f = fm_f.unpack().map(|i| i as f32).ok();
                    sensor_data.fm_o = fm_o.unpack().map(|i| i as f32).ok();
                    sensor_data.thrust_load_cell = load1.unpack().map(|i| i as f32).ok();
                    sensor_data.nitrous_load_cell = load2.unpack().map(|i| i as f32).ok();
                }
                SensorDataKind::Thermo1(data) => {
                    let pi_sensor::Thermo1 { tc1_e, tc1_f, tc2_f, tc1_o  } = data;
                    sensor_data.tc1_e = tc1_e.unpack().map(|i| i as f32).ok();
                    sensor_data.tc1_f = tc1_f.unpack().map(|i| i as f32).ok();
                    sensor_data.tc2_f = tc2_f.unpack().map(|i| i as f32).ok();
                    sensor_data.tc1_o = tc1_o.unpack().map(|i| i as f32).ok();
                }
                SensorDataKind::Thermo2(data) => {
                    let pi_sensor::Thermo2 { tc5_o } = data;
                    sensor_data.tc5_o = tc5_o.unpack().map(|i| i as f32).ok();
                }
            }

            update_count += 1;
            if update_count > 16 {
                break;
            }
        }

        sensor_data.timestamp = (most_recent_update_time - start_time).as_millis() as u32;

        let msg = Message::Text(serde_json::to_string(&sensor_data)?);
        writer.send(msg).await?;
    }
}

async fn parse_msg(msg: Result<Message, tungstenite::Error>) -> Result<Command> {
    let msg = msg?.into_text()?;
    let msg = serde_json::from_str(&msg)
        .context("failed to parse ws message as json")?;

    Ok(msg)
}

async fn incoming(
    gs_to_output: Arc<AtomicRingBuffer<Vec<pi_output::Command>>>,
    mut reader: impl Stream<Item = Result<Message, tungstenite::Error>> + Unpin,
) -> Result<()> {
    while let Some(msg) = reader.next().await {
        let cmd = match parse_msg(msg).await {
            Ok(cmd) => cmd,
            Err(e) => {
                eprintln!("{:?}", e);
                continue;
            }
        };

        match cmd {
            Command::SetValves(states) => {
                fn two_way(state: TwoWayState) -> shared::TwoWay {
                    match state {
                        TwoWayState::Open => shared::TwoWay::Open,
                        TwoWayState::Closed => shared::TwoWay::Closed,
                    }
                }

                fn three_way(state: ThreeWayState) -> shared::ThreeWay {
                    match state {
                        ThreeWayState::NitrogenPathway => shared::ThreeWay::NitrogenPathway,
                        ThreeWayState::FuelOxidizer => shared::ThreeWay::FuelOxidizer,
                    }
                }

                let valves = shared::Valves {
                    fo_fp: two_way(states.fo_fp),
                    fc_fp: two_way(states.fc_fp),
                    fc_p: two_way(states.fc_p),
                    fc1_f: two_way(states.fc1_f),
                    fo_p1: two_way(states.fo_p1),
                    fo2_o: two_way(states.fo2_o),
                    fc4_o: two_way(states.fc4_o),
                    fc3_o: two_way(states.fc3_o),
                    pv_f: three_way(states.pv_f),
                    pv_o: three_way(states.pv_o),
                };

                gs_to_output.push_overwrite(vec![pi_output::Command::SetValves(valves.into())]);
            },
            Command::HardAbort => {
                let valves = shared::Valves::default();
                gs_to_output.push_overwrite(vec![pi_output::Command::SetValves(valves.into())]);
            },
            Command::SoftAbort => todo!(),
            Command::Ignite => todo!(),
        }
    }

    Ok(())
}

async fn accept_connection(cs: ConnSide) -> Result<()> {
    let ws = tokio_tungstenite::accept_async(cs.stream).await?;
    let (writer, reader) = ws.split();

    try_join!(outgoing(cs.sensor_to_gs, writer, cs.start_time), incoming(cs.gs_to_output, reader))?;

    Ok(())
}
