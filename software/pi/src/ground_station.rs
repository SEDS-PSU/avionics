use std::{thread::{self, JoinHandle}, time::Instant};
use futures_util::{StreamExt, try_join, SinkExt, Sink, Stream};
use shared::{pi_sensor, pi_output};
use tokio::{net::{TcpListener, TcpStream}, task, runtime};
use tungstenite::Message;
use anyhow::{Result, Context};
use serde::{Serialize, Deserialize};

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

#[derive(Debug, Deserialize, Serialize)]
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
    tx: flume::Sender<pi_sensor::AllSensors>,
    rx: flume::Receiver<Vec<pi_output::Command>>,
}

impl GroundStation {
    /// Ideally, the ground station would be hosting the server and the test-stand/rocket
    /// would connect to that as a client (over UDP). However, the test-stand hosting the server itself
    /// is easier given current constraints.
    pub fn connect() -> Result<GroundStation> {
        let (in_tx, in_rx) = flume::unbounded();
        let (out_tx, out_rx) = flume::unbounded();

        let handle = thread::spawn(move || {
            run_ground_station(out_tx, in_rx).unwrap();
        });

        Ok(Self {
            _handle: handle,
            tx: in_tx,
            rx: out_rx,
        })
    }

    pub fn send_sensor_data(&self, data: pi_sensor::AllSensors) {
        self.tx.send(data).expect("this should never fail");
    }

    pub fn read_new_commands(&self) -> Option<Vec<pi_output::Command>> {
        self.rx.try_recv().ok()
    }
}

struct ConnSide {
    stream: TcpStream,
    tx: flume::Sender<Vec<pi_output::Command>>,
    rx: flume::Receiver<pi_sensor::AllSensors>,
    start_time: Instant,
}

fn run_ground_station(tx: flume::Sender<Vec<pi_output::Command>>, rx: flume::Receiver<pi_sensor::AllSensors>) -> Result<()> {
    let rt = runtime::Builder::new_current_thread().enable_io().build()?;
    let local = task::LocalSet::new();

    let start_time = Instant::now();

    local.block_on(&rt, async {
        let server = TcpListener::bind("0.0.0.0:8080").await?;
        println!("listening");
    
        for (stream, addr) in server.accept().await {
            println!("accepted connection from {}", addr);

            let conn_side = ConnSide {
                stream,
                tx: tx.clone(),
                rx: rx.clone(),
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
    rx: flume::Receiver<pi_sensor::AllSensors>,
    mut writer: impl Sink<Message, Error = tungstenite::Error> + Unpin,
    start_time: Instant,
) -> Result<()> {
    while let Ok(sensors) = rx.recv_async().await {
        let sensor_data = SensorData {
            timestamp: start_time.elapsed().as_millis() as u32,
            pt1_f: sensors.pt1_f.unpack().map(|i| i as f32).ok(),
            pt2_f: sensors.pt2_f.unpack().map(|i| i as f32).ok(),
            pt2_o: sensors.pt2_o.unpack().map(|i| i as f32).ok(),
            pt3_o: sensors.pt3_o.unpack().map(|i| i as f32).ok(),
            pt4_o: sensors.pt4_o.unpack().map(|i| i as f32).ok(),
            pt2_p: sensors.pt2_p.unpack().map(|i| i as f32).ok(),
            pt1_e: sensors.pt1_e.unpack().map(|i| i as f32).ok(),
            tc1_f: sensors.tc1_f.unpack().map(|i| i as f32).ok(),
            tc2_f: sensors.tc2_f.unpack().map(|i| i as f32).ok(),
            tc1_o: sensors.tc1_o.unpack().map(|i| i as f32).ok(),
            tc5_o: sensors.tc5_o.unpack().map(|i| i as f32).ok(),
            tc1_e: sensors.tc1_e.unpack().map(|i| i as f32).ok(),
            fm_f: sensors.fm_f.unpack().map(|i| i as f32).ok(),
            fm_o: sensors.fm_o.unpack().map(|i| i as f32).ok(),
            thrust_load_cell: sensors.load1.unpack().map(|i| i as f32).ok(),
            nitrous_load_cell: sensors.load2.unpack().map(|i| i as f32).ok(),
        };

        let msg = Message::Text(serde_json::to_string(&sensor_data)?);

        writer.send(msg).await?;
    }

    Ok(())
}

async fn parse_msg(msg: Result<Message, tungstenite::Error>) -> Result<Command> {
    let msg = msg?.into_text()?;
    let msg = serde_json::from_str(&msg)
        .context("failed to parse ws message as json")?;

    Ok(msg)
}

async fn incoming(
    tx: flume::Sender<Vec<pi_output::Command>>,
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

                tx.send_async(vec![pi_output::Command::SetValves(valves.into())]).await?;
            },
            Command::HardAbort => {
                let valves = shared::Valves::default();
                tx.send_async(vec![pi_output::Command::SetValves(valves.into())]).await?;
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

    try_join!(outgoing(cs.rx, writer, cs.start_time), incoming(cs.tx, reader))?;

    Ok(())
}
