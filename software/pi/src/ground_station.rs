use std::{error::Error, thread::{self, JoinHandle}, sync::{atomic::{AtomicBool, Ordering}, Arc}, time::Duration};
use futures_util::{StreamExt, try_join, SinkExt};
use tokio::{net::{TcpListener, TcpStream}, task, runtime, time};
use tungstenite::Message;
use serde::{Serialize, Deserialize};

#[derive(Debug, Deserialize, Serialize)]
enum OpenClosed {
    #[serde(rename = "closed")]
    Closed,
    #[serde(rename = "open")]
    Open,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct Valves {
    #[serde(rename = "FO_FP")]
    fo_fp: OpenClosed,
    #[serde(rename = "FC_FP")]
    fc_fp: OpenClosed,
    #[serde(rename = "FC_P")]
    fc_p: OpenClosed,
    #[serde(rename = "FC3_O")]
    fc3_o: OpenClosed,
    #[serde(rename = "FO2_O")]
    fo2_o: OpenClosed,
    #[serde(rename = "FO_P1")]
    fo_p1: OpenClosed,
    #[serde(rename = "FO_P2")]
    fo_p2: OpenClosed,
    #[serde(rename = "FC1_F")]
    fc1_f: OpenClosed,
    #[serde(rename = "PV_F")]
    pv_f: OpenClosed,
    #[serde(rename = "PV_O")]
    pv_o: OpenClosed,
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
    #[serde(rename = "PT1_F")]
    pt1_f: Option<f32>,
    #[serde(rename = "PT2_F")]
    pt2_f: Option<f32>,
    #[serde(rename = "PT1_O")]
    pt1_o: Option<f32>,
    #[serde(rename = "PT2_O")]
    pt2_o: Option<f32>,
    #[serde(rename = "PT3_O")]
    pt3_o: Option<f32>,
    #[serde(rename = "PT4_O")]
    pt4_o: Option<f32>,
    #[serde(rename = "PT1_P")]
    pt1_p: Option<f32>,
    #[serde(rename = "PT2_P")]
    pt2_p: Option<f32>,
    #[serde(rename = "PT1_E")]
    pt1_e: Option<f32>,
    #[serde(rename = "PT2_E")]
    pt2_e: Option<f32>,
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
    #[serde(rename = "TC2_E")]
    tc2_e: Option<f32>,
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
    handle: JoinHandle<()>,
    tx: flume::Sender<()>,
    rx: flume::Receiver<()>,
    connected: Arc<AtomicBool>,
}

impl GroundStation {
    /// Ideally, the ground station would be hosting the server and the test-stand/rocket
    /// would connect to that as a client (over UDP). However, the test-stand hosting the server itself
    /// is easier given current constraints.
    pub fn connect() -> Result<GroundStation, Box<dyn Error>> {
        let (in_tx, in_rx) = flume::unbounded();
        let (out_tx, out_rx) = flume::unbounded();

        let connected = Arc::new(AtomicBool::new(false));

        let connected_clone = connected.clone();
        let handle = thread::spawn(move || {
            run_ground_station(out_tx, in_rx, connected_clone).unwrap();
        });

        Ok(Self {
            handle,
            tx: in_tx,
            rx: out_rx,
            connected,
        })
    }

    pub fn send_new_data(&self, data: ()) {
        if !self.connected.load(Ordering::SeqCst) {
            return;
        }
        
        todo!()
    }

    pub fn read_new_commands(&self) -> Option<()> {
        if !self.connected.load(Ordering::SeqCst) {
            return None;
        }
        
        todo!()
    }
}

fn run_ground_station(tx: flume::Sender<()>, rx: flume::Receiver<()>, connected: Arc<AtomicBool>) -> Result<(), Box<dyn Error>> {
    let rt = runtime::Builder::new_current_thread().build()?;
    let local = task::LocalSet::new();

    local.block_on(&rt, async {
        let server = TcpListener::bind("0.0.0.0:8080").await?;

        let mut abort_delayer: Option<task::JoinHandle<_>> = None;
    
        for (stream, addr) in server.accept().await {
            println!("accepted connection from {}", addr);
            connected.store(true, Ordering::SeqCst);

            if let Some(abort_task) = abort_delayer.take() {
                abort_task.abort();
            }
            
            if let Err(e) = accept_connection(stream, tx.clone(), rx.clone()).await {
                eprintln!("disconnected from ground station: {}", e);
            }

            // Do a soft-abort if the ground-station disconnects and doesn't reconnect
            // within 500 ms.
            let abort_tx = tx.clone();
            abort_delayer = Some(task::spawn(async move {
                time::sleep(Duration::from_millis(500)).await;

                // TODO: Send a soft-abort message.
                abort_tx.send_async(()).await;
            }));

            connected.store(false, Ordering::SeqCst);
        }

        Ok(())
    })
}

async fn accept_connection(stream: TcpStream, tx: flume::Sender<()>, rx: flume::Receiver<()>) -> Result<(), Box<dyn Error>> {
    let ws = tokio_tungstenite::accept_async(stream).await?;
    let (mut writer, mut reader) = ws.split();

    let writer_task = task::spawn_local(async move {
        while let Ok(_outgoing) = rx.recv_async().await {
            writer.send(todo!()).await?;
        }

        Ok::<_, Box<dyn Error>>(())
    });

    let reader_task = task::spawn_local(async move {
        while let Some(msg) = reader.next().await {
            let msg = msg?;
            let s = match msg {
                Message::Text(s) => s,
                _ => {
                    eprintln!("wrong websocket message format");
                    continue;
                },
            };

            
        }

        Ok::<_, Box<dyn Error>>(())
    });

    let (writer_res, reader_res) = try_join!(writer_task, reader_task)?;
    writer_res?;
    reader_res?;

    Ok(())
}
