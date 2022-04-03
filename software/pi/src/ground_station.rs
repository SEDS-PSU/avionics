use std::{error::Error, thread::{self, JoinHandle}, sync::{atomic::{AtomicBool, Ordering}, Arc}, time::Duration};
use core_affinity::CoreId;
use futures_util::{StreamExt, try_join, SinkExt};
use tokio::{net::{TcpListener, TcpStream}, task, runtime, time};
use tungstenite::Message;

pub enum AbortType {
    /// Full purge.
    Hard,
    /// Pause and change valves to safe but non-purging.
    Soft,
}

enum Incoming {
    Abort {
        ty: AbortType,
    },
    StartSequence {
        name: String,
    },
}

pub enum AbortCause {

}

enum Anomaly {
    /// A sensor is out-of-range.
    SensorOOR {
        sensor: String,
        value: f32,
    },
}

enum Outgoing {
    Aborted {
        cause: Anomaly,
        ty: AbortType,
    },
    Warning {
        cause: Anomaly,
    },
    SensorData {
        // TODO
    },
    ValveSequenceChanged {
        old: String,
        new: String,
    },
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
    pub fn connect(cpu_core: CoreId) -> Result<GroundStation, Box<dyn Error>> {
        let (in_tx, in_rx) = flume::unbounded();
        let (out_tx, out_rx) = flume::unbounded();

        let connected = Arc::new(AtomicBool::new(false));

        let connected_clone = connected.clone();
        let handle = thread::spawn(move || {
            core_affinity::set_for_current(cpu_core);

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
