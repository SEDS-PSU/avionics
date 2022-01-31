#![no_main]
#![no_std]

use defmt_rtt as _;
use dwt_systick_monotonic::fugit;
// global logger
use panic_probe as _;
use stm32f1xx_hal as _; // memory layout

mod util;

const FREQUENCY: u32 = 36_000_000; // Hz

type Instant = fugit::Instant<u32, 1, FREQUENCY>;
type Duration = fugit::Duration<u32, 1, FREQUENCY>;

const RASPI_ID: bxcan::StandardId = if let Some(id) = bxcan::StandardId::new(shared::RASPI_ID) {
    id
} else {
    panic!("RASPI_ID is not a valid standard CAN ID");
};
const SENSOR_BOARD_ID: bxcan::StandardId = if let Some(id) = bxcan::StandardId::new(shared::SENSOR_BOARD_ID) {
    id
} else {
    panic!("SENSOR_ID is not a valid standard CAN ID");
};

#[rtic::app(device = stm32f1xx_hal::pac, dispatchers = [EXTI0])]
mod app {
    use crate::{SENSOR_BOARD_ID, FREQUENCY};
    use bxcan::{filter::Mask32, Frame, Interrupts, Rx, Tx, StandardId};
    use dwt_systick_monotonic::DwtSystick;
    use heapless::{spsc::Queue};
    use postcard::MaxSize;
    use shared::pi_sensor;
    use stm32f1xx_hal::{
        can::Can,
        pac::{Interrupt, CAN1},
        prelude::*,
    };

    use crate::{Instant, RASPI_ID};

    // The monotonic scheduler type
    #[monotonic(binds = SysTick, default = true)]
    type Mono = DwtSystick<FREQUENCY /* Hz */>;

    // Shared resources go here
    #[shared]
    struct Shared {
        /// The CAN frame queue.
        /// This has a capacity of 15, according to the documentation.
        can_tx_queue: Queue<Frame, 16>,

        sensor_data: pi_sensor::AllSensors,
    }

    // Local resources go here
    #[local]
    struct Local {
        can_tx: Tx<Can<CAN1>>,
        can_rx: Rx<Can<CAN1>>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");

        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();
        let mut exti = cx.device.EXTI;

        let clocks = rcc
            .cfgr
            .use_hse(25.mhz())
            // do we need other stuff here?
            .sysclk(FREQUENCY.hz())
            .pclk1(18.mhz())
            .freeze(&mut flash.acr);

        // Set up CAN bus.
        // Based on https://github.com/stm32-rs/stm32f1xx-hal/blob/master/examples/can-rtic.rs.
        let can = Can::new(cx.device.CAN1);

        // Pins
        let mut afio = cx.device.AFIO.constrain();
        let mut gpioa = cx.device.GPIOA.split();
        let mut gpiob = cx.device.GPIOB.split();

        let can_rx_pin = gpioa.pa11.into_floating_input(&mut gpioa.crh);
        let can_tx_pin = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);

        can.assign_pins((can_tx_pin, can_rx_pin), &mut afio.mapr);

        // APB1 (PCLK1): 16MHz, Bit rate: 1000kBit/s, Sample Point 87.5%
        // Value was calculated with http://www.bittiming.can-wiki.info/
        let mut can = bxcan::Can::builder(can)
            .set_bit_timing(0x001e0000)
            .leave_disabled();

        // Only recieve frames intended for the sensor board.
        can.modify_filters().enable_bank(0, Mask32::frames_with_std_id(SENSOR_BOARD_ID, StandardId::MAX));

        // Sync to the bus and start normal operation.
        can.enable_interrupts(
            Interrupts::TRANSMIT_MAILBOX_EMPTY | Interrupts::FIFO0_MESSAGE_PENDING,
        );
        nb::block!(can.enable_non_blocking()).unwrap();

        let (can_tx, can_rx) = can.split();

        // Set up monotonic scheduler.
        let mut dcb = cx.core.DCB;
        let dwt = cx.core.DWT;
        let systick = cx.core.SYST;

        defmt::info!("syclk: {}", clocks.sysclk().0);

        let mono = DwtSystick::new(&mut dcb, dwt, systick, clocks.sysclk().0);

        // Setup the monotonic timer
        (
            Shared {
                can_tx_queue: Queue::new(),
                sensor_data: Default::default(),
            },
            Local {
                can_tx,
                can_rx,
            },
            init::Monotonics(mono),
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }

    /// This is fired every time the CAN controller has finished a frame transmission
    /// or after the USB_HP_CAN_TX ISR is pended.
    #[task(binds = USB_HP_CAN_TX, local = [can_tx], shared = [can_tx_queue])]
    fn can_tx(cx: can_tx::Context) {
        defmt::info!("can_tx");

        let mut tx_queue = cx.shared.can_tx_queue;
        let tx = cx.local.can_tx;

        tx.clear_interrupt_flags();

        tx_queue.lock(|tx_queue| {
            while let Some(frame) = tx_queue.peek() {
                match tx.transmit(frame) {
                    Ok(status) => match status.dequeued_frame() {
                        None => {
                            // Frame was placed in a transmit buffer.
                            tx_queue.dequeue();
                        }
                        Some(pending_frame) => {
                            // A lower priority frame was replaced with our higher-priority frame.
                            // Put the lower priority frame back into the transmit queue.
                            tx_queue.dequeue();
                            enqueue_frame(tx_queue, pending_frame.clone());
                        }
                    },
                    Err(nb::Error::WouldBlock) => break,
                    Err(_) => unreachable!(),
                }
            }
        });
    }

    fn enqueue_frame(queue: &mut Queue<Frame, 16>, frame: Frame) {
        queue.enqueue(frame).unwrap();
        rtic::pend(Interrupt::USB_HP_CAN_TX);
    }

    #[task(shared = [can_tx_queue, sensor_data])]
    fn send_sensor_data(cx: send_sensor_data::Context) {
        let send_sensor_data::SharedResources { mut can_tx_queue, mut sensor_data } = cx.shared;

        let sensor_data = sensor_data.lock(|data| data.clone());

        let header = pi_sensor::ResponseHeader {
            doing_good: true, // temporary
        };

        let mut buf = [0; pi_sensor::ResponseHeader::POSTCARD_MAX_SIZE];
        if let Err(_) = postcard::to_slice(&header, &mut buf) {
            defmt::error!("Could not serialize sensor data header");
            return;
        }

        let header_frame = bxcan::Frame::new_data(RASPI_ID, buf);

        let mut buf = [0; pi_sensor::AllSensors::POSTCARD_MAX_SIZE];
        if let Err(_) = postcard::to_slice(&sensor_data, &mut buf) {
            defmt::error!("Could not serialize sensor data");
            return;
        }

        can_tx_queue.lock(|tx_queue| {
            enqueue_frame(tx_queue, header_frame);

            let mut chunks = buf.chunks(8);

            for chunk in &mut chunks {
                let data = bxcan::Data::new(chunk).unwrap();
                let frame = bxcan::Frame::new_data(RASPI_ID, data);
                enqueue_frame(tx_queue, frame);
            }
        });
    }

    #[task(binds = USB_LP_CAN_RX0,
        local = [
            can_rx,
        ]
    )]
    fn can_rx0(cx: can_rx0::Context) {
        let can_rx0::LocalResources {
            can_rx,
        } = cx.local;

        loop {
            match can_rx.receive() {
                Ok(frame) => {
                    defmt::info!("received a CAN frame");

                    let data = if let Some(data) = frame.data() {
                        data.as_ref()
                    } else {
                        continue; // go to next loop iteration
                    };

                    let request = if let Ok(r) = postcard::from_bytes(data) { r } else {
                        defmt::error!("failed to deserialize frame");
                        continue;
                    };

                    match request {
                        pi_sensor::Request::GetSensorData => {
                            send_sensor_data::spawn().expect("failed to spawn task `send_sensor_data`");
                        }
                        pi_sensor::Request::Reset => {
                            defmt::info!("resetting");
                            defmt::flush();
                            cortex_m::peripheral::SCB::sys_reset()
                        }
                    }
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => {} // Ignore overrun errors.
            }
        }
    }
}
