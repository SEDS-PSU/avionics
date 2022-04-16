#![no_main]
#![no_std]

use defmt_rtt as _;
use dwt_systick_monotonic::fugit;
// global logger
use panic_probe as _;
use stm32f1xx_hal as _; // memory layout

mod util;

const FREQUENCY: u32 = 36_000_000; // Hz

type Duration = fugit::Duration<u32, 1, FREQUENCY>;

const RASPI_ID: bxcan::StandardId = if let Some(id) = bxcan::StandardId::new(shared::RASPI_ID) {
    id
} else {
    panic!("RASPI_ID is not a valid standard CAN ID");
};
const SENSOR_BOARD_ID: bxcan::StandardId =
    if let Some(id) = bxcan::StandardId::new(shared::SENSOR_BOARD_ID) {
        id
    } else {
        panic!("SENSOR_ID is not a valid standard CAN ID");
    };

#[rtic::app(device = stm32f1xx_hal::pac, dispatchers = [EXTI0])]
mod app {
    use core::fmt;

    use crate::{Duration, FREQUENCY, SENSOR_BOARD_ID};
    use bxcan::{filter::Mask32, Frame, Interrupts, Rx, StandardId, Tx};
    use dwt_systick_monotonic::DwtSystick;
    use heapless::spsc::Queue;
    use mcp96x::{FilterCoefficient, ThermocoupleType, MCP96X};
    use postcard::MaxSize;
    use shared::pi_sensor::{self, Force, SensorError, SensorReading, Temperature};
    use stm32f1xx_hal::{
        adc::{Adc, SampleTime},
        can::Can,
        gpio::{
            gpioa::{PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7},
            gpiob::{PB0, PB1, PB6, PB7},
            gpioc::{PC0, PC1, PC2, PC3},
            Alternate, Analog, OpenDrain,
        },
        i2c::BlockingI2c,
        pac::{Interrupt, ADC1, CAN1, I2C1},
        prelude::*,
    };

    use crate::RASPI_ID;

    // The monotonic scheduler type
    #[monotonic(binds = SysTick, default = true)]
    type Mono = DwtSystick<FREQUENCY /* Hz */>;

    type I2c1 = BlockingI2c<I2C1, (PB6<Alternate<OpenDrain>>, PB7<Alternate<OpenDrain>>)>;
    type I2c1Proxy = shared_bus::I2cProxy<'static, shared_bus::AtomicCheckMutex<I2c1>>;

    pub struct AnalogPins {
        load_cell1: PA0<Analog>,
        load_cell2: PA1<Analog>,

        pressure1: PA6<Analog>,
        pressure2: PA7<Analog>,
        pressure3: PB0<Analog>,
        pressure4: PB1<Analog>,
        pressure5: PA4<Analog>,
        pressure6: PA5<Analog>,
        pressure7: PA2<Analog>,
        pressure8: PA3<Analog>,
        pressure9: PC0<Analog>,

        flow1: PC1<Analog>,
        flow2: PC2<Analog>,

        battery: PC3<Analog>, // Assignment may change.
    }

    pub struct Thermocouples {
        thermo1: Option<MCP96X<I2c1Proxy, 0b1100_000>>,
        thermo2: Option<MCP96X<I2c1Proxy, 0b1100_001>>,
        thermo3: Option<MCP96X<I2c1Proxy, 0b1100_010>>,
        thermo4: Option<MCP96X<I2c1Proxy, 0b1100_011>>,
        thermo5: Option<MCP96X<I2c1Proxy, 0b1100_100>>,
        thermo6: Option<MCP96X<I2c1Proxy, 0b1100_101>>,
    }

    // Shared resources go here
    #[shared]
    struct Shared {
        /// The CAN frame queue.
        /// This has a capacity of 15, according to the documentation.
        can_tx_queue: Queue<Frame, 16>,

        battery_millivolts: Option<u16>,
        sensor_data: pi_sensor::AllSensors,
    }

    // Local resources go here
    #[local]
    struct Local {
        can_tx: Tx<Can<CAN1>>,
        can_rx: Rx<Can<CAN1>>,

        adc1: Adc<ADC1>,
        analog_pins: AnalogPins,
        thermocouples: Thermocouples,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");

        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        let clocks = rcc
            .cfgr
            .use_hse(36.mhz())
            // do we need other stuff here?
            .sysclk(FREQUENCY.hz())
            .pclk1(16.mhz())
            .freeze(&mut flash.acr);

        // Set up CAN bus.
        // Based on https://github.com/stm32-rs/stm32f1xx-hal/blob/master/examples/can-rtic.rs.
        let can = Can::new(cx.device.CAN1);

        // Set up the ADC.
        let mut adc1 = Adc::adc1(cx.device.ADC1, clocks.clone());
        adc1.set_sample_time(SampleTime::T_239);

        // Pins
        let mut afio = cx.device.AFIO.constrain();
        let mut gpioa = cx.device.GPIOA.split();
        let mut gpiob = cx.device.GPIOB.split();
        let mut gpioc = cx.device.GPIOC.split();

        let analog_pins = AnalogPins {
            load_cell1: gpioa.pa0.into_analog(&mut gpioa.crl),
            load_cell2: gpioa.pa1.into_analog(&mut gpioa.crl),

            pressure1: gpioa.pa6.into_analog(&mut gpioa.crl),
            pressure2: gpioa.pa7.into_analog(&mut gpioa.crl),
            pressure3: gpiob.pb0.into_analog(&mut gpiob.crl),
            pressure4: gpiob.pb1.into_analog(&mut gpiob.crl),
            pressure5: gpioa.pa4.into_analog(&mut gpioa.crl),
            pressure6: gpioa.pa5.into_analog(&mut gpioa.crl),
            pressure7: gpioa.pa2.into_analog(&mut gpioa.crl),
            pressure8: gpioa.pa3.into_analog(&mut gpioa.crl),
            pressure9: gpioc.pc0.into_analog(&mut gpioc.crl),

            flow1: gpioc.pc1.into_analog(&mut gpioc.crl),
            flow2: gpioc.pc2.into_analog(&mut gpioc.crl),

            battery: gpioc.pc3.into_analog(&mut gpioc.crl),
        };

        let mut dwt = cx.core.DWT;

        // Set up the I2C bus.
        dwt.enable_cycle_counter();

        let bus_manager: &'static _ = {
            let i2c_scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
            let i2c_sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);

            let i2c1 = BlockingI2c::i2c1(
                cx.device.I2C1,
                (i2c_scl, i2c_sda),
                &mut afio.mapr,
                100.khz(),
                clocks.clone(),
                1000,
                10,
                1000,
                1000,
            );

            shared_bus::new_atomic_check!(I2c1 = i2c1).unwrap()
        };

        fn to_opt<T, E: fmt::Debug>(res: Result<T, E>, n: usize) -> Option<T> {
            match res {
                Ok(t) => Some(t),
                Err(e) => {
                    defmt::error!(
                        "failed to initialize thermocouple {}: {:?}",
                        n,
                        defmt::Debug2Format(&e)
                    );
                    None
                }
            }
        }

        let thermocouples = Thermocouples {
            thermo1: to_opt(
                MCP96X::new(
                    bus_manager.acquire_i2c(),
                    ThermocoupleType::K,
                    FilterCoefficient::Maximum,
                ),
                1,
            ),
            thermo2: to_opt(
                MCP96X::new(
                    bus_manager.acquire_i2c(),
                    ThermocoupleType::K,
                    FilterCoefficient::Maximum,
                ),
                2,
            ),
            thermo3: to_opt(
                MCP96X::new(
                    bus_manager.acquire_i2c(),
                    ThermocoupleType::K,
                    FilterCoefficient::Maximum,
                ),
                3,
            ),
            thermo4: to_opt(
                MCP96X::new(
                    bus_manager.acquire_i2c(),
                    ThermocoupleType::K,
                    FilterCoefficient::Maximum,
                ),
                4,
            ),
            thermo5: to_opt(
                MCP96X::new(
                    bus_manager.acquire_i2c(),
                    ThermocoupleType::K,
                    FilterCoefficient::Maximum,
                ),
                5,
            ),
            thermo6: to_opt(
                MCP96X::new(
                    bus_manager.acquire_i2c(),
                    ThermocoupleType::K,
                    FilterCoefficient::Maximum,
                ),
                6,
            ),
        };

        let can_rx_pin = gpioa.pa11.into_floating_input(&mut gpioa.crh);
        let can_tx_pin = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);

        can.assign_pins((can_tx_pin, can_rx_pin), &mut afio.mapr);

        // APB1 (PCLK1): 16MHz, Bit rate: 1000kBit/s, Sample Point 87.5%
        // Value was calculated with http://www.bittiming.can-wiki.info/
        let mut can = bxcan::Can::builder(can)
            .set_bit_timing(0x001c0000)
            .leave_disabled();

        // Only recieve frames intended for the sensor board.
        can.modify_filters().enable_bank(
            0,
            Mask32::frames_with_std_id(SENSOR_BOARD_ID, StandardId::MAX),
        );

        // Sync to the bus and start normal operation.
        can.enable_interrupts(
            Interrupts::TRANSMIT_MAILBOX_EMPTY | Interrupts::FIFO0_MESSAGE_PENDING,
        );
        nb::block!(can.enable_non_blocking()).unwrap();

        let (can_tx, can_rx) = can.split();

        // Set up monotonic scheduler.
        let mut dcb = cx.core.DCB;
        let systick = cx.core.SYST;

        defmt::info!("syclk: {}", clocks.sysclk().0);

        let mono = DwtSystick::new(&mut dcb, dwt, systick, clocks.sysclk().0);

        sensor_poll::spawn().unwrap();
        send_sensor_data::spawn_after(Duration::secs(2)).unwrap();

        // Setup the monotonic timer
        (
            Shared {
                can_tx_queue: Queue::new(),
                battery_millivolts: None,
                sensor_data: Default::default(),
            },
            Local {
                can_tx,
                can_rx,

                adc1,
                analog_pins,

                thermocouples,
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

    #[task]
    fn sensor_poll(_cx: sensor_poll::Context) {
        sensor_poll::spawn_after(Duration::millis(1000)).unwrap();

        read_adc1::spawn().expect("failed to spawn task `poll_adc1`");
        read_thermocouples::spawn().expect("failed to spawn task `poll_thermocouples`");
    }

    #[task(local = [adc1, analog_pins], shared = [sensor_data, battery_millivolts])]
    fn read_adc1(cx: read_adc1::Context) {
        // defmt::info!("read_adc1");
        let read_adc1::LocalResources { adc1, analog_pins } = cx.local;
        let read_adc1::SharedResources {
            sensor_data,
            battery_millivolts,
        } = cx.shared;

        let s = |res: nb::Result<u16, ()>| match res {
            Ok(v) => SensorReading::new(v),
            Err(_) => SensorReading::new_error(SensorError::Unknown),
        };

        let l = |res| match res {
            Ok(v) => Force::new(v),
            Err(_) => Force::new_error(SensorError::Unknown),
        };

        // This is the source of truth for these sensor to pin mappings.

        let fm_f = s(adc1.read(&mut analog_pins.flow1));
        let fm_o = s(adc1.read(&mut analog_pins.flow2));

        // TODO: Do conversions into newtons.
        let load1 = l(adc1.read(&mut analog_pins.load_cell1));
        let load2 = l(adc1.read(&mut analog_pins.load_cell2));

        let pt1_f = s(adc1.read(&mut analog_pins.pressure1));
        let pt2_f = s(adc1.read(&mut analog_pins.pressure2));
        let pt1_e = s(adc1.read(&mut analog_pins.pressure3));
        let pt2_e = s(adc1.read(&mut analog_pins.pressure4));
        let pt1_o = s(adc1.read(&mut analog_pins.pressure5));
        let pt2_o = s(adc1.read(&mut analog_pins.pressure6));
        let pt4_o = s(adc1.read(&mut analog_pins.pressure7));
        let pt1_p = s(adc1.read(&mut analog_pins.pressure8));
        let pt2_p = s(adc1.read(&mut analog_pins.pressure9));

        let bat_voltage = adc1
            .read(&mut analog_pins.battery)
            .ok()
            .map(|s: u16| ((s as u32) * 1200 / (adc1.read_vref() as u32)).try_into().expect("calculation for battery voltage was wrong"));

        (sensor_data, battery_millivolts).lock(|sensor_data, bat| {
            sensor_data.fm_f = fm_f;
            sensor_data.fm_o = fm_o;

            sensor_data.load1 = load1;
            sensor_data.load2 = load2;

            sensor_data.pt1_f = pt1_f;
            sensor_data.pt2_f = pt2_f;
            sensor_data.pt1_e = pt1_e;
            sensor_data.pt2_e = pt2_e;
            sensor_data.pt1_o = pt1_o;
            sensor_data.pt2_o = pt2_o;
            sensor_data.pt4_o = pt4_o;
            sensor_data.pt1_p = pt1_p;
            sensor_data.pt2_p = pt2_p;

            *bat = bat_voltage;
        });
    }

    #[task(local = [thermocouples], shared = [sensor_data])]
    fn read_thermocouples(cx: read_thermocouples::Context) {
        // defmt::info!("read_thermocouples");
        let tc = cx.local.thermocouples;
        let mut sensor_data = cx.shared.sensor_data;

        let c = |opt: Option<_>| {
            opt.map(|res| match res {
                Ok(v) => Temperature::new(v),
                Err(_) => Temperature::new_error(SensorError::Unknown),
            })
        };

        // This is the source of truth for these sensor to pin mappings.

        let t1 = c(tc.thermo1.as_mut().map(|t1| t1.read()));
        let t2 = c(tc.thermo2.as_mut().map(|t2| t2.read()));
        let t3 = c(tc.thermo3.as_mut().map(|t3| t3.read()));
        let t4 = c(tc.thermo4.as_mut().map(|t4| t4.read()));
        let t5 = c(tc.thermo5.as_mut().map(|t5| t5.read()));
        let t6 = c(tc.thermo6.as_mut().map(|t6| t6.read()));

        sensor_data.lock(|s| {
            s.tc1_e = t1.unwrap_or(Temperature::new_error(SensorError::NoData));
            s.tc2_e = t2.unwrap_or(Temperature::new_error(SensorError::NoData));
            s.tc1_f = t3.unwrap_or(Temperature::new_error(SensorError::NoData));
            s.tc2_f = t4.unwrap_or(Temperature::new_error(SensorError::NoData));
            s.tc1_o = t5.unwrap_or(Temperature::new_error(SensorError::NoData));
            s.tc5_o = t6.unwrap_or(Temperature::new_error(SensorError::NoData));
        });
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
        queue.enqueue(frame).expect("the frame queue is full");
        rtic::pend(Interrupt::USB_HP_CAN_TX);
    }

    #[task(shared = [can_tx_queue, sensor_data, battery_millivolts])]
    fn send_sensor_data(cx: send_sensor_data::Context) {
        send_sensor_data::spawn_after(Duration::secs(2)).unwrap(); // temporary

        let send_sensor_data::SharedResources {
            mut can_tx_queue,
            sensor_data,
            battery_millivolts,
        } = cx.shared;

        let (sensor_data, battery_millivolts) =
            (sensor_data, battery_millivolts).lock(|data, bat| (data.clone(), *bat));

        defmt::info!("sensor_data: {:#?}", defmt::Debug2Format(&sensor_data));

        let header = pi_sensor::ResponseHeader {
            doing_good: true, // temporary
            battery_millivolts,
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
        let can_rx0::LocalResources { can_rx } = cx.local;

        loop {
            match can_rx.receive() {
                Ok(frame) => {
                    defmt::info!("received a CAN frame");

                    let data = if let Some(data) = frame.data() {
                        data.as_ref()
                    } else {
                        continue; // go to next loop iteration
                    };

                    let request = if let Ok(r) = postcard::from_bytes(data) {
                        r
                    } else {
                        defmt::error!("failed to deserialize frame");
                        continue;
                    };

                    match request {
                        pi_sensor::Request::GetSensorData => {
                            send_sensor_data::spawn()
                                .expect("failed to spawn task `send_sensor_data`");
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
