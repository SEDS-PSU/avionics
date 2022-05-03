#![no_main]
#![no_std]

use defmt_rtt as _;
// global logger
use panic_probe as _;
use stm32f1xx_hal as _; // memory layout

mod util;
mod hx711;

const SENSOR_BOARD_ID: bxcan::StandardId =
    if let Some(id) = bxcan::StandardId::new(shared::Id::SensorBoard as u16) {
        id
    } else {
        panic!("SENSOR_ID is not a valid standard CAN ID");
    };

#[rtic::app(device = stm32f1xx_hal::pac, dispatchers = [EXTI0, EXTI1, EXTI2])]
mod app {
    use core::{fmt, sync::atomic::{AtomicU16, Ordering}};

    use crate::{SENSOR_BOARD_ID, hx711::{DualHx711, Mode as Hx711Mode}};
    use bxcan::{filter::Mask32, Frame, Interrupts, Rx, StandardId, Tx};
    use embedded_hal::adc::Channel;
    use heapless::spsc::Queue;
    use mcp96x::{FilterCoefficient, ThermocoupleType, MCP96X};
    use shared::{
        pi_sensor::{self, Force, SensorError, SensorReading, Temperature, Pressure},
        Id,
    };
    use stm32f1xx_hal::{
        adc::{Adc, SampleTime},
        can::Can,
        gpio::{
            gpioa::{PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7},
            gpiob::{PB0, PB1, PB6, PB7},
            gpioc::{PC0, PC1, PC2},
            Alternate, Analog, OpenDrain, PushPull, Output, Input, PullDown, PC3,
        },
        i2c::BlockingI2c,
        pac::{Interrupt, ADC1, CAN1, I2C1, TIM1, TIM2},
        timer::{MonoTimerUs, DelayUs},
        prelude::*,
    };

    const FREQUENCY: u32 = 36_000_000; // Hz

    // Granularity of one microsecond
    type Instant = fugit::Instant<u32, 1, 1_000_000>;
    // type Duration = fugit::Duration<u32, 1, 1_000_000>;

    // The monotonic scheduler type
    #[monotonic(binds = TIM2, default = true)]
    type Mono = MonoTimerUs<TIM2>;

    type I2c1 = BlockingI2c<I2C1, (PB6<Alternate<OpenDrain>>, PB7<Alternate<OpenDrain>>)>;
    type I2c1Proxy = shared_bus::I2cProxy<'static, shared_bus::AtomicCheckMutex<I2c1>>;

    pub struct AnalogPins {
        // load_cell1: PA0<Analog>,
        // load_cell2: PA1<Analog>,

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

        // battery: PC3<Analog>, // Assignment may change.
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
        /// This has a capacity of 31, according to the documentation.
        can_tx_queue: Queue<Frame, 32>,

        battery_millivolts: Option<u16>,

        fast_sensing_ms_remaining: AtomicU16,
        medium_sensing_ms_remaining: AtomicU16,
        slow_sensing_ms_remaining: AtomicU16,
    }

    // Local resources go here
    #[local]
    struct Local {
        can_tx: Tx<Can<CAN1>>,
        can_rx: Rx<Can<CAN1>>,

        adc1: Adc<ADC1>,
        analog_pins: AnalogPins,
        thermocouples: Thermocouples,
        dual_hx711: Option<DualHx711<
            DelayUs<TIM1>,
            PA0<Input<PullDown>>,
            PA1<Input<PullDown>>,
            PC3<Output<PushPull>>,
        >>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");

        // Allow RTT to work during `wfe`, `wfi`
        cx.device.DBGMCU.cr.modify(|_, w| {
            w.dbg_sleep().set_bit();
            w.dbg_standby().set_bit();
            w.dbg_stop().set_bit()
        });
        cx.device.RCC.ahbenr.modify(|_, w| w.dma1en().enabled());

        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        let clocks = rcc
            .cfgr
            .use_hse(36.MHz())
            // do we need other stuff here?
            .sysclk(FREQUENCY.Hz())
            .pclk1(9.MHz())
            .freeze(&mut flash.acr);

        let tim1 = cx.device.TIM1;
        let tim2 = cx.device.TIM2;
        let delay = tim1.delay(&clocks);
        let mut mono = tim2.monotonic(&clocks);

        // Set up CAN bus.
        // Based on https://github.com/stm32-rs/stm32f1xx-hal/blob/master/examples/can-rtic.rs.
        let can = Can::new(cx.device.CAN1);

        // Set up the ADC.
        let mut adc1 = Adc::adc1(cx.device.ADC1, clocks.clone());
        adc1.set_sample_time(SampleTime::T_71);

        // Pins
        let mut afio = cx.device.AFIO.constrain();
        let mut gpioa = cx.device.GPIOA.split();
        let mut gpiob = cx.device.GPIOB.split();
        let mut gpioc = cx.device.GPIOC.split();

        let analog_pins = AnalogPins {
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

            // battery: gpioc.pc3.into_analog(&mut gpioc.crl),
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
                150.kHz(),
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

        let dual_hx711 = DualHx711::new(
            delay,
            gpioa.pa0.into_pull_down_input(&mut gpioa.crl),
            gpioa.pa1.into_pull_down_input(&mut gpioa.crl),
            gpioc.pc3.into_push_pull_output(&mut gpioc.crl),
            Hx711Mode::ChAGain128,
        ).map_err(|e| {
            defmt::error!("failed to initialize dual HX711: {:?}", e);
            ()
        }).ok();

        let can_rx_pin = gpioa.pa11.into_floating_input(&mut gpioa.crh);
        let can_tx_pin = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);

        can.assign_pins((can_tx_pin, can_rx_pin), &mut afio.mapr);

        // APB1 (PCLK1): 9MHz, Bit rate: 500kBit/s, Sample Point 87.5%
        // Value was calculated with http://www.bittiming.can-wiki.info/
        let mut can = bxcan::Can::builder(can)
            .set_bit_timing(0x001e0000)
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
        // let mut dcb = cx.core.DCB;
        // let systick = cx.core.SYST;

        // let mut mono = DwtSystick::new(&mut dcb, dwt, systick, clocks.sysclk().0);

        fast_sensor_poll::spawn(mono.now()).unwrap();
        slow_sensor_poll::spawn(mono.now()).unwrap();

        (
            Shared {
                can_tx_queue: Queue::new(),
                battery_millivolts: None,
                fast_sensing_ms_remaining: AtomicU16::new(0),
                medium_sensing_ms_remaining: AtomicU16::new(0),
                slow_sensing_ms_remaining: AtomicU16::new(0),
            },
            Local {
                can_tx,
                can_rx,

                adc1,
                analog_pins,

                thermocouples,

                dual_hx711,
            },
            init::Monotonics(mono),
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            // cortex_m::asm::wfi();
            continue;
        }
    }

    fn enqueue_frame(queue: &mut Queue<Frame, 32>, frame: Frame) {
        queue.enqueue(frame).expect("the frame queue is full");
        rtic::pend(Interrupt::USB_HP_CAN_TX);
    }

    const fn can_id(id: shared::Id) -> bxcan::StandardId {
        if let Some(id) = bxcan::StandardId::new(id as u16) {
            id
        } else {
            panic!("RASPI_ID is not a valid standard CAN ID");
        }
    }

    /// Runs every 10 ms.
    #[task(
        priority = 3,
        local = [adc1, analog_pins],
        shared = [can_tx_queue, &fast_sensing_ms_remaining],
    )]
    fn fast_sensor_poll(cx: fast_sensor_poll::Context, instant: Instant) {
        let fast_sensor_poll::LocalResources { adc1, analog_pins } = cx.local;
        let fast_sensor_poll::SharedResources {
            mut can_tx_queue,
            fast_sensing_ms_remaining,
        } = cx.shared;

        let next_instant = instant + 10.millis();
        fast_sensor_poll::spawn_at(next_instant, next_instant).unwrap();

        if fast_sensing_ms_remaining.load(Ordering::SeqCst) < 10 {
            return;
        }

        defmt::info!("fast_sensor_poll");
        
        fn read_adc(adc: &mut Adc<ADC1>, pin: &mut impl Channel<ADC1, ID = u8>) -> Result<u16, ()> {
            let mut acc = 0u32;
            for _ in 0..4 {
                let sample: u32 = adc.read(pin).map_err(|_| ())?;
                acc += sample;
            }

            Ok((acc / 4) as u16)
        }

        let s = |res| match res {
            Ok(v) => SensorReading::new(v),
            Err(_) => SensorReading::new_error(SensorError::Unknown),
        };

        let fm_f = s(read_adc(adc1, &mut analog_pins.flow1));
        let fm_o = s(read_adc(adc1, &mut analog_pins.flow2));

        fn pt_to_psig(reading: u16) -> Pressure {
            // The reading should range from ~496 to ~2480, which corresponds
            // to 0 to 5000 psig.
            // The line between those points is
            //      y = (625 x)/248 - 1250

            match (((reading as u32) * 625) / 248).saturating_sub(1250).try_into() {
                Ok(psig) => Pressure::new(psig),
                Err(_) => Pressure::new_error(SensorError::OutOfRange),
            }
        }

        let pt = |res: Result<u16, ()>| match res {
            Ok(v) => pt_to_psig(v),
            Err(_) => Pressure::new_error(SensorError::Unknown),
        };

        let pres1 = pt(read_adc(adc1, &mut analog_pins.pressure1));
        let pres2 = pt(read_adc(adc1, &mut analog_pins.pressure2));
        let pres3 = pt(read_adc(adc1, &mut analog_pins.pressure3));
        // let pres4 = pt(read_adc(adc1, &mut analog_pins.pressure4));
        let pres5 = pt(read_adc(adc1, &mut analog_pins.pressure5));
        let pres6 = pt(read_adc(adc1, &mut analog_pins.pressure6));
        // let pres7 = pt(read_adc(adc1, &mut analog_pins.pressure7));
        let pres8 = pt(read_adc(adc1, &mut analog_pins.pressure8));
        let pres9 = pt(read_adc(adc1, &mut analog_pins.pressure9));

        let pressure1_data = pi_sensor::Pressure1 {
            pt1_f: pres1,
            pt2_f: pres2,
            pt1_e: pres3,
            pt2_o: pres5,
        };

        let pressure2_data = pi_sensor::Pressure2 {
            pt3_o: pres6,
            pt4_o: pres8,
            pt2_p: pres9,
        };

        let flow = pi_sensor::Flow {
            fm_f,
            fm_o,
        };

        let mut buf = [0u8; 8];

        macro_rules! serialize {
            ($id:expr, $data:expr) => {{
                let slice = postcard::to_slice(&$data, &mut buf).expect("failed to serialize");
                bxcan::Frame::new_data(can_id($id), bxcan::Data::new(slice).unwrap())
            }};
        }

        let frame1 = serialize!(Id::RaspiPressure1, pressure1_data);
        let frame2 = serialize!(Id::RaspiPressure2, pressure2_data);
        let frame3 = serialize!(Id::RaspiFlow, flow);

        can_tx_queue.lock(|tx| {
            enqueue_frame(tx, frame1);
            enqueue_frame(tx, frame2);
            enqueue_frame(tx, frame3);
        });

        fast_sensing_ms_remaining.fetch_sub(10, Ordering::SeqCst);

        defmt::info!("fast_sensor_poll done");
    }

    /// Runs every 20 ms.
    #[task(
        priority = 2,
        local = [dual_hx711],
        shared = [can_tx_queue, &medium_sensing_ms_remaining],
    )]
    fn medium_sensor_poll(cx: medium_sensor_poll::Context, instant: Instant) {
        let medium_sensor_poll::LocalResources { dual_hx711 } = cx.local;
        let medium_sensor_poll::SharedResources {
            mut can_tx_queue,
            medium_sensing_ms_remaining,
        } = cx.shared;

        let next_instant = instant + 20.millis();
        medium_sensor_poll::spawn_at(next_instant, next_instant).unwrap();

        if medium_sensing_ms_remaining.load(Ordering::SeqCst) < 20 {
            return;
        }

        defmt::info!("medium_sensor_poll");

        let samples = dual_hx711
            .as_mut()
            .map(|dual_hx711| dual_hx711.retrieve())
            .transpose()
            .ok()
            .flatten();

        if let Some((load1, load2)) = samples {
            let load1 = if let Ok(load1) = load1.try_into() {
                Force::new(load1)
            } else {
                Force::new_error(SensorError::OutOfRange)
            };
            let load2 = if let Ok(load2) = load2.try_into() {
                Force::new(load2)
            } else {
                Force::new_error(SensorError::OutOfRange)
            };

            let load = pi_sensor::Load {
                load1,
                load2,
            };

            let mut buf = [0u8; 8];

            macro_rules! serialize {
                ($id:expr, $data:expr) => {{
                    let slice = postcard::to_slice(&$data, &mut buf).expect("failed to serialize");
                    bxcan::Frame::new_data(can_id($id), bxcan::Data::new(slice).unwrap())
                }};
            }

            let frame = serialize!(Id::RaspiLoad, load);
            can_tx_queue.lock(|tx_queue| {
                enqueue_frame(tx_queue, frame);
            });
        }

        medium_sensing_ms_remaining.fetch_sub(20, Ordering::SeqCst);
        defmt::info!("medium_sensor_poll done");
    }

    /// Runs every 50 ms.
    #[task(
        priority = 1,
        local = [thermocouples],
        shared = [can_tx_queue, &slow_sensing_ms_remaining],
    )]
    fn slow_sensor_poll(cx: slow_sensor_poll::Context, instant: Instant) {
        let slow_sensor_poll::LocalResources { thermocouples } = cx.local;
        let slow_sensor_poll::SharedResources {
            mut can_tx_queue,
            slow_sensing_ms_remaining,
        } = cx.shared;

        let next_instant = instant + 50.millis();
        slow_sensor_poll::spawn_at(next_instant, next_instant).unwrap();

        if slow_sensing_ms_remaining.load(Ordering::SeqCst) < 50 {
            return;
        }

        defmt::info!("slow_sensor_poll");

        // Thermocouples

        let c = |opt: Option<_>| {
            opt.map(|res| match res {
                Ok(v) => Temperature::new(v),
                Err(_) => Temperature::new_error(SensorError::Unknown),
            })
        };

        let t1 = c(thermocouples.thermo1.as_mut().map(|t1| t1.read()))
            .unwrap_or(Temperature::new_error(SensorError::NoData));
        // let t2 = c(thermocouples.thermo2.as_mut().map(|t2| t2.read()))
        //     .unwrap_or(Temperature::new_error(SensorError::NoData));
        let t3 = c(thermocouples.thermo3.as_mut().map(|t3| t3.read()))
            .unwrap_or(Temperature::new_error(SensorError::NoData));
        let t4 = c(thermocouples.thermo4.as_mut().map(|t4| t4.read()))
            .unwrap_or(Temperature::new_error(SensorError::NoData));
        let t5 = c(thermocouples.thermo5.as_mut().map(|t5| t5.read()))
            .unwrap_or(Temperature::new_error(SensorError::NoData));
        let t6 = c(thermocouples.thermo6.as_mut().map(|t6| t6.read()))
            .unwrap_or(Temperature::new_error(SensorError::NoData));

        let mut buf = [0u8; 8];

        macro_rules! serialize {
            ($id:expr, $data:expr) => {{
                let slice = postcard::to_slice(&$data, &mut buf).expect("failed to serialize");
                bxcan::Frame::new_data(can_id($id), bxcan::Data::new(slice).unwrap())
            }};
        }

        let thermo1 = pi_sensor::Thermo1 {
            tc1_e: t1,
            tc1_f: t3,
            tc2_f: t4,
            tc1_o: t5
        };
        let thermo2 = pi_sensor::Thermo2 { tc5_o: t6 };

        let frame1 = serialize!(Id::RaspiThermo1, thermo1);
        let frame2 = serialize!(Id::RaspiThermo2, thermo2);

        can_tx_queue.lock(|tx_queue| {
            enqueue_frame(tx_queue, frame1);
            enqueue_frame(tx_queue, frame2);
        });

        slow_sensing_ms_remaining.fetch_sub(50, Ordering::SeqCst);

        defmt::info!("slow_sensor_poll done");
    }

    // #[task(
    //     local = [adc1, analog_pins, thermocouples],
    //     // shared = [sensor_data]
    // )]
    // fn sensor_poll(cx: sensor_poll::Context, instant: Instant) {
    //     let next_instant = instant + Duration::millis(10);
    //     sensor_poll::spawn_at(next_instant, next_instant).unwrap();

    //     // read_adc1::spawn().expect("failed to spawn task `poll_adc1`");
    //     // read_thermocouples::spawn().expect("failed to spawn task `poll_thermocouples`");

    //     let sensor_poll::LocalResources { adc1, analog_pins, thermocouples } = cx.local;
    //     let sensor_poll::SharedResources {
    //         mut sensor_data,
    //     } = cx.shared;

    //     fn read_adc(adc: &mut Adc<ADC1>, pin: &mut impl Channel<ADC1, ID = u8>) -> Result<u16, ()> {
    //         let mut acc = 0u32;
    //         for _ in 0..4 {
    //             let sample: u32 = adc.read(pin).map_err(|_| ())?;
    //             acc += sample;
    //         }

    //         Ok((acc / 4) as u16)
    //     }

    //     let s = |res| match res {
    //         Ok(v) => SensorReading::new(v),
    //         Err(_) => SensorReading::new_error(SensorError::Unknown),
    //     };

    //     let l = |res| match res {
    //         Ok(v) => Force::new(v),
    //         Err(_) => Force::new_error(SensorError::Unknown),
    //     };

    //     // This is the source of truth for these sensor to pin mappings.

    //     let fm_f = s(read_adc(adc1, &mut analog_pins.flow1));
    //     let fm_o = s(read_adc(adc1, &mut analog_pins.flow2));

    //     // TODO: Do conversions into newtons.
    //     let load1 = l(read_adc(adc1, &mut analog_pins.load_cell1));
    //     let load2 = l(read_adc(adc1, &mut analog_pins.load_cell2));

    //     fn pt_to_psig(reading: u16) -> Pressure {
    //         // The reading should range from ~496 to ~2480, which corresponds
    //         // to 0 to 5000 psig.
    //         // The line between those points is
    //         //      y = (625 x)/248 - 1250

    //         match (((reading as u32) * 625) / 248).saturating_sub(1250).try_into() {
    //             Ok(psig) => Pressure::new(psig),
    //             Err(_) => Pressure::new_error(SensorError::OutOfRange),
    //         }
    //     }

    //     let pt = |res: Result<u16, ()>| match res {
    //         Ok(v) => pt_to_psig(v),
    //         Err(_) => Pressure::new_error(SensorError::Unknown),
    //     };

    //     let pres1 = pt(read_adc(adc1, &mut analog_pins.pressure1));
    //     let pres2 = pt(read_adc(adc1, &mut analog_pins.pressure2));
    //     let pres3 = pt(read_adc(adc1, &mut analog_pins.pressure3));
    //     let pres4 = pt(read_adc(adc1, &mut analog_pins.pressure4));
    //     let pres5 = pt(read_adc(adc1, &mut analog_pins.pressure5));
    //     let pres6 = pt(read_adc(adc1, &mut analog_pins.pressure6));
    //     let pres7 = pt(read_adc(adc1, &mut analog_pins.pressure7));
    //     let pres8 = pt(read_adc(adc1, &mut analog_pins.pressure8));
    //     let pres9 = pt(read_adc(adc1, &mut analog_pins.pressure9));

    //     // --------------------------------------------------------------------
    //     // Thermocouples

    //     let c = |opt: Option<_>| {
    //         opt.map(|res| match res {
    //             Ok(v) => Temperature::new(v),
    //             Err(_) => Temperature::new_error(SensorError::Unknown),
    //         })
    //     };

    //     // This is the source of truth for these sensor to pin mappings.

    //     let t1 = c(thermocouples.thermo1.as_mut().map(|t1| t1.read()));
    //     let t2 = c(thermocouples.thermo2.as_mut().map(|t2| t2.read()));
    //     let t3 = c(thermocouples.thermo3.as_mut().map(|t3| t3.read()));
    //     let t4 = c(thermocouples.thermo4.as_mut().map(|t4| t4.read()));
    //     let t5 = c(thermocouples.thermo5.as_mut().map(|t5| t5.read()));
    //     let t6 = c(thermocouples.thermo6.as_mut().map(|t6| t6.read()));

    //     sensor_data.lock(|sensor_data| {
    //         sensor_data.fm_f = fm_f;
    //         sensor_data.fm_o = fm_o;

    //         sensor_data.load1 = load1;
    //         sensor_data.load2 = load2;

    //         sensor_data.pt1_f = pres1;
    //         sensor_data.pt2_f = pres2;
    //         sensor_data.pt1_e = pres3;
    //         sensor_data.pt2_o = pres4;
    //         sensor_data.pres5 = pres5;
    //         sensor_data.pt3_o = pres6;
    //         sensor_data.pres7 = pres7;
    //         sensor_data.pt4_o = pres8;
    //         sensor_data.pt2_p = pres9;

    //         sensor_data.tc1_e = t1.unwrap_or(Temperature::new_error(SensorError::NoData));
    //         sensor_data.thermo2 = t2.unwrap_or(Temperature::new_error(SensorError::NoData));
    //         sensor_data.tc1_f = t3.unwrap_or(Temperature::new_error(SensorError::NoData));
    //         sensor_data.tc2_f = t4.unwrap_or(Temperature::new_error(SensorError::NoData));
    //         sensor_data.tc1_o = t5.unwrap_or(Temperature::new_error(SensorError::NoData));
    //         sensor_data.tc5_o = t6.unwrap_or(Temperature::new_error(SensorError::NoData));
    //     });
    // }

    /// This is fired every time the CAN controller has finished a frame transmission
    /// or after the USB_HP_CAN_TX ISR is pended.
    #[task(
        binds = USB_HP_CAN_TX,
        priority = 4,
        local = [can_tx],
        shared = [can_tx_queue]
    )]
    fn can_tx(cx: can_tx::Context) {
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
                            // A lower priority frame was replaced with our high priority frame.
                            // Put the low priority frame back in the transmit queue.
                            tx_queue.dequeue();
                            enqueue_frame(tx_queue, pending_frame.clone());
                        }
                    },
                    Err(nb::Error::WouldBlock) => {
                        break;
                    },
                    Err(_) => unreachable!(),
                }
            }
        });
    }

    // #[task(shared = [can_tx_queue, sensor_data, battery_millivolts], priority = 2)]
    // fn send_sensor_data(cx: send_sensor_data::Context) {
    //     let send_sensor_data::SharedResources {
    //         mut can_tx_queue,
    //         sensor_data,
    //         battery_millivolts,
    //     } = cx.shared;

    //     let (sensor_data, _battery_millivolts) =
    //         (sensor_data, battery_millivolts).lock(|data, bat| (data.clone(), *bat));

    //     let mut buf = [0; pi_sensor::AllSensors::POSTCARD_MAX_SIZE];
    //     if let Err(_) = postcard::to_slice(&sensor_data, &mut buf) {
    //         defmt::error!("Could not serialize sensor data");
    //         return;
    //     }

    //     can_tx_queue.lock(|tx_queue| {
    //         let mut chunks = buf.chunks(8);

    //         for chunk in &mut chunks {
    //             assert_eq!(chunk.len(), 8);
    //             let data = bxcan::Data::new(chunk).unwrap();
    //             let frame = bxcan::Frame::new_data(RASPI_ID, data);
    //             enqueue_frame(tx_queue, frame);
    //         }
    //     });
    // }

    #[task(
        binds = USB_LP_CAN_RX0,
        priority = 4,
        local = [
            can_rx,
        ],
        shared = [
            &fast_sensing_ms_remaining,
            &slow_sensing_ms_remaining,
        ]
    )]
    fn can_rx0(cx: can_rx0::Context) {
        let can_rx0::LocalResources { can_rx } = cx.local;
        let can_rx0::SharedResources {
            fast_sensing_ms_remaining,
            slow_sensing_ms_remaining,
        } = cx.shared;

        loop {
            match can_rx.receive() {
                Ok(frame) => {
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
                        pi_sensor::Request::StartSensing => {
                            fast_sensing_ms_remaining.store(1000, Ordering::SeqCst);
                            slow_sensing_ms_remaining.store(1000, Ordering::SeqCst);
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
