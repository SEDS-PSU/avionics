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

const COMMAND_TIMEOUT: Duration = Duration::millis(5);

const RASPI_ID: bxcan::StandardId = if let Some(id) = bxcan::StandardId::new(shared::RASPI_ID) {
    id
} else {
    panic!("RASPI_ID is not a valid standard CAN ID");
};
const OUTPUT_BOARD_ID: bxcan::StandardId = if let Some(id) = bxcan::StandardId::new(shared::OUTPUT_BOARD_ID) {
    id
} else {
    panic!("OUTPUT_ID is not a valid standard CAN ID");
};

#[rtic::app(device = stm32f1xx_hal::pac, dispatchers = [EXTI0])]
mod app {
    use core::mem;

    use crate::{Duration, OUTPUT_BOARD_ID, FREQUENCY};
    use bxcan::{filter::Mask32, Frame, Interrupts, Rx, Tx, StandardId};
    use dwt_systick_monotonic::DwtSystick;
    use heapless::{spsc::Queue, Deque};
    use shared::{pi_output, PackedValves, ThreeWay, TwoWay, Valves};
    use stm32f1xx_hal::{
        can::Can,
        gpio::{
            gpioa::PA1,
            gpiob::{PB0, PB1, PB10, PB11, PB12, PB15, PB2, PB3, PB4, PB5, PB6, PB7},
            Edge, ExtiPin, Input, Output, PinState, PullDown, PullUp, PushPull,
        },
        pac::{Interrupt, CAN1},
        prelude::*,
    };
    use postcard::MaxSize;

    use crate::{Instant, COMMAND_TIMEOUT, RASPI_ID};

    // The monotonic scheduler type
    #[monotonic(binds = SysTick, default = true)]
    type Mono = DwtSystick<FREQUENCY /* Hz */>;

    pub enum CommandState {
        Loading,
        Executing {
            handle: Option<execute_commands::SpawnHandle>,
            ignition_delay: Option<u16>,
        },
    }

    pub struct ActuationPins {
        solenoid1: PB1<Output<PushPull>>,
        solenoid2: PB2<Output<PushPull>>,
        solenoid3: PB3<Output<PushPull>>,
        solenoid4: PB4<Output<PushPull>>,
        solenoid5: PB5<Output<PushPull>>,
        solenoid6: PB6<Output<PushPull>>,
        solenoid7: PB7<Output<PushPull>>,
        solenoid8: PB0<Output<PushPull>>,
        // solenoid9: PB8<Output<PushPull>>,
        // solenoid10: PB9<Output<PushPull>>,

        // These are walled-off by the arming mosfet.
        main_fuel_solenoid: PB12<Output<PushPull>>,
        main_oxidizer_solenoid: PB11<Output<PushPull>>,
    }

    // Shared resources go here
    #[shared]
    struct Shared {
        valve_states: Option<Valves>,

        /// CAN frame queue
        /// This has a capacity of 3, according to the documentation.
        /// I don't think this will ever have more than one item in it
        /// at a time, but it's a good idea to have a small slack.
        can_tx_queue: Queue<Frame, 4>,

        arming_switch: PB15<Input<PullDown>>,
        ignition_detection: PA1<Input<PullUp>>,

        /// The `#[lock_free]` attribute makes sure that access to this
        /// is always deterministic (since only tasks at a single priority
        /// can access it).
        #[lock_free]
        commands: (CommandState, &'static mut Deque<pi_output::Command, 256>),
    }

    // Local resources go here
    #[local]
    struct Local {
        can_tx: Tx<Can<CAN1>>,
        can_rx: Rx<Can<CAN1>>,

        commands_start: Option<Instant>,
        commands_count: u8,

        actuation_pins: ActuationPins,
        /// This is walled-off by the arming mosfet.
        igniter_pin: PB10<Output<PushPull>>,
    }

    #[init(local = [command_list: Deque<pi_output::Command, 256> = Deque::new()])]
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

        // The pb3 and pb4 pins are used by the JTAG debugger initially.
        // We need to disable the JTAG peripheral to use them.
        let (_, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        let arming_switch = gpiob.pb15.into_pull_down_input(&mut gpiob.crh);
        let mut ignition_detection = gpioa.pa1.into_pull_up_input(&mut gpioa.crl);

        let actuation_pins = ActuationPins {
            solenoid1: gpiob.pb1.into_push_pull_output(&mut gpiob.crl),
            solenoid2: gpiob.pb2.into_push_pull_output(&mut gpiob.crl),
            solenoid3: pb3.into_push_pull_output(&mut gpiob.crl),
            solenoid4: pb4.into_push_pull_output(&mut gpiob.crl),
            solenoid5: gpiob.pb5.into_push_pull_output(&mut gpiob.crl),
            solenoid6: gpiob.pb6.into_push_pull_output(&mut gpiob.crl),
            solenoid7: gpiob.pb7.into_push_pull_output(&mut gpiob.crl),
            solenoid8: gpiob.pb0.into_push_pull_output(&mut gpiob.crl),
            // solenoid9: gpiob.pb8.into_push_pull_output(&mut gpiob.crh),
            // solenoid10: gpiob.pb9.into_push_pull_output(&mut gpiob.crh),
            main_fuel_solenoid: gpiob.pb12.into_push_pull_output(&mut gpiob.crh),
            main_oxidizer_solenoid: gpiob.pb11.into_push_pull_output(&mut gpiob.crh),
        };

        let igniter_pin = gpiob.pb10.into_push_pull_output(&mut gpiob.crh);

        // Set up the ignition detection interrupt handler.
        // This will trigger on EXTI1.
        ignition_detection.make_interrupt_source(&mut afio);
        ignition_detection.enable_interrupt(&mut exti);
        ignition_detection.trigger_on_edge(&mut exti, Edge::Rising);

        let can_rx_pin = gpioa.pa11.into_floating_input(&mut gpioa.crh);
        let can_tx_pin = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);

        can.assign_pins((can_tx_pin, can_rx_pin), &mut afio.mapr);

        // APB1 (PCLK1): 16MHz, Bit rate: 1000kBit/s, Sample Point 87.5%
        // Value was calculated with http://www.bittiming.can-wiki.info/
        let mut can = bxcan::Can::builder(can)
            .set_bit_timing(0x001e0000)
            .leave_disabled();

        // Only recieve frames intended for the output board.
        can.modify_filters().enable_bank(0, Mask32::frames_with_std_id(OUTPUT_BOARD_ID, StandardId::MAX));

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
                // Initialization of shared resources go here
                valve_states: None,
                can_tx_queue: Queue::new(),

                arming_switch,
                ignition_detection,

                commands: (CommandState::Loading, cx.local.command_list),
            },
            Local {
                can_tx,
                can_rx,
                commands_start: None,
                commands_count: 0,
                actuation_pins,
                igniter_pin,
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

    fn enqueue_frame(queue: &mut Queue<Frame, 4>, frame: Frame) {
        queue.enqueue(frame).unwrap();
        rtic::pend(Interrupt::USB_HP_CAN_TX);
    }

    #[task(local = [igniter_pin])]
    fn ignite(cx: ignite::Context) {
        defmt::info!("igniting!");

        let igniter_pin = cx.local.igniter_pin;

        igniter_pin.set_high();
    }

    #[task(shared = [valve_states], local = [actuation_pins])]
    fn set_valves(cx: set_valves::Context, new_states: PackedValves) {
        defmt::info!("setting values states");

        let set_valves::SharedResources { mut valve_states } = cx.shared;
        let set_valves::LocalResources { actuation_pins } = cx.local;

        let new_states = new_states.into();

        valve_states.lock(|valve_states| *valve_states = Some(new_states));

        // This is the source of truth for mapping valves to output board connectors.

        fn a(v: TwoWay) -> PinState {
            match v {
                TwoWay::Open => PinState::Low,
                TwoWay::Closed => PinState::High,
            }
        }

        fn b(v: ThreeWay) -> PinState {
            match v {
                ThreeWay::NitrogenPathway => PinState::Low,
                ThreeWay::FuelOxidizer => PinState::High,
            }
        }

        actuation_pins.solenoid1.set_state(a(new_states.fc_fp));
        actuation_pins.solenoid2.set_state(a(new_states.fc_op));
        actuation_pins.solenoid3.set_state(a(new_states.fo_p));
        actuation_pins.solenoid4.set_state(a(new_states.fo_fp));
        actuation_pins.solenoid5.set_state(a(new_states.fc_p));
        actuation_pins.solenoid6.set_state(a(new_states.fc_op));
        actuation_pins.solenoid7.set_state(a(new_states.fc1_o));
        actuation_pins.solenoid8.set_state(a(new_states.fc2_o));

        actuation_pins
            .main_fuel_solenoid
            .set_state(b(new_states.pv_f));
        actuation_pins
            .main_oxidizer_solenoid
            .set_state(b(new_states.pv_o));
    }

    #[task(shared = [commands])]
    fn execute_commands(cx: execute_commands::Context) {
        let execute_commands::SharedResources {
            commands: (command_state, command_list),
        } = cx.shared;

        let (old_spawn_handle, ignition_delay) = match command_state {
            CommandState::Loading => return,
            CommandState::Executing {
                handle: maybe_spawn_handle,
                ignition_delay,
            } => (maybe_spawn_handle, ignition_delay),
        };

        *old_spawn_handle = if let Some(command) = command_list.pop_front() {
            match command {
                pi_output::Command::SetValves { states, wait } => {
                    defmt::info!("setting valves");
                    set_valves::spawn(states).expect("failed to spawn the `set_valves` task");
                    *ignition_delay = None;

                    if let pi_output::Wait::WaitMs(ms) = wait {
                        let handle =
                            execute_commands::spawn_after(Duration::millis(ms.get() as u32))
                                .expect("failed to spawn `execute_commands` task");
                        Some(handle)
                    } else {
                        None
                    }
                }
                pi_output::Command::Ignite { delay } => {
                    defmt::info!("Igniting the engine with a delay of {}ms", delay);
                    *ignition_delay = Some(delay);

                    ignite::spawn().expect("failed to spawn the `ignite` task");

                    None
                }
            }
        } else {
            *ignition_delay = None;
            None
        };
    }

    #[task(shared = [can_tx_queue, valve_states, &arming_switch, ignition_detection])]
    fn send_status(cx: send_status::Context) {
        let mut state = pi_output::State::new();
        let mut ignition_detect = cx.shared.ignition_detection;

        if cx.shared.arming_switch.is_high() {
            state = state.set_armed();
        }

        ignition_detect.lock(|ignition_detect| {
            if ignition_detect.is_high() {
                state = state.set_ignited();
            }
        });

        let mut states = cx.shared.valve_states;
        let states = states.lock(|states| states.map(Into::into));

        let status = pi_output::Status {
            states,
            state,
            error: None, // for now
        };

        let mut data = [0; pi_output::Status::POSTCARD_MAX_SIZE];

        if let Err(_) = postcard::to_slice(&status, &mut data) {
            defmt::error!("failed to serialize `pi_output::Status`");
            return;
        }

        let frame = bxcan::Frame::new_data(RASPI_ID, data);

        let mut tx_queue = cx.shared.can_tx_queue;

        tx_queue.lock(|tx_queue| {
            enqueue_frame(tx_queue, frame);
        });
    }

    enum CanRxError {
        Postcard(postcard::Error),
        WrongState,
    }

    impl From<postcard::Error> for CanRxError {
        fn from(e: postcard::Error) -> Self {
            CanRxError::Postcard(e)
        }
    }

    fn process_can_rx(
        data: &[u8],
        command_state: &mut CommandState,
        command_list: &mut Deque<pi_output::Command, 256>,
        commands_start: &mut Option<Instant>,
        commands_count: &mut u8,
    ) -> Result<(), CanRxError> {
        if let Some(timeout_begin) = *commands_start {
            if monotonics::now() - timeout_begin < COMMAND_TIMEOUT
                && *commands_count > 0
            {
                if let CommandState::Loading = command_state {
                    // We're in the wrong state.
                    defmt::error!(
                        "attempted to load commands while executing commands"
                    );
                    return Err(CanRxError::WrongState);
                }

                assert!(*commands_count > 0);

                let command = postcard::from_bytes(data)?;

                // This can't overflow, since the length is a max of 255.
                command_list.push_back(command).unwrap();
                *commands_count -= 1;

                return Ok(())
            } else {
                // We timed out, so let's assume this frame is a request.
                *commands_start = None;
            }
        }

        // Assume this frame is a request.
        let request = postcard::from_bytes(data)?;

        match request {
            pi_output::Request::GetStatus => {
                send_status::spawn().expect("failed to spawn the `send_status` task")
            }
            pi_output::Request::SetValvesImmediately(new_states) => {
                defmt::info!("setting valve states");

                set_valves::spawn(new_states)
                    .expect("failed to spawn the `set_valves` task");

                // Kill the currently executing list of commands.
                if let CommandState::Executing {
                    handle: Some(handle),
                    ..
                } = mem::replace(command_state, CommandState::Loading)
                {
                    defmt::info!("killing the currently executing command task");
                    let _ = handle.cancel();
                }
                command_list.clear();
            }
            pi_output::Request::BeginSequence { length } => {
                defmt::info!("beginning a sequence of {} commands", length);
                *commands_start = Some(monotonics::now());
                *commands_count = length;

                // Kill the currently executing list of commands.
                if let CommandState::Executing {
                    handle: Some(handle),
                    ..
                } = mem::replace(command_state, CommandState::Loading)
                {
                    defmt::info!("killing the currently executing command task");
                    let _ = handle.cancel();
                }
                command_list.clear();
            }
            pi_output::Request::Reset => {
                defmt::info!("resetting");
                defmt::flush();
                cortex_m::peripheral::SCB::sys_reset()
            }
        }

        Ok(())
    }

    #[task(binds = USB_LP_CAN_RX0,
        local = [
            can_rx,
            commands_start,
            commands_count,
        ],
        shared = [
            commands,
        ]
    )]
    fn can_rx0(cx: can_rx0::Context) {
        let (command_state, command_list) = cx.shared.commands;
        let can_rx0::LocalResources {
            can_rx,
            commands_start,
            commands_count,
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

                    match process_can_rx(
                        data,
                        command_state,
                        command_list,
                        commands_start,
                        commands_count,
                    ) {
                        Ok(_) => (),
                        Err(CanRxError::Postcard(_)) => {
                            defmt::error!("failed to deserialize frame");
                        }
                        Err(CanRxError::WrongState) => {
                            defmt::error!("received a command in the wrong state");
                        }
                    }
                    
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => {} // Ignore overrun errors.
            }
        }
    }

    #[task(binds = EXTI1, shared = [ignition_detection, commands])]
    fn ignition_detect(cx: ignition_detect::Context) {
        let ignition_detect::SharedResources {
            mut ignition_detection,
            commands: (command_state, _),
        } = cx.shared;

        ignition_detection.lock(|ignition_detection| {
            ignition_detection.clear_interrupt_pending_bit();

            defmt::info!("ignition detection triggered");

            match command_state {
                CommandState::Executing {
                    handle,
                    ignition_delay,
                } => {
                    if let Some(delay) = *ignition_delay {
                        assert!(
                            handle.is_none(),
                            "There should be no execute command task scheduled"
                        );

                        // Now that ignition has been detected, schedule the next command to run in `delay` ms.
                        *handle = Some(
                            execute_commands::spawn_after(Duration::millis(delay as u32))
                                .expect("failed to spawn the `schedule_command` task"),
                        );
                        *ignition_delay = None;
                    }
                }
                _ => {}
            }
        });
    }
}
