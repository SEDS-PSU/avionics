#![no_main]
#![no_std]

use defmt_rtt as _;
use dwt_systick_monotonic::fugit;
// global logger
use panic_probe as _;
use stm32f1xx_hal as _; // memory layout

mod util;

const FREQUENCY: u32 = 36_000_000; // Hz

// type Instant = fugit::Instant<u32, 1, FREQUENCY>;
type Duration = fugit::Duration<u32, 1, FREQUENCY>;

const RASPI_ID_OUTPUT_STATUS: bxcan::StandardId = if let Some(id) = bxcan::StandardId::new(shared::Id::RaspiOutputStatus as u16) {
    id
} else {
    panic!("RASPI_ID is not a valid standard CAN ID");
};
const OUTPUT_BOARD_ID: bxcan::StandardId =
    if let Some(id) = bxcan::StandardId::new(shared::Id::OutputBoard as u16) {
        id
    } else {
        panic!("OUTPUT_ID is not a valid standard CAN ID");
    };

#[rtic::app(device = stm32f1xx_hal::pac, dispatchers = [EXTI0])]
mod app {
    use core::num::NonZeroU8;

    use crate::{Duration, FREQUENCY, OUTPUT_BOARD_ID};
    use bxcan::{filter::Mask32, Frame, Interrupts, Rx, StandardId, Tx};
    use dwt_systick_monotonic::DwtSystick;
    use heapless::{spsc::Queue, Deque};
    use postcard::MaxSize;
    use shared::{pi_output, PackedValves, ThreeWay, TwoWay, Valves};
    use stm32f1xx_hal::{
        can::Can,
        gpio::{
            gpiob::{PB0, PB1, PB10, PB11, PB12, PB15, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9},
            gpiod::PD12,
            Edge, ExtiPin, Input, Output, PinState, PullDown, PushPull,
        },
        pac::{Interrupt, CAN1},
        // rcc::{self, HPre, PPre},
        prelude::*,
    };

    use crate::RASPI_ID_OUTPUT_STATUS;

    // The monotonic scheduler type
    #[monotonic(binds = SysTick, default = true)]
    type Mono = DwtSystick<FREQUENCY /* Hz */>;

    pub enum CommandState {
        Loading {
            commands_remaining: NonZeroU8,
        },
        Executing {
            handle: Option<execute_commands::SpawnHandle>,
            wait_for_ignition: bool,
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
        solenoid9: PB8<Output<PushPull>>,
        solenoid10: PB9<Output<PushPull>>,

        // These are walled-off by the arming mosfet.
        main_fuel_solenoid: PB10<Output<PushPull>>,
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
        ignition_detection: PD12<Input<PullDown>>,

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

        actuation_pins: ActuationPins,

        /// This is walled-off by the arming mosfet.
        igniter_pin: PB12<Output<PushPull>>,
    }

    #[init(local = [command_list: Deque<pi_output::Command, 256> = Deque::new()])]
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
        let mut exti = cx.device.EXTI;

        let clocks = rcc
            .cfgr
            .use_hse(36.mhz())
            // do we need other stuff here?
            .sysclk(FREQUENCY.hz())
            .pclk1(9.mhz())
            .freeze(&mut flash.acr);

        // Set up CAN bus.
        // Based on https://github.com/stm32-rs/stm32f1xx-hal/blob/master/examples/can-rtic.rs.
        let can = Can::new(cx.device.CAN1);

        // Pins
        let mut afio = cx.device.AFIO.constrain();
        let mut gpioa = cx.device.GPIOA.split();
        let mut gpiob = cx.device.GPIOB.split();
        let mut gpiod = cx.device.GPIOD.split();

        // The pb3 and pb4 pins are used by the JTAG debugger initially.
        // We need to disable the JTAG peripheral to use them.
        let (_, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        let actuation_pins = ActuationPins {
            solenoid1: gpiob.pb1.into_push_pull_output(&mut gpiob.crl),
            solenoid2: gpiob.pb2.into_push_pull_output(&mut gpiob.crl),
            solenoid3: pb3.into_push_pull_output(&mut gpiob.crl),
            solenoid4: pb4.into_push_pull_output(&mut gpiob.crl),
            solenoid5: gpiob.pb5.into_push_pull_output(&mut gpiob.crl),
            solenoid6: gpiob.pb6.into_push_pull_output(&mut gpiob.crl),
            solenoid7: gpiob.pb7.into_push_pull_output(&mut gpiob.crl),
            solenoid8: gpiob.pb0.into_push_pull_output(&mut gpiob.crl),
            solenoid9: gpiob.pb8.into_push_pull_output(&mut gpiob.crh),
            solenoid10: gpiob.pb9.into_push_pull_output(&mut gpiob.crh),
            main_fuel_solenoid: gpiob.pb10.into_push_pull_output(&mut gpiob.crh),
            main_oxidizer_solenoid: gpiob.pb11.into_push_pull_output(&mut gpiob.crh),
        };

        let igniter_pin = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);

        let arming_switch = gpiob.pb15.into_pull_down_input(&mut gpiob.crh);
        let mut ignition_detection = gpiod.pd12.into_pull_down_input(&mut gpiod.crh);

        // Set up the ignition detection interrupt handler.
        // This will trigger on EXTI1.
        ignition_detection.make_interrupt_source(&mut afio);
        ignition_detection.enable_interrupt(&mut exti);
        ignition_detection.trigger_on_edge(&mut exti, Edge::Falling);

        let can_rx_pin = gpioa.pa11.into_floating_input(&mut gpioa.crh);
        let can_tx_pin = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);

        can.assign_pins((can_tx_pin, can_rx_pin), &mut afio.mapr);

        // APB1 (PCLK1): 9MHz, Bit rate: 500kBit/s, Sample Point 87.5%
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

        let mono = DwtSystick::new(&mut dcb, dwt, systick, clocks.sysclk().0);

        (
            Shared {
                // Initialization of shared resources go here
                valve_states: None,
                can_tx_queue: Queue::new(),

                arming_switch,
                ignition_detection,

                commands: (
                    CommandState::Executing {
                        handle: None,
                        wait_for_ignition: false,
                    },
                    cx.local.command_list,
                ),
            },
            Local {
                can_tx,
                can_rx,
                actuation_pins,
                igniter_pin,
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

    /// This is fired every time the CAN controller has finished a frame transmission
    /// or after the USB_HP_CAN_TX ISR is pended.
    #[task(binds = USB_HP_CAN_TX, local = [can_tx], shared = [can_tx_queue])]
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
    fn igniter(cx: igniter::Context, igniter: pi_output::Igniter) {
        let igniter_pin = cx.local.igniter_pin;

        match igniter {
            pi_output::Igniter::Activate => {
                igniter_pin.set_high();
            }
            pi_output::Igniter::Deactivate => {
                igniter_pin.set_low();
            }
        }

        execute_commands::spawn().unwrap();
    }

    #[task(shared = [valve_states], local = [actuation_pins])]
    fn set_valves(cx: set_valves::Context, new_states: PackedValves) {
        defmt::info!("setting values states");

        let set_valves::SharedResources { mut valve_states } = cx.shared;
        let set_valves::LocalResources { actuation_pins } = cx.local;

        let new_states = new_states.into();

        valve_states.lock(|valve_states| *valve_states = Some(new_states));

        // This is the source of truth for mapping valves to output board connectors.
        fn fc(v: TwoWay) -> PinState {
            match v {
                TwoWay::Open => PinState::High,
                TwoWay::Closed => PinState::Low,
            }
        }

        fn fo(v: TwoWay) -> PinState {
            match v {
                TwoWay::Open => PinState::Low,
                TwoWay::Closed => PinState::High,
            }
        }

        fn tw(v: ThreeWay) -> PinState {
            match v {
                ThreeWay::NitrogenPathway => PinState::Low,
                ThreeWay::FuelOxidizer => PinState::High,
            }
        }

        actuation_pins.solenoid1.set_state(fo(new_states.fo_fp));
        actuation_pins.solenoid2.set_state(fc(new_states.fc_fp));
        actuation_pins.solenoid3.set_state(fc(new_states.fc_p));
        actuation_pins.solenoid4.set_state(fc(new_states.fc1_f));
        actuation_pins.solenoid5.set_state(fo(new_states.fo_p1));
        actuation_pins.solenoid6.set_state(fo(new_states.fo2_o));
        actuation_pins.solenoid7.set_state(fc(new_states.fc4_o));
        actuation_pins.solenoid8.set_state(fc(new_states.fc3_o));
        actuation_pins.solenoid9.set_low();
        actuation_pins.solenoid10.set_low();

        actuation_pins
            .main_fuel_solenoid
            .set_state(tw(new_states.pv_f));
        actuation_pins
            .main_oxidizer_solenoid
            .set_state(tw(new_states.pv_o));

        execute_commands::spawn().unwrap();
    }

    #[task(shared = [commands])]
    fn execute_commands(cx: execute_commands::Context) {
        let execute_commands::SharedResources {
            commands: (command_state, command_list),
        } = cx.shared;

        if let CommandState::Loading { .. } = command_state {
            defmt::warn!("`execute_commands` ran while the system is in the command loading state");
            return
        }

        if let Some(command) = command_list.pop_front() {
            defmt::info!("Executing command: {:#?}", command);
            match command {
                pi_output::Command::SetValves(states) => {
                    set_valves::spawn(states).expect("failed to spawn the `set_valves` task");
                }
                pi_output::Command::Igniter(igniter) => {
                    igniter::spawn(igniter).expect("failed to spawn the `igniter` task");
                }
                pi_output::Command::WaitForIgnitionDetected => {
                    *command_state = CommandState::Executing {
                        handle: None,
                        wait_for_ignition: true,
                    };
                }
                pi_output::Command::Wait(delay) => {
                    let handle = execute_commands::spawn_after(Duration::millis(delay.get().into())).unwrap();
                    *command_state = CommandState::Executing {
                        handle: Some(handle),
                        wait_for_ignition: false,
                    };
                }
            }
        }
    }

    #[task(shared = [can_tx_queue, valve_states, &arming_switch, ignition_detection])]
    fn send_status(cx: send_status::Context) {
        let mut state = pi_output::State::new();
        let mut ignition_detect = cx.shared.ignition_detection;

        if cx.shared.arming_switch.is_high() {
            state = state.set_armed();
        }

        ignition_detect.lock(|ignition_detect| {
            if ignition_detect.is_low() {
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

        let frame = bxcan::Frame::new_data(RASPI_ID_OUTPUT_STATUS, data);

        let mut tx_queue = cx.shared.can_tx_queue;

        tx_queue.lock(|tx_queue| {
            enqueue_frame(tx_queue, frame);
        });
    }

    enum CanRxError {
        Postcard(postcard::Error),
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
    ) -> Result<(), CanRxError> {
        match command_state {
            CommandState::Loading { commands_remaining } => {
                let command = postcard::from_bytes(data)?;

                command_list.push_back(command).unwrap();
                if let Some(remaining) = NonZeroU8::new(commands_remaining.get() - 1) {
                    *commands_remaining = remaining;
                } else {
                    // No remaining commands to read, start executing them.
                    execute_commands::spawn().expect("failed to spawn `execute_commands`");
                    *command_state = CommandState::Executing {
                        handle: None,
                        wait_for_ignition: false,
                    };
                }
            }
            CommandState::Executing { handle, .. } => {
                // Assume this frame is a request.
                let request = postcard::from_bytes(data)?;

                match request {
                    pi_output::Request::GetStatus => {
                        send_status::spawn().expect("failed to spawn the `send_status` task")
                    }
                    pi_output::Request::SetValvesImmediately(new_states) => {
                        defmt::info!("setting valve states");

                        // Cancel the current task, if it exists.
                        handle.take().map(|handle| handle.cancel());

                        *command_state = CommandState::Executing { handle: None, wait_for_ignition: false };
                        command_list.clear();

                        set_valves::spawn(new_states).expect("failed to spawn the `set_valves` task");
                    }
                    pi_output::Request::BeginSequence { length } => {
                        defmt::info!("beginning a sequence of {} commands", length);

                        // Cancel the current task, if it exists.
                        handle.take().map(|handle| handle.cancel());

                        *command_state = CommandState::Loading { commands_remaining: length };
                        command_list.clear();
                    }
                    pi_output::Request::Reset => {
                        defmt::info!("resetting");
                        defmt::flush();
                        cortex_m::peripheral::SCB::sys_reset()
                    }
                }
            },
        }
        Ok(())
    }

    #[task(binds = USB_LP_CAN_RX0,
        local = [
            can_rx,
        ],
        shared = [
            commands,
        ]
    )]
    fn can_rx0(cx: can_rx0::Context) {
        let (command_state, command_list) = cx.shared.commands;
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

                    match process_can_rx(
                        data,
                        command_state,
                        command_list,
                    ) {
                        Ok(_) => (),
                        Err(CanRxError::Postcard(_)) => {
                            defmt::error!("failed to deserialize frame");
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

        defmt::info!("ignition detection triggered");

        ignition_detection.lock(|ignition_detection| {
            ignition_detection.clear_interrupt_pending_bit();

            match command_state {
                CommandState::Executing {
                    handle: None,
                    wait_for_ignition: true,
                } => {
                    // Now that ignition has been detected, schedule the next command to run.
                    execute_commands::spawn()
                        .expect("failed to spawn the `schedule_command` task");

                    *command_state = CommandState::Executing {
                        handle: None,
                        wait_for_ignition: false,
                    };
                }
                _ => {}
            }
        });
    }
}
