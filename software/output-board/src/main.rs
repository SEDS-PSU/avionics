#![no_main]
#![no_std]

use defmt_rtt as _; use dwt_systick_monotonic::fugit;
// global logger
use stm32f1xx_hal as _; // memory layout
use panic_probe as _;

mod util;

type Instant = fugit::Instant<u32, 1, 72_000_000>;
type Duration = fugit::Duration<u32, 1, 72_000_000>;

const COMMAND_TIMEOUT: Duration = Duration::millis(5);

const RASPI_ID: bxcan::StandardId = if let Some(id) = bxcan::StandardId::new(shared::RASPI_ID) {
    id
} else {
    panic!("RASPI_ID is not a valid standard CAN ID");
};

// TODO: Replace `some_hal::pac` with the path to the PAC
#[rtic::app(device = stm32f1xx_hal::pac, dispatchers = [EXTI0])]
mod app {
    use bxcan::{Interrupts, filter::Mask32, Tx, Rx, Frame};
    use heapless::{spsc::Queue, Deque};
    use shared::{pi_output, ValveStates};
    use stm32f1xx_hal::{prelude::*, can::Can, pac::{CAN1, Interrupt}, gpio::{PullDown, Input, gpiob::PB15, gpioa::PA1, PullUp}};
    use dwt_systick_monotonic::DwtSystick;
    use crate::Duration;

    use crate::{Instant, COMMAND_TIMEOUT, RASPI_ID};

    // The monotonic scheduler type
    #[monotonic(binds = SysTick, default = true)]
    type Mono = DwtSystick<72_000_000 /* Hz */>;

    pub enum CommandState {
        Loading,
        Executing(Option<execute_commands::SpawnHandle>),
    }

    // Shared resources go here
    #[shared]
    struct Shared {
        valve_states: Option<ValveStates>,

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
    }

    #[init(local = [command_list: Deque<pi_output::Command, 256> = Deque::new()])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("init");

        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        let clocks = rcc.cfgr
            // do we need other stuff here?
            .sysclk(72.mhz())
            .pclk1(16.mhz())
            .freeze(&mut flash.acr);

        // Set up CAN bus.
        // Based on https://github.com/stm32-rs/stm32f1xx-hal/blob/master/examples/can-rtic.rs.
        let can = Can::new(cx.device.CAN1);

        // Pins
        let mut gpioa = cx.device.GPIOA.split();
        let mut gpiob = cx.device.GPIOB.split();

        let arming_switch = gpiob.pb15.into_pull_down_input(&mut gpiob.crh);
        let ignition_detection = gpioa.pa1.into_pull_up_input(&mut gpioa.crl);

        // TODO: Assign CAN pins
        
        // APB1 (PCLK1): 16MHz, Bit rate: 1000kBit/s, Sample Point 87.5%
        // Value was calculated with http://www.bittiming.can-wiki.info/
        let mut can = bxcan::Can::builder(can)
            .set_bit_timing(0x001c_0000)
            .leave_disabled();
        
        can.modify_filters().enable_bank(0, Mask32::accept_all());

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
                    }
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

    fn set_valves(valve_states: &mut Option<ValveStates>, new_states: ValveStates) {
        defmt::info!("setting values states");
        *valve_states = Some(new_states);
        
        todo!()
    }

    #[task(shared = [commands, valve_states])]
    fn execute_commands(cx: execute_commands::Context) {
        let execute_commands::SharedResources { commands: (command_state, command_list), mut valve_states } = cx.shared;

        let old_spawn_handle = match command_state {
            CommandState::Loading => return,
            CommandState::Executing(maybe_spawn_handle) => maybe_spawn_handle,
        };

        *old_spawn_handle = if let Some(command) = command_list.pop_front() {
            match command {
                pi_output::Command::SetValves { states, wait } => {
                    defmt::info!("setting valves");
                    valve_states.lock(|valve_states| {
                        set_valves(valve_states, states);
                    });
                    
                    if let pi_output::Wait::WaitMs(ms) = wait {
                        let handle = execute_commands::spawn_after(Duration::millis(ms.get() as u32)).expect("failed to spawn `execute_commands` task");
                        Some(handle)
                    } else {
                        None
                    }
                },
                pi_output::Command::Ignite { timeout } => {
                    todo!("timeout: {}ms", timeout);
                },
            }
        } else {
            None
        };
    }

    #[task(shared = [can_tx_queue, valve_states, &arming_switch, &ignition_detection])]
    fn send_status(cx: send_status::Context) {
        let mut state = pi_output::State::new();

        if cx.shared.arming_switch.is_high() {
            state = state.set_armed();
        }

        if cx.shared.ignition_detection.is_high() {
            state = state.set_ignited();
        }

        let mut states = cx.shared.valve_states;
        let states = states.lock(|states| *states);

        let status = pi_output::Status {
            states,
            state,
            error: None, // for now
        };

        let frame = bxcan::Frame::new_data(RASPI_ID, status.as_bytes());
        
        let mut tx_queue = cx.shared.can_tx_queue;

        tx_queue.lock(|tx_queue| {
            enqueue_frame(tx_queue, frame);
        });
    }

    #[task(binds = USB_LP_CAN_RX0,
        local = [
            can_rx,
            commands_start,
            commands_count,
        ],
        shared = [
            valve_states,
            commands,
        ]
    )]
    fn can_rx0(cx: can_rx0::Context) {
        let (command_state, command_list) = cx.shared.commands;
        let mut valve_states = cx.shared.valve_states;
        let can_rx0::LocalResources { can_rx, commands_start, commands_count } = cx.local;

        loop {
            match can_rx.receive() {
                Ok(frame) => {
                    defmt::info!("received a CAN frame");

                    let data = if let Some(data) = frame.data() {
                        data.as_ref()
                    } else {
                        continue; // go to next loop iteration
                    };

                    if let Some(timeout_begin) = *commands_start {
                        if monotonics::now() - timeout_begin < COMMAND_TIMEOUT && *commands_count > 0 {
                            if let CommandState::Loading = command_state{
                                // We're in the wrong state.
                                defmt::error!("attempted to load commands while executing commands");
                                continue;
                            }

                            assert!(*commands_count > 0);

                            let data = if let Ok(data) = data.try_into() {
                                data
                            } else {
                                defmt::error!("frame data is not 4 bytes");
                                continue;
                            };

                            let command = pi_output::Command::from_bytes(data);
                            // This can't overflow, since the length is a max of 255.
                            command_list.push_back(command).unwrap();
                            *commands_count -= 1;

                            continue;
                        } else {
                            // We timed out, so let's assume this frame is a request.
                            *commands_start = None;
                        }
                    }

                    // Assume this frame is a request.

                    // Make sure the frame has a size of 4.
                    let data = if let Ok(data) = data.try_into() {
                        data
                    } else {
                        defmt::error!("frame data is not 4 bytes");
                        continue;
                    };

                    let request = pi_output::Request::from_bytes(data);

                    match request {
                        pi_output::Request::GetStatus => send_status::spawn().expect("failed to spawn the `send_status` task"),
                        pi_output::Request::SetValvesImmediately(new_states) => {
                            defmt::info!("setting valve states");
                            valve_states.lock(|valve_states| {
                                set_valves(valve_states, new_states);
                            });
                            
                            // Kill the currently executing list of commands.
                            *command_state = CommandState::Loading;
                            command_list.clear();
                        }
                        pi_output::Request::BeginSequence { length } => {
                            defmt::info!("beginning a sequence of {} commands", length);
                            *commands_start = Some(monotonics::now());
                            *commands_count = length;

                            // Kill the currently executing list of commands.
                            *command_state = CommandState::Loading;
                            command_list.clear();
                        },
                        pi_output::Request::Reset => {
                            defmt::info!("resetting");
                            defmt::flush();
                            cortex_m::peripheral::SCB::sys_reset()
                        },
                    }

                },
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => {}, // Ignore overrun errors.
            }
        }
    }
}
