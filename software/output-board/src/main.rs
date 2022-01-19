#![no_main]
#![no_std]

use defmt_rtt as _; // global logger
use stm32f1xx_hal as _; // memory layout
use panic_probe as _;

mod util;

// TODO: Replace `some_hal::pac` with the path to the PAC
#[rtic::app(device = stm32f1xx_hal::pac)]
mod app {
    use bxcan::{Interrupts, filter::Mask32, Tx, Rx};
    use stm32f1xx_hal::{prelude::*, can::Can, pac::CAN1};
    use dwt_systick_monotonic::DwtSystick;

    // The monotonic scheduler type
    #[monotonic(binds = SysTick, default = true)]
    type Mono = DwtSystick<72_000_000 /* Hz */>;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
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

        let clocks = rcc.cfgr
            // do we need other stuff here?
            .sysclk(72.mhz())
            .pclk1(16.mhz())
            .freeze(&mut flash.acr);

        // Set up CAN bus.
        // Based on https://github.com/stm32-rs/stm32f1xx-hal/blob/master/examples/can-rtic.rs.
        let can = Can::new(cx.device.CAN1);

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

    #[task(binds = USB_LP_CAN_RX0, local = [can_rx])]
    fn can_rx0(cx: can_rx0::Context) {
        loop {
            match cx.local.can_rx.receive() {
                Ok(_frame) => {
                    defmt::info!("Received a CAN frame!");
                },
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => {}, // Ignore overrun errors.
            }
        }
    }
}
