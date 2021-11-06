// We don't have a `main` function (the initiation is handled by the rtic framework and by the microcontroller directly).
#![no_main]
// We're not running this in an operating system, so we only have access to the `core` library, not `std` (of which `core` is a subset).
#![no_std]

mod defmt_setup;
mod readings;
use readings::Adc1Readings;
use stm32f4xx_hal::{adc::Adc, dma::{PeripheralToMemory, Stream0, Transfer}, pac::{ADC1, DMA2}};

// We're going to be using this type a couple of times and I don't want to type the whole thing out every time.
//
// ┌──────────────────────────────────────┐
// │There are several DMA peripherals     │
// │on the MCU. We're using DMA2.         │    ┌─────────────────────────────────┐
// │                                      │    │This indicates that this transfer│
// │Each DMA has multiple streams and for │    │is from a peripheral (ADC1) to   │
// │various reasons, we're using stream 0.│    │a memory buffer.                 │
// └───┬──────────────────────────────────┘    └──────────────┬──────────────────┘
//     │                                                      │
//     │    ┌──────────────────────────────────┐              │
//     │    │This particular MCU has multiple  │              │    ┌───────────────────────┐
//     │    │ADC peripherals. We're using ADC1.│              │    │Here's the type of that│
//     │    │There's an `Adc<T>` type that's   │              │    |memory buffer.         │
//     │    │generic on the particular ADC you │              │    └────────────┬──────────┘
//     │    │use.                              │              │                 │
//     │    └────────────────────────────┬─────┘              │                 │
//     │                                 │                    │                 └──────────┐
//     └─────────────────────────────┐   └──────┐             │                            │
//                                   ▼          ▼             ▼                            ▼
type DMATransfer = Transfer<Stream0<DMA2>, Adc<ADC1>, PeripheralToMemory, &'static mut Adc1Readings, 0>;
//                                                                                                   ▲
//                        ┌──────────────────────────────────────────────────┐                       │
//                        │This device only has 8 channels that DMA transfers│                       │
//                        │can use at a time, so this number indicates that  ├───────────────────────┘
//                        │this transfer is going to use channel 0.          │
//                        └──────────────────────────────────────────────────┘

/// This declares the actual application and the code that it contains. `mod` is a keyword to make a
/// regular rust module, but the `rtic::app` attribute macro (from the `rtic` crate) does source-level
/// transformation to turn it into an RTIC application.
///
/// The `device` parameter gives the macro a "path" to the PAC (peripheral access crate) for the stm32f4 device.
/// The dispatchers` parameter tells the RTIC application to use the EXTI0 interrupt to run the task system behind
/// the scenes.
#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [EXTI0])]
mod app {
    use crate::{DMATransfer, Adc1Readings};
    use dwt_systick_monotonic::DwtSystick;
    use rtic::rtic_monotonic::Milliseconds;
    use stm32f4xx_hal::{adc::{Adc, config::{AdcConfig, Dma, SampleTime, Scan, Sequence}}, dma::{StreamsTuple, Transfer, config::DmaConfig}, gpio::GpioExt, pac, rcc::RccExt, time::U32Ext};

    /// This device can run at a maximum of 180 MHz.
    const SYSCLK_HZ: u32 = 180_000_000; // 180 MHz

    /// RTIC needs to know what monotonic timer we're going to use.
    /// The `monotonic` attribute is parsed by the `rtic::app` macro.
    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<SYSCLK_HZ>;

    /// The items in this struct are accessible from any task.
    /// The `lock_free` attribute just tells rtic that we want
    /// those particular fields to be accessible without locks
    /// so rtic restricts their usage a little to make that safe.
    #[shared]
    struct Shared {
        #[lock_free]
        transfer: DMATransfer,
        #[lock_free]
        adc1_readings: Option<&'static mut Adc1Readings>,
    }

    /// Any fields in this struct would be accessible to only one function each.
    #[local]
    struct Local {}

    /// This is the "init" function and it runs first.
    /// You can see that it returns a tuple containing `Shared`, `Local`, and `Monotonics` types,
    /// so it must construct and return those types.
    ///
    /// The "local" syntax you see here is a little different the `#[local]` struct from earlier:
    /// It's also supplied by RTIC, but it's creating these two values and storing them in static memory.
    /// They're only accessible directly inside this function (`cx.local.adc1_readings0` for example),
    /// but `cx.local.adc1_readings0` has type `&'static mut Adc1Readings` not just `Adc1Readings`, so they're
    /// alive for the entire program. We store them in the `Shared` struct so they can be accessed by other tasks.
    #[init(local = [
        adc1_readings0: Adc1Readings = Adc1Readings::new(),
        adc1_readings1: Adc1Readings = Adc1Readings::new(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let device: pac::Peripherals = cx.device;

        // These are, for the most part, zero-sized tokens that can only be moved
        // around through ownership. As you can see further down, we're required to
        // move them into types that use these various features.
        let mut dcb = cx.core.DCB;
        let dwt = cx.core.DWT;
        let systick = cx.core.SYST;

        // This creates the monotonic timer. You can see it temporarily
        // needs unique access to `dcb` (whatever that is), ownership of the
        // DWT (Data Watchpoint and Trace) timer and the system timer.
        let mono = DwtSystick::new(&mut dcb, dwt, systick, SYSCLK_HZ);

        // Let's set up the clock, which is needed for a couple things later on.
        let rcc = device.RCC.constrain();
        let _clocks = rcc
            .cfgr
            // We have an external oscillator that runs at 25 MHz.
            .use_hse(25.mhz())
            // Does something with a PLL
            .require_pll48clk()
            // Setting the system clock (180 MHz)
            .sysclk(SYSCLK_HZ)
            // Takes ownership and finalizes the clock settings
            .freeze();

        // Set up the DMA (Direct Memory Access) controller.
        let dma = StreamsTuple::new(device.DMA2);
        let dma_config = DmaConfig::default()
            // We want an interrupt when the transfer is complete.
            .transfer_complete_interrupt(true)
            // Increment the pointer every time so it doesn't overwrite
            // the same location everytime.
            .memory_increment(true)
            // We don't want to double-buffer.
            .double_buffer(false);
        
        // Set up the ADC (Analog-Digital Controller)
        let adc_config = AdcConfig::default()
            // Write converted values into memory using DMA continuously.
            .dma(Dma::Continuous)
            // Scan through all the ADC channels/pins automatically.
            .scan(Scan::Enabled);

        let gpioa = device.GPIOA.split();
        let gpiob = device.GPIOB.split();
        
        // Setting all the pins we're using to analog.
        let pa0 = gpioa.pa0.into_analog();
        let pa1 = gpioa.pa1.into_analog();
        let pa2 = gpioa.pa2.into_analog();
        let pa3 = gpioa.pa3.into_analog();
        let pa4 = gpioa.pa4.into_analog();
        let pa5 = gpioa.pa5.into_analog();
        let pa6 = gpioa.pa6.into_analog();
        let pa7 = gpioa.pa7.into_analog();

        let pb0 = gpiob.pb0.into_analog();
        let pb1 = gpiob.pb1.into_analog();

        // Enable Triple-ADC mode.
        device.ADC_COMMON.ccr.modify(|_, w| w
            .multi()
            .triple_r()
        );
        
        // Actually set up the ADCs.
        let mut adc1 = Adc::adc1(device.ADC1, true, adc_config);
        let mut adc2 = Adc::adc2(device.ADC2, true, adc_config);
        let mut adc3 = Adc::adc3(device.ADC3, true, adc_config);

        // Configure ADC1 to convert each of these pins in a sequence in a DMA transfer.
        adc1.configure_channel(&pa0, Sequence::One, SampleTime::Cycles_480);
        adc1.configure_channel(&pa1, Sequence::Two, SampleTime::Cycles_480);
        adc1.configure_channel(&pa2, Sequence::Three, SampleTime::Cycles_480);
        adc1.configure_channel(&pa3, Sequence::Four, SampleTime::Cycles_480);
        adc1.configure_channel(&pa4, Sequence::Five, SampleTime::Cycles_480);
        adc1.configure_channel(&pa5, Sequence::Six, SampleTime::Cycles_480);
        adc1.configure_channel(&pa6, Sequence::Seven, SampleTime::Cycles_480);
        adc1.configure_channel(&pa7, Sequence::Eight, SampleTime::Cycles_480);

        adc2.configure_channel(&pb0, Sequence::One, SampleTime::Cycles_480);
        adc2.configure_channel(&pb1, Sequence::Two, SampleTime::Cycles_480);
        
        // Set up a DMA transfer which is going to handle writing the ADC conversions to memory.
        let transfer = Transfer::init_peripheral_to_memory(dma.0, adc1, cx.local.adc1_readings0, None, dma_config);

        // Spawn the `polling` task right after this.
        polling::spawn().unwrap();

        (
            Shared {
                transfer,
                adc1_readings: Some(cx.local.adc1_readings1),
            },
            Local {},
            init::Monotonics(mono)
        )
    }

    /// Gets called once every 10 milliseconds (100 Hz).
    #[task(shared = [transfer, adc1_readings])]
    fn polling(cx: polling::Context) {
        // This task runs every 10 ms because we spawn it again 10 ms in the future.
        polling::spawn_after(Milliseconds(10_u32)).unwrap();
    
        let transfer: &mut DMATransfer = cx.shared.transfer;

        let adc1_readings: Adc1Readings = if let Some(adc1_readings) = cx.shared.adc1_readings.as_ref().map(|a| **a) {
            adc1_readings
        } else {
            // We don't have any existing data yet, so start the ADC conversion
            // and the DMA transfer.
            transfer.start(|adc| adc.start_conversion());
            // Wait until the next time polling is spawned.
            return;
        };

        // Start the DMA transfer and the ADC conversions.
        transfer.start(|adc| {
            adc.start_conversion();
        });

        defmt::info!("adc1 readings: {:?}", adc1_readings);
    }

    /// Gets triggered when the DMA transfer for ADC1 is completed.
    ///
    /// Since we called `.transfer_complete_interrupt(true)` on the `DmaConfig`,
    /// we know that the `DMA2_STREAM0` interrupt will be dispatched when the transfer
    /// has finished.
    ///
    /// We're also saying that we need access to the `transfer` and `adc1_readings`
    /// fields of the `Shared` struct in this task.s
    #[task(binds = DMA2_STREAM0, shared = [transfer, adc1_readings])]
    fn adc1_dma(cx: adc1_dma::Context) {
        let transfer: &mut DMATransfer = cx.shared.transfer;

        // Essentially just flip buffers so we can have access to the conversions
        // from the last time through while still doing a conversion at the same time.
        let (buffer, _) = transfer
            .next_transfer(cx.shared.adc1_readings.take().unwrap())
            .unwrap();
        *cx.shared.adc1_readings = Some(buffer);
    }
}
