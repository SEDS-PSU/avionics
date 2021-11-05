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

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [EXTI0])]
mod app {
    use crate::{DMATransfer, Adc1Readings};
    use dwt_systick_monotonic::DwtSystick;
    use rtic::rtic_monotonic::Milliseconds;
    use stm32f4xx_hal::{adc::{Adc, config::{AdcConfig, Dma, SampleTime, Scan, Sequence}}, dma::{StreamsTuple, Transfer, config::DmaConfig}, gpio::GpioExt, pac, rcc::RccExt, time::U32Ext};

    const SYSCLK_HZ: u32 = 180_000_000; // 180 MHz

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<SYSCLK_HZ>;

    #[shared]
    struct Shared {
        #[lock_free]
        transfer: DMATransfer,
        #[lock_free]
        adc1_readings: Option<&'static mut Adc1Readings>,
    }

    #[local]
    struct Local {}

    #[init(local = [
        adc1_readings0: Adc1Readings = Adc1Readings::new(),
        adc1_readings1: Adc1Readings = Adc1Readings::new(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let device: pac::Peripherals = cx.device;

        let mut dcb = cx.core.DCB;
        let dwt = cx.core.DWT;
        let systick = cx.core.SYST;

        let mono = DwtSystick::new(&mut dcb, dwt, systick, SYSCLK_HZ);

        let rcc = device.RCC.constrain();
        let _clocks = rcc
            .cfgr
            .use_hse(25.mhz())
            .require_pll48clk()
            .sysclk(SYSCLK_HZ)
            .freeze();

        let dma = StreamsTuple::new(device.DMA2);
        let dma_config = DmaConfig::default()
            .transfer_complete_interrupt(true)
            .memory_increment(true)
            .double_buffer(false);
        
        let adc_config = AdcConfig::default()
            .dma(Dma::Continuous)
            .scan(Scan::Enabled);

        let gpioa = device.GPIOA.split();
        let gpiob = device.GPIOB.split();
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
        

        let transfer = Transfer::init_peripheral_to_memory(dma.0, adc1, cx.local.adc1_readings0, None, dma_config);

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
        polling::spawn_after(Milliseconds(10_u32)).unwrap();
    
        let transfer: &mut DMATransfer = cx.shared.transfer;

        let adc1_readings: Adc1Readings = if let Some(adc1_readings) = cx.shared.adc1_readings.as_ref().map(|a| **a) {
            adc1_readings
        } else {
            // We don't have any existing data yet.
            transfer.start(|adc| adc.start_conversion());
            return;
        };

        transfer.start(|adc| {
            adc.start_conversion();
        });

        defmt::info!("adc1 readings: {:?}", adc1_readings);
    }

    /// Gets triggered when the DMA transfer for ADC1 is completed.
    #[task(binds = DMA2_STREAM0, shared = [transfer, adc1_readings])]
    fn adc1_dma(cx: adc1_dma::Context) {
        let transfer: &mut DMATransfer = cx.shared.transfer;

        let (buffer, _) = transfer
            .next_transfer(cx.shared.adc1_readings.take().unwrap())
            .unwrap();
        *cx.shared.adc1_readings = Some(buffer);
    }
}
