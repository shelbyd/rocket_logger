#![no_std]
#![no_main]

mod fmt;

mod sd_card;

use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{self, Channel},
};
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use assign_resources::assign_resources;
use embassy_executor::Spawner;
use embassy_stm32::{
    adc::{Adc, AdcChannel, SampleTime},
    gpio::{Level, Output, Speed},
    peripherals, sdmmc, Config,
};
use embassy_time::{Duration, Instant, Ticker};
use fmt::{info, warn};

assign_resources! {
    analog_read: Analog {
        rx_dma: DMA2_CH0,
        adc: ADC1,
        pin_a: PA0,
        pin_b: PA1,
    },
    led: Led {
        led: PE3,
    },
    sd_card: SdCard {
        sdmmc: SDMMC1,
        clk: PC12,
        cmd: PD2,
        d0: PC8,
        d1: PC9,
        d2: PC10,
        d3: PC11,
    },
}

const SAMPLE_EVERY: Duration = Duration::from_micros(50);

const BUFFER_LEN: usize = 8 * 1024;
pub type Sample = (u16, u16);
type M = ThreadModeRawMutex;

static CHANNEL: Channel<M, Sample, BUFFER_LEN> = Channel::new();

pub type Receiver = channel::Receiver<'static, M, Sample, BUFFER_LEN>;
pub type Sender = channel::Sender<'static, M, Sample, BUFFER_LEN>;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config: Config = Default::default();

    {
        use embassy_stm32::rcc::*;

        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV4), // default clock chosen by SDMMCSEL. 200 Mhz
            divr: None,
        });

        config.rcc.pll2 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            divp: Some(PllDiv::DIV8), // 100mhz
            divq: None,
            divr: None,
        });
    }

    let p = embassy_stm32::init(config);

    info!("Starting up");

    let r = split_resources!(p);

    spawner.must_spawn(blink(r.led, Duration::from_secs(1)));

    let publish_analog = publish_analog(r.analog_read, SAMPLE_EVERY, CHANNEL.sender());
    let write_to_sd_card = sd_card::write_to_sd_card(r.sd_card, CHANNEL.receiver());

    spawner.must_spawn(publish_analog);
    spawner.must_spawn(write_to_sd_card);

    info!("All tasks spawned");
}

#[embassy_executor::task]
async fn blink(led: Led, loop_time: Duration) {
    let mut led = Output::new(led.led, Level::High, Speed::Low);
    let mut ticker = Ticker::every(loop_time / 2);

    loop {
        led.set_high();
        ticker.next().await;

        led.set_low();
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn publish_analog(mut analog: Analog, loop_time: Duration, sender: Sender) {
    let mut adc = Adc::new(analog.adc);

    let mut pin_a = analog.pin_a.degrade_adc();
    let mut pin_b = analog.pin_b.degrade_adc();

    let mut measurements = [0; 2];

    await_something_reading(&sender).await;
    let mut ticker = Ticker::every(loop_time);

    loop {
        adc.read(
            &mut analog.rx_dma,
            [
                (&mut pin_a, SampleTime::CYCLES2_5),
                (&mut pin_b, SampleTime::CYCLES2_5),
            ]
            .into_iter(),
            &mut measurements,
        )
        .await;
        let sample = (measurements[0], measurements[1]);

        if let Err(_) = sender.try_send(sample) {
            let start = Instant::now();
            sender.send((measurements[0], measurements[1])).await;
            warn!(
                "Sample sender blocked for {}us",
                start.elapsed().as_micros()
            );
        }

        ticker.next().await;
    }
}

async fn await_something_reading(sender: &Sender) {
    while let Ok(_) = sender.try_send(Default::default()) {}

    sender.send(Default::default()).await;
}

embassy_stm32::bind_interrupts!(struct Irqs {
    SDMMC1 => sdmmc::InterruptHandler<peripherals::SDMMC1>;
});
