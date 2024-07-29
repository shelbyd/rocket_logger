#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use assign_resources::assign_resources;
use embassy_executor::Spawner;
use embassy_stm32::{
    adc::{Adc, AdcChannel, SampleTime},
    gpio::{Level, Output, Speed},
    peripherals, Config,
};
use embassy_time::{Duration, Ticker};
use fmt::info;

assign_resources! {
    analog_read: Analog {
        rx_dma: DMA2_CH0,
        adc: ADC1,
        pin: PA0,
    },
    led: Led {
        led: PE3,
    },
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config: Config = Default::default();

    {
        use embassy_stm32::rcc::*;
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
    spawner.must_spawn(log_analog(r.analog_read, Duration::from_millis(100)));

    info!("All tasks spawned");
}

#[embassy_executor::task]
async fn blink(led: Led, loop_time: Duration) {
    let mut led = Output::new(led.led, Level::High, Speed::Low);
    let mut ticker = Ticker::every(loop_time / 2);

    loop {
        info!("LED High");
        led.set_high();
        ticker.next().await;

        info!("LED Low");
        led.set_low();
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn log_analog(mut analog: Analog, loop_time: Duration) {
    let mut adc = Adc::new(analog.adc);
    let mut pin = analog.pin.degrade_adc();
    let mut measurements = [0; 1];

    let mut ticker = Ticker::every(loop_time);

    loop {
        adc.read(
            &mut analog.rx_dma,
            [(&mut pin, SampleTime::CYCLES16_5)].into_iter(),
            &mut measurements,
        )
        .await;

        info!("Read: {}", measurements);
        ticker.next().await;
    }
}
