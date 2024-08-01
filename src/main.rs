#![no_std]
#![no_main]

mod fmt;
mod sd_card;

use core::{
    cell::RefCell,
    pin::Pin,
    task::{Context, Poll},
};

use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel, mutex::Mutex};
use heapless::Deque;
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use assign_resources::assign_resources;
use embassy_executor::Spawner;
use embassy_stm32::{
    adc::{self, Adc, AdcChannel, AnyAdcChannel, SampleTime},
    gpio::{Level, Output, Speed},
    peripherals, sdmmc, Config,
};
use embassy_time::{Duration, Instant, Ticker, TICK_HZ};
use fmt::{info, unwrap, warn};
use futures::{
    stream::{self, Fuse},
    Stream, StreamExt, TryStreamExt,
};

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

const SAMPLE_FREQUENCY: u64 = 14_000;
const WARN_BLOCKED_SENDS: bool = false;

const BUFFER_LEN: usize = 8 * 1024;
pub type Sample = (u16, u16);
type M = ThreadModeRawMutex;

static SAMPLE_BUFFER: Mutex<ThreadModeRawMutex, Deque<Sample, BUFFER_LEN>> =
    Mutex::new(Deque::new());

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

    info!("Embassy clock ticking at {} hz", TICK_HZ);

    let r = split_resources!(p);

    spawner.must_spawn(blink(r.led, Duration::from_secs(1)));

    let samples = RefCell::new(AdcSamples::new(r.analog_read));
    let writer = RefCell::new(crate::sd_card::sector_writer(r.sd_card).await);

    let mut logger = FrequencyLogger::new("Writing to SD");
    let mut deque = SAMPLE_BUFFER.lock().await;

    let sample_every = Duration::from_nanos(1_000_000_000 / SAMPLE_FREQUENCY);
    let throttle = RefCell::new(Ticker::every(sample_every));

    unwrap!(
        stream::repeat(())
            .then(|_| { async { throttle.borrow_mut().next().await } })
            .then(|_| async { samples.borrow_mut().read().await })
            // .then(|_| async { [0, 0] })
            .map(crate::Sample::from)
            .buffer(&mut deque)
            .then(|sample| {
                let writer = &writer;
                async move { writer.borrow_mut().write(sample).await }
                // async { Ok(()) }
            })
            .inspect(|_: &Result<(), sdmmc::Error>| logger.tick())
            .try_collect::<()>()
            .await
    );
}

struct AdcSamples<I: adc::Instance> {
    adc: Adc<'static, I>,
    pin_a: AnyAdcChannel<I>,
    pin_b: AnyAdcChannel<I>,
    rx_dma: peripherals::DMA2_CH0, // TODO(shelbyd): Generic?
}

impl AdcSamples<peripherals::ADC1> {
    fn new(analog: Analog) -> Self {
        let adc = Adc::new(analog.adc);

        let pin_a = analog.pin_a.degrade_adc();
        let pin_b = analog.pin_b.degrade_adc();
        let rx_dma = analog.rx_dma;

        AdcSamples {
            adc,
            pin_a,
            pin_b,
            rx_dma,
        }
    }

    async fn read(&mut self) -> [u16; 2] {
        let mut measurements = [0; 2];
        self.adc
            .read(
                &mut self.rx_dma,
                [
                    (&mut self.pin_a, SampleTime::CYCLES2_5),
                    (&mut self.pin_b, SampleTime::CYCLES2_5),
                ]
                .into_iter(),
                &mut measurements,
            )
            .await;
        measurements
    }
}

trait MyStreamExt: Stream + Sized {
    fn buffer<'d, const N: usize>(
        self,
        deque: &'d mut Deque<Self::Item, N>,
    ) -> Buffered<'d, N, Self>;
}

impl<S: Stream + Sized> MyStreamExt for S {
    fn buffer<'d, const N: usize>(self, deque: &'d mut Deque<S::Item, N>) -> Buffered<'d, N, Self>
    where
        Self: Stream + Sized,
    {
        Buffered {
            s: self.fuse(),
            buffer: deque,
        }
    }
}

#[pin_project::pin_project]
struct Buffered<'d, const N: usize, S: Stream> {
    #[pin]
    s: Fuse<S>,

    buffer: &'d mut Deque<S::Item, N>,
}

impl<'d, const N: usize, S: Stream> Stream for Buffered<'d, N, S> {
    type Item = S::Item;

    fn poll_next(self: Pin<&mut Self>, ctx: &mut Context) -> Poll<Option<Self::Item>> {
        let mut this = self.project();

        while this.buffer.len() < this.buffer.capacity() {
            match this.s.as_mut().poll_next(ctx) {
                Poll::Pending => break,
                Poll::Ready(None) => break,

                Poll::Ready(Some(v)) => {
                    let _checked_spare_capacity = this.buffer.push_back(v);
                }
            }
        }

        if WARN_BLOCKED_SENDS && this.buffer.len() == this.buffer.capacity() {
            warn!("Buffer full");
        }

        if let Some(f) = this.buffer.pop_front() {
            return Poll::Ready(Some(f));
        }

        this.s.poll_next(ctx)
    }
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

embassy_stm32::bind_interrupts!(struct Irqs {
    SDMMC1 => sdmmc::InterruptHandler<peripherals::SDMMC1>;
});

struct FrequencyLogger<'s> {
    prefix: &'s str,
    start: Instant,
    remaining: u64,
    total: u64,
}

impl<'s> FrequencyLogger<'s> {
    fn new(prefix: &'s str) -> Self {
        Self {
            prefix,
            start: Instant::now(),
            remaining: 100,
            total: 100,
        }
    }

    fn tick(&mut self) {
        self.remaining -= 1;
        if self.remaining != 0 {
            return;
        }

        let elapsed = self.start.elapsed();
        let frequency = self.total * 1_000_000 / elapsed.as_micros();
        info!("{} @ {}hz", self.prefix, frequency);

        self.start = Instant::now();
        self.total = frequency;
        self.remaining = self.total;
    }
}
