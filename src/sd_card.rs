use core::borrow::BorrowMut;

use embassy_stm32::{
    peripherals,
    sdmmc::{DataBlock, Error, Instance, Sdmmc},
    time::mhz,
};
use embassy_time::Instant;

use crate::{
    fmt::{info, unwrap},
    Irqs, SdCard,
};

pub async fn sector_writer(
    sd_card: SdCard,
) -> SectorWriter<crate::Sample, peripherals::SDMMC1, Sdmmc<'static, peripherals::SDMMC1>> {
    let mut sdmmc = Sdmmc::new_4bit(
        sd_card.sdmmc,
        Irqs,
        sd_card.clk,
        sd_card.cmd,
        sd_card.d0,
        sd_card.d1,
        sd_card.d2,
        sd_card.d3,
        Default::default(),
    );

    unwrap!(sdmmc.init_card(mhz(25)).await);

    unwrap!(SectorWriter::<crate::Sample, _, _>::new(sdmmc).await)
}

pub struct SectorWriter<T, I: Instance, S: BorrowMut<Sdmmc<'static, I>>> {
    header: u8,
    next_block: u32,
    buffer: Buffer,
    sdmmc: S,
    _i: core::marker::PhantomData<I>,
    _t: core::marker::PhantomData<T>,
}

impl<T, I: Instance, S: BorrowMut<Sdmmc<'static, I>>> SectorWriter<T, I, S> {
    async fn new(mut sdmmc: S) -> Result<Self, Error> {
        let mut buffer = Buffer::new(1);

        sdmmc.borrow_mut().read_block(0, &mut buffer.block).await?;

        let last_header = buffer.block[0];
        let header = match last_header {
            0xFF => 1,
            h => h + 1,
        };

        Ok(SectorWriter {
            header,
            next_block: 0,
            buffer,
            sdmmc,
            _i: core::marker::PhantomData,
            _t: core::marker::PhantomData,
        })
    }

    pub async fn write(&mut self, data: T) -> Result<(), Error>
    where
        T: ToBytes,
    {
        let bytes = data.to_bytes();
        let mut next_extend = bytes.as_ref();

        loop {
            let Err((block, e)) = self.buffer.extend(next_extend) else {
                return Ok(());
            };
            // info!("Flushing block {}", self.next_block);
            next_extend = e;

            block[0] = self.header;
            self.sdmmc
                .borrow_mut()
                .write_block(self.next_block, block)
                .await?;

            self.next_block += 1;
        }
    }
}

pub trait ToBytes {
    type Result: AsRef<[u8]>;

    fn to_bytes(&self) -> Self::Result;
}

impl ToBytes for u8 {
    type Result = [u8; 1];

    fn to_bytes(&self) -> Self::Result {
        [*self]
    }
}

impl ToBytes for u16 {
    type Result = [u8; 2];

    fn to_bytes(&self) -> Self::Result {
        self.to_le_bytes()
    }
}

impl ToBytes for (u16, u16) {
    type Result = [u8; 4];

    fn to_bytes(&self) -> Self::Result {
        let a = self.0.to_le_bytes();
        let b = self.1.to_le_bytes();
        [a[0], a[1], b[0], b[1]]
    }
}

struct Buffer {
    header_len: usize,
    block: DataBlock,
    len: usize,
}

impl Buffer {
    fn new(header_len: usize) -> Self {
        Buffer {
            header_len,
            block: DataBlock([0; 512]),
            len: 0,
        }
    }

    fn extend<'s, 'd>(&'s mut self, data: &'d [u8]) -> Result<(), (&'s mut DataBlock, &'d [u8])> {
        let (append, next) = data.split_at(core::cmp::min(self.remaining(), data.len()));

        let next_write = self.next_write();
        self.block[next_write..][..append.len()].copy_from_slice(append);
        self.len += append.len();

        if self.remaining() > 0 {
            return Ok(());
        }

        self.len = 0;

        Err((&mut self.block, next))
    }

    fn remaining(&self) -> usize {
        self.block.len() - self.len - self.header_len
    }

    fn next_write(&self) -> usize {
        self.header_len + self.len
    }
}

#[allow(unused)]
async fn benchmark_sample_writing<T, I: Instance>(
    sdmmc: &mut Sdmmc<'static, I>,
    n: u64,
) -> Result<(), Error>
where
    T: Default + Copy + ToBytes,
{
    let mut writer = SectorWriter::<T, _, _>::new(sdmmc).await?;

    let start = Instant::now();
    for _ in 0..n {
        writer.write(T::default()).await?;
    }

    let elapsed = start.elapsed();
    let samples_per_s = (n * 1_000_000) / elapsed.as_micros();
    info!(
        "Wrote {} samples at {:?} hz; {} in {}ms",
        core::any::type_name::<T>(),
        samples_per_s,
        n,
        elapsed.as_millis()
    );

    Ok(())
}
