// W25Q128FV
// https://www.pjrc.com/teensy/W25Q128FV.pdf
#![no_std]

extern crate embedded_hal;
use embedded_hal::blocking::spi;
use embedded_hal::digital::v2::OutputPin;

pub const PAGE_SIZE: usize = 256;
pub const TOTAL_SIZE: usize = 16777216;

const BUSY_BIT_MASK: u8 = 0x01;

#[derive(Debug, Clone, Copy)]
pub enum Error<PinError, SpiError> {
    BlockAddrNotAligned,
    Pin(PinError),
    Spi(SpiError),
}

pub enum EraseUnit {
    Block4k(u32),
    Block32k(u32),
    Block64k(u32),
    All,
}

enum Cmd {
    WriteEnable = 0x06,
    ReadData = 0x03,
    PageProgram = 0x02,
    SectorErase = 0x20,   // 4kB
    BlockErase32K = 0x52, // 32kB
    BlockErase64K = 0xD8, // 64kB
    ChipErase  = 0xC7,
    ReadStatusRegister1 = 0x05,
    // Add here any additional cmd
}

pub struct Winbond25Q128<SPI, CS, WP, RST> {
    spi: SPI,
    cs: CS,
    wp: Option<WP>,
    rst: Option<RST>,
    busy: bool,
}

impl <SPI, CS, WP, RST, SpiError, PinError> Winbond25Q128<SPI, CS, WP, RST>
    where
        SPI: spi::Transfer<u8, Error = SpiError> + spi::Write<u8, Error = SpiError>,
        CS: OutputPin<Error = PinError>,
        WP: OutputPin<Error = PinError>,
        RST: OutputPin<Error = PinError>,
{
    pub fn new(spi: SPI, cs: CS, wp: Option<WP>, rst: Option<RST>) -> Result<Self, Error<PinError, SpiError>> {
        let mut flash = Self {
            spi,
            cs,
            wp,
            rst,
            busy: true
        };

        if let Some(ref mut wp_pin) = flash.wp {
            wp_pin.set_high().map_err(Error::Pin)?;
        }

        if let Some(ref mut reset_pin) = flash.rst {
            reset_pin.set_high().map_err(Error::Pin)?;
        }

        Ok(flash)
    }

    pub fn read(&mut self, addr: u32, buf: &mut [u8]) -> Result<usize, Error<PinError, SpiError>> {
        // If flash is busy - we can't read at this moment.
        // But this is't error, just need to wait.
        if self.is_busy()? {
            return Ok(0);
        }

        self.cmd_read(Cmd::ReadData, Some(addr), buf)
    }

    pub fn erase(&mut self, unit: EraseUnit) -> Result<bool, Error<PinError, SpiError>> {
        if self.is_busy()? {
            return Ok(false);
        }

        let cmd;
        let address;

        match unit {
            EraseUnit::Block4k(addr) => {
                if (addr & 0xFFF) != 0 {
                    return Err(Error::BlockAddrNotAligned);
                }
                cmd = Cmd::SectorErase;
                address = Some(addr);
            },
            EraseUnit::Block32k(addr) => {
                if (addr & 0x7FFF) != 0 {
                    return Err(Error::BlockAddrNotAligned);
                }
                cmd = Cmd::BlockErase32K;
                address = Some(addr);
            },
            EraseUnit::Block64k(addr) => {
                assert!(addr & 0xFFFF == 0);
                if (addr & 0xFFFF) != 0 {
                    return Err(Error::BlockAddrNotAligned);
                }
                cmd = Cmd::BlockErase64K;
                address = Some(addr);
            },
            EraseUnit::All => {
                cmd = Cmd::ChipErase;
                address = None;
            },
        }

        self.cmd_write(Cmd::WriteEnable, None, None)?;
        self.cmd_write(cmd, address, None)?;
        // After erase operation, busy bit is true
        self.busy = true;
        Ok(true)
    }

    // function can write up to 256 bytes (Page Size)
    pub fn write(&mut self, addr: u32, buf: &[u8]) -> Result<usize, Error<PinError, SpiError>> {
        // Unable to write when busy bit is 1, need wait
        if self.is_busy()? {
            return Ok(0);
        }

        self.cmd_write(Cmd::WriteEnable, None, None)?;
        let len = core::cmp::min(buf.len(), PAGE_SIZE);
        self.cmd_write(Cmd::PageProgram, Some(addr), Some(&buf[..len]))?;
        // After write operation, busy bit is 1
        self.busy = true;
        Ok(len)
    }

    pub fn is_busy(&mut self) -> Result<bool, Error<PinError, SpiError>> {
        if self.busy {
            let mut reg = [0x00];
            self.cmd_read(Cmd::ReadStatusRegister1, None, &mut reg)?;
            self.busy = (reg[0] & BUSY_BIT_MASK) != 0x00;
        }

        Ok(self.busy)
    }

    fn cmd_read(&mut self, cmd: Cmd, addr: Option<u32>, buf: &mut[u8]) -> Result<usize, Error<PinError, SpiError>> {
        let cmd_buf = [cmd as u8; 1];
        self.cs.set_low().map_err(Error::Pin)?;
        self.spi.write(&cmd_buf).map_err(Error::Spi)?;

        if let Some(addr) = addr {
            self.spi.write(&addr_to_bytes(addr)).map_err(Error::Spi)?;
        }

        self.spi.transfer(buf).map_err(Error::Spi)?;
        self.cs.set_high().map_err(Error::Pin)?;
        Ok(buf.len())
    }

    fn cmd_write(&mut self, cmd: Cmd, addr: Option<u32>, buf: Option<&[u8]>) -> Result<usize, Error<PinError, SpiError>> {
        let cmd_buf = [cmd as u8; 1];
        self.cs.set_low().map_err(Error::Pin)?;
        self.spi.write(&cmd_buf).map_err(Error::Spi)?;
        
        if let Some(addr) = addr {
            self.spi.write(&addr_to_bytes(addr)).map_err(Error::Spi)?;
        }

        let mut len = 0;

        if let Some(data) = buf {
            self.spi.write(data).map_err(Error::Spi)?;
            len = data.len();
        }
        
        self.cs.set_high().map_err(Error::Pin)?;
        Ok(len)
    }
}

fn addr_to_bytes(addr: u32) -> [u8; 3] {
    [
        (addr >> 16) as u8,
        (addr >> 8) as u8,
        addr as u8
    ]
}