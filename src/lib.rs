//! Driver for W25Q128FV (Winbond Flash memory IC, 128M-BIT) over the SPI interface.
//! Uses embedded-hal. 
//!
//! Support blocking and non-blocking usage.
//! 
//! # Example of blocking usage:
//! ```
//! let mut flash = : Winbond25Q128::new(spi, cs, wp, rst).unwrap();
//! // erase first 4k block before writing
//! flash.erase(EraseUnit::Block4k(0x00)).unwrap();
//! flash.wait_if_busy().unwrap();
//! // write bytes to 0x00 address
//! let data = b"Hello";
//! flash.write(0x00, &data).unwrap();
//! flash.wait_if_busy().unwrap();
//! // read 5 bytes from address 0x00
//! let mut read_data = [0u8; 5];
//! flash.read(0x00, &mut read_data).unwrap();
//!
//! for c in read_data.iter() {
//!     println!("{}", c as char);
//! }
//! ```
// Datashee: https://www.pjrc.com/teensy/W25Q128FV.pdf

#![no_std]

extern crate embedded_hal;
use embedded_hal::blocking::spi;
use embedded_hal::digital::v2::OutputPin;

extern crate embedded_storage;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash, ErrorType, NorFlashError, NorFlashErrorKind};

/// Maximum len of write operation
pub const PAGE_SIZE: usize = 256;
/// Total flash size is 16 Mega Bytes
pub const TOTAL_SIZE: usize = 16777216;

const BUSY_BIT_MASK: u8 = 0x01;

/// Possible errors during interaction with flash IC.
#[derive(Debug, Clone, Copy)]
pub enum Error {
    /// Address passed to [`EraseUnit`] doesn't aligned to block size
    BlockAddrNotAligned,
    /// GPIO error
    PinError,
    /// SPI error
    SpiError,
}


/// Define the erase block. Flash IC support different block sizes
/// for erasing: 4kBytes, 32kBytes, 64kBytes, and all flash.
/// All addresses MUST be aligned to block size.
pub enum EraseUnit {
    /// Valid addresses: 0, 4096, 8192, 12288, 16384, ...
    /// Erase time 45-400 mS
    Block4k(u32),
    /// Valid addresses: 0, 32768, 65536, 98304, 131072, ...
    /// Erase time 120-1600 mS
    Block32k(u32),
    /// Valid addresses: 0, 65536, 131072, 196608, 262144, ...
    /// Erase time 150-2000 mS
    Block64k(u32),
    /// All flash memory
    /// Erase time 40mS-200S
    All,
}

// Flash IC commands
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

/// Flash IC instance.
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
    /// Create new instance. Requires SPI interface and CS 
    /// (chip select) pin. WP (write protect) and RST (reset)
    /// pins are oprional. If you don't pass this pins to function, 
    /// make sure that they in high state.
    pub fn new(spi: SPI, cs: CS, wp: Option<WP>, rst: Option<RST>) -> Result<Self, Error> {
        let mut flash = Self {
            spi,
            cs,
            wp,
            rst,
            busy: true
        };

        if let Some(ref mut wp_pin) = flash.wp {
            if wp_pin.set_high().is_err() {
                return Err(Error::PinError);
            }
        }

        if let Some(ref mut reset_pin) = flash.rst {
            if reset_pin.set_high().is_err() {
                return Err(Error::PinError);
            }
        }

        Ok(flash)
    }

    /// Erasing dedicated area. Before writing, block must be erased.
    /// After erasing, busy bit set to 1. If flash is busy, when user
    /// calls this function - it will return Ok(false) in other case Ok(true). 
    pub fn erase_unit(&mut self, unit: EraseUnit) -> Result<(), Error> {
        self.wait_if_busy()?;

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
        Ok(())
    }

    /// Write bytes to valid address. Area muste be eresed with 
    /// [`Self::erase()`] function before writing. Up to 256 bytes 
    /// can be writing per one call. After writing busy bit is 1.
    /// If user trying to write, when flash is busy function will 
    /// return Ok(0).

    /// Return current busy status. Flash IC can be busy after erase or 
    /// write operation.
    pub fn is_busy(&mut self) -> Result<bool, Error> {
        if self.busy {
            let mut reg = [0x00];
            self.cmd_read(Cmd::ReadStatusRegister1, None, &mut reg)?;
            self.busy = (reg[0] & BUSY_BIT_MASK) != 0x00;
        }

        Ok(self.busy)
    }

    /// Wait while flash IC is in a busy state. Useful when working with flash in a blocking manner.
    pub fn wait_if_busy(&mut self) -> Result<(), Error> {
        loop {
            if self.is_busy()? == false {
                return Ok(());
            }
        }
    }

    /// Returns resources.
    pub fn inner(self) -> (SPI, CS, Option<WP>, Option<RST>) {
        (self.spi, self.cs, self.wp, self.rst)
    }

    fn cmd_read(&mut self, cmd: Cmd, addr: Option<u32>, buf: &mut[u8]) -> Result<(), Error> {
        let cmd_buf = [cmd as u8; 1];
        
        if self.cs.set_low().is_err() {
            return Err(Error::PinError);
        }
        
        if self.spi.write(&cmd_buf).is_err() {
            return Err(Error::SpiError);
        }

        if let Some(addr) = addr {
            if self.spi.write(&addr_to_bytes(addr)).is_err() {
                return Err(Error::SpiError);
            }
        }

        if self.spi.transfer(buf).is_err() {
            return Err(Error::SpiError);
        }

        if self.cs.set_high().is_err() {
            return Err(Error::PinError);
        }

        Ok(())
    }

    fn cmd_write(&mut self, cmd: Cmd, addr: Option<u32>, buf: Option<&[u8]>) -> Result<(), Error> {
        let cmd_buf = [cmd as u8; 1];

        if self.cs.set_low().is_err() {
            return Err(Error::PinError);
        }

        if self.spi.write(&cmd_buf).is_err() {
            return Err(Error::SpiError);
        }
        
        if let Some(addr) = addr {
            if self.spi.write(&addr_to_bytes(addr)).is_err() {
                return Err(Error::SpiError);
            }
        }

        if let Some(data) = buf {
            if self.spi.write(data).is_err() {
                return Err(Error::SpiError);
            }
        }
        
        if self.cs.set_high().is_err() {
            return Err(Error::PinError);
        }

        Ok(())
    }
}

// Convert address to BE format
fn addr_to_bytes(addr: u32) -> [u8; 3] {
    [
        (addr >> 16) as u8,
        (addr >> 8) as u8,
        addr as u8
    ]
}


impl NorFlashError for Error {
    fn kind(&self) -> NorFlashErrorKind {
        match *self {
            Error::BlockAddrNotAligned => {
                NorFlashErrorKind::NotAligned
            },
            Error::PinError | Error::SpiError => {
                NorFlashErrorKind::Other
            }
        }
    }
}

impl <SPI, CS, WP, RST, SpiError, PinError> ErrorType for Winbond25Q128<SPI, CS, WP, RST>
    where
        SPI: spi::Transfer<u8, Error = SpiError> + spi::Write<u8, Error = SpiError>,
        CS: OutputPin<Error = PinError>,
        WP: OutputPin<Error = PinError>,
        RST: OutputPin<Error = PinError>,
{
    type Error = Error;
}

impl <SPI, CS, WP, RST, SpiError, PinError> ReadNorFlash for Winbond25Q128<SPI, CS, WP, RST>
    where
        SPI: spi::Transfer<u8, Error = SpiError> + spi::Write<u8, Error = SpiError>,
        CS: OutputPin<Error = PinError>,
        WP: OutputPin<Error = PinError>,
        RST: OutputPin<Error = PinError>,
{
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.wait_if_busy()?;
        self.cmd_read(Cmd::ReadData, Some(offset), bytes)
    }

    fn capacity(&self) -> usize {
        TOTAL_SIZE
    }
}

impl <SPI, CS, WP, RST, SpiError, PinError> NorFlash for Winbond25Q128<SPI, CS, WP, RST>
    where
        SPI: spi::Transfer<u8, Error = SpiError> + spi::Write<u8, Error = SpiError>,
        CS: OutputPin<Error = PinError>,
        WP: OutputPin<Error = PinError>,
        RST: OutputPin<Error = PinError>,
{
    const ERASE_SIZE: usize = 4096;
    const WRITE_SIZE: usize = 256;

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.wait_if_busy()?;
        self.cmd_write(Cmd::WriteEnable, None, None)?;
        let len = core::cmp::min(bytes.len(), PAGE_SIZE);
        self.cmd_write(Cmd::PageProgram, Some(offset), Some(&bytes[..len]))?;
        // After write operation, busy bit is 1
        self.busy = true;
        Ok(())
    }

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        match to - from {
            4096 => {
                self.erase_unit(EraseUnit::Block4k(from))
            },
            32768 => {
                self.erase_unit(EraseUnit::Block32k(from))
            },
            65536 => {
                self.erase_unit(EraseUnit::Block64k(from))
            },
            _ => {
                Err(Error::BlockAddrNotAligned)
            }
        }
    }
}