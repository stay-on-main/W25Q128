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

/// Maximum len of write operation
pub const PAGE_SIZE: usize = 256;
/// Total flash size is 16 Mega Bytes
pub const TOTAL_SIZE: usize = 16777216;

const BUSY_BIT_MASK: u8 = 0x01;

/// Possible errors during interaction with flash IC.
#[derive(Debug, Clone, Copy)]
pub enum Error<PinError, SpiError> {
    /// Address passed to [`EraseUnit`] doesn't aligned to block size
    BlockAddrNotAligned,
    /// GPIO error
    Pin(PinError),
    /// SPI error
    Spi(SpiError),
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

    /// Reads bytes from valid flash IC address to buffer. 
    /// Return bytes read count. Ok(0) means, that flash IC is busy,
    /// and you need to call this function later.  (Flash IC can
    /// be busy after erase or write operation).
    pub fn read(&mut self, addr: u32, buf: &mut [u8]) -> Result<usize, Error<PinError, SpiError>> {
        // If flash is busy - we can't read at this moment.
        // But this is't error, just need to wait.
        if self.is_busy()? {
            return Ok(0);
        }

        self.cmd_read(Cmd::ReadData, Some(addr), buf)
    }

    /// Erasing dedicated area. Before writing, block must be erased.
    /// After erasing, busy bit set to 1. If flash is busy, when user
    /// calls this function - it will return Ok(false) in other case Ok(true). 
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

    /// Write bytes to valid address. Area muste be eresed with 
    /// [`Self::erase()`] function before writing. Up to 256 bytes 
    /// can be writing per one call. After writing busy bit is 1.
    /// If user trying to write, when flash is busy function will 
    /// return Ok(0).
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

    /// Return current busy status. Flash IC can be busy after erase or 
    /// write operation.
    pub fn is_busy(&mut self) -> Result<bool, Error<PinError, SpiError>> {
        if self.busy {
            let mut reg = [0x00];
            self.cmd_read(Cmd::ReadStatusRegister1, None, &mut reg)?;
            self.busy = (reg[0] & BUSY_BIT_MASK) != 0x00;
        }

        Ok(self.busy)
    }

    /// Wait while flash IC is in a busy state. Useful when working with flash in a blocking manner.
    pub fn wait_if_busy(&mut self) -> Result<(), Error<PinError, SpiError>> {
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

// Convert address to BE format
fn addr_to_bytes(addr: u32) -> [u8; 3] {
    [
        (addr >> 16) as u8,
        (addr >> 8) as u8,
        addr as u8
    ]
}