//! A platform agnostic driver to interface the Sensirion SPS30 (UART Particulate Matter Sensor)
//!
//! This driver was built using [`embedded-hal`] traits.
//!  
//!
//! # References
//!
//! - [SPS30 data sheet][1]
//!
//! [1]: https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/0_Datasheets/Particulate_Matter/Sensirion_PM_Sensors_SPS30_Datasheet.pdf

#![deny(unsafe_code)]
#![deny(missing_docs)]
#![no_std]

use arrayvec::ArrayVec;
use core::convert::From;
use ieee754::*;
use nb::Error as nbError;
use sensirion_hdlc::{decode, encode, HDLCError, SpecialChars};

/// Max characters to read for a frame detection
const MAX_BUFFER: usize = 600;

/// Errors for this crate
#[derive(Debug)]
pub enum Error<E, F> {
    /// Serial bus read error
    SerialR(nb::Error<F>),
    /// Serial bus write error
    SerialW(E),
    /// SHDLC decode error
    SHDLC(HDLCError),
    /// No valid frame read.
    ///
    /// Input function read more than 600 characters without seeing two 0x7e
    InvalidFrame,
    /// Result is empty
    EmptyResult,
    /// Checksum failed, after shdlc decode
    ChecksumFailed,
    /// Response is for another CommandType
    InvalidRespose,
    /// Device returned an Error (State field of MISO Frame is not 0)
    StatusError,
}

impl<E, F> From<nbError<F>> for Error<E, F> {
    fn from(f: nbError<F>) -> Self {
        Error::SerialR(f)
    }
}

/// Types of information device holds
#[repr(u8)]
pub enum DeviceInfo {
    /// Product Name
    ProductName = 1,
    /// Article Code
    ArticleCode = 2,
    /// Serial Number
    SerialNumber = 3,
}

/// Available commands
#[repr(u8)]
pub enum CommandType {
    /// Start measurement
    StartMeasurement = 0,
    /// Stop measurement
    StopMeasurement = 1,
    ///  Read measurement
    ReadMeasuredData = 3,
    /// Read/Write Auto Cleaning Interval
    ReadWriteAutoCleaningInterval = 0x80,
    /// Start Fan Cleaning
    StartFanCleaning = 0x56,
    /// Device Information
    DeviceInformation = 0xD0,
    /// Reset
    Reset = 0xD3,
}

/// Checksum implemented as per section 4.1 from spec
fn compute_cksum(data: &[u8]) -> u8 {
    let mut cksum: u8 = 0;
    for &byte in data.iter() {
        let val: u16 = cksum as u16 + byte as u16;
        let lsb = val % 256;
        cksum = lsb as u8;
    }

    255 - cksum
}

/// Sps30 driver
#[derive(Debug, Default)]
pub struct Sps30<SERIAL> {
    /// The concrete Serial device implementation.
    serial: SERIAL,
}

impl<SERIAL, E, F> Sps30<SERIAL>
where
    SERIAL: embedded_hal::blocking::serial::Write<u8, Error = E>
        + embedded_hal::serial::Read<u8, Error = F>,
{
    /// Create new instance of the Sps30 device
    pub fn new(serial: SERIAL) -> Self {
        Sps30 { serial }
    }

    /// Send data through serial interface
    fn send_uart_data(&mut self, data: &[u8]) -> Result<(), Error<E, F>> {
        let s_chars = SpecialChars::default();
        let output = encode(&data, s_chars).unwrap();
        //extern crate std;
        //std::println!("Write {:x?}", output);
        self.serial.bwrite_all(&output).map_err(Error::SerialW)
    }

    /// Read from serial until two 0x7e are seen
    ///
    /// No more than MAX_BUFFER=600 u8 will be read
    /// After a MISO Frame is received, result is SHDLC decoded
    /// Checksum for decoded frame is verified
    fn read_uart_data(&mut self) -> Result<ArrayVec<[u8; 1024]>, Error<E, F>> {
        let mut output = ArrayVec::<[u8; 1024]>::new();

        let mut seen = 0;
        while seen != 2 {
            let byte = self.serial.read();
            match byte {
                Ok(value) => {
                    if value == 0x7e {
                        seen += 1;
                    }
                    output.push(value);
                }
                Err(e) => {
                    return Err(Error::from(e));
                }
            }
            if output.len() > MAX_BUFFER {
                return Err(Error::InvalidFrame);
            }
        }

        match decode(&output, SpecialChars::default()) {
            Ok(v) => {
                if v[v.len() - 1] == compute_cksum(&v[..v.len() - 1]) {
                    return Ok(v);
                }

                Err(Error::ChecksumFailed)
            }
            Err(e) => Err(Error::SHDLC(e)),
        }
    }

    /// Perform checks on MISO Frame
    ///  * lenght >=5
    ///  * CMD must match sent MOSI Frame CMD
    ///  * State should be 0 (No Error)
    ///  * L(ength) must be valid
    fn check_miso_frame<'a>(
        &self,
        data: &'a [u8],
        cmd_type: CommandType,
    ) -> Result<&'a [u8], Error<E, F>> {
        if data.len() < 5 {
            return Err(Error::InvalidRespose);
        }

        if data[1] != cmd_type as u8 {
            return Err(Error::InvalidRespose);
        }
        if data[2] != 0 {
            return Err(Error::StatusError);
        }

        if data[3] as usize != data.len() - 5 {
            return Err(Error::InvalidRespose);
        }

        //extern crate std;
        //std::println!("Read: {:x?}", &data);
        Ok(data)
    }

    /// Start measuring
    pub fn start_measurement(&mut self) -> Result<(), Error<E, F>> {
        let mut output = ArrayVec::<[u8; 1024]>::new();
        let cmd = [0x00, 0x00, 0x02, 0x01, 0x03];
        for item in &cmd {
            output.push(*item);
        }
        output.push(compute_cksum(&output));
        self.send_uart_data(&output)?;

        match self.read_uart_data() {
            Ok(response) => self
                .check_miso_frame(&response, CommandType::StartMeasurement)
                .map(|_| ()),
            Err(e) => Err(e),
        }
    }

    /// Stop measuring
    pub fn stop_measurement(&mut self) -> Result<(), Error<E, F>> {
        let mut output = ArrayVec::<[u8; 1024]>::new();
        let cmd = [0x00, 0x01, 0x00];
        for item in &cmd {
            output.push(*item);
        }
        output.push(compute_cksum(&output));
        self.send_uart_data(&output)?;

        match self.read_uart_data() {
            Ok(response) => self
                .check_miso_frame(&response, CommandType::StopMeasurement)
                .map(|_| ()),
            Err(e) => Err(e),
        }
    }

    /// Read measuring
    pub fn read_measurement(&mut self) -> Result<[f32; 10], Error<E, F>> {
        let mut output = ArrayVec::<[u8; 1024]>::new();
        let cmd = [0x00, 0x03, 0x00];
        for item in &cmd {
            output.push(*item);
        }
        output.push(compute_cksum(&cmd));
        self.send_uart_data(&output)?;

        let data = self.read_uart_data();

        let mut res: [f32; 10] = [0.0; 10];
        match data {
            Ok(v) => match v.len() {
                45 => {
                    self.check_miso_frame(&v, CommandType::ReadMeasuredData)?;
                    for i in 0..res.len() {
                        let mut bits: u32 = 0;
                        for &byte in v[4 + 4 * i..4 + 4 * (i + 1)].iter() {
                            bits = (bits << 8) + byte as u32;
                        }
                        res[i] = Ieee754::from_bits(bits);
                    }
                    Ok(res)
                }
                5 => Err(Error::EmptyResult),
                _ => Err(Error::InvalidFrame),
            },
            Err(e) => Err(e),
        }
    }

    /// Read cleaning interval
    pub fn read_cleaning_interval(&mut self) -> Result<u32, Error<E, F>> {
        let mut output = ArrayVec::<[u8; 1024]>::new();
        let cmd = [0x00, 0x80, 0x01, 0x00];
        for item in &cmd {
            output.push(*item);
        }
        output.push(compute_cksum(&output));
        self.send_uart_data(&output)?;

        match self.read_uart_data() {
            Ok(response) => {
                match self.check_miso_frame(&response, CommandType::ReadWriteAutoCleaningInterval) {
                    Ok(v) => {
                        if v[3] != 4 {
                            return Err(Error::InvalidRespose);
                        }

                        let mut ret: u32 = 0;
                        for &byte in v[4..8].iter() {
                            ret = ret * 256 + byte as u32;
                        }
                        Ok(ret)
                    }
                    Err(e) => Err(e),
                }
            }
            Err(e) => Err(e),
        }
    }

    /// Write cleaning interval
    pub fn write_cleaning_interval(&mut self, val: u32) -> Result<(), Error<E, F>> {
        let mut output = ArrayVec::<[u8; 1024]>::new();
        let cmd = [0x00, 0x80, 0x05, 0x00];
        for item in &cmd {
            output.push(*item);
        }
        for item in &val.to_be_bytes() {
            output.push(*item);
        }
        output.push(compute_cksum(&output));
        self.send_uart_data(&output)?;

        match self.read_uart_data() {
            Ok(response) => {
                match self.check_miso_frame(&response, CommandType::ReadWriteAutoCleaningInterval) {
                    Ok(v) => {
                        if v[3] != 0 {
                            return Err(Error::InvalidRespose);
                        }
                        Ok(())
                    }
                    Err(e) => Err(e),
                }
            }
            Err(e) => Err(e),
        }
    }

    /// Start fan cleaning
    pub fn start_fan_cleaning(&mut self) -> Result<(), Error<E, F>> {
        let mut output = ArrayVec::<[u8; 1024]>::new();
        let cmd = [0x00, 0x56, 0x00];
        for item in &cmd {
            output.push(*item);
        }
        output.push(compute_cksum(&output));
        self.send_uart_data(&output)?;

        match self.read_uart_data() {
            Ok(response) => self
                .check_miso_frame(&response, CommandType::StartFanCleaning)
                .map(|_| ()),
            Err(e) => Err(e),
        }
    }

    /// Get info
    ///
    /// Return a [u8;32] with info
    pub fn device_info(&mut self, info: DeviceInfo) -> Result<[u8; 32], Error<E, F>> {
        let mut output = ArrayVec::<[u8; 1024]>::new();
        let cmd = [0x00, 0xD0, 0x01];
        for item in &cmd {
            output.push(*item);
        }
        output.push(info as u8);
        output.push(compute_cksum(&output));
        self.send_uart_data(&output)?;

        match self.read_uart_data() {
            Ok(response) => {
                match self.check_miso_frame(&response, CommandType::DeviceInformation) {
                    Ok(val) => {
                        let mut ret: [u8; 32] = [0; 32];
                        if val[3] < 33 {
                            for i in 0..val[3] {
                                ret[i as usize] = val[3 + i as usize];
                            }
                            return Ok(ret);
                        }
                        Err(Error::EmptyResult)
                    }
                    Err(e) => Err(e),
                }
            }
            Err(e) => Err(e),
        }
    }

    /// Reset device
    ///
    /// After calling this function, caller must sleep before issuing more commands
    pub fn reset(&mut self) -> Result<(), Error<E, F>> {
        let mut output = ArrayVec::<[u8; 1024]>::new();
        let cmd = [0x00, 0xD3, 0x00];
        for item in &cmd {
            output.push(*item);
        }
        output.push(compute_cksum(&output));
        self.send_uart_data(&output)?;

        match self.read_uart_data() {
            Ok(response) => self
                .check_miso_frame(&response, CommandType::Reset)
                .map(|_| ()),
            Err(e) => Err(e),
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
