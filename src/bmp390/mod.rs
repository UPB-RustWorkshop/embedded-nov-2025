//! A simplified platform agnostic driver to interface with the BMP390 (pressure and temperature sensor)
//!
//! This driver is built using [`embedded-hal-async`] traits.

use defmt::Format;

pub mod i2c;
// pub mod spi;

/// Internal registers of the BMP390
#[allow(non_camel_case_types)]
#[derive(Debug, Copy, Clone, Format)]
pub enum Register {
    chip_id = 0x00,
    /// Pressure XLSB
    data0 = 0x04,
    /// Pressure LSB
    data1 = 0x05,
    /// Pressure MSB
    data2 = 0x06,
    /// Temperature XLSB
    data3 = 0x07,
    /// Temperature LSB
    data4 = 0x08,
    /// Temperature MSB
    data5 = 0x09,
    pwr_ctrl = 0x1B,
    /// Calibration coefficient T1 LSB
    nvm_par_t1_lsb = 0x31,
    /// Calibration coefficient T1 MSB
    nvm_par_t1_msb = 0x32,
    /// Calibration coefficient T2 LSB
    nvm_par_t2_lsb = 0x33,
    /// Calibration coefficient T2 MSB
    nvm_par_t2_msb = 0x34,
    /// Calibration coefficient T3
    nvm_par_t3 = 0x35,
}

pub trait ByteRegister {
    fn from_byte(byte: u8) -> Self;
    fn into_byte(&self) -> u8;
}

// --- PWR_CTRL --- //

/// PWR_CTRL status
#[derive(Debug, Clone, Copy, Format)]
pub struct PowerControl {
    pub pressure_enabled: bool,
    pub temperature_enabled: bool,
    pub mode: PowerMode,
}

impl ByteRegister for PowerControl {
    fn from_byte(byte: u8) -> Self {
        let pressure_enabled = (byte & 0b0000_0001) != 0;
        let temperature_enabled = (byte & 0b0000_0010) != 0;
        let mode_raw = (byte & 0b0011_0000) >> 4;
        let mode = match mode_raw {
            0b00 => PowerMode::Sleep,
            0b01 => PowerMode::Force,
            0b10 => PowerMode::ForceAlt,
            0b11 => PowerMode::Normal,
            _ => unreachable!(),
        };

        Self {
            pressure_enabled,
            temperature_enabled,
            mode,
        }
    }

    fn into_byte(&self) -> u8 {
        let mut byte: u8 = 0;
        if self.pressure_enabled {
            byte |= 0b0000_0001;
        }
        if self.temperature_enabled {
            byte |= 0b0000_0010;
        }
        byte |= (self.mode as u8) << 4;
        byte
    }
}

#[derive(Debug, Clone, Copy, Format)]
pub enum PowerMode {
    Sleep = 0b00,
    Force = 0b01,
    /// Alternative representation for forced. The chip supports both and may
    /// output both.
    ForceAlt = 0b10,
    Normal = 0b11,
}

#[derive(Debug, Clone, Copy, Format)]
pub struct TemperatureCalibration {
    pub par_t1: f32,
    pub par_t2: f32,
    pub par_t3: f32,
}
