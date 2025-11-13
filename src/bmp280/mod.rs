//! A WIP platform agnostic driver to interface with the BMP280 (pressure sensor)
//!
//! This driver is built using [`embedded-hal-async`] traits.

use core::fmt;

pub mod i2c;
pub mod spi;

#[derive(Debug, Copy, Clone)]
/// Control
pub struct Control {
    /// Temperature oversampling
    pub osrs_t: Oversampling,
    /// Pressure oversampling
    pub osrs_p: Oversampling,
    /// Powermode
    pub mode: PowerMode,
}

#[derive(Debug, Copy, Clone)]
#[allow(non_camel_case_types)]
/// Standby time in ms
pub enum Standby {
    /// ms0_5
    ms0_5 = 0b000,
    /// ms62_5
    ms62_5 = 0b001,
    /// ms125_5
    ms125 = 0b010,
    /// ms250
    ms250 = 0b011,
    /// ms500
    ms500 = 0b100,
    /// ms1000
    ms1000 = 0b101,
    /// ms2000
    ms2000 = 0b110,
    /// ms4000
    ms4000 = 0b111,
    /// unknown
    unknown,
}

#[derive(Debug, Copy, Clone)]
#[allow(non_camel_case_types)]
/// The time constant of IIR filter
pub enum Filter {
    /// off
    off = 0x00,
    /// c2
    c2 = 0x01,
    /// c4
    c4 = 0x02,
    /// c8
    c8 = 0x03,
    /// c16
    c16 = 0x04,
    /// unknown
    unknown,
}

/// Configuration register, sets the rate, filter and interface options
/// of the device. Note that writing to this register while device in normal
/// mode may be ignored. Writes in sleep mode are not ignored.
///
/// spi3w_en is intentionally left out of this implementation.
#[derive(Debug, Copy, Clone)]
pub struct Config {
    /// Controls inactive duration in normal mode
    pub t_sb: Standby,
    /// Controls the time constant of IIR filter
    pub filter: Filter,
}

/// Status
#[derive(Debug, Copy, Clone)]
pub struct Status {
    /// measuring
    measuring: bool,
    /// im update
    im_update: bool,
}

impl fmt::Display for Status {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        core::write!(
            f,
            "conversion is running: {}, NVM data being copied: {}",
            self.measuring,
            self.im_update
        )
    }
}

#[derive(Debug, Copy, Clone)]
#[allow(non_camel_case_types)]
/// Oversampling
pub enum Oversampling {
    /// skipped
    skipped = 0b000,
    /// x1
    x1 = 0b001,
    /// x2
    x2 = 0b010,
    /// x4
    x4 = 0b011,
    /// x8
    x8 = 0b100,
    /// x16
    x16 = 0b101,
}

#[derive(Debug, Copy, Clone)]
/// PowerMode
pub enum PowerMode {
    /// Sleep
    Sleep = 0b00,
    /// Forced
    Forced = 0b01,
    /// Normal
    Normal = 0b11,
}

#[allow(non_camel_case_types)]
enum Register {
    id = 0xD0,
    reset = 0xE0,
    status = 0xF3,
    ctrl_meas = 0xF4,
    config = 0xF5,
    press = 0xF7,
    calib00 = 0x88,
}
