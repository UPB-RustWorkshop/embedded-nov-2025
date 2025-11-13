//! A platform agnostic driver to interface with the BMP280 (pressure sensor)
//!
//! This driver is built using [`embedded-hal`] traits.

use embassy_futures::block_on;
use embassy_stm32::gpio::Output;

use super::{Config, Control, Filter, Oversampling, PowerMode, Register, Standby, Status};

/// BMP280 driver
pub struct BMP280<Spi: embedded_hal_async::spi::SpiBus> {
    com: Spi,
    cs: Output<'static>,
    // Temperature compensation
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
    t_fine: i32,
    // Pressure calibration
    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,
}

impl<Spi: embedded_hal_async::spi::SpiBus> BMP280<Spi> {
    /// Creates new BMP280 driver
    pub fn new<E>(spi: Spi, cs: Output<'static>) -> Result<BMP280<Spi>, E>
    where
        Spi: embedded_hal_async::spi::SpiBus<Error = E>,
    {
        let mut chip = BMP280 {
            com: spi,
            cs,
            dig_t1: 0,
            dig_t2: 0,
            dig_t3: 0,
            t_fine: 0,
            dig_p1: 0,
            dig_p2: 0,
            dig_p3: 0,
            dig_p4: 0,
            dig_p5: 0,
            dig_p6: 0,
            dig_p7: 0,
            dig_p8: 0,
            dig_p9: 0,
        };

        block_on(chip.read_calibration());

        Ok(chip)
    }
}

impl<Spi: embedded_hal_async::spi::SpiBus> BMP280<Spi> {
    async fn read_calibration(&mut self) {
        let mut buff: [u8; 25] = [0; 25];
        buff[0] = Register::calib00 as u8;
        self.cs.set_low();
        let _ = self.com.transfer_in_place(&mut buff).await;
        self.cs.set_high();
        let data = &buff[1..];

        self.dig_t1 = ((data[1] as u16) << 8) | (data[0] as u16);
        self.dig_t2 = ((data[3] as i16) << 8) | (data[2] as i16);
        self.dig_t3 = ((data[5] as i16) << 8) | (data[4] as i16);

        self.dig_p1 = ((data[7] as u16) << 8) | (data[6] as u16);
        self.dig_p2 = ((data[9] as i16) << 8) | (data[8] as i16);
        self.dig_p3 = ((data[11] as i16) << 8) | (data[10] as i16);
        self.dig_p4 = ((data[13] as i16) << 8) | (data[12] as i16);
        self.dig_p5 = ((data[15] as i16) << 8) | (data[14] as i16);
        self.dig_p6 = ((data[17] as i16) << 8) | (data[16] as i16);
        self.dig_p7 = ((data[19] as i16) << 8) | (data[18] as i16);
        self.dig_p8 = ((data[21] as i16) << 8) | (data[20] as i16);
        self.dig_p9 = ((data[23] as i16) << 8) | (data[22] as i16);
    }

    /// Reads and returns pressure
    pub async fn pressure(&mut self) -> f64 {
        let mut buff: [u8; 7] = [Register::press as u8, 0, 0, 0, 0, 0, 0];
        self.cs.set_low();
        let _ = self.com.transfer_in_place(&mut buff).await;
        self.cs.set_high();
        let data = &buff[1..];
        let press = (data[0] as u32) << 12 | (data[1] as u32) << 4 | (data[2] as u32) >> 4;

        let mut var1 = ((self.t_fine as f64) / 2.0) - 64000.0;
        let mut var2 = var1 * var1 * (self.dig_p6 as f64) / 32768.0;
        var2 += var1 * (self.dig_p5 as f64) * 2.0;
        var2 = (var2 / 4.0) + ((self.dig_p4 as f64) * 65536.0);
        var1 = ((self.dig_p3 as f64) * var1 * var1 / 524288.0 + (self.dig_p2 as f64) * var1)
            / 524288.0;
        var1 = (1.0 + var1 / 32768.0) * (self.dig_p1 as f64);
        let mut pressure = 1048576.0 - (press as f64);
        if var1 != 0.0 {
            pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
            var1 = (self.dig_p9 as f64) * pressure * pressure / 2147483648.0;
            var2 = pressure * (self.dig_p8 as f64) / 32768.0;
            pressure += (var1 + var2 + (self.dig_p7 as f64)) / 16.0;
        }
        pressure
    }

    /// Reads and returns temperature
    pub async fn temp(&mut self) -> f64 {
        let mut buff: [u8; 7] = [Register::press as u8, 0, 0, 0, 0, 0, 0];
        self.cs.set_low();
        let _ = self.com.transfer_in_place(&mut buff).await;
        self.cs.set_high();
        let data = &buff[1..];
        let _pres = (data[0] as u32) << 12 | (data[1] as u32) << 4 | (data[2] as u32) >> 4;
        let temp = (data[3] as u32) << 12 | (data[4] as u32) << 4 | (data[5] as u32) >> 4;

        let v1 = ((temp as f64) / 16384.0 - (self.dig_t1 as f64) / 1024.0) * (self.dig_t2 as f64);
        let v2 = (((temp as f64) / 131072.0 - (self.dig_t1 as f64) / 8192.0)
            * ((temp as f64) / 131072.0 - (self.dig_t1 as f64) / 8192.0))
            * (self.dig_t3 as f64);
        self.t_fine = (v1 + v2) as i32;

        (v1 + v2) / 5120.0
    }

    /// Returns current config
    pub async fn config(&mut self) -> Config {
        let config = self.read_byte(Register::config).await;
        let t_sb = match (config & (0b111 << 5)) >> 5 {
            x if x == Standby::ms0_5 as u8 => Standby::ms0_5,
            x if x == Standby::ms62_5 as u8 => Standby::ms62_5,
            x if x == Standby::ms125 as u8 => Standby::ms125,
            x if x == Standby::ms250 as u8 => Standby::ms250,
            x if x == Standby::ms500 as u8 => Standby::ms500,
            x if x == Standby::ms1000 as u8 => Standby::ms1000,
            x if x == Standby::ms2000 as u8 => Standby::ms2000,
            x if x == Standby::ms4000 as u8 => Standby::ms4000,
            _ => Standby::unknown,
        };
        let filter = match (config & (0b111 << 2)) >> 2 {
            x if x == Filter::off as u8 => Filter::off,
            x if x == Filter::c2 as u8 => Filter::c2,
            x if x == Filter::c4 as u8 => Filter::c4,
            x if x == Filter::c8 as u8 => Filter::c8,
            x if x == Filter::c16 as u8 => Filter::c16,
            _ => Filter::unknown,
        };
        Config { t_sb, filter }
    }

    /// Sets configuration
    pub async fn set_config(&mut self, new: Config) {
        let config: u8 = 0x00;
        let t_sb = (new.t_sb as u8) << 5;
        let filter = (new.filter as u8) << 2;
        self.write_byte(Register::config, config | t_sb | filter)
            .await;
    }

    /// Sets control
    pub async fn set_control(&mut self, new: Control) {
        let osrs_t: u8 = (new.osrs_t as u8) << 5;
        let osrs_p: u8 = (new.osrs_p as u8) << 2;
        let control: u8 = osrs_t | osrs_p | (new.mode as u8);
        self.write_byte(Register::ctrl_meas, control).await;
    }

    /// Returns control
    pub async fn control(&mut self) -> Control {
        let config = self.read_byte(Register::ctrl_meas).await;
        let osrs_t = match (config & (0b111 << 5)) >> 5 {
            x if x == Oversampling::skipped as u8 => Oversampling::skipped,
            x if x == Oversampling::x1 as u8 => Oversampling::x1,
            x if x == Oversampling::x2 as u8 => Oversampling::x2,
            x if x == Oversampling::x4 as u8 => Oversampling::x4,
            x if x == Oversampling::x8 as u8 => Oversampling::x8,
            _ => Oversampling::x16,
        };
        let osrs_p = match (config & (0b111 << 2)) >> 2 {
            x if x == Oversampling::skipped as u8 => Oversampling::skipped,
            x if x == Oversampling::x1 as u8 => Oversampling::x1,
            x if x == Oversampling::x2 as u8 => Oversampling::x2,
            x if x == Oversampling::x4 as u8 => Oversampling::x4,
            x if x == Oversampling::x8 as u8 => Oversampling::x8,
            _ => Oversampling::x16,
        };
        let mode = match config & 0b11 {
            x if x == PowerMode::Sleep as u8 => PowerMode::Sleep,
            x if x == PowerMode::Forced as u8 => PowerMode::Forced,
            x if x == PowerMode::Normal as u8 => PowerMode::Normal,
            _ => PowerMode::Forced,
        };

        Control {
            osrs_t,
            osrs_p,
            mode,
        }
    }

    /// Returns device status
    pub async fn status(&mut self) -> Status {
        let status = self.read_byte(Register::status).await;
        Status {
            measuring: 0 != (status & 0b00001000),
            im_update: 0 != (status & 0b00000001),
        }
    }

    /// Returns device id
    pub async fn id(&mut self) -> u8 {
        self.read_byte(Register::id).await
    }

    /// Software reset, emulates POR
    pub async fn reset(&mut self) {
        self.write_byte(Register::reset, 0xB6).await; // Magic from documentation
    }

    async fn write_byte(&mut self, reg: Register, byte: u8) {
        self.cs.set_low();
        let _ = self.com.write(&[reg as u8, byte]).await;
        self.cs.set_high();
    }

    async fn read_byte(&mut self, reg: Register) -> u8 {
        let mut data: [u8; 2] = [reg as u8, 0];
        self.cs.set_low();
        let _ = self.com.transfer_in_place(&mut data).await;
        self.cs.set_high();
        data[1]
    }
}
