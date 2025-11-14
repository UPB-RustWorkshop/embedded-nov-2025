use crate::bmp390::{ByteRegister, PowerControl, Register, TemperatureCalibration};

/// The default address for the BMP390 if SDO pin is low
const DEFAULT_ADDRESS_LOW: u8 = 0b1110110;
/// The default address for the BMP390 if SDO pin is high
const DEFAULT_ADDRESS_HIGH: u8 = 0b1110111;

/// BMP390 driver
pub struct BMP390<I2C: embedded_hal_async::i2c::I2c> {
    com: I2C,
    addr: u8,
    temp_calibration: Option<TemperatureCalibration>,
}

impl<I2C: embedded_hal_async::i2c::I2c> BMP390<I2C> {
    /// Creates new BMP390 driver with the specified address
    pub async fn new_with_address<E>(i2c: I2C, addr: u8) -> Result<BMP390<I2C>, E>
    where
        I2C: embedded_hal_async::i2c::I2c<Error = E>,
    {
        let mut chip = BMP390 {
            com: i2c,
            addr,
            temp_calibration: None,
        };

        chip.temp_calibration = Some(chip.read_calibration().await);

        Ok(chip)
    }

    /// Create a new BMP390 driver with the default address (assuming SDO pin is low)
    pub async fn new_sdo_low<E>(i2c: I2C) -> Result<BMP390<I2C>, E>
    where
        I2C: embedded_hal_async::i2c::I2c<Error = E>,
    {
        Self::new_with_address(i2c, DEFAULT_ADDRESS_LOW).await
    }

    /// Create a new BMP390 driver with the default address (assuming SDO pin is high)
    pub async fn new_sdo_high<E>(i2c: I2C) -> Result<BMP390<I2C>, E>
    where
        I2C: embedded_hal_async::i2c::I2c<Error = E>,
    {
        Self::new_with_address(i2c, DEFAULT_ADDRESS_HIGH).await
    }
}

impl<I2C: embedded_hal_async::i2c::I2c> BMP390<I2C> {
    async fn read_calibration(&mut self) -> TemperatureCalibration {
        let mut nvm_par_t1 = 0u16;
        let mut nvm_par_t2 = 0u16;
        let nvm_par_t3: i8;

        nvm_par_t1 |= (self.read_byte(Register::nvm_par_t1_msb).await as u16) << 8;
        nvm_par_t1 |= self.read_byte(Register::nvm_par_t1_lsb).await as u16;
        nvm_par_t2 |= (self.read_byte(Register::nvm_par_t2_msb).await as u16) << 8;
        nvm_par_t2 |= self.read_byte(Register::nvm_par_t2_lsb).await as u16;
        nvm_par_t3 = self.read_byte(Register::nvm_par_t3).await as i8;

        let par_t1 = (nvm_par_t1 as f32) * ((1u32 << 8) as f32);
        let par_t2 = (nvm_par_t2 as f32) / ((1u64 << 30) as f32);
        let par_t3 = (nvm_par_t3 as f32) / ((1u64 << 48) as f32);

        TemperatureCalibration {
            par_t1,
            par_t2,
            par_t3,
        }
    }

    /// Returns device id
    pub async fn id(&mut self) -> u8 {
        self.read_byte(Register::chip_id).await
    }

    pub async fn read_power_control(&mut self) -> PowerControl {
        let byte = self.read_byte(Register::pwr_ctrl).await;
        PowerControl::from_byte(byte)
    }

    pub async fn write_power_control(&mut self, pwr_ctrl: PowerControl) {
        let byte = pwr_ctrl.into_byte();
        self.write_byte(Register::pwr_ctrl, byte).await;
    }

    pub async fn raw_temperature(&mut self) -> u32 {
        let mut result: u32 = 0;

        result |= (self.read_byte(Register::data5).await as u32) << 16;
        result |= (self.read_byte(Register::data4).await as u32) << 8;
        result |= self.read_byte(Register::data3).await as u32;

        result
    }

    pub async fn read_temperature(&mut self) -> f32 {
        let raw_temp = self.raw_temperature().await as f32;
        let calib = self.temp_calibration.unwrap();

        let partial_data1 = raw_temp as f32 - calib.par_t1;
        let partial_data2 = partial_data1 * calib.par_t2;
        let temperature = partial_data2 + (partial_data1 * partial_data1) * calib.par_t3;

        temperature
    }

    /// Software reset, emulates POR
    // pub async fn reset(&mut self) {
    //     self.write_byte(Register::reset, 0xB6).await; // Magic from documentation
    // }

    async fn write_byte(&mut self, reg: Register, byte: u8) {
        self.com.write(self.addr, &[reg as u8, byte]).await.unwrap();
    }

    async fn read_byte(&mut self, reg: Register) -> u8 {
        let mut data: [u8; 1] = [0];
        self.com
            .write_read(self.addr, &[reg as u8], &mut data)
            .await
            .unwrap();
        data[0]
    }
}
