#![no_std]

pub mod registers;

use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

const LSM6_SA0_HIGH_ADDRESS: u8 = 0b1101011;
const LSM6_SA0_LOW_ADDRESS: u8 = 0b1101010;
const LSM6_WHO_ID: u8 = 0x69;

/// Different modes and frequency that the accelerometer can run at.
pub enum AccelerometerMode {
    PowerDown,
    LowPower13Hz,
    LowPower26Hz,
    LowPower52Hz,
    Normal104Hz,
    Normal208Hz,
    HighPerformance416Hz,
    HighPerformance833Hz,
    HighPerformance1660Hz,
    HighPerformance3330Hz,
    HighPerformance6660Hz,
}

impl AccelerometerMode {
    fn to_bitcode(self) -> u8 {
        match self {
            AccelerometerMode::PowerDown => 0,
            AccelerometerMode::LowPower13Hz => 1,
            AccelerometerMode::LowPower26Hz => 0b10,
            AccelerometerMode::LowPower52Hz => 0b11,
            AccelerometerMode::Normal104Hz => 0b100,
            AccelerometerMode::Normal208Hz => 0b101,
            AccelerometerMode::HighPerformance416Hz => 0b110,
            AccelerometerMode::HighPerformance833Hz => 0b111,
            AccelerometerMode::HighPerformance1660Hz => 0b1000,
            AccelerometerMode::HighPerformance3330Hz => 0b1001,
            AccelerometerMode::HighPerformance6660Hz => 0b1010,
        }
    }
}

/// Different modes and frequency that the gyroscope can run at.
pub enum GyroscopeMode {
    PowerDown,
    LowPower13Hz,
    LowPower26Hz,
    LowPower52Hz,
    Normal104Hz,
    Normal208Hz,
    HighPerformance416Hz,
    HighPerformance833Hz,
    HighPerformance1660Hz,
}

impl GyroscopeMode {
    fn to_bitcode(self) -> u8 {
        match self {
            GyroscopeMode::PowerDown => 0,
            GyroscopeMode::LowPower13Hz => 1,
            GyroscopeMode::LowPower26Hz => 0b10,
            GyroscopeMode::LowPower52Hz => 0b11,
            GyroscopeMode::Normal104Hz => 0b100,
            GyroscopeMode::Normal208Hz => 0b101,
            GyroscopeMode::HighPerformance416Hz => 0b110,
            GyroscopeMode::HighPerformance833Hz => 0b111,
            GyroscopeMode::HighPerformance1660Hz => 0b1000,
        }
    }
}

pub struct LSM6<E, I: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>> {
    address: u8,
    i2c: I,
}

impl<E, I: Clone + Read<Error = E> + Write<Error = E> + WriteRead<Error = E>> Clone for LSM6<E, I> {
    fn clone(&self) -> Self {
        LSM6 {
            address: self.address.clone(),
            i2c: self.i2c.clone(),
        }
    }
}

impl<E, I: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>> LSM6<E, I> {
    /// Create a new `LSMD6` from an i2c implementor.
    /// This function will automatically set the slave address.
    /// This will also set the CTR3_C register of the LSM6 to 4,
    /// but it will NOT set the mode of either sensor or turn them on.
    pub fn new(mut i2c: I) -> Result<Option<Self>, E> {
        // Get the correct address for the lsm6 that is being used
        let address = if test_lsm6_addr(&mut i2c, LSM6_SA0_HIGH_ADDRESS)? {
            LSM6_SA0_HIGH_ADDRESS
        } else if test_lsm6_addr(&mut i2c, LSM6_SA0_LOW_ADDRESS)? {
            LSM6_SA0_LOW_ADDRESS
        } else {
            return Ok(None);
        };

        // Set automatic register incrementing between reads
        let mut this = Self { address, i2c };
        this.set_register(registers::CTRL3_C, 4)?;

        Ok(Some(this))
    }

    /// Turns on both sensors in high performance mode.
    pub fn init_default(&mut self) -> Result<(), E> {
        self.set_accel_mode(AccelerometerMode::HighPerformance1660Hz)?;
        self.set_gyro_mode(GyroscopeMode::HighPerformance1660Hz)
    }

    /// Powers down both sensors.
    pub fn full_power_down(&mut self) -> Result<(), E> {
        self.set_accel_mode(AccelerometerMode::PowerDown)?;
        self.set_gyro_mode(GyroscopeMode::PowerDown)
    }

    /// This overwrites the CTRL1_XL register.
    pub fn set_accel_mode(&mut self, mode: AccelerometerMode) -> Result<(), E> {
        self.set_register(registers::CTRL1_XL, mode.to_bitcode() << 4)
    }

    /// This overwrites the CTRL2_G register.
    pub fn set_gyro_mode(&mut self, mode: GyroscopeMode) -> Result<(), E> {
        self.set_register(registers::CTRL2_G, mode.to_bitcode() << 4)
    }

    /// Sets which axes of the accelerometer are enabled. 
    /// The result of `LSM6::read_accel` will remain structurally the same,
    /// although the output it gives for a disabled axis should be ignored.
    /// This overwrites the CTRL9_XL register.
    pub fn set_accel_axes(&mut self, x: bool, y: bool, z: bool) -> Result<(), E> {
        self.set_register(
            registers::CTRL9_XL,
            if x { 0b100000 } else { 0 } | if y { 0b10000 } else { 0 } | if z { 0b1000 } else { 0 },
        )
    }

    /// Sets which axes of the gyroscope are enabled. 
    /// The result of `LSM6::read_gyro` will remain structurally the same,
    /// although the output it gives for a disabled axis should be ignored.
    /// This overwrites the CTRL10_C register.
    pub fn set_gyro_axes(&mut self, x: bool, y: bool, z: bool) -> Result<(), E> {
        let prev = self.read_register(registers::CTRL10_C)?;
        self.set_register(
            registers::CTRL10_C,
            if x { 0b100000 } else { 0 }
                | if y { 0b10000 } else { 0 }
                | if z { 0b1000 } else { 0 }
                | (prev & 7),
        )
    }

    /// Set one of the LSM6's register to a certain value.
    /// Be wary when using this manually, as you may override
    /// an important setting.
    pub fn set_register(&mut self, reg: u8, value: u8) -> Result<(), E> {
        self.i2c.write(self.address, &[reg, value])
    }

    /// Read one of the LSM6's registers.
    pub fn read_register(&mut self, reg: u8) -> Result<u8, E> {
        let mut resp = [0];
        self.i2c.write_read(self.address, &[reg], &mut resp)?;
        Ok(resp[0])
    }

    /// Reads the latest acceleration data, returning `Ok(None)` if any is not ready.
    /// A `None` return does not necessarily indicate that anything has failed,
    /// and this function can be called immediately afterwards.
    /// This method of extracting measurements only works if bit 2 (0-indexed) of the CTRL_3C register is set to 1
    /// (which automatically happens in `LSMG::new`). It also assumes that the data is given in little endian, which is true
    /// when bit 1 of the CTRL_3C register is set to 0. 
    pub fn read_gyro(&mut self) -> Result<Option<(i16, i16, i16)>, E> {
        if self.read_register(registers::STATUS_REG)? & 0b10 != 0b10 {
            return Ok(None);
        }
        self.incremental_read_measurements(registers::OUTX_L_G)
            .map(|o| Some(o))
    }

    /// Reads the latest gyroscopic data, returning `Ok(None)` if any is not ready.
    /// A `None` return does not necessarily indicate that anything has failed,
    /// and this function can be called immediately afterwards.
    /// This method of extracting measurements only works if the 2nd bit (0-indexed) of the CTRL_3C register is set to 1
    /// (which automatically happens in `LSMG::new`). It also assumes that the data is given in little endian, which is true
    /// when bit 1 of the CTRL_3C register is set to 0. 
    pub fn read_accel(&mut self) -> Result<Option<(i16, i16, i16)>, E> {
        if self.read_register(registers::STATUS_REG)? & 0b1 != 1 {
            return Ok(None);
        }
        self.incremental_read_measurements(registers::OUTX_L_XL)
            .map(|o| Some(o))
    }

    /// This method of extracting measurements only works if the 2nd bit (0-indexed) of the CTRL_3C register is set to 1.
    fn incremental_read_measurements(&mut self, start_reg: u8) -> Result<(i16, i16, i16), E> {
        let mut values = [0; 6];
        self.i2c
            .write_read(self.address, &[start_reg], &mut values)?;

        Ok((
            (values[1] as i16) << 8 | values[0] as i16,
            (values[3] as i16) << 8 | values[2] as i16,
            (values[5] as i16) << 8 | values[4] as i16,
        ))
    }
}

fn test_lsm6_addr<I: WriteRead>(i2c: &mut I, address: u8) -> Result<bool, I::Error> {
    let mut resp = [LSM6_WHO_ID + 1];
    i2c.write_read(address, &[registers::WHO_AM_I], &mut resp)?;
    Ok(resp[0] == LSM6_WHO_ID)
}
