pub mod registers;

use i2c::{Address, ReadWrite};

const LSM6_SA0_HIGH_ADDRESS: u16 = 0b1101011;
const LSM6_SA0_LOW_ADDRESS: u16 = 0b1101010;
const LSM6_WHO_ID: u8 = 0x69;

pub struct LSM6<I: Address + ReadWrite> {
    i2c: I,
}

impl<I: Address + ReadWrite> LSM6<I> {
    pub fn new(mut i2c: I) -> Result<Option<Self>, I::Error> {
        // Set the i2c's address to the correct value
        if !(test_lsm6_addr(&mut i2c, LSM6_SA0_HIGH_ADDRESS)? || test_lsm6_addr(&mut i2c, LSM6_SA0_LOW_ADDRESS)?) {
            return Ok(None);
        };

        // Set automatic register incrementing between reads
        let mut this = Self { i2c };
        this.set_register(registers::CTRL3_C, 0x4)?;

        Ok(Some(this))
    }

    /// Set the accelerometer and gyroscope to 1.66 kHz
    pub fn use_default_res(&mut self) -> Result<(), I::Error> {
        // Accelerometer frequency
        self.set_register(registers::CTRL1_XL, 0x80)?;
        // Gyroscope frequency
        self.set_register(registers::CTRL2_G, 0x80)?;

        Ok(())
    }

    /// Set one of the LSM6's register to a certain value
    pub fn set_register(&mut self, reg: u8, value: u8) -> Result<(), I::Error> {
        self.i2c.i2c_write(&[reg, value])
    }

    /// Read one of the LSM6's registers
    pub fn read_register(&mut self, reg: u8) -> Result<u8, I::Error> {
        self.i2c.i2c_write(&[reg])?;

        let mut resp = [0];
        self.i2c.i2c_read(&mut resp)?;
        Ok(resp[0])
    }

    /// Reads the latest acceleration data, returning `None` if any is not ready.
    /// A `None` return does not necessarily indicate that anything has failed,
    /// and this function can be called immediately afterwards.
    /// This method of extracting measurements only works if the 2nd bit (0-indexed) of the CTRL_3C register is set to 1
    /// (which automatically happens in `LSMG::new`).
    pub fn read_gyro(&mut self) -> Result<Option<(u16, u16, u16)>, I::Error> {
        self.incremental_read_measurements(registers::OUTX_L_G)
    }

    /// Reads the latest gyroscopic data, returning `None` if any is not ready.
    /// A `None` return does not necessarily indicate that anything has failed,
    /// and this function can be called immediately afterwards.
    /// This method of extracting measurements only works if the 2nd bit (0-indexed) of the CTRL_3C register is set to 1
    /// (which automatically happens in `LSMG::new`).
    pub fn read_accel(&mut self) -> Result<Option<(u16, u16, u16)>, I::Error> {
        self.incremental_read_measurements(registers::OUTX_L_XL)
    }

    /// This method of extracting measurements only works if the 2nd bit (0-indexed) of the CTRL_3C register is set to 1.
    fn incremental_read_measurements(
        &mut self,
        start_reg: u8,
    ) -> Result<Option<(u16, u16, u16)>, I::Error> {
        if self.read_register(registers::STATUS_REG)? & 0b1 != 1 {
            return Ok(None);
        }

        self.i2c.i2c_write(&[start_reg])?;

        let mut values = [0; 6];
        self.i2c.i2c_read(&mut values)?;

        Ok(Some((
            (values[1] as u16) << 8 | values[0] as u16,
            (values[3] as u16) << 8 | values[2] as u16,
            (values[5] as u16) << 8 | values[4] as u16,
        )))
    }
}

fn test_lsm6_addr<I: Address + ReadWrite>(i2c: &mut I, address: u16) -> Result<bool, I::Error> {
    i2c.set_slave_address(address, false)?;
    i2c.i2c_write(&[registers::WHO_AM_I])?;

    let mut resp = [LSM6_WHO_ID + 1];
    i2c.i2c_read(&mut resp)?;
    Ok(resp[0] == LSM6_WHO_ID)
}
