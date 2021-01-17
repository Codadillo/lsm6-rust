# lsmd6-rust
This is a library for interacting over i2c with the LSM6DS33 gyroscope and acclerometer. It is based on the [manufacterer's aruduino specific library](https://github.com/pololu/lsm6-arduino) and the [datasheet](https://www.pololu.com/file/0J1088/LSM6DS33-AN4682.pdf). The methods that this library provide are abstracted away from a specific i2c implementation using traits from [embedded-hal](https://crates.io/crates/embedded-hal).


```rust
use lsmd6::LSMD6;
use std::{time::Duration, thread};

fn main() {
    // Get an i2c interface from elsewhere.
    let i2c = something_from_another_library();

    let lsmd6 = LSMD6::new(i2c).unwrap();
    
    // Physically turn on the gyroscope and accelerometer
    // in high performance mode.
    lsmd6.init_default();

    for _ in 0..100 {
        // If any accelerometer measurements are ready log them
        if let Some((x, y, z)) = lsmd6.read_accel().unwrap() {
            println!("A: x={}, y={}, z={}", x, y, z);
        }
    
        // If any gyroscope measurements are ready log them
        if let Some((x, y, z)) = lsmd6.read_gyro().unwrap() {
            println!("G: x={}, y={}, z={}", x, y, z);
        }
    
        thread::sleep(Duration::from_millis(500));
    }

    // Physically turn off the gyroscope and accelerometer
    lsmd6.full_power_down();
}
```
