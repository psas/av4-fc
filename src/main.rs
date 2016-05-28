#![deny(missing_docs)]

//! This program reads measurements from an MPU-9150 inertial
//! measurement unit attached via I2C.

extern crate byteorder;
extern crate i2cdev;

use byteorder::{BigEndian, ReadBytesExt};
use i2cdev::core::*;
use i2cdev::linux::*;
use std::env;
use std::error::Error;
use std::io;
use std::time::Duration;
use std::thread::sleep;

/// Read a contiguous series of `buf.length` registers from the given
/// I2C device `bus`, starting with `reg`.
fn read_reg<E: Error>(bus: &mut I2CDevice<Error=E>, reg: u8, buf: &mut [u8]) -> Result<(), E> {
	try!(bus.write(&[reg]));
	bus.read(buf)
}

/// Set up an MPU-9150's configuration registers.
pub fn setup<E: Error + From<io::Error>>(bus: &mut I2CDevice<Error=E>) -> Result<(), E> {
	// This sensor has a "WhoAmI" register that, when read, should
	// always return 0x68. If we read that register and get a
	// different value, then this isn't an MPU-family IMU and we
	// shouldn't try to poke at it further.
	let mut buf = [0u8; 1];
	try!(read_reg(bus, 0x75, &mut buf));
	if buf[0] != 0x68 {
		return Err(io::Error::new(io::ErrorKind::NotFound, "MPU-9150 WhoAmI returned wrong value").into());
	}

	// Wake device up, using internal oscillator.
	try!(bus.write(&[0x6b, 0x00]));

	// Set configuration:
	// - Sample rate divider: 1kHz / 200
	// - Config: no FSYNC, low-pass filter at 5Hz
	// - Gyro config: full scale range at +/- 250 dps
	// - Accel config: full scale range at +/- 2g
	bus.write(&[0x19, 199, 0x06, 0x00, 0x00])
}

/// Structure to hold measurements in real units.
#[derive(Debug)]
pub struct MPUSample {
	/// Acceleration X/Y/Z in g's
	pub accel: [f32; 3],
	/// Temperature in degrees Celsius
	pub temp: f32,
	/// Rotational velocity X/Y/Z in degrees/second
	pub gyro: [f32; 3],
}

/// Read an `MPUSample` from the given I2C device, which must have been
/// initialized first using `setup`.
pub fn read_sample<E: Error + From<io::Error>>(bus: &mut I2CDevice<Error=E>) -> Result<MPUSample, E> {
	// This sensor family places the measured values in a contiguous
	// block of registers, which allows us to do a bulk read of all
	// of them at once. And it's important to do the read in bulk,
	// because this hardware locks the register values while we're
	// reading them so that none of the sampled values change
	// mid-read. If we read them byte-at-a-time, we could get a
	// high-order byte from an old sample and a low-order byte from
	// a new sample, and wind up with nonsense numbers.
	let mut buf = [0u8; (3 + 1 + 3) * 2];
	try!(read_reg(bus, 0x3b, &mut buf));

	// If read_i16 returns an error, it will be of type io::Error.
	// However, we're supposed to return errors of the type
	// associated with the I2CDevice implementation we're using. So
	// above we constrained type E to have an implementation of the
	// From trait, which the try! macro will use to convert
	// io::Error to E as needed.
	let mut rdr = io::Cursor::new(buf);
	Ok(MPUSample {
		accel: [
			(try!(rdr.read_i16::<BigEndian>()) as f32) / 16384.0,
			(try!(rdr.read_i16::<BigEndian>()) as f32) / 16384.0,
			(try!(rdr.read_i16::<BigEndian>()) as f32) / 16384.0,
		],
		temp: (try!(rdr.read_i16::<BigEndian>()) as f32) / 340.0 + 35.0,
		gyro: [
			(try!(rdr.read_i16::<BigEndian>()) as f32) / 131.0,
			(try!(rdr.read_i16::<BigEndian>()) as f32) / 131.0,
			(try!(rdr.read_i16::<BigEndian>()) as f32) / 131.0,
		],
	})
}

fn main() {
	let dev = env::args().nth(1)
		.expect(&format!("Usage: {} /dev/i2c-?",
			env::args().nth(0).unwrap_or("program".into())
		));
	let mut bus = LinuxI2CDevice::new(&dev, 0x68)
		.expect(&format!("opening {} failed", &dev));

	setup(&mut bus).unwrap();

	let delay = Duration::from_millis(200);
	while let Ok(sample) = { sleep(delay); read_sample(&mut bus) } {
		println!("{:?}", sample);
	}
}
