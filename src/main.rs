extern crate byteorder;
extern crate i2cdev;

use byteorder::{BigEndian, ReadBytesExt};
use i2cdev::core::*;
use i2cdev::linux::*;
use std::env;
use std::error::Error;
use std::io;

fn read_reg<E: Error>(bus: &mut I2CDevice<Error=E>, reg: u8, buf: &mut [u8]) -> Result<(), E> {
	try!(bus.write(&[reg]));
	bus.read(buf)
}

fn setup<E: Error + From<io::Error>>(bus: &mut I2CDevice<Error=E>) -> Result<(), E> {
	let mut buf = [0u8; 1];
	try!(read_reg(bus, 0x75, &mut buf));
	if buf[0] != 0x68 {
		return Err(io::Error::new(io::ErrorKind::NotFound, "MPU-9150 WhoAmI returned wrong value").into());
	}

	// Wake device up, and sample at 5Hz
	bus.write(&[0x6b, 0x20, 0x40])
}

#[derive(Debug)]
struct MPUSample {
	accel: [f32; 3],
	temp: f32,
	gyro: [f32; 3],
}

fn parse_sample(buf: [u8; (3 + 1 + 3) * 2]) -> Result<MPUSample, io::Error> {
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

fn read_sample<E: Error + From<io::Error>>(bus: &mut I2CDevice<Error=E>) -> Result<MPUSample, E> {
	let mut buf = [0u8; (3 + 1 + 3) * 2];
	try!(read_reg(bus, 0x3b, &mut buf));
	parse_sample(buf).map_err(Into::into)
}

fn main() {
	let dev = env::args().nth(1).unwrap();
	let mut bus = LinuxI2CDevice::new(dev, 0x68).unwrap();

	setup(&mut bus).unwrap();

	if let Ok(sample) = read_sample(&mut bus) {
		println!("{:?}", sample);
	}
}
