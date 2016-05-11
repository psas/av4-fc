extern crate byteorder;
extern crate i2cdev;

use byteorder::{BigEndian, ReadBytesExt};
use i2cdev::core::*;
use i2cdev::linux::*;
use std::env;
use std::io::Cursor;

fn setup() -> LinuxI2CDevice {
	let dev = env::args().nth(1).unwrap();
	let mut bus = LinuxI2CDevice::new(dev, 0x68).unwrap();

	bus.write(&[0x75]);
	let mut buf = [0u8; 1];
	bus.read(&mut buf);
	assert!(buf[0] == 0x68);

	bus.write(&[0x6b, 0x20]);

	bus
}

fn main() {
	let mut bus = setup();

	bus.write(&[0x3b]);
	let mut buf = [0u8; 6];
	bus.read(&mut buf);

	let mut rdr = Cursor::new(buf);
	for _ in 0..3 {
		let v = rdr.read_i16::<BigEndian>().unwrap();
		println!("{}", (v as f32) / 16384.0);
	}
}
