#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use core::fmt::Write;
use cortex_m_rt::entry;
use cortex_m::prelude::_embedded_hal_serial_Read;
use cortex_m::prelude::_embedded_hal_serial_Write;

use rp_pico as bsp;	/* Remove at some point... */
use rp2040_hal as hal;
use rp2040_hal::gpio::Pin;
use rp2040_hal::gpio::PinId;
use rp2040_hal::gpio::PinMode;
use rp2040_hal::gpio::ValidPinMode;

use bsp::hal::{
	sio::Sio,
	pac,

	clocks::Clock,
	clocks::init_clocks_and_plls,

	watchdog::Watchdog
};

use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use embedded_hal::digital::v2::ToggleableOutputPin;

fn read_uart<D : hal::uart::UartDevice>(uart : &mut hal::uart::UartPeripheral<hal::uart::Enabled, D>) -> u8 {
	let content = uart.read();
	match content {
		Ok(val) => { val }
		Err(_) => { 0 }
	}
}

#[entry]
fn main() -> ! {
	/* Let's assume this is ok. If it wasn't, we probably need a new board. */
	let mut pac = pac::Peripherals::take().unwrap();
	let mut watchdog = Watchdog::new(pac.WATCHDOG);

	let core = pac::CorePeripherals::take().unwrap();

	let extclock_freq : u32 = 12_000_000;
	let clocks = init_clocks_and_plls(
		extclock_freq, pac.XOSC, pac.CLOCKS, pac.PLL_SYS, pac.PLL_USB,
		&mut pac.RESETS, &mut watchdog
	).ok().unwrap();

	let sio = Sio::new(pac.SIO);
	let pins = bsp::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
	let mut led = pins.led.into_push_pull_output();
	let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

	let mut uart = hal::uart::UartPeripheral::<_,_>::new(pac.UART0, &mut pac.RESETS)
		.enable(hal::uart::common_configs::_115200_8_N_1, clocks.peripheral_clock.into()).unwrap();

	let _tx = pins.gpio0.into_mode::<hal::gpio::FunctionUart>();
	let _rx = pins.gpio1.into_mode::<hal::gpio::FunctionUart>();

	loop {
		let cur_char = read_uart(&mut uart);
		if cur_char != 0 {
			if (cur_char == 10 || cur_char == 13) {
				let _ = uart.write_str("\r\n");
			} else { 
				let _ = uart.write(cur_char);
			}

			info!("Toggle!");
			let _ = led.toggle();			
		}
	}
}