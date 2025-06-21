#![no_main]
#![no_std]

extern crate alloc;

use bevy_ecs::system::{NonSendMut, ResMut};
use core::sync::atomic::{AtomicUsize, Ordering};
use defmt_brtt as _;
use rp235x_hal::i2c::Controller;
use rp235x_hal::pac::I2C1; // global logger

use embedded_hal::spi::MODE_3;
use panic_probe as _;

use rp235x_hal::I2C;
// Alias for our HAL crate
use rp235x_hal::{self as hal, gpio::FunctionSioOutput};

use bevy_app::{App, Plugin};
use bevy_ecs::prelude::Resource;
// use defmt_rtt as _;
use display_interface_spi::SPIInterface;
use hal::fugit::RateExtU32;
use hal::gpio::bank0::{Gpio10, Gpio11, Gpio12, Gpio13, Gpio14, Gpio6, Gpio7};
use hal::gpio::{FunctionI2C, FunctionSpi, Pin, PullDown, PullUp};
use hal::spi::Enabled;
use hal::Spi;
use hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use ili9486::{color::PixelFormat, io::shim::OutputOnlyIoPin};
use ili9486::{Command, Commands, ILI9486};
use pac::SPI1;

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

pub const _REG_VER: u8 = 0x01; // fw version
pub const _REG_CFG: u8 = 0x02; //  config
pub const _REG_INT: u8 = 0x03; //  interrupt status
pub const _REG_KEY: u8 = 0x04; //  key status
pub const _REG_BKL: u8 = 0x05; //  backlight
pub const _REG_DEB: u8 = 0x06; //  debounce cfg
pub const _REG_FRQ: u8 = 0x07; //  poll freq cfg
pub const _REG_RST: u8 = 0x08; //  reset
pub const _REG_FIF: u8 = 0x09; //  fifo
pub const _REG_BK2: u8 = 0x0A; //  backlight 2
pub const _REG_BAT: u8 = 0x0B; //  battery
pub const _REG_DIR: u8 = 0x0C; //  gpio direction
pub const _REG_PUE: u8 = 0x0D; //  gpio input pull enable
pub const _REG_PUD: u8 = 0x0E; //  gpio input pull direction
pub const _REG_GIO: u8 = 0x0F; //  gpio value
pub const _REG_GIC: u8 = 0x10; //  gpio interrupt config
pub const _REG_GIN: u8 = 0x11; //  gpio interrupt status
pub const _KEY_COUNT_MASK: u8 = 0x1F;
pub const _WRITE_MASK: u8 = 1 << 7;
pub const _STATE_IDLE: u8 = 0;
pub const _STATE_PRESS: u8 = 1;
pub const _STATE_LONG_PRESS: u8 = 2;
pub const _STATE_RELEASE: u8 = 3;

pub struct PiPicoDemoPlugin;

impl Plugin for PiPicoDemoPlugin {
    fn build(&self, app: &mut App) {
        let mut pac = pac::Peripherals::take().unwrap();
        let mut watchdog = Watchdog::new(pac.WATCHDOG);
        let sio = Sio::new(pac.SIO);

        let clocks = init_clocks_and_plls(
            XTAL_FREQ_HZ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

        let pins = hal::gpio::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        // SETUP SCREEN
        // These are implicitly used by the spi driver if they are in the correct mode
        let dc = pins.gpio14.into_push_pull_output();
        let cs = pins.gpio13.into_push_pull_output();
        let rst = pins.gpio15.into_push_pull_output();
        let spi_mosi = pins.gpio11.into_function::<hal::gpio::FunctionSpi>();
        let spi_miso = pins.gpio12.into_function::<hal::gpio::FunctionSpi>();
        // #define RST_PIN 15
        let spi_sclk = pins.gpio10.into_function::<hal::gpio::FunctionSpi>();
        let spi_bus = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI1, (spi_mosi, spi_miso, spi_sclk));

        // Exchange the uninitialised SPI driver for an initialised one
        let spi = spi_bus.init(
            &mut pac.RESETS,
            clocks.peripheral_clock.freq(),
            200_000_000u32.Hz(),
            MODE_3,
        );

        let display_spi = SPIInterface::new(spi, dc, cs);

        let mut lcd_driver = ILI9486::new(
            &mut timer,
            PixelFormat::Rgb565,
            display_spi,
            OutputOnlyIoPin::new(rst),
        )
        .unwrap();

        // reset
        lcd_driver.write_command(Command::Nop, &[]).unwrap();
        lcd_driver.write_command(Command::SleepOut, &[]).unwrap();

        lcd_driver
            .write_command(Command::DisplayInversionOn, &[])
            .unwrap();

        // MADCTL settings
        lcd_driver
            .write_command(Command::MemoryAccessControl, &[0b01001000])
            .unwrap();

        lcd_driver.clear_screen().unwrap();

        // turn on display
        lcd_driver
            .write_command(Command::NormalDisplayMode, &[])
            .unwrap();
        lcd_driver.write_command(Command::DisplayOn, &[]).unwrap();
        lcd_driver.write_command(Command::IdleModeOff, &[]).unwrap();
        lcd_driver
            .write_command(Command::TearingEffectLineOn, &[])
            .unwrap();

        // SETUP KEEB
        let keeb_addr = 0x1f;
        let i2c_freq = 200_000;
        let sda_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio6.reconfigure();
        let scl_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio7.reconfigure();

        // Create the I²C drive, using the two pre-configured pins. This will fail
        // at compile time if the pins are in the wrong mode, or if this I²C
        // peripheral isn't available on these pins!
        let i2c = hal::I2C::i2c1(
            pac.I2C1,
            sda_pin,
            scl_pin,
            i2c_freq.Hz(),
            &mut pac.RESETS,
            &clocks.system_clock,
        );

        app.set_runner(|mut app| loop {
            app.update();

            if let Some(exit) = app.should_exit() {
                return exit;
            }
        })
        .insert_non_send_resource(Keeb {
            i2c,
            adr: keeb_addr,
            // is_shift: false,
            // is_ctrl: false,
            // is_alt: false,
        })
        .insert_non_send_resource(Display { output: lcd_driver })
        .insert_non_send_resource(timer)
        .insert_resource(KeyPresses::default());
    }
}

pub fn get_key_report(mut key_presses: ResMut<KeyPresses>, mut keyboard: NonSendMut<Keeb>) {
    let num_keys = keyboard.key_count();
    key_presses.step();

    for _ in 0..num_keys {
        if let Some(key_got) = keyboard.key_event() {
            let state = key_got[0];
            let key = key_got[1];

            if state == _STATE_PRESS {
                key_presses.press_key(key);
            } else if state == _STATE_LONG_PRESS {
                key_presses.long_press_key(key);
            } else if state == _STATE_RELEASE {
                key_presses.release_key(key);
            } else {
                key_presses.press_key(key);
            }

            // if state == _STATE_PRESS || state == _STATE_LONG_PRESS {
            //     match key {
            //         0xa2 | 0xa3 => self.is_shift = true,
            //         0xa5 => self.is_ctrl = true,
            //         0xa1 => self.is_alt = true,
            //     };
            // } else {
            //     match key {
            //         0xa2 | 0xa3 => self.is_shift = false,
            //         0xa5 => self.is_ctrl = false,
            //         0xa1 => self.is_alt = false,
            //         _ => {}
            //     };
            // }
        }
    }
}

#[derive(Default, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum KeyState {
    JustPressed,
    JustLongPressed,
    LongPressed,
    Pressed,
    JustReleased,
    JustLongReleased,
    #[default]
    Released,
}

#[derive(Resource)]
pub struct KeyPresses {
    keys: [KeyState; 256],
}

impl Default for KeyPresses {
    fn default() -> Self {
        Self {
            keys: [KeyState::Released; 256],
        }
    }
}

impl KeyPresses {
    pub fn press_key(&mut self, key: impl Into<u8>) {
        let key: u8 = key.into();

        self.keys[key as usize] = KeyState::JustPressed;
    }

    pub fn long_press_key(&mut self, key: impl Into<u8>) {
        let key: u8 = key.into();

        self.keys[key as usize] = KeyState::JustLongPressed;
    }

    pub fn release_key(&mut self, key: impl Into<u8>) {
        let key: u8 = key.into();
        let old = self.keys[key as usize];

        match old {
            KeyState::JustLongPressed | KeyState::LongPressed => {
                self.keys[key as usize] = KeyState::JustLongReleased
            }
            KeyState::JustPressed | KeyState::Pressed => {
                self.keys[key as usize] = KeyState::JustReleased
            }
            _ => self.keys[key as usize] = KeyState::Released,
        }
    }

    pub fn step(&mut self) {
        self.keys.iter_mut().for_each(|state| match state {
            KeyState::JustLongPressed => *state = KeyState::LongPressed,
            KeyState::JustLongReleased => *state = KeyState::Released,
            KeyState::JustPressed => *state = KeyState::Pressed,
            KeyState::JustReleased => *state = KeyState::Released,
            _ => {}
        });
    }

    pub fn is_pressed(&self, key: impl Into<u8>) -> bool {
        let key: u8 = key.into();
        let state = self.keys[key as usize];

        match state {
            KeyState::JustLongPressed | KeyState::JustPressed | KeyState::Pressed => true,
            _ => false,
        }
    }
}

pub struct Keeb {
    i2c: I2C<
        I2C1,
        (
            Pin<Gpio6, FunctionI2C, PullUp>,
            Pin<Gpio7, FunctionI2C, PullUp>,
        ),
        Controller,
    >,
    adr: u8,
    // is_shift: bool,
    // is_ctrl: bool,
    // is_alt: bool,
}

impl Keeb {
    pub fn read_reg16(&mut self, reg: u8) -> Option<[u8; 2]> {
        let temp = reg;
        let mut buffer = [0; 2];
        self.i2c
            .write_iter_read(self.adr, [temp], &mut buffer)
            .map_or(None, |_| Some(buffer))
    }

    pub fn key_count(&mut self) -> u8 {
        if let Some(buf) = self.read_reg16(_REG_KEY) {
            buf[0] & _KEY_COUNT_MASK
        } else {
            0
        }
    }

    pub fn key_event(&mut self) -> Option<[u8; 2]> {
        // if self.key_count() == 0 {
        // return None;
        // } else {
        self.read_reg16(_REG_FIF)
        // }
    }

    // pub fn get_key_report(&mut self) {
    //     let num_keys = self.key_count();
    //
    //     for _ in 0..num_keys {
    //         if let Some(key_got) = self.key_event() {
    //             let state = key_got[0];
    //             let key = key_got[1];
    //
    //             // if state == _STATE_PRESS || state == _STATE_LONG_PRESS {
    //             //     match key {
    //             //         0xa2 | 0xa3 => self.is_shift = true,
    //             //         0xa5 => self.is_ctrl = true,
    //             //         0xa1 => self.is_alt = true,
    //             //     };
    //             // } else {
    //             //     match key {
    //             //         0xa2 | 0xa3 => self.is_shift = false,
    //             //         0xa5 => self.is_ctrl = false,
    //             //         0xa1 => self.is_alt = false,
    //             //         _ => {}
    //             //     };
    //             // }
    //         }
    //     }
    // }
}

pub struct Display {
    pub output: ILI9486<
        SPIInterface<
            Spi<
                Enabled,
                SPI1,
                (
                    Pin<Gpio11, FunctionSpi, PullDown>,
                    Pin<Gpio12, FunctionSpi, PullDown>,
                    Pin<Gpio10, FunctionSpi, PullDown>,
                ),
            >,
            Pin<Gpio14, FunctionSioOutput, PullDown>,
            Pin<Gpio13, FunctionSioOutput, PullDown>,
        >,
        u8,
    >,
    // pub spi: SPIInterface<
    //     Spi<
    //         Enabled,
    //         SPI1,
    //         (
    //             Pin<Gpio11, FunctionSpi, PullDown>,
    //             Pin<Gpio12, FunctionSpi, PullDown>,
    //             Pin<Gpio10, FunctionSpi, PullDown>,
    //         ),
    //     >,
    //     Pin<Gpio14, FunctionSioOutput, PullDown>,
    //     Pin<Gpio13, FunctionSioOutput, PullDown>,
    // >,
}

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
});

/// Terminates the application and makes `probe-rs` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
