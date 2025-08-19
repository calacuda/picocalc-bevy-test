#![no_main]
#![no_std]

extern crate alloc;

use bevy::prelude::*;
use core::convert::Infallible;
use core::sync::atomic::{AtomicUsize, Ordering};
use defmt_brtt as _;
use embedded_gfx::{K3dengine, PI};
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::{pixelcolor::Rgb565, prelude::*, Pixel};
use embedded_hal::spi::MODE_3;
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{BlockDevice, SdCard, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use fugit::Rate;
use hal::gpio::FunctionSioOutput;
use hal::i2c::Controller;
use hal::pac::I2C1;
use hal::pac::SPI0;
use hal::timer::CopyableTimer0;
use hal::{Timer, I2C};
use ili9486::Command;
use nalgebra::{OPoint, Point3};
use panic_probe as _;
// use defmt_rtt as _;
use display_interface_spi::SPIInterface;
use hal::fugit::RateExtU32;
use hal::gpio::bank0::*;
use hal::gpio::{FunctionI2C, FunctionSpi, Pin, PullDown, PullUp};
use hal::spi::Enabled;
use hal::Spi;
use hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    powman::Powman,
    sio::Sio,
    watchdog::Watchdog,
};
pub use ili9486 as screen;
use ili9486::{color::PixelFormat, io::shim::OutputOnlyIoPin, Commands, ILI9486};
use pac::SPI1;
pub use rp235x_hal as hal; // Alias for our HAL crate
use usb_device::bus::UsbBusAllocator;
use usb_device::device::{StringDescriptors, UsbDeviceBuilder, UsbVidPid};
use usbd_serial::SerialPort;
pub mod keys;

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
pub const XTAL_FREQ_HZ: u32 = 12_000_000u32;

// Keyboard I2C stuff
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

pub struct PicoCalcDefaultPlugins;

impl Plugin for PicoCalcDefaultPlugins {
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
        // Pin<Gpio11, FunctionSpi, PullDown>,
        // Pin<Gpio12, FunctionSpi, PullDown>,
        // Pin<Gpio10, FunctionSpi, PullDown>,

        // These are implicitly used by the spi driver if they are in the correct mode
        // These are implicitly used by the spi driver if they are in the correct mode
        let dc = pins.gpio14.into_push_pull_output();
        let cs = pins.gpio13.into_push_pull_output();
        let mut rst = OutputOnlyIoPin::new(pins.gpio15.into_push_pull_output());
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

        let mut lcd_driver =
            ILI9486::new(&mut timer, PixelFormat::Rgb565, display_spi, &mut rst).unwrap();

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
        let i2c_freq = 200_000.Hz();
        let sda_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio6.reconfigure();
        let scl_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio7.reconfigure();

        // Create the I²C drive, using the two pre-configured pins. This will fail
        // at compile time if the pins are in the wrong mode, or if this I²C
        // peripheral isn't available on these pins!
        let i2c = hal::I2C::i2c1(
            pac.I2C1,
            sda_pin,
            scl_pin,
            i2c_freq,
            &mut pac.RESETS,
            &clocks.system_clock,
        );

        // TIMER setup
        let powman = Powman::new(pac.POWMAN, None);
        let pico_timer = PicoTimer::new(powman);

        // SD card osetup
        let cs = pins.gpio17.into_push_pull_output();
        let spi_mosi = pins.gpio19.into_function::<hal::gpio::FunctionSpi>();
        let spi_miso = pins.gpio16.into_function::<hal::gpio::FunctionSpi>();
        let spi_sclk = pins.gpio18.into_function::<hal::gpio::FunctionSpi>();
        let spi_bus = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sclk));

        let spi = spi_bus.init(
            &mut pac.RESETS,
            clocks.peripheral_clock.freq(),
            1_000_000.Hz(), // card initialization happens at low baud rate
            embedded_hal::spi::MODE_0,
        );
        let spi = ExclusiveDevice::new(spi, cs, timer).unwrap();
        let sdcard = SdCard::new(spi, timer);
        let volume_mgr = VolumeManager::new(sdcard, DummyTimesource::default());
        let fs = FileSystemStruct(volume_mgr);

        app.set_runner(move |mut app| {
            // usb logging
            let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
                pac.USB,
                pac.USB_DPRAM,
                clocks.usb_clock,
                true,
                &mut pac.RESETS,
            ));
            let mut serial = SerialPort::new(&usb_bus);

            let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
                .strings(&[StringDescriptors::default()
                    .manufacturer("calacuda")
                    .product("Ferris")
                    .serial_number("TEST")])
                .unwrap()
                .device_class(2) // 2 for the CDC, from: https://www.usb.org/defined-class-codes
                .build();
            let _ = usb_dev.poll(&mut [&mut serial]);

            serial.write(b"starting bevy\n").unwrap();

            loop {
                let _ = usb_dev.poll(&mut [&mut serial]);

                app.update();

                let mut log_events = {
                    let world = app.world_mut();
                    world.get_resource_mut::<Events<LoggingEnv>>()
                };

                if let Some(ref mut events) = log_events {
                    for event in events.iter_current_update_events() {
                        let _ = serial.write(&event.msg.clone().into_bytes());
                        let _ = serial.write(&['\n' as u8, '\r' as u8]);
                    }

                    events.update();
                }

                if let Some(exit) = app.should_exit() {
                    return exit;
                }
            }
        })
        .add_event::<LoggingEnv>()
        .insert_non_send_resource(Keeb {
            i2c,
            adr: keeb_addr,
            speed: i2c_freq,
        })
        .insert_non_send_resource(Display { output: lcd_driver })
        .insert_non_send_resource(timer)
        .insert_non_send_resource(pico_timer)
        .insert_non_send_resource(rst)
        // TODO: make a non_send_resource to hold the unused pins which are exposed on the side of
        // the device, the remaining I2C interface, the unused UART, & PIO state machines.
        .insert_non_send_resource(fs)
        .insert_resource(KeyPresses::default())
        // .insert_resource(DoubleFrameBuffer::new(320, 320))
        // .insert_non_send_resource(DoubleFrameBuffer::new(lcd_driver, 320, 320))
        .add_systems(Startup, (start_timer, tick_timer, clear_display))
        .add_systems(Update, get_key_report)
        // .add_systems(Update, usb_poll)
        .add_systems(PostUpdate, tick_timer);
    }
}

#[derive(Default, Clone, PartialEq, PartialOrd, Resource)]
pub struct PlayerLocation {
    pub pos: Point3<f32>,
    pub dir: f32,
    pub head: f32,
    pub looking_at: OPoint<f32, nalgebra::Const<3>>,
}

#[derive(Resource)]
pub struct Engine3d {
    pub engine: K3dengine,
    pub changed: bool,
}

impl Engine3d {
    pub fn new(w: u16, h: u16) -> Self {
        let mut engine = K3dengine::new(w, h);
        engine.camera.set_fovy(PI / 4.0);

        Self {
            engine,
            changed: true,
        }
    }
}

#[derive(Resource)]
pub struct DoubleBufferRes<RES>
where
    RES: Resource + Clone + PartialEq,
{
    pub res: [RES; 2],
    i: usize,
    new: bool,
}

impl<RES> DoubleBufferRes<RES>
where
    RES: Resource + Clone + PartialEq,
{
    pub fn new(res: RES) -> Self {
        let res = [res.clone(), res.clone()];

        Self {
            res,
            i: 0,
            new: true,
        }
    }

    pub fn get_active(&mut self) -> &RES {
        &self.res[self.i]
    }

    pub fn get_active_mut(&mut self) -> &mut RES {
        &mut self.res[self.i]
    }

    pub fn get_inactive(&mut self) -> &RES {
        &self.res[(self.i + 1) % 2]
    }

    pub fn switch(&mut self) {
        self.res[(self.i + 1) % 2] = self.res[self.i].clone();

        self.i += 1;
        self.i %= 2;

        self.new = false;
    }

    pub fn was_updated(&self) -> bool {
        self.res[0] != self.res[1] || self.new
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Component)]
pub struct Visible {
    is_visible: bool,
    updated: bool,
}

impl Default for Visible {
    fn default() -> Self {
        Self {
            is_visible: true,
            updated: true,
        }
    }
}

impl Visible {
    pub fn new(is_visible: bool) -> Self {
        Self {
            is_visible,
            updated: true,
        }
    }

    pub fn set_visible(&mut self, is_seen: bool) {
        self.is_visible = is_seen;
        self.updated = true;
    }

    pub fn should_show(&self) -> bool {
        self.is_visible
    }

    pub fn should_rm(&self) -> bool {
        self.is_visible || (!self.is_visible && self.updated)
    }

    pub fn was_rendered(&mut self) {
        self.updated = false;
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
                // key_presses.press_key(key);
            }
        }
    }
}

pub fn start_timer(mut timer: NonSendMut<PicoTimer>) {
    timer.start();
}

pub fn tick_timer(mut timer: NonSendMut<PicoTimer>) {
    timer.tick();
}

#[derive(Default)]
pub struct DummyTimesource();

impl TimeSource for DummyTimesource {
    // In theory you could use the RTC of the rp2040 here, if you had
    // any external time synchronizing device.
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

pub type FS = FileSystem;
pub type FileSystem = FileSystemStruct<
    SdCard<
        ExclusiveDevice<
            // SPI0,
            // SpiDevice<
            Spi<
                Enabled,
                SPI0,
                (
                    Pin<Gpio19, FunctionSpi, PullDown>,
                    Pin<Gpio16, FunctionSpi, PullDown>,
                    Pin<Gpio18, FunctionSpi, PullDown>,
                ),
                8,
            >,
            // >,
            Pin<Gpio17, FunctionSioOutput, PullDown>,
            Timer<CopyableTimer0>,
        >,
        Timer<CopyableTimer0>,
    >,
    DummyTimesource,
>;

// #[derive(Resource)]
pub struct FileSystemStruct<SdCardDev, TimeSourceDev>(pub VolumeManager<SdCardDev, TimeSourceDev>)
where
    SdCardDev: BlockDevice,
    TimeSourceDev: TimeSource;

impl<SdCardDev, TimeSourceDev> FileSystemStruct<SdCardDev, TimeSourceDev>
where
    SdCardDev: BlockDevice,
    TimeSourceDev: TimeSource,
{
    pub fn read_from_file(&mut self, path: impl ToString) -> Result<Vec<u8>, String> {
        let volume0 = self
            .0
            .open_volume(VolumeIdx(0))
            .map_err(|e| format!("{e:?}"))?;
        let root_dir = volume0.open_root_dir().map_err(|e| format!("{e:?}"))?;
        let file = root_dir
            .open_file_in_dir(path.to_string().as_str(), embedded_sdmmc::Mode::ReadOnly)
            .map_err(|e| format!("{e:?}"))?;

        let mut contents = Vec::with_capacity(32);

        while !file.is_eof() {
            let mut buffer = [0u8; 32];
            let num_read = file.read(&mut buffer).unwrap();
            contents.extend(&buffer[0..num_read]);
        }

        Ok(contents)
    }

    pub fn read_text_file(&mut self, path: impl ToString) -> Result<Vec<char>, String> {
        Ok(self
            .read_from_file(path)
            .map(|bytes| bytes.into_iter().map(|byte| byte as char).collect())?)
    }
}
// pub struct FrameBuffer(Vec<Pixel<Rgb565>>);
pub type FrameBuffer = Vec<(bool, Pixel<Rgb565>)>;

// #[derive(Resource)]
pub struct DoubleFrameBuffer {
    fbs: FrameBuffer,
    i: usize,
    w: usize,
    h: usize,
    pub display: RawDisplay,
}

impl DoubleFrameBuffer {
    pub fn new(display: RawDisplay, w: usize, h: usize) -> Self {
        let fb = FrameBuffer::with_capacity(w);
        // let mut fb = [Pixel(Point::new(0, 0), Rgb565::BLACK); 320 * 320];

        // for i in 0..(w * h) {
        //     let x = i % w;
        //     let y = i / h;
        //     fb[i] = Pixel(Point::new(x as i32, y as i32), Rgb565::BLACK);
        // }

        Self {
            fbs: fb,
            i: 0,
            // len: w * h,
            w,
            h,
            display,
        }
    }

    pub fn switch_buffer(&mut self) {
        self.i += 1;
        self.i %= 2;

        self.fbs = FrameBuffer::default();
    }

    pub fn full_clear(&mut self) {
        _ = self.display.clear(Rgb565::BLACK);
    }

    fn clear(&mut self) {
        // _ = self.display.clear(Rgb565::BLACK);
        // if let Err(e) = self
        //     .fbs
        //     .iter()
        //     .map(|p| {
        //         let mut p = p.clone();
        //         p.1 = Rgb565::BLACK;
        //         p
        //     })
        //     .collect::<Vec<_>>()
        //     .into_iter()
        //     .draw(&mut self.display)
        // {
        //     logger.write(LoggingEnv {
        //         msg: format!("{e}"),
        //     });
        // }

        // self.fbs.clear();

        self.fbs.retain(|(displayed, _)| !*displayed);
    }

    pub fn draw_frame(&mut self, logger: &mut EventWriter<LoggingEnv>)
    /* -> impl Iterator<Item = Pixel<<Self as DrawTarget>::Color>> */
    {
        // let fb = FrameBuffer::with_capacity(self.len);
        // &self.fbs[self.i]
        // let frame_buf = self.fbs[self.i]
        //     .clone()
        //     .into_iter()
        //     .zip(self.fbs[(self.i + 1) % 2].clone().into_iter())
        //     // .into_iter()
        //     .filter_map(|(new, old)| {
        //         // if new.0 != old.0 || new.1 != old.1 {
        //         if new != old {
        //             Some(new)
        //         } else {
        //             None
        //         }
        //     })
        //     .collect::<Vec<_>>();

        // let new_pixels = self.fbs[self.i].len();
        // let old_pixels = self.fbs[(self.i + 1) % 2].len();

        // let frame_buf = self.fbs[self.i]
        //     // .clone()
        //     .iter()
        //     // .zip(self.fbs[(self.i + 1) % 2].clone().into_iter())
        //     // .into_iter()
        //     .filter_map(|new_pixel| {
        //         // if new.0 != old.0 || new.1 != old.1 {
        //         if !self.fbs[(self.i + 1) % 2].clone().contains(&new_pixel) {
        //             Some(new_pixel.clone())
        //         } else {
        //             None
        //         }
        //     })
        //     .collect::<Vec<_>>();

        // let n_pixels = frame_buf.len();

        if let Err(e) = self
            .fbs
            .iter()
            .map(|(_, p)| p.clone())
            .draw(&mut self.display)
        {
            logger.write(LoggingEnv {
                msg: format!("{e}"),
            });
        } else {
            // logger.write(LoggingEnv {
            //     msg: format!("rendered {n_pixels} pixels (new: {new_pixels}, old: {old_pixels})"),
            // });
        }

        self.clear();

        // self.fbs.iter_mut().for_each(|(display, _)| );
        self.fbs.iter_mut().for_each(|(displayed, p)| {
            // let mut p = p.clone();
            (*p).1 = Rgb565::BLACK;
            *displayed = true;
            // p
        });

        // self.fbs.clear();

        // fb
    }

    pub fn frame_len(&mut self, logger: &mut EventWriter<LoggingEnv>) {
        let new_pixels = self.fbs.len();
        // let old_pixels = self.fbs[(self.i + 1) % 2].len();

        logger.write(LoggingEnv {
            msg: format!("pixels {new_pixels}"),
        });
    }
}

impl Dimensions for DoubleFrameBuffer {
    fn bounding_box(&self) -> embedded_graphics::primitives::Rectangle {
        Rectangle::with_corners(Point::new(0, 0), Point::new(self.w as i32, self.h as i32))
    }
}

impl DrawTarget for DoubleFrameBuffer {
    type Color = Rgb565;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> core::result::Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        // let w = self.w;
        // let h = self.h;

        for Pixel(coord, color) in pixels.into_iter() {
            // let x = coord.x;
            // let y = coord.y;

            // if x >= w as i32 || y >= h as i32 || x < 0 || y < 0 {
            // let index: usize = x as usize + y as usize * w;
            // self.fbs[self.i][index] = Pixel(coord, color);
            if color != Rgb565::BLACK {
                if let Some(index) = self.fbs.iter().position(|(_, p)| p.0 == coord) {
                    self.fbs[index].0 = false;
                    self.fbs[index].1 .1 = color;
                } else {
                    self.fbs.push((false, Pixel(coord, color)));
                }
            }
            // }
        }
        // self.fbs[self.i].extend(&mut pixels.into_iter());

        Ok(())
    }
}

// #[derive(Resource)]
pub struct PicoTimer {
    pub powman: Powman,
    last_tick_time: u64,
    this_tick_time: u64,
}

impl PicoTimer {
    pub fn new(powman: Powman) -> Self {
        let this_tick_time = powman.aot_get_time();

        Self {
            powman,
            last_tick_time: 0,
            this_tick_time,
        }
    }

    fn start(&mut self) {
        self.powman.aot_start();
    }

    fn tick(&mut self) {
        self.last_tick_time = self.this_tick_time;

        self.this_tick_time = self.powman.aot_get_time();
    }

    pub fn delta_millis(&self) -> u64 {
        self.this_tick_time - self.last_tick_time
    }

    pub fn get_on_time_secs(&self) -> f32 {
        self.this_tick_time as f32 / 1_000.0
    }

    pub fn get_fps(&self) -> f32 {
        1_000.0 / (self.this_tick_time - self.last_tick_time) as f32
    }
}

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
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

#[derive(Resource, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
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

        if !self.is_pressed(key) {
            self.keys[key as usize] = KeyState::JustPressed;
        }
    }

    pub fn long_press_key(&mut self, key: impl Into<u8>) {
        let key: u8 = key.into();

        if !self.is_pressed(key) {
            self.keys[key as usize] = KeyState::JustLongPressed;
        }
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
            _ => {
                if !self.is_pressed(key) {
                    self.keys[key as usize] = KeyState::Released
                }
            }
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

    pub fn get_key_state(&self, key: impl Into<u8>) -> KeyState {
        let key: u8 = key.into();
        self.keys[key as usize]
    }

    pub fn is_pressed(&self, key: impl Into<u8>) -> bool {
        match self.get_key_state(key) {
            KeyState::JustLongPressed | KeyState::JustPressed | KeyState::Pressed => true,
            _ => false,
        }
    }

    pub fn is_long_pressed(&self, key: impl Into<u8>) -> bool {
        match self.get_key_state(key) {
            KeyState::LongPressed => true,
            _ => false,
        }
    }

    pub fn is_released(&self, key: impl Into<u8>) -> bool {
        match self.get_key_state(key) {
            _ => false,
        }
    }

    pub fn just_pressed(&self, key: impl Into<u8>) -> bool {
        match self.get_key_state(key) {
            KeyState::JustPressed => true,
            _ => false,
        }
    }

    pub fn just_long_pressed(&self, key: impl Into<u8>) -> bool {
        match self.get_key_state(key) {
            KeyState::JustLongPressed => true,
            _ => false,
        }
    }

    pub fn just_released(&self, key: impl Into<u8>) -> bool {
        match self.get_key_state(key) {
            KeyState::JustReleased => true,
            _ => false,
        }
    }

    pub fn just_long_released(&self, key: impl Into<u8>) -> bool {
        match self.get_key_state(key) {
            KeyState::JustLongReleased => true,
            _ => false,
        }
    }
}

pub struct Keeb {
    pub i2c: I2C<
        I2C1,
        (
            Pin<Gpio6, FunctionI2C, PullUp>,
            Pin<Gpio7, FunctionI2C, PullUp>,
        ),
        Controller,
    >,
    pub adr: u8,
    pub speed: Rate<u32, 1, 1>,
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
        self.read_reg16(_REG_FIF)
    }
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
}

pub type RawDisplay = ILI9486<
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
>;

pub fn clear_display(mut display: NonSendMut<Display>) {
    let Display { output } = display.as_mut();
    _ = output.clear(Rgb565::BLACK);
}

#[derive(Event, Clone, Eq, PartialEq, PartialOrd, Ord)]
pub struct LoggingEnv {
    pub msg: String,
}

impl LoggingEnv {
    pub fn new(msg: impl core::fmt::Display) -> Self {
        let msg = msg.to_string();

        Self { msg }
    }

    pub fn debug(msg: impl core::fmt::Display) -> Self {
        let msg = format!("[DEBUG] -> {msg}");

        Self { msg }
    }

    pub fn info(msg: impl core::fmt::Display) -> Self {
        let msg = format!("[INFO] -> {msg}");

        Self { msg }
    }

    pub fn warn(msg: impl core::fmt::Display) -> Self {
        let msg = format!("[WARN] -> {msg}");

        Self { msg }
    }

    pub fn error(msg: impl core::fmt::Display) -> Self {
        let msg = format!("[ERROR] -> {msg}");

        Self { msg }
    }
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
