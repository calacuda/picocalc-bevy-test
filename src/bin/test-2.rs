#![no_std]
#![no_main]

extern crate alloc;

use alloc::format;
use bevy::prelude::*;
// use defmt_rtt as _;
use embedded_alloc::LlffHeap as Heap;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X12, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};
use panic_probe as _;
use picocalc_bevy_test::{
    // get_key_report,
    keys::KEY_ENTER,
    Display,
    KeyPresses,
    PicoCalcDefaultPlugins,
};
use rp235x_hal as hal;

// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[global_allocator]
static HEAP: Heap = Heap::empty();
const HEAP_SIZE: usize = 96 * 1024;

const BEVY: &[u8] = include_bytes!("../../assets/bevy_bird_dark.data");

#[derive(Resource, Default)]
pub struct Counter(pub u32);

#[derive(Default, Resource, Deref, DerefMut)]
pub struct DrawBird(pub bool);

#[hal::entry]
fn main() -> ! {
    init_heap();

    App::new()
        .add_plugins(PicoCalcDefaultPlugins)
        .init_resource::<Counter>()
        .insert_resource(DrawBird(true))
        .add_systems(Startup, clear_display)
        .add_systems(Update, (set_draw_bird, draw_scene, update_counter).chain())
        .run();

    loop {}
}

fn set_draw_bird(mut draw_bird: ResMut<DrawBird>, key_presses: Res<KeyPresses>) {
    if key_presses.just_pressed(KEY_ENTER) {
        **draw_bird = !**draw_bird;
    }
}

fn draw_scene(mut display: NonSendMut<Display>, counter: Res<Counter>, draw_bird: Res<DrawBird>) {
    let Display { output: display } = display.as_mut();
    let mut style = MonoTextStyle::new(&FONT_6X12, Rgb565::GREEN);
    style.background_color = Some(Rgb565::BLACK);

    Text::new("Bevy?!", Point::new(10, 30), style)
        .draw(display)
        .unwrap();

    Text::new("Not a toaster...", Point::new(10, 50), style)
        .draw(display)
        .unwrap();

    Text::new("...but still small!", Point::new(10, 70), style)
        .draw(display)
        .unwrap();

    let draw_bird = **draw_bird;

    for (index, value) in BEVY.iter().step_by(2).enumerate() {
        let x = index % 64;
        let y = (index - x) / 64;

        if *value == 0 && draw_bird {
            Pixel(Point::new(32 + x as i32, 96 + y as i32), Rgb565::GREEN)
                .draw(display)
                .unwrap();
        } else {
            Pixel(Point::new(32 + x as i32, 96 + y as i32), Rgb565::BLACK)
                .draw(display)
                .unwrap();
        }
    }

    Text::new("Frames Rendered:", Point::new(10, 200), style)
        .draw(display)
        .unwrap();

    let message = format!("{}", counter.0);

    Text::new(&message, Point::new(10, 220), style)
        .draw(display)
        .unwrap();

    Text::new("Heap:", Point::new(10, 240), style)
        .draw(display)
        .unwrap();

    let message = format!("{} / {} KiB", HEAP.free() / 1024, HEAP_SIZE / 1024);

    Text::new(&message, Point::new(10, 260), style)
        .draw(display)
        .unwrap();
}

fn clear_display(mut display: NonSendMut<Display>) {
    let Display { output } = display.as_mut();
    _ = output.clear(Rgb565::BLACK);
}

fn update_counter(mut counter: ResMut<Counter>) {
    counter.0 += 1;
}

#[allow(static_mut_refs)]
fn init_heap() {
    use core::mem::MaybeUninit;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"Blinky Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];
