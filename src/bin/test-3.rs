#![no_std]
#![no_main]

extern crate alloc;

use alloc::format;
use bevy::prelude::*;
// use defmt_rtt as _;
use embedded_alloc::LlffHeap as Heap;
use embedded_gfx::{
    draw::draw,
    mesh::{Geometry, K3dMesh, RenderMode},
    K3dengine, PI,
};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X12, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};
use nalgebra::Point3;
use nalgebra::{ComplexField, OPoint};
use panic_probe as _;
use picocalc_bevy_test::{
    // get_key_report,
    keys::{KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_UP},
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

#[derive(Resource, Default)]
pub struct Counter(pub u32);

#[derive(Default, Resource)]
pub struct GroundPlane(pub Vec<[f32; 3]>, pub Vec<[usize; 2]>);

#[derive(Default, Resource)]
pub struct PlayerLocation {
    pub pos: Point3<f32>,
    pub dir: f32,
    pub head: f32,
    pub looking_at: OPoint<f32, nalgebra::Const<3>>,
}

#[hal::entry]
fn main() -> ! {
    init_heap();

    let (ground_vertices, ground_lines) = make_xz_plane();
    let pos = Point3::new(0.0, 2.0, 0.0);

    // TODO: add a framebuffer that get written to then send to the display, to avoid ghosting.

    App::new()
        .add_plugins(PicoCalcDefaultPlugins)
        .init_resource::<Counter>()
        .insert_resource(GroundPlane(ground_vertices, ground_lines))
        .insert_resource(PlayerLocation {
            pos,
            looking_at: pos + nalgebra::Vector3::new((0.0).cos(), (0.0).sin(), 0.0_f32.sin()),
            ..default()
        })
        .add_systems(Startup, clear_display)
        .add_systems(Update, (walk, draw_scene, update_counter).chain())
        .run();

    loop {}
}

fn walk(mut player: ResMut<PlayerLocation>, keys: Res<KeyPresses>) {
    let ft = 20;
    let dt = ft as f32 / 60.0; // 1_000_000.0;
    let walking_speed = 5.0 * dt;
    let turning_speed = 0.75 * dt;

    if keys.is_pressed(KEY_UP) {
        player.pos.x += player.dir.cos() * walking_speed;
        player.pos.z += player.dir.sin() * walking_speed;
    }

    if keys.is_pressed(KEY_DOWN) {
        player.pos.x -= player.dir.cos() * walking_speed;
        player.pos.z -= player.dir.sin() * walking_speed;
    }

    if keys.is_pressed(KEY_LEFT) {
        player.dir -= turning_speed;
    }

    if keys.is_pressed(KEY_RIGHT) {
        player.dir += turning_speed;
    }

    player.looking_at =
        player.pos + nalgebra::Vector3::new(player.dir.cos(), player.head.sin(), player.dir.sin());
}

fn make_xz_plane() -> (Vec<[f32; 3]>, Vec<[usize; 2]>) {
    let step = 1.0;
    let nsteps = 32;

    let mut vertices = Vec::new();

    for i in 0..nsteps {
        for j in 0..nsteps {
            vertices.push([
                (i as f32 - nsteps as f32 / 2.0) * step,
                0.0,
                (j as f32 - nsteps as f32 / 2.0) * step,
            ]);
        }
    }

    let mut lines = Vec::new();

    for i in 0..nsteps {
        for j in 0..nsteps {
            let p1 = i * nsteps + j;
            let p2 = p1 + 1;
            let p3 = p1 + nsteps;
            // let p4 = p3 + (nsteps - 1);

            if p1 < vertices.len() && p2 < vertices.len() && p2 % nsteps != 0 {
                lines.push([p1, p2]);
            }

            if p1 < vertices.len() && p3 < vertices.len() {
                lines.push([p1, p3]);
            }
        }
    }

    (vertices, lines)
}

fn draw_scene(
    mut display: NonSendMut<Display>,
    counter: Res<Counter>,
    ground: Res<GroundPlane>,
    player: Res<PlayerLocation>,
) {
    let Display { output: display } = display.as_mut();
    // _ = display.write_command(Command::SoftReset, &[]);
    let (ground_vertices, ground_lines) = (&ground.0, &ground.1);
    let mut style = MonoTextStyle::new(&FONT_6X12, Rgb565::GREEN);
    style.background_color = Some(Rgb565::BLACK);

    // Text::new("Frames Rendered:", Point::new(10, 200), style)
    Text::new("Frames Rendered:", Point::new(10, 10), style)
        .draw(display)
        .unwrap();

    let message = format!("{}", counter.0);

    // Text::new(&message, Point::new(10, 220), style)
    Text::new(&message, Point::new(10, 30), style)
        .draw(display)
        .unwrap();

    // Text::new("Heap:", Point::new(10, 240), style)
    Text::new("Heap:", Point::new(10, 50), style)
        .draw(display)
        .unwrap();

    let message = format!("{} / {} KiB", HEAP.free() / 1024, HEAP_SIZE / 1024);

    // Text::new(&message, Point::new(10, 260), style)
    Text::new(&message, Point::new(10, 70), style)
        .draw(display)
        .unwrap();

    let mut ground = K3dMesh::new(Geometry {
        vertices: ground_vertices,
        faces: &[],
        colors: &[],
        lines: ground_lines,
        normals: &[],
    });
    ground.set_render_mode(RenderMode::Lines);
    // ground.set_scale(1.0);
    ground.set_color(Rgb565::new(0, 127, 255));

    let mut engine = K3dengine::new(320, 320);
    engine.camera.set_position(player.pos);
    // engine.camera.set_target(Point3::new(0.0, 0.0, 0.0));
    engine.camera.set_fovy(PI / 4.0);

    let lookat = player.looking_at;
    engine.camera.set_target(lookat);

    engine.render([&ground], |p| draw(p, display));
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
