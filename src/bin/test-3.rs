#![no_std]
#![no_main]

extern crate alloc;

use alloc::format;
use bevy::prelude::*;
// use defmt_rtt as _;
use embedded_alloc::LlffHeap as Heap;
use embedded_gfx::{
    // camera::Camera,
    draw::draw,
    mesh::{Geometry, K3dMesh, RenderMode},
    K3dengine,
    PI,
};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X12, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};
// use embedded_hal::digital::OutputPin;
// use ili9486::io::{shim::OutputOnlyIoPin, IoPin};
use nalgebra::Point3;
use nalgebra::{ComplexField, OPoint};
use panic_probe as _;
use picocalc_bevy_test::{
    keys::{KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_UP},
    Display, KeyPresses, PicoCalcDefaultPlugins, PicoTimer,
};
use rp235x_hal::{self as hal};

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

#[derive(Default, Clone, PartialEq, PartialOrd, Resource)]
pub struct PlayerLocation {
    pub pos: Point3<f32>,
    pub dir: f32,
    pub head: f32,
    pub looking_at: OPoint<f32, nalgebra::Const<3>>,
}

#[derive(Component, Default)]
pub struct Renderable;

#[derive(Component)]
// #[require(Renderable)]
pub struct TextComponent {
    pub text: String,
    pub point: Point,
}

impl TextComponent {
    pub fn set_text(&mut self, text: &str) {
        self.text = text.into();
    }
}

#[derive(Component)]
// #[require(Renderable)]
pub struct Mesh3D {
    pub vertices: Vec<[f32; 3]>,
    pub lines: Vec<[usize; 2]>,
    pub faces: Vec<[usize; 3]>,
    pub render_mode: RenderMode,
    pub scale: f32,
    pub color: Rgb565,
}

impl Default for Mesh3D {
    fn default() -> Self {
        Self {
            vertices: Vec::new(),
            lines: Vec::new(),
            faces: Vec::new(),
            render_mode: RenderMode::Lines,
            scale: 1.0,
            color: Rgb565::GREEN,
        }
    }
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

#[derive(Component)]
pub struct FpsText;

#[derive(Component)]
pub struct HeapText;

#[hal::entry]
fn main() -> ! {
    init_heap();

    let pos = Point3::new(0.0, 2.0, 0.0);

    App::new()
        .add_plugins(PicoCalcDefaultPlugins)
        .init_resource::<Counter>()
        .insert_resource(DoubleBufferRes::new(PlayerLocation {
            pos,
            looking_at: pos + nalgebra::Vector3::new((0.0).cos(), (0.0).sin(), 0.0_f32.sin()),
            ..default()
        }))
        .insert_resource(Engine3d::new(320, 320))
        .add_systems(Startup, (clear_display, setup))
        .add_systems(Update, (walk, draw_fps, draw_heap, update_counter).chain())
        .add_systems(PostUpdate, render)
        .run();

    loop {}
}

fn setup(mut cmds: Commands) {
    cmds.spawn(TextComponent {
        text: "Frames Rendered:".into(),
        point: Point::new(10, 10),
    });
    cmds.spawn((
        TextComponent {
            text: "".into(),
            point: Point::new(10, 20),
        },
        FpsText,
    ));
    cmds.spawn(TextComponent {
        text: "Heap:".into(),
        point: Point::new(10, 35),
    });
    cmds.spawn((
        TextComponent {
            text: "".into(),
            point: Point::new(10, 45),
        },
        HeapText,
    ));

    let (vertices, lines) = make_xz_plane();

    cmds.spawn(Mesh3D {
        vertices,
        lines,
        scale: 2.0,
        color: Rgb565::new(0, 127, 255),
        ..default()
    });
}

fn walk(
    mut player_buf: ResMut<DoubleBufferRes<PlayerLocation>>,
    keys: Res<KeyPresses>,
    time: NonSend<PicoTimer>,
) {
    let player = player_buf.get_active_mut();

    let ft = time.delta_millis();
    let dt = ft as f32 / 1000.0;
    let walking_speed = 5.0 * dt;
    let turning_speed = 0.6 * dt;

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

    let new_look_at =
        player.pos + nalgebra::Vector3::new(player.dir.cos(), player.head.sin(), player.dir.sin());

    if player.looking_at != new_look_at {
        player.looking_at = new_look_at;
    }
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

fn draw_fps(
    counter: Res<Counter>,
    time: NonSend<PicoTimer>,
    mut fps: Query<&mut TextComponent, (With<FpsText>, Without<HeapText>)>,
) {
    let message = format!(
        "{} | {:.2}: fps",
        counter.0,
        counter.0 as f32 / time.get_on_time_secs()
    );
    _ = fps
        .single_mut()
        .map(|ref mut counter| counter.set_text(&message));
}

fn draw_heap(mut heap: Query<&mut TextComponent, (With<HeapText>, Without<FpsText>)>) {
    let message = format!("{} / {} KiB", HEAP.free() / 1024, HEAP_SIZE / 1024);
    _ = heap
        .single_mut()
        .map(|ref mut counter| counter.set_text(&message));
}

fn render(
    mut display: NonSendMut<Display>,
    mut camera: ResMut<Engine3d>,
    mut player_buf: ResMut<DoubleBufferRes<PlayerLocation>>,
    text_comps: Query<&TextComponent, Changed<TextComponent>>,
    mesh_comps: Query<Ref<Mesh3D>>,
) {
    let Display { output: display } = display.as_mut();
    let cam_changed = camera.changed || player_buf.was_updated();

    let setup_cam = |player: &PlayerLocation, camera: &mut ResMut<Engine3d>| {
        camera.engine.camera.set_position(player.pos);
        let lookat = player.looking_at;
        camera.engine.camera.set_target(lookat);
    };

    if cam_changed {
        setup_cam(player_buf.get_inactive(), &mut camera);
        // "unrender" all meshes if changed or camera changed
        for mesh in mesh_comps {
            let mut renderable = K3dMesh::new(Geometry {
                vertices: &mesh.vertices,
                faces: &[],
                colors: &[],
                lines: &mesh.lines,
                normals: &[],
            });
            renderable.set_render_mode(mesh.render_mode);
            renderable.set_scale(mesh.scale);
            renderable.set_color(Rgb565::BLACK);
            camera.engine.render([&renderable], |p| draw(p, display))
        }
    }

    // "rerender" all renderables if changed or camera changed

    setup_cam(player_buf.get_active(), &mut camera);

    for mesh in mesh_comps {
        if mesh.is_changed() || cam_changed {
            let mut renderable = K3dMesh::new(Geometry {
                vertices: &mesh.vertices,
                faces: &[],
                colors: &[],
                lines: &mesh.lines,
                normals: &[],
            });
            renderable.set_render_mode(mesh.render_mode);
            renderable.set_scale(mesh.scale);
            renderable.set_color(mesh.color);
            camera.engine.render([&renderable], |p| draw(p, display))
        }
    }

    let mut style = MonoTextStyle::new(&FONT_6X12, Rgb565::GREEN);
    style.background_color = Some(Rgb565::BLACK);

    for text in text_comps {
        Text::new(&text.text, text.point, style)
            .draw(display)
            .unwrap();
    }

    // call DoubleBufferRes::switch()
    player_buf.switch();
    camera.changed = false;
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
    hal::binary_info::rp_program_description!(c"Bevy test-3"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];
