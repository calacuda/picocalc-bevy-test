#![no_std]
#![no_main]

extern crate alloc;

// use defmt_rtt as _;
use alloc::format;
use bevy::prelude::*;
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
    keys::{KEY_DOWN, KEY_ENTER, KEY_LEFT, KEY_RIGHT, KEY_UP},
    Display, KeyPresses, LoggingEnv, PicoCalcDefaultPlugins, PicoTimer,
};
use rp235x_hal as hal;

// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[global_allocator]
static HEAP: Heap = Heap::empty();
const HEAP_SIZE: usize = 128 * 1024;

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
#[require(Renderable)]
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
#[require(Renderable)]
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

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Component)]
pub struct Visible {
    is_visible: bool,
    updated: bool,
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

    fn was_rendered(&mut self) {
        self.updated = false;
    }
}

#[derive(Component)]
pub struct FpsText;

#[derive(Component)]
pub struct HeapText;

#[derive(Component)]
pub struct HeapHud;

#[hal::entry]
fn main() -> ! {
    init_heap();

    let pos = Point3::new(0.0, 2.0, 0.0);
    let looking_at = pos + nalgebra::Vector3::new(0.0_f32.cos(), 0.0_f32.sin(), 0.0_f32.sin());

    App::new()
        .add_plugins(PicoCalcDefaultPlugins)
        .init_resource::<Counter>()
        .insert_resource(DoubleBufferRes::new(PlayerLocation {
            pos,
            looking_at,
            ..default()
        }))
        .insert_resource(Engine3d::new(320, 320))
        .add_systems(Startup, (clear_display, setup))
        .add_systems(
            Update,
            (
                walk,
                toggle_heap_hud,
                (draw_fps, draw_heap, update_counter).chain(),
            ),
        )
        .add_systems(PostUpdate, render)
        .run();

    loop {}
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
    // for i in (0..1) {
    cmds.spawn((
        TextComponent {
            text: "Free Heap Mem:".into(),
            point: Point::new(10, 45),
        },
        Visible::new(false),
        HeapHud,
    ));
    // }
    cmds.spawn((
        TextComponent {
            text: "".into(),
            point: Point::new(10, 55),
        },
        Visible::new(false),
        HeapText,
        HeapHud,
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

fn toggle_heap_hud(
    keys: Res<KeyPresses>,
    heap_hud: Query<&mut Visible>,
    mut logger: EventWriter<LoggingEnv>,
) {
    if keys.just_pressed(KEY_ENTER) {
        let mut get_visible = {
            let mut visible = None;

            move |vis: &mut Visible| -> bool {
                if let Some(visible) = visible {
                    logger.write(LoggingEnv {
                        msg: format!("turning heap hud: {}", if visible { "on" } else { "off" }),
                    });
                    visible
                } else {
                    let tmp_vis = !vis.should_show();
                    visible.replace(tmp_vis);
                    tmp_vis
                }
            }
        };

        for ref mut vis in heap_hud {
            let visible = get_visible(vis);

            vis.set_visible(visible);
        }
    }
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

fn draw_fps(
    counter: Res<Counter>,
    time: NonSend<PicoTimer>,
    mut fps: Query<&mut TextComponent, (With<FpsText>, Without<HeapText>)>,
    // mut logger: EventWriter<LoggingEnv>,
) {
    let n = format!("{} ", counter.0);
    let message = format!(
        "{n}| avg-FPS => {:.2}\n{}| cur-FPS => {:.2}",
        counter.0 as f32 / time.get_on_time_secs(),
        (0..n.len()).map(|_| ' ').collect::<String>(),
        time.get_fps(),
    );
    _ = fps
        .single_mut()
        .map(|ref mut counter| counter.set_text(&message));
    // logger.write(LoggingEnv {
    //     msg: format!("fps => {}", time.get_fps()),
    // });
}

fn draw_heap(
    mut heap: Query<&mut TextComponent, (With<HeapText>, Without<FpsText>)>,
    // mut logger: EventWriter<LoggingEnv>,
) {
    let message = format!("{} / {} KiB", HEAP.free() / 1024, HEAP_SIZE / 1024);
    _ = heap
        .single_mut()
        .map(|ref mut counter| counter.set_text(&message));
    // logger.write(LoggingEnv { msg: message });
}

fn render(
    mut display: NonSendMut<Display>,
    // mut display: NonSendMut<DoubleFrameBuffer>,
    mut camera: ResMut<Engine3d>,
    mut player_buf: ResMut<DoubleBufferRes<PlayerLocation>>,
    text_comps: Query<
        (&TextComponent, Option<&mut Visible>),
        (
            Or<(Changed<TextComponent>, Changed<Visible>)>,
            Without<Mesh3D>,
        ),
    >,
    mesh_comps: Query<(Ref<Mesh3D>, Option<&mut Visible>), Without<TextComponent>>,
    // mut logger: EventWriter<LoggingEnv>,
) {
    let Display { output: display } = display.as_mut();
    // let display = &mut *display;

    let cam_changed = camera.changed || player_buf.was_updated();

    let setup_cam = |player: &PlayerLocation, camera: &mut ResMut<Engine3d>| {
        camera.engine.camera.set_position(player.pos);
        let lookat = player.looking_at;
        camera.engine.camera.set_target(lookat);
    };

    for (mesh, vis) in mesh_comps {
        // "unrender" all meshes if changed or camera changed
        if cam_changed && (vis.is_none() || vis.as_ref().is_some_and(|vis| vis.should_rm())) {
            setup_cam(player_buf.get_inactive(), &mut camera);
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

        // "rerender" a ll renderables if changed or camera changed
        if (vis.is_none() || vis.as_ref().is_some_and(|vis| vis.should_show()))
            && (mesh.is_changed() || cam_changed)
        {
            setup_cam(player_buf.get_active(), &mut camera);
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

        vis.map(|ref mut vis| vis.was_rendered());
    }

    let mut style = MonoTextStyle::new(&FONT_6X12, Rgb565::GREEN);
    style.background_color = Some(Rgb565::BLACK);

    for (text, vis) in text_comps {
        let point = text.point;

        if vis.is_none() || vis.as_ref().is_some_and(|vis| vis.should_show()) {
            // let text = text.text.clone();
            Text::new(&text.text, point, style).draw(display).unwrap();
        } else if vis.as_ref().is_some_and(|vis| vis.should_rm()) {
            let text: String = text
                .text
                .chars()
                .map(|c| if !c.is_whitespace() { ' ' } else { c })
                .collect();
            Text::new(&text, point, style).draw(display).unwrap();
        }

        vis.map(|ref mut vis| vis.was_rendered());
    }

    // display.frame_len(&mut logger);
    // display.draw_frame(&mut logger);

    // call DoubleBufferRes::switch()
    player_buf.switch();
    camera.changed = false;
}

fn clear_display(mut display: NonSendMut<Display>) {
    let Display { output } = display.as_mut();
    _ = output.clear(Rgb565::BLACK);
}

// fn clear_display(mut display: NonSendMut<DoubleFrameBuffer>) {
//     display.clear();
// }

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
