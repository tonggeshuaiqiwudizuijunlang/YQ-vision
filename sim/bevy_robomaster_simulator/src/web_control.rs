use avian3d::prelude::*;
use bevy::prelude::*;
use serde::{Deserialize, Serialize};
use std::fs::{self, File};
use std::io::{BufWriter, Read, Seek, SeekFrom, Write};
use std::net::{TcpListener, TcpStream};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::f32::consts::FRAC_PI_2;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::components::{
    ActiveSlapper, Controlled, GameLayer, Infantry, InfantryChassis, InfantryGimbal,
    InfantryLaunchOffset, SlapperInfantry,
};
use crate::robomaster::prelude::HERO_ROBOT_CONFIG;
use crate::statistic::ProjectileStatistics;
use crate::systems::{
    ControllerState, GameplaySystems, GimbalAimTracker, sample_gamepad_controller,
    sample_keyboard_controller, update_auto_aim_subscription,
};

const ARENA_Y: f32 = 20.0;
const ROBOT_SPAWN_Y: f32 = ARENA_Y + 0.55;

#[derive(Component)]
struct FlatArenaFloor;

#[derive(Clone, Debug, Default, Deserialize)]
struct HoldPayload {
    #[serde(default)]
    ctrl_move: [f32; 2],
    #[serde(default)]
    ctrl_gimbal: [f32; 2],
    #[serde(default)]
    ctrl_shoot: bool,
    #[serde(default)]
    ctrl_boost: bool,
    #[serde(default)]
    remote_move: [f32; 2],
    #[serde(default)]
    remote_gimbal: [f32; 2],
    #[serde(default)]
    remote_boost: bool,
}

#[derive(Clone, Debug, Deserialize)]
struct ActionPayload {
    action: String,
    value: Option<f32>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct ArenaConfig {
    #[serde(default = "default_distance")]
    distance_m: f32,
    #[serde(default)]
    move_speed: f32,
    #[serde(default = "default_move_span")]
    move_span: f32,
    #[serde(default)]
    spin_omega: f32,
    #[serde(default)]
    self_spin: bool,
    #[serde(default)]
    self_move_speed: f32,
    #[serde(default = "default_move_span")]
    self_move_span: f32,
    #[serde(default = "default_target_type")]
    target_type: String, // "infantry" | "hero"
    #[serde(default = "default_duration")]
    duration_s: f32, // 0 = continuous
}

fn default_distance() -> f32 {
    5.0
}
fn default_move_span() -> f32 {
    3.0
}
fn default_target_type() -> String {
    "infantry".to_string()
}
fn default_duration() -> f32 {
    15.0
}

impl Default for ArenaConfig {
    fn default() -> Self {
        Self {
            distance_m: default_distance(),
            move_speed: 0.0,
            move_span: default_move_span(),
            spin_omega: 0.0,
            self_spin: false,
            self_move_speed: 0.0,
            self_move_span: default_move_span(),
            target_type: default_target_type(),
            duration_s: default_duration(),
        }
    }
}

#[derive(Clone, Debug, Serialize)]
struct BenchSummary {
    id: u32,
    label: String,
    duration_s: f32,
    launch_count: u32,
    hit_count: u32,
    hit_rate_pct: f32,
    mean_abs_yaw_err: f32,
    mean_abs_pitch_err: f32,
    fire_mean_abs_yaw_err: f32,
    fire_mean_abs_pitch_err: f32,
    csv_file: String,
}

#[derive(Clone, Debug, Serialize)]
struct StateSnapshot {
    auto_aim: bool,
    controlled_spin: bool,
    remote_spin: bool,
    remote_patrol: bool,
    max_fps: u32,
    gimbal_sens: f32,
    ec_yaw_deg: f32,
    ec_pitch_deg: f32,
    vision_yaw_deg: f32,
    vision_pitch_deg: f32,
    vision_valid: bool,
    vision_fire: bool,
    vision_age_ms: u64,
    launch_count: u32,
    hit_count: u32,
    hit_rate_pct: f32,
    arena_active: bool,
    arena_config: ArenaConfig,
    bench_running: bool,
    bench_elapsed_s: f32,
    bench_remaining_s: f32,
    bench_csv: String,
    bench_history: Vec<BenchSummary>,
}

struct ActiveBench {
    id: u32,
    label: String,
    start: Instant,
    duration_s: f32,
    csv_name: String,
    writer: Option<BufWriter<File>>,
    sample_count: u64,
    sum_abs_yaw: f64,
    sum_abs_pitch: f64,
    fire_count: u64,
    fire_sum_abs_yaw: f64,
    fire_sum_abs_pitch: f64,
}

struct SharedWebState {
    hold: HoldPayload,
    hold_updated_at: Instant,
    pending_toggle_auto_aim: bool,
    pending_toggle_ctrl_spin: bool,
    pending_toggle_remote_spin: bool,
    pending_switch_slapper: bool,
    pending_switch_camera: bool,
    pending_dart: bool,
    pending_reset_stats: bool,
    pending_apply_arena: bool,
    pending_exit_arena: bool,
    remote_patrol: bool,
    patrol_start: Instant,
    max_fps: u32,
    gimbal_sens: f32,
    auto_aim_mirror: bool,
    ctrl_spin_mirror: bool,
    remote_spin_mirror: bool,
    initialized_auto_aim: bool,
    ec_yaw_deg: f32,
    ec_pitch_deg: f32,
    vision_yaw_deg: f32,
    vision_pitch_deg: f32,
    vision_valid: bool,
    vision_fire: bool,
    vision_updated_at: Option<Instant>,
    launch_count: u32,
    hit_count: u32,
    arena_active: bool,
    arena_config: ArenaConfig,
    arena_move_dir: f32,
    self_move_dir: f32,
    bench_seq: u32,
    active_bench: Option<ActiveBench>,
    bench_history: Vec<BenchSummary>,
}

impl SharedWebState {
    fn new(max_fps: u32) -> Self {
        Self {
            hold: HoldPayload::default(),
            hold_updated_at: Instant::now() - Duration::from_secs(10),
            pending_toggle_auto_aim: false,
            pending_toggle_ctrl_spin: false,
            pending_toggle_remote_spin: false,
            pending_switch_slapper: false,
            pending_switch_camera: false,
            pending_dart: false,
            pending_reset_stats: false,
            pending_apply_arena: false,
            pending_exit_arena: false,
            remote_patrol: false,
            patrol_start: Instant::now(),
            max_fps,
            gimbal_sens: 0.45,
            auto_aim_mirror: true,
            ctrl_spin_mirror: false,
            remote_spin_mirror: false,
            initialized_auto_aim: false,
            ec_yaw_deg: 0.0,
            ec_pitch_deg: 0.0,
            vision_yaw_deg: 0.0,
            vision_pitch_deg: 0.0,
            vision_valid: false,
            vision_fire: false,
            vision_updated_at: None,
            launch_count: 0,
            hit_count: 0,
            arena_active: false,
            arena_config: ArenaConfig::default(),
            arena_move_dir: 1.0,
            self_move_dir: 1.0,
            bench_seq: 0,
            active_bench: None,
            bench_history: Vec::new(),
        }
    }

    fn snapshot(&self) -> StateSnapshot {
        let age_ms = self
            .vision_updated_at
            .map(|t| t.elapsed().as_millis() as u64)
            .unwrap_or(99999);
        let hit_rate_pct = if self.launch_count == 0 {
            0.0
        } else {
            (self.hit_count as f32 / self.launch_count as f32) * 100.0
        };
        let (bench_running, bench_elapsed_s, bench_remaining_s, bench_csv) =
            if let Some(ref b) = self.active_bench {
                let elapsed = b.start.elapsed().as_secs_f32();
                let rem = if b.duration_s > 0.0 {
                    (b.duration_s - elapsed).max(0.0)
                } else {
                    0.0
                };
                (true, elapsed, rem, b.csv_name.clone())
            } else {
                (false, 0.0, 0.0, String::new())
            };

        StateSnapshot {
            auto_aim: self.auto_aim_mirror,
            controlled_spin: self.ctrl_spin_mirror,
            remote_spin: self.remote_spin_mirror,
            remote_patrol: self.remote_patrol,
            max_fps: self.max_fps,
            gimbal_sens: self.gimbal_sens,
            ec_yaw_deg: self.ec_yaw_deg,
            ec_pitch_deg: self.ec_pitch_deg,
            vision_yaw_deg: self.vision_yaw_deg,
            vision_pitch_deg: self.vision_pitch_deg,
            vision_valid: self.vision_valid && age_ms < 1000,
            vision_fire: self.vision_fire && age_ms < 1000,
            vision_age_ms: age_ms,
            launch_count: self.launch_count,
            hit_count: self.hit_count,
            hit_rate_pct,
            arena_active: self.arena_active,
            arena_config: self.arena_config.clone(),
            bench_running,
            bench_elapsed_s,
            bench_remaining_s,
            bench_csv,
            bench_history: self.bench_history.clone(),
        }
    }
}

#[derive(Resource, Clone)]
pub struct WebControlBridge(Arc<Mutex<SharedWebState>>);

impl WebControlBridge {
    pub fn record_vision_cmd(&self, yaw_deg: f32, pitch_deg: f32, valid: bool, fire: bool) {
        if let Ok(mut st) = self.0.lock() {
            st.vision_yaw_deg = yaw_deg;
            st.vision_pitch_deg = pitch_deg;
            st.vision_valid = valid;
            st.vision_fire = fire;
            st.vision_updated_at = Some(Instant::now());
        }
    }
}

#[derive(Resource)]
struct FrameLimiterState {
    last_frame: Instant,
}

pub struct WebControlPlugin;

impl Plugin for WebControlPlugin {
    fn build(&self, app: &mut App) {
        let max_fps = std::env::var("DAEDALUS_MAX_FPS")
            .ok()
            .and_then(|v| v.parse::<u32>().ok())
            .unwrap_or(60);
        let port = std::env::var("DAEDALUS_WEB_PORT")
            .ok()
            .and_then(|v| v.parse::<u16>().ok())
            .unwrap_or(9001);

        let shared = Arc::new(Mutex::new(SharedWebState::new(max_fps)));
        spawn_http_server(port, shared.clone());

        app.insert_resource(WebControlBridge(shared))
            .insert_resource(FrameLimiterState {
                last_frame: Instant::now(),
            })
            .add_systems(Startup, spawn_flat_arena_floor)
            .add_systems(
                Update,
                sample_web_controller
                    .after(sample_keyboard_controller)
                    .after(sample_gamepad_controller)
                    .before(update_auto_aim_subscription)
                    .in_set(GameplaySystems::Input),
            )
            .add_systems(
                Update,
                drive_flat_arena.in_set(GameplaySystems::GameLogic),
            )
            .add_systems(PostUpdate, sample_ec_gimbal_and_stats)
            .add_systems(Last, limit_frame_rate);
    }
}

fn spawn_flat_arena_floor(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let layers = GameLayer::environment_collision_layers();
    let mesh = meshes.add(Cuboid::new(44.0, 0.4, 44.0));
    let mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.28, 0.30, 0.34),
        perceptual_roughness: 0.9,
        ..default()
    });
    commands.spawn((
        FlatArenaFloor,
        Mesh3d(mesh),
        MeshMaterial3d(mat),
        Transform::from_xyz(0.0, ARENA_Y - 0.2, 0.0),
        RigidBody::Static,
        Collider::cuboid(44.0, 0.4, 44.0),
        layers,
        Friction::new(0.5),
    ));
}

fn bench_dir() -> PathBuf {
    let home = std::env::var("HOME").unwrap_or_else(|_| "/tmp".to_string());
    PathBuf::from(home).join("sim_benchmarks")
}

fn start_new_bench(st: &mut SharedWebState) {
    st.bench_seq += 1;
    let id = st.bench_seq;
    let c = &st.arena_config;
    let label = format!(
        "{} D={:.1}m v={:.1} L={:.1}m ω={:.1}{}",
        if c.target_type == "hero" {
            "英雄"
        } else {
            "步兵"
        },
        c.distance_m,
        c.move_speed,
        c.move_span,
        c.spin_omega,
        if c.self_move_speed > 0.01 {
            format!(" 己v={:.1}", c.self_move_speed)
        } else {
            String::new()
        }
    );
    let ts = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    let csv_name = format!(
        "bench_{}_{}_D{:.1}_v{:.1}_w{:.1}.csv",
        ts, c.target_type, c.distance_m, c.move_speed, c.spin_omega
    );
    let dir = bench_dir();
    let _ = fs::create_dir_all(&dir);
    let writer = File::create(dir.join(&csv_name)).ok().map(|f| {
        let mut bw = BufWriter::new(f);
        let _ = writeln!(
            bw,
            "t_s,dist_m,ec_yaw_deg,ec_pitch_deg,vis_yaw_deg,vis_pitch_deg,err_yaw_deg,err_pitch_deg,vis_valid,vis_fire,target_x,target_y,target_z,target_vx,target_vyaw,launch_count,hit_count"
        );
        bw
    });
    st.active_bench = Some(ActiveBench {
        id,
        label,
        start: Instant::now(),
        duration_s: c.duration_s,
        csv_name,
        writer,
        sample_count: 0,
        sum_abs_yaw: 0.0,
        sum_abs_pitch: 0.0,
        fire_count: 0,
        fire_sum_abs_yaw: 0.0,
        fire_sum_abs_pitch: 0.0,
    });
}

fn finalize_bench(st: &mut SharedWebState) {
    let Some(mut b) = st.active_bench.take() else {
        return;
    };
    if let Some(ref mut w) = b.writer {
        let _ = w.flush();
    }
    let elapsed = b.start.elapsed().as_secs_f32();
    let hit_rate_pct = if st.launch_count == 0 {
        0.0
    } else {
        (st.hit_count as f32 / st.launch_count as f32) * 100.0
    };
    let mean_abs_yaw_err = if b.sample_count > 0 {
        (b.sum_abs_yaw / b.sample_count as f64) as f32
    } else {
        0.0
    };
    let mean_abs_pitch_err = if b.sample_count > 0 {
        (b.sum_abs_pitch / b.sample_count as f64) as f32
    } else {
        0.0
    };
    let fire_mean_abs_yaw_err = if b.fire_count > 0 {
        (b.fire_sum_abs_yaw / b.fire_count as f64) as f32
    } else {
        0.0
    };
    let fire_mean_abs_pitch_err = if b.fire_count > 0 {
        (b.fire_sum_abs_pitch / b.fire_count as f64) as f32
    } else {
        0.0
    };
    st.bench_history.insert(
        0,
        BenchSummary {
            id: b.id,
            label: b.label,
            duration_s: elapsed,
            launch_count: st.launch_count,
            hit_count: st.hit_count,
            hit_rate_pct,
            mean_abs_yaw_err,
            mean_abs_pitch_err,
            fire_mean_abs_yaw_err,
            fire_mean_abs_pitch_err,
            csv_file: b.csv_name,
        },
    );
    if st.bench_history.len() > 15 {
        st.bench_history.truncate(15);
    }
}

fn drive_flat_arena(
    time: Res<Time>,
    bridge: Res<WebControlBridge>,
    mut commands: Commands,
    children: Query<&Children>,
    mut stats: Option<ResMut<ProjectileStatistics>>,
    mut controller: ResMut<ControllerState>,
    mut robots: Query<
        (
            Entity,
            &Infantry,
            &mut Transform,
            &mut LinearVelocity,
            &mut AngularVelocity,
            Option<&Controlled>,
            Option<&SlapperInfantry>,
        ),
        (Without<InfantryGimbal>, Without<InfantryChassis>, Without<InfantryLaunchOffset>),
    >,
    mut gimbals: Query<
        (Entity, &mut Transform, &mut InfantryGimbal),
        (With<Controlled>, Without<Infantry>, Without<InfantryChassis>, Without<InfantryLaunchOffset>),
    >,
    launch_offsets: Query<
        &Transform,
        (With<Controlled>, With<InfantryLaunchOffset>, Without<Infantry>, Without<InfantryGimbal>, Without<InfantryChassis>),
    >,
    mut chassis_q: Query<
        (&mut Transform, &mut InfantryChassis, Option<&Controlled>, Option<&ActiveSlapper>),
        (Without<Infantry>, Without<InfantryGimbal>, Without<InfantryLaunchOffset>),
    >,
) {
    let Ok(mut st) = bridge.0.lock() else {
        return;
    };

    if st.pending_exit_arena {
        st.pending_exit_arena = false;
        st.arena_active = false;
        finalize_bench(&mut st);
        for (_e, inf, mut tf, mut lin, mut ang, is_ctrl, is_slapper) in robots.iter_mut() {
            lin.0 = Vec3::ZERO;
            ang.0 = Vec3::ZERO;
            if is_ctrl.is_some() {
                tf.translation = Vec3::new(0.0, 1.0, 0.0);
                tf.rotation = Quat::IDENTITY;
            } else if is_slapper.is_some() {
                let is_hero = inf.config == HERO_ROBOT_CONFIG;
                tf.translation = if is_hero {
                    Vec3::new(2.0, 1.0, 1.0)
                } else {
                    Vec3::new(1.0, 1.0, 1.0)
                };
                tf.rotation = Quat::IDENTITY;
            }
        }
        return;
    }

    if st.pending_apply_arena {
        st.pending_apply_arena = false;
        st.arena_active = true;
        st.arena_move_dir = 1.0;
        st.self_move_dir = 1.0;
        let dist = st.arena_config.distance_m.clamp(2.0, 15.0);
        let want_hero = st.arena_config.target_type == "hero";

        for (e, inf, mut tf, mut lin, mut ang, is_ctrl, is_slapper) in robots.iter_mut() {
            lin.0 = Vec3::ZERO;
            ang.0 = Vec3::ZERO;
            if is_ctrl.is_some() {
                tf.translation = Vec3::new(0.0, ROBOT_SPAWN_Y, 0.0);
                tf.rotation = Quat::IDENTITY;
            } else if is_slapper.is_some() {
                let is_hero = inf.config == HERO_ROBOT_CONFIG;
                if is_hero == want_hero {
                    tf.translation = Vec3::new(0.0, ROBOT_SPAWN_Y, -dist);
                    tf.rotation = Quat::IDENTITY;
                    commands.entity(e).insert(ActiveSlapper);
                    for desc in children.iter_descendants(e) {
                        commands.entity(desc).insert(ActiveSlapper);
                    }
                } else {
                    tf.translation = Vec3::new(0.0, ROBOT_SPAWN_Y, 16.0);
                    tf.rotation = Quat::IDENTITY;
                    commands.entity(e).remove::<ActiveSlapper>();
                    for desc in children.iter_descendants(e) {
                        commands.entity(desc).remove::<ActiveSlapper>();
                    }
                }
            }
        }

        // Aim controlled gimbal horizontally toward -Z (facing the target at (0, _, -dist))
        let launch_rot = launch_offsets
            .iter()
            .next()
            .map(|t| t.rotation)
            .unwrap_or(Quat::IDENTITY);
        let target_muzzle_rot = Quat::from_euler(EulerRot::YXZ, 0.0, -FRAC_PI_2, 0.0);
        let desired_gimbal_rot = (target_muzzle_rot * launch_rot.inverse()).normalize();
        let (init_yaw, init_pitch, _) = desired_gimbal_rot.to_euler(EulerRot::YXZ);

        for (ge, mut gtf, mut ig) in gimbals.iter_mut() {
            gtf.rotation = desired_gimbal_rot;
            ig.local_yaw = init_yaw;
            ig.pitch = init_pitch;
            commands.entity(ge).remove::<GimbalAimTracker>();
        }

        for (mut ctf, mut ch, _, _) in chassis_q.iter_mut() {
            ch.yaw = 0.0;
            ch.yaw_velocity = 0.0;
            ch.pitch = 0.0;
            ch.roll = 0.0;
            ctf.rotation = Quat::IDENTITY;
        }

        controller.set_keyboard_auto_aim(true);
        if (st.arena_config.spin_omega.abs() > 0.001) != controller.remote_chassis_spin() {
            controller.toggle_remote_chassis_spin();
        }
        if st.arena_config.self_spin != controller.controlled_chassis_spin() {
            controller.toggle_controlled_chassis_spin();
        }
        if let Some(ref mut ps) = stats {
            ps.launch_count = 0;
            ps.accurate_count = 0;
        }
        st.launch_count = 0;
        st.hit_count = 0;
        start_new_bench(&mut st);
        return;
    }

    if !st.arena_active {
        return;
    }

    let dt = time.delta_secs();
    let dist = st.arena_config.distance_m.clamp(2.0, 15.0);
    let speed = st.arena_config.move_speed.clamp(0.0, 5.0);
    let half_span = (st.arena_config.move_span.clamp(0.5, 16.0)) * 0.5;
    let self_speed = st.arena_config.self_move_speed.clamp(0.0, 5.0);
    let self_half_span = (st.arena_config.self_move_span.clamp(0.5, 16.0)) * 0.5;
    let mut omega = st.arena_config.spin_omega.clamp(-20.0, 20.0);
    if omega.abs() <= 0.001 && controller.remote_chassis_spin() {
        omega = 7.42;
    }
    let self_spin = st.arena_config.self_spin || controller.controlled_chassis_spin();
    let want_hero = st.arena_config.target_type == "hero";

    for (_e, inf, mut tf, mut lin, mut ang, is_ctrl, is_slapper) in robots.iter_mut() {
        if is_ctrl.is_some() {
            tf.translation.z = 0.0;
            if tf.translation.y < ARENA_Y - 1.0 {
                tf.translation.y = ROBOT_SPAWN_Y;
            }
            lin.0.z = 0.0;
            ang.0 = Vec3::ZERO;
            tf.rotation = Quat::IDENTITY;
            if self_speed > 0.001 {
                if tf.translation.x >= self_half_span {
                    st.self_move_dir = -1.0;
                } else if tf.translation.x <= -self_half_span {
                    st.self_move_dir = 1.0;
                }
                lin.0.x = st.self_move_dir * self_speed;
            } else {
                tf.translation.x = 0.0;
                lin.0.x = 0.0;
            }
        } else if is_slapper.is_some() {
            let is_hero = inf.config == HERO_ROBOT_CONFIG;
            if is_hero == want_hero {
                if tf.translation.y < ARENA_Y - 1.0 {
                    tf.translation.y = ROBOT_SPAWN_Y;
                }
                tf.translation.z = -dist;
                lin.0.z = 0.0;
                ang.0 = Vec3::ZERO;
                tf.rotation = Quat::IDENTITY;

                if speed > 0.001 {
                    if tf.translation.x >= half_span {
                        st.arena_move_dir = -1.0;
                    } else if tf.translation.x <= -half_span {
                        st.arena_move_dir = 1.0;
        st.self_move_dir = 1.0;
                    }
                    lin.0.x = st.arena_move_dir * speed;
                } else {
                    tf.translation.x = 0.0;
                    lin.0.x = 0.0;
                }
            } else {
                tf.translation.x = 0.0;
                tf.translation.z = 16.0;
                if tf.translation.y < ARENA_Y - 1.0 {
                    tf.translation.y = ROBOT_SPAWN_Y;
                }
                lin.0 = Vec3::ZERO;
                ang.0 = Vec3::ZERO;
            }
        }
    }

    // Drive exact chassis spin for target (and optionally self) via InfantryChassis
    for (mut ctf, mut ch, is_ctrl, is_active_slapper) in chassis_q.iter_mut() {
        if is_ctrl.is_some() {
            if self_spin {
                ch.yaw += 6.0 * dt;
                ch.yaw_velocity = 0.0;
                ctf.rotation = Quat::from_euler(EulerRot::YXZ, ch.yaw, 0.0, 0.0);
            } else {
                ch.yaw = 0.0;
                ch.yaw_velocity = 0.0;
                ctf.rotation = Quat::IDENTITY;
            }
        } else if is_active_slapper.is_some() {
            if omega.abs() > 0.001 {
                ch.yaw += omega * dt;
                ch.yaw_velocity = 0.0;
                ctf.rotation = Quat::from_euler(EulerRot::YXZ, ch.yaw, 0.0, 0.0);
            } else {
                ch.yaw = 0.0;
                ch.yaw_velocity = 0.0;
                ctf.rotation = Quat::IDENTITY;
            }
        }
    }
}

fn quat_to_ec_yaw_pitch_deg(q: Quat) -> (f32, f32) {
    let w = q.w;
    let x = q.x;
    let y = q.y;
    let z = q.z;
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny_cosp.atan2(cosy_cosp);

    let sinp = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0);
    let pitch = -sinp.asin();

    (yaw.to_degrees(), pitch.to_degrees())
}

fn angle_diff_deg(a: f32, b: f32) -> f32 {
    let mut d = (a - b + 180.0).rem_euclid(360.0) - 180.0;
    if d < -180.0 {
        d += 360.0;
    }
    d
}

fn sample_ec_gimbal_and_stats(
    bridge: Res<WebControlBridge>,
    mut stats: Option<ResMut<ProjectileStatistics>>,
    muzzle: Option<
        Single<
            &GlobalTransform,
            (
                With<Controlled>,
                With<InfantryLaunchOffset>,
                Without<InfantryChassis>,
                Without<InfantryGimbal>,
            ),
        >,
    >,
    targets: Query<(&GlobalTransform, &LinearVelocity, &AngularVelocity), With<ActiveSlapper>>,
) {
    let Ok(mut st) = bridge.0.lock() else {
        return;
    };

    if let Some(muzzle_tf) = muzzle {
        let (_, rot, _) = muzzle_tf.to_scale_rotation_translation();
        let dir = rot.mul_vec3(Vec3::Y).normalize_or_zero();
        if dir.length_squared() > 0.5 {
            st.ec_yaw_deg = (-dir.x).atan2(-dir.z).to_degrees();
            st.ec_pitch_deg = dir.y.clamp(-1.0, 1.0).asin().to_degrees();
        }
    }

    if let Some(ref mut ps) = stats {
        if st.pending_reset_stats {
            st.pending_reset_stats = false;
            ps.launch_count = 0;
            ps.accurate_count = 0;
        }
        st.launch_count = ps.launch_count;
        st.hit_count = ps.accurate_count;
    }

    // Record telemetry CSV & accumulate benchmark stats if running
    let mut should_finish = false;
    let ec_yaw = st.ec_yaw_deg;
    let ec_pitch = st.ec_pitch_deg;
    let vis_yaw = st.vision_yaw_deg;
    let vis_pitch = st.vision_pitch_deg;
    let age_ms = st
        .vision_updated_at
        .map(|t| t.elapsed().as_millis() as u64)
        .unwrap_or(99999);
    let vis_valid = st.vision_valid && age_ms < 1000;
    let vis_fire = st.vision_fire && age_ms < 1000;
    let dist_m = st.arena_config.distance_m;
    let arena_omega = if st.arena_active { Some(st.arena_config.spin_omega) } else { None };
    let launch_c = st.launch_count;
    let hit_c = st.hit_count;

    let (tx, ty, tz, tvx, tvyaw) = targets
        .iter()
        .next()
        .map(|(gt, lv, av)| {
            let t = gt.translation();
            (t.x, t.y, t.z, lv.0.x, av.0.y)
        })
        .unwrap_or((0.0, 0.0, 0.0, 0.0, 0.0));

    if let Some(ref mut b) = st.active_bench {
        let t_s = b.start.elapsed().as_secs_f32();
        let err_yaw = if vis_valid {
            angle_diff_deg(vis_yaw, ec_yaw)
        } else {
            f32::NAN
        };
        let err_pitch = if vis_valid {
            vis_pitch - ec_pitch
        } else {
            f32::NAN
        };
        if vis_valid && t_s > 0.5 {
            b.sample_count += 1;
            b.sum_abs_yaw += err_yaw.abs() as f64;
            b.sum_abs_pitch += err_pitch.abs() as f64;
            if vis_fire {
                b.fire_count += 1;
                b.fire_sum_abs_yaw += err_yaw.abs() as f64;
                b.fire_sum_abs_pitch += err_pitch.abs() as f64;
            }
        }
        if let Some(ref mut w) = b.writer {
            let _ = writeln!(
                w,
                "{:.4},{:.2},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{},{},{:.3},{:.3},{:.3},{:.3},{:.3},{},{}",
                t_s,
                dist_m,
                ec_yaw,
                ec_pitch,
                vis_yaw,
                vis_pitch,
                if err_yaw.is_finite() { err_yaw } else { 0.0 },
                if err_pitch.is_finite() { err_pitch } else { 0.0 },
                u8::from(vis_valid),
                u8::from(vis_fire),
                tx,
                ty,
                tz,
                tvx,
                arena_omega.unwrap_or(tvyaw),
                launch_c,
                hit_c
            );
        }
        if b.duration_s > 0.0 && t_s >= b.duration_s {
            should_finish = true;
        }
    }
    if should_finish {
        finalize_bench(&mut st);
    }
}

fn sample_web_controller(
    bridge: Res<WebControlBridge>,
    mut controller: ResMut<ControllerState>,
) {
    let Ok(mut st) = bridge.0.lock() else {
        return;
    };

    if !st.initialized_auto_aim {
        st.initialized_auto_aim = true;
        let default_on = std::env::var("DAEDALUS_AUTO_AIM_DEFAULT")
            .map(|v| v != "0")
            .unwrap_or(true);
        if default_on {
            controller.set_keyboard_auto_aim(true);
        }
    }

    if st.pending_toggle_auto_aim {
        st.pending_toggle_auto_aim = false;
        controller.toggle_keyboard_auto_aim();
    }
    if st.pending_toggle_ctrl_spin {
        st.pending_toggle_ctrl_spin = false;
        controller.toggle_controlled_chassis_spin();
    }
    if st.pending_toggle_remote_spin {
        st.pending_toggle_remote_spin = false;
        controller.toggle_remote_chassis_spin();
        if st.arena_active {
            if controller.remote_chassis_spin() {
                if st.arena_config.spin_omega.abs() <= 0.001 {
                    st.arena_config.spin_omega = 7.42;
                }
            } else {
                st.arena_config.spin_omega = 0.0;
            }
        }
    }

    if !st.arena_active {
        if controller.controlled_chassis_spin() && controller.controlled.chassis_yaw == 0.0 {
            controller.controlled.add_chassis(1.0, 0.0, 0.0);
        }
        if controller.remote_chassis_spin() && controller.remote.chassis_yaw == 0.0 {
            controller.remote.add_chassis(1.0, 0.0, 0.0);
        }
    }

    if st.pending_switch_slapper {
        st.pending_switch_slapper = false;
        controller.controlled.switch_slapper_just_pressed = true;
    }
    if st.pending_switch_camera {
        st.pending_switch_camera = false;
        controller.controlled.switch_camera_just_pressed = true;
    }
    if st.pending_dart {
        st.pending_dart = false;
        controller.controlled.dart_just_pressed = true;
    }

    if st.hold_updated_at.elapsed() < Duration::from_millis(350) {
        let h = &st.hold;
        let sens = st.gimbal_sens.clamp(0.05, 1.0);
        controller
            .controlled
            .add_movement(Vec2::new(h.ctrl_move[0], h.ctrl_move[1]));
        controller
            .controlled
            .add_gimbal(Vec2::new(h.ctrl_gimbal[0] * sens, h.ctrl_gimbal[1] * sens));
        controller.controlled.shoot |= h.ctrl_shoot;
        controller.controlled.boost |= h.ctrl_boost;

        controller
            .remote
            .add_movement(Vec2::new(h.remote_move[0], h.remote_move[1]));
        controller
            .remote
            .add_gimbal(Vec2::new(h.remote_gimbal[0] * sens, h.remote_gimbal[1] * sens));
        controller.remote.boost |= h.remote_boost;
    }

    if st.remote_patrol && !st.arena_active {
        let t = st.patrol_start.elapsed().as_secs_f32();
        let vx = if (t / 2.2) as i32 % 2 == 0 { 0.65 } else { -0.65 };
        controller.remote.add_movement(Vec2::new(vx, 0.0));
    }

    st.auto_aim_mirror = controller.auto_aim_active();
    st.ctrl_spin_mirror = controller.controlled_chassis_spin();
    st.remote_spin_mirror = controller.remote_chassis_spin();
}

fn limit_frame_rate(bridge: Res<WebControlBridge>, mut limiter: ResMut<FrameLimiterState>) {
    let max_fps = bridge.0.lock().map(|s| s.max_fps).unwrap_or(60);
    if max_fps == 0 {
        limiter.last_frame = Instant::now();
        return;
    }
    let target = Duration::from_secs_f64(1.0 / max_fps as f64);
    let elapsed = limiter.last_frame.elapsed();
    if elapsed < target {
        std::thread::sleep(target - elapsed);
    }
    limiter.last_frame = Instant::now();
}

fn spawn_http_server(port: u16, shared: Arc<Mutex<SharedWebState>>) {
    std::thread::spawn(move || {
        let addr = format!("0.0.0.0:{port}");
        let Ok(listener) = TcpListener::bind(&addr) else {
            eprintln!("[web_control] failed to bind {addr}");
            return;
        };
        println!("[web_control] Daedalus Web Control listening on http://{addr}");
        for stream in listener.incoming().flatten() {
            let shared = shared.clone();
            std::thread::spawn(move || handle_client(stream, shared));
        }
    });
}

fn read_awakening_jpeg() -> Option<Vec<u8>> {
    let mut f = File::open("/dev/shm/awakening_frame").ok()?;
    let mut len_buf = [0u8; 4];
    f.seek(SeekFrom::Start(0)).ok()?;
    f.read_exact(&mut len_buf).ok()?;
    let jpg_size = u32::from_le_bytes(len_buf) as usize;
    if jpg_size == 0 || jpg_size > (2 * 1024 * 1024 - 4) {
        return None;
    }
    let mut data = vec![0u8; jpg_size];
    f.read_exact(&mut data).ok()?;
    if data.len() >= 2 && data[0] == 0xff && data[1] == 0xd8 {
        Some(data)
    } else {
        None
    }
}

fn stream_mjpeg(mut stream: TcpStream) {
    let _ = stream.set_write_timeout(Some(Duration::from_secs(3)));
    let header = "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\nCache-Control: no-cache\r\nConnection: close\r\n\r\n";
    if stream.write_all(header.as_bytes()).is_err() {
        return;
    }
    let interval = Duration::from_millis(33);
    loop {
        let start = Instant::now();
        if let Some(jpg) = read_awakening_jpeg() {
            let part_hdr = format!(
                "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n",
                jpg.len()
            );
            if stream.write_all(part_hdr.as_bytes()).is_err()
                || stream.write_all(&jpg).is_err()
                || stream.write_all(b"\r\n").is_err()
            {
                break;
            }
        }
        let elapsed = start.elapsed();
        if elapsed < interval {
            std::thread::sleep(interval - elapsed);
        }
    }
}

fn handle_client(mut stream: TcpStream, shared: Arc<Mutex<SharedWebState>>) {
    let _ = stream.set_read_timeout(Some(Duration::from_secs(2)));
    let mut buf = [0u8; 16384];
    let Ok(n) = stream.read(&mut buf) else {
        return;
    };
    if n == 0 {
        return;
    }
    let req = String::from_utf8_lossy(&buf[..n]);
    let mut lines = req.lines();
    let Some(first_line) = lines.next() else {
        return;
    };
    let mut parts = first_line.split_whitespace();
    let method = parts.next().unwrap_or("");
    let path = parts.next().unwrap_or("/");
    let body = req.split("\r\n\r\n").nth(1).unwrap_or("");

    match (method, path) {
        ("GET", "/") => send_resp(&mut stream, "200 OK", "text/html; charset=utf-8", HTML_PAGE),
        ("GET", "/video") => stream_mjpeg(stream),
        ("GET", "/api/state") => {
            let snap = shared.lock().map(|s| s.snapshot()).ok();
            let json = serde_json::to_string(&snap).unwrap_or_else(|_| "{}".into());
            send_resp(&mut stream, "200 OK", "application/json", &json);
        }
        ("POST", "/api/arena") => {
            if let Ok(cfg) = serde_json::from_str::<ArenaConfig>(body) {
                if let Ok(mut st) = shared.lock() {
                    st.arena_config = cfg;
                    st.pending_apply_arena = true;
                    let json = serde_json::to_string(&st.snapshot()).unwrap_or_else(|_| "{}".into());
                    send_resp(&mut stream, "200 OK", "application/json", &json);
                    return;
                }
            }
            send_resp(&mut stream, "400 Bad Request", "application/json", r#"{"error":"bad arena config"}"#);
        }
        ("POST", "/api/action") => {
            if let Ok(act) = serde_json::from_str::<ActionPayload>(body) {
                if let Ok(mut st) = shared.lock() {
                    match act.action.as_str() {
                        "toggle_auto_aim" => st.pending_toggle_auto_aim = true,
                        "toggle_ctrl_spin" => st.pending_toggle_ctrl_spin = true,
                        "toggle_remote_spin" => st.pending_toggle_remote_spin = true,
                        "toggle_remote_patrol" => {
                            st.remote_patrol = !st.remote_patrol;
                            st.patrol_start = Instant::now();
                        }
                        "switch_slapper" => st.pending_switch_slapper = true,
                        "switch_camera" => st.pending_switch_camera = true,
                        "dart" => st.pending_dart = true,
                        "reset_stats" => st.pending_reset_stats = true,
                        "exit_arena" => st.pending_exit_arena = true,
                        "stop_bench" => finalize_bench(&mut st),
                        "set_max_fps" => {
                            if let Some(v) = act.value {
                                st.max_fps = (v as u32).clamp(0, 300);
                            }
                        }
                        "set_gimbal_sens" => {
                            if let Some(v) = act.value {
                                st.gimbal_sens = v.clamp(0.05, 1.0);
                            }
                        }
                        _ => {}
                    }
                    let json = serde_json::to_string(&st.snapshot()).unwrap_or_else(|_| "{}".into());
                    send_resp(&mut stream, "200 OK", "application/json", &json);
                    return;
                }
            }
            send_resp(&mut stream, "400 Bad Request", "application/json", r#"{"error":"bad action"}"#);
        }
        ("POST", "/api/hold") => {
            if let Ok(hold) = serde_json::from_str::<HoldPayload>(body) {
                if let Ok(mut st) = shared.lock() {
                    st.hold = hold;
                    st.hold_updated_at = Instant::now();
                }
                send_resp(&mut stream, "200 OK", "application/json", r#"{"ok":true}"#);
                return;
            }
            send_resp(&mut stream, "400 Bad Request", "application/json", r#"{"error":"bad hold"}"#);
        }
        _ => send_resp(&mut stream, "404 Not Found", "text/plain", "Not Found"),
    }
}

fn send_resp(stream: &mut TcpStream, status: &str, ctype: &str, body: &str) {
    let resp = format!(
        "HTTP/1.1 {status}\r\nContent-Type: {ctype}\r\nContent-Length: {}\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n{body}",
        body.len()
    );
    let _ = stream.write_all(resp.as_bytes());
}

const HTML_PAGE: &str = r#"<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Daedalus 自瞄算法验证台</title>
<style>
  * { box-sizing:border-box; }
  body { margin:0; font-family:system-ui,-apple-system,sans-serif; background:#0f172a; color:#e2e8f0; }
  .wrap { display:grid; grid-template-columns: 1.2fr 1fr; gap:12px; padding:12px; max-width:1800px; margin:auto; }
  @media (max-width: 1080px) { .wrap { grid-template-columns: 1fr; } }
  .card { background:#1e293b; border:1px solid #334155; border-radius:10px; padding:12px; margin-bottom:12px; }
  h2 { margin:0 0 8px; font-size:14px; color:#93c5fd; display:flex; justify-content:space-between; align-items:center; gap:8px; }
  .stream { width:100%; aspect-ratio:4/3; background:#020617; border-radius:8px; object-fit:contain; display:block; }
  .row { display:flex; flex-wrap:wrap; gap:6px; margin-bottom:8px; align-items:center; }
  button { background:#334155; color:#f8fafc; border:1px solid #475569; border-radius:7px; padding:6px 10px; cursor:pointer; font-size:12px; user-select:none; }
  button:hover { background:#475569; }
  button.on { background:#16a34a; border-color:#22c55e; }
  button.primary { background:#2563eb; border-color:#3b82f6; font-weight:600; }
  button.primary:hover { background:#1d4ed8; }
  button.danger { background:#991b1b; border-color:#dc2626; }
  .hint { font-size:12px; color:#94a3b8; line-height:1.45; }
  .stats-grid { display:grid; grid-template-columns:repeat(4,1fr); gap:8px; margin-bottom:10px; }
  .tbox { background:#0f172a; border:1px solid #334155; border-radius:8px; padding:7px 9px; }
  .tlabel { font-size:11px; color:#94a3b8; margin-bottom:3px; display:flex; justify-content:space-between; align-items:center; gap:4px; }
  .tval { font-family:ui-monospace,Consolas,monospace; font-size:13px; line-height:1.45; }
  .big { font-size:20px; font-weight:700; color:#4ade80; }
  .badge { font-size:11px; padding:1px 6px; border-radius:4px; background:#334155; color:#cbd5e1; }
  .badge.ok { background:#14532d; color:#86efac; }
  .badge.fire { background:#7f1d1d; color:#fca5a5; }
  .badge.run { background:#1e3a8a; color:#93c5fd; }
  .chart-wrap { background:#020617; border:1px solid #334155; border-radius:8px; padding:6px 8px; margin-bottom:8px; }
  .chart-hdr { display:flex; justify-content:space-between; align-items:center; font-size:12px; margin-bottom:4px; color:#cbd5e1; }
  .legend { display:flex; gap:10px; font-size:11px; color:#94a3b8; align-items:center; }
  .dot { display:inline-block; width:10px; height:3px; border-radius:2px; margin-right:4px; vertical-align:middle; }
  canvas { width:100%; height:155px; display:block; }
  .arena-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:8px; margin-bottom:8px; }
  .field { background:#0f172a; border:1px solid #334155; border-radius:7px; padding:6px 8px; }
  .field label { display:block; font-size:11px; color:#94a3b8; margin-bottom:4px; }
  .field input, .field select { width:100%; background:#1e293b; color:#f8fafc; border:1px solid #475569; border-radius:5px; padding:4px 6px; font-size:13px; }
  table { width:100%; border-collapse:collapse; font-size:12px; font-family:ui-monospace,Consolas,monospace; }
  th, td { border-bottom:1px solid #334155; padding:5px 6px; text-align:left; }
  th { color:#94a3b8; font-weight:600; }
</style>
</head>
<body>
<div class="wrap">
  <div>
    <div class="card">
      <h2>
        <span>自瞄实时画面（红方识别蓝方）</span>
        <span style="display:flex;gap:6px;align-items:center">
          <span id="arenaBadge" class="badge">常规场地</span>
          <span id="statusBadge" class="badge">等待视觉</span>
          <span id="fpsBadge" class="hint"></span>
        </span>
      </h2>
      <img id="video" class="stream" src="/video" alt="等待自瞄画面流..."/>
      <div class="row" style="margin-top:10px">
        <button id="btnAutoAim" onclick="act('toggle_auto_aim')">自瞄订阅 (F5)</button>
        <button id="btnCtrlSpin" onclick="act('toggle_ctrl_spin')">己方小陀螺 (Q)</button>
        <button id="btnRemSpin" onclick="act('toggle_remote_spin')">假人小陀螺 (U)</button>
        <button id="btnPatrol" onclick="act('toggle_remote_patrol')">假人自动巡航</button>
        <button onclick="act('switch_slapper')">切换假人 (Tab)</button>
        <button onclick="act('switch_camera')">切换视角 (F3)</button>
      </div>
      <div class="row">
        <span class="hint">帧率：</span>
        <button id="fps60" onclick="setFps(60)">60</button>
        <button id="fps90" onclick="setFps(90)">90</button>
        <button id="fps120" onclick="setFps(120)">120</button>
        <button id="fps0" onclick="setFps(0)">不限</button>
        <span class="hint" style="margin-left:8px">云台灵敏度：<b id="sensVal">0.45</b></span>
        <input id="sensSlider" type="range" min="0.1" max="1.0" step="0.05" value="0.45" oninput="setSens(this.value)" style="width:110px"/>
      </div>
      <div class="hint">
        键盘直控：<b>WASD</b> 己方移动 | <b>方向键</b> 手动云台 | <b>Space</b> 开火 | <b>IJKL</b> 假人移动 | <b>F5</b> 自瞄开关
      </div>
    </div>

    <div class="card">
      <h2>
        <span>平地标准靶场模式（锁死参数可复现评测）</span>
        <span id="benchBadge" class="badge">未运行</span>
      </h2>
      <div class="row">
        <span class="hint">静止：</span>
        <button onclick="preset(3, 0, 2.5, 0, 10)">3m 静止</button>
        <button onclick="preset(5, 0, 3.5, 0, 10)">5m 静止</button>
        <button onclick="preset(7, 0, 5.0, 0, 10)">7m 静止</button>
        <span class="hint" style="margin-left:6px">横移：</span>
        <button onclick="preset(3, 1.5, 2.5, 0, 15)">3m 横移</button>
        <button onclick="preset(5, 1.8, 3.5, 0, 15)">5m 横移</button>
        <button onclick="preset(7, 2.0, 5.0, 0, 15)">7m 横移</button>
      </div>
      <div class="row">
        <span class="hint">陀螺：</span>
        <button onclick="preset(3, 0, 2.5, 7.5, 15)">3m 小陀螺</button>
        <button onclick="preset(5, 0, 3.5, 7.5, 15)">5m 小陀螺</button>
        <button onclick="preset(7, 0, 5.0, 7.5, 15)">7m 小陀螺</button>
        <span class="hint" style="margin-left:6px">综合：</span>
        <button onclick="preset(3, 1.2, 2.5, 7.5, 15)">3m 移动+陀螺</button>
        <button onclick="preset(5, 1.5, 3.5, 7.5, 15)">5m 移动+陀螺</button>
        <button onclick="preset(7, 1.8, 5.0, 7.5, 15)">7m 移动+陀螺</button>
      </div>
      <div class="arena-grid">
        <div class="field">
          <label>目标距离 D (m)</label>
          <input id="inDist" type="number" min="2" max="14" step="0.5" value="5.0"/>
        </div>
        <div class="field">
          <label>假人平移速度 v (m/s)</label>
          <input id="inSpeed" type="number" min="0" max="5" step="0.2" value="0.0"/>
        </div>
        <div class="field">
          <label>假人平移距离 L (m)</label>
          <input id="inSpan" type="number" min="0.5" max="16" step="0.5" value="3.5"/>
        </div>
        <div class="field">
          <label>假人陀螺角速度 &omega; (rad/s)</label>
          <input id="inOmega" type="number" min="-15" max="15" step="0.5" value="0.0"/>
        </div>
        <div class="field">
          <label>己方平移速度 (m/s)</label>
          <input id="inSelfSpeed" type="number" min="0" max="5" step="0.2" value="0.0"/>
        </div>
        <div class="field">
          <label>己方平移距离 (m)</label>
          <input id="inSelfSpan" type="number" min="0.5" max="16" step="0.5" value="3.0"/>
        </div>
        <div class="field">
          <label>目标兵种</label>
          <select id="inTarget">
            <option value="infantry">蓝方步兵（小装甲）</option>
            <option value="hero">蓝方英雄（大装甲）</option>
          </select>
        </div>
        <div class="field">
          <label>评测时长 T (秒, 0=不限)</label>
          <input id="inDur" type="number" min="0" max="120" step="5" value="15"/>
        </div>
        <div class="field" style="display:flex;align-items:center">
          <label style="margin:0;display:flex;align-items:center;gap:6px;cursor:pointer;font-size:12px;color:#e2e8f0">
            <input id="inSelfSpin" type="checkbox" style="width:auto"/> 己方同步小陀螺
          </label>
        </div>
      </div>
      <div class="row">
        <button class="primary" onclick="applyArena()">进入平地靶场 &amp; 开始评测</button>
        <button onclick="act('stop_bench')">立即结算</button>
        <button class="danger" onclick="act('exit_arena')">退出回常规场地</button>
      </div>
      <div id="benchCsvHint" class="hint">评测时会自动将每帧 Ground Truth / 云台 / 视觉数据写入 NUC 的 <code>~/sim_benchmarks/*.csv</code>，可直接让我读取分析调参。</div>
    </div>
  </div>

  <div>
    <div class="card">
      <h2>
        <span>追踪曲线与命中率</span>
        <span style="display:flex;gap:6px">
          <button id="btnPause" onclick="togglePause()">暂停波形</button>
          <button onclick="clearCharts()">清空波形</button>
          <button class="primary" onclick="resetStats()">清零命中率</button>
        </span>
      </h2>

      <div class="stats-grid">
        <div class="tbox">
          <div class="tlabel"><span>命中率</span><span id="hitCount" class="badge">0/0</span></div>
          <div class="tval big" id="hitRate">0.0%</div>
        </div>
        <div class="tbox">
          <div class="tlabel"><span>电控当前</span><span class="badge">云台</span></div>
          <div class="tval">Y: <b id="ecYaw">--</b>&deg;<br/>P: <b id="ecPitch">--</b>&deg;</div>
        </div>
        <div class="tbox">
          <div class="tlabel"><span>视觉下发</span><span id="visBadge" class="badge">未锁定</span></div>
          <div class="tval">Y: <b id="visYaw">--</b>&deg;<br/>P: <b id="visPitch">--</b>&deg;</div>
        </div>
        <div class="tbox">
          <div class="tlabel"><span>追踪偏差</span><span id="fireBadge" class="badge">停火</span></div>
          <div class="tval">&Delta;Y: <b id="errYaw">--</b>&deg;<br/>&Delta;P: <b id="errPitch">--</b>&deg;</div>
        </div>
      </div>

      <div class="chart-wrap">
        <div class="chart-hdr">
          <b>Yaw 实时追踪曲线 (&deg;)</b>
          <div class="legend">
            <span><i class="dot" style="background:#38bdf8"></i>电控 Yaw</span>
            <span><i class="dot" style="background:#f97316"></i>视觉 Yaw</span>
            <span><i class="dot" style="background:rgba(34,197,94,0.45);height:8px"></i>开火区间</span>
          </div>
        </div>
        <canvas id="yawCanvas"></canvas>
      </div>

      <div class="chart-wrap">
        <div class="chart-hdr">
          <b>Pitch 实时追踪曲线 (&deg;)</b>
          <div class="legend">
            <span><i class="dot" style="background:#38bdf8"></i>电控 Pitch</span>
            <span><i class="dot" style="background:#f97316"></i>视觉 Pitch</span>
            <span><i class="dot" style="background:rgba(34,197,94,0.45);height:8px"></i>开火区间</span>
          </div>
        </div>
        <canvas id="pitchCanvas"></canvas>
      </div>

      <div class="chart-wrap" style="margin-bottom:0">
        <div class="chart-hdr">
          <b>追踪误差 &Delta;Yaw / &Delta;Pitch (&deg;) &amp; 开火区间</b>
          <div class="legend">
            <span><i class="dot" style="background:#a78bfa"></i>&Delta;Yaw</span>
            <span><i class="dot" style="background:#facc15"></i>&Delta;Pitch</span>
            <span><i class="dot" style="background:rgba(34,197,94,0.45);height:8px"></i>开火区间</span>
          </div>
        </div>
        <canvas id="errCanvas" style="height:120px"></canvas>
      </div>
    </div>

    <div class="card">
      <h2><span>历史评测记录（锁死参数横向对比）</span></h2>
      <table>
        <thead>
          <tr><th>#</th><th>场景参数</th><th>命中率</th><th>均值|&Delta;Y|/|&Delta;P|</th><th>开火|&Delta;Y|/|&Delta;P|</th><th>CSV</th></tr>
        </thead>
        <tbody id="benchTable">
          <tr><td colspan="6" class="hint">暂无记录，点击左侧「进入平地靶场 &amp; 开始评测」后自动结算</td></tr>
        </tbody>
      </table>
    </div>
  </div>
</div>

<script>
const MAX_POINTS = 160;
const history = [];
let paused = false;
const keys = new Set();

function fmt(v) { return Number.isFinite(v) ? v.toFixed(2) : '--'; }
function angleDiff(a, b) {
  let d = (a - b + 180) % 360;
  if (d < 0) d += 360;
  return d - 180;
}

function togglePause() {
  paused = !paused;
  const btn = document.getElementById('btnPause');
  btn.textContent = paused ? '继续波形' : '暂停波形';
  btn.classList.toggle('on', paused);
}
function clearCharts() {
  history.length = 0;
  drawAllCharts();
}
function resetStats() {
  act('reset_stats');
  clearCharts();
}
function preset(d, v, span, w, dur) {
  document.getElementById('inDist').value = d;
  document.getElementById('inSpeed').value = v;
  document.getElementById('inSpan').value = span;
  document.getElementById('inOmega').value = w;
  document.getElementById('inDur').value = dur;
  applyArena();
}
async function applyArena() {
  const cfg = {
    distance_m: parseFloat(document.getElementById('inDist').value) || 5.0,
    move_speed: parseFloat(document.getElementById('inSpeed').value) || 0.0,
    move_span: parseFloat(document.getElementById('inSpan').value) || 3.5,
    spin_omega: parseFloat(document.getElementById('inOmega').value) || 0.0,
    self_spin: document.getElementById('inSelfSpin').checked,
    self_move_speed: parseFloat(document.getElementById('inSelfSpeed').value) || 0.0,
    self_move_span: parseFloat(document.getElementById('inSelfSpan').value) || 3.0,
    target_type: document.getElementById('inTarget').value,
    duration_s: parseFloat(document.getElementById('inDur').value) || 0.0,
  };
  clearCharts();
  const r = await fetch('/api/arena', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify(cfg)
  });
  renderState(await r.json());
}

function drawDualChart(canvasId, getA, getB, colorA, colorB, minSpan) {
  const canvas = document.getElementById(canvasId);
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth || 500;
  const h = canvas.clientHeight || 155;
  if (canvas.width !== Math.floor(w * dpr) || canvas.height !== Math.floor(h * dpr)) {
    canvas.width = Math.floor(w * dpr);
    canvas.height = Math.floor(h * dpr);
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);

  const padL = 44, padR = 10, padT = 10, padB = 16;
  const plotW = w - padL - padR;
  const plotH = h - padT - padB;

  const vals = [];
  for (const p of history) {
    const a = getA(p), b = getB(p);
    if (Number.isFinite(a)) vals.push(a);
    if (Number.isFinite(b)) vals.push(b);
  }
  let vMin = vals.length ? Math.min(...vals) : -5;
  let vMax = vals.length ? Math.max(...vals) : 5;
  if (vMax - vMin < minSpan) {
    const mid = (vMax + vMin) / 2;
    vMin = mid - minSpan / 2;
    vMax = mid + minSpan / 2;
  } else {
    const m = (vMax - vMin) * 0.12;
    vMin -= m;
    vMax += m;
  }
  const toY = v => padT + (1 - (v - vMin) / (vMax - vMin)) * plotH;
  const toX = i => padL + (history.length <= 1 ? plotW : (i / (MAX_POINTS - 1)) * plotW);

  ctx.strokeStyle = '#1e293b';
  ctx.fillStyle = '#94a3b8';
  ctx.font = '11px ui-monospace, Consolas, monospace';
  ctx.lineWidth = 1;
  for (let g = 0; g <= 4; g++) {
    const y = padT + (g / 4) * plotH;
    const val = vMax - (g / 4) * (vMax - vMin);
    ctx.beginPath();
    ctx.moveTo(padL, y);
    ctx.lineTo(padL + plotW, y);
    ctx.stroke();
    ctx.fillText(val.toFixed(1), 4, y + 4);
  }

  if (vMin < 0 && vMax > 0) {
    const zy = toY(0);
    ctx.strokeStyle = '#475569';
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(padL, zy);
    ctx.lineTo(padL + plotW, zy);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  if (history.length === 0) return;
  const offset = MAX_POINTS - history.length;
  const stepW = Math.max(2, plotW / (MAX_POINTS - 1));

  ctx.fillStyle = 'rgba(34, 197, 94, 0.24)';
  for (let i = 0; i < history.length; i++) {
    if (history[i].fire) {
      const x = toX(offset + i);
      ctx.fillRect(x - stepW * 0.5, padT, stepW + 0.5, plotH);
    }
  }

  const drawSeries = (getter, color, width) => {
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.beginPath();
    let started = false;
    for (let i = 0; i < history.length; i++) {
      const v = getter(history[i]);
      if (!Number.isFinite(v)) { started = false; continue; }
      const x = toX(offset + i);
      const y = toY(v);
      if (!started) { ctx.moveTo(x, y); started = true; }
      else { ctx.lineTo(x, y); }
    }
    ctx.stroke();
  };

  drawSeries(getB, colorB, 2.0);
  drawSeries(getA, colorA, 2.0);
}

function drawAllCharts() {
  drawDualChart('yawCanvas', p => p.ecYaw, p => p.visYaw, '#38bdf8', '#f97316', 4.0);
  drawDualChart('pitchCanvas', p => p.ecPitch, p => p.visPitch, '#38bdf8', '#f97316', 2.0);
  drawDualChart('errCanvas', p => p.errYaw, p => p.errPitch, '#a78bfa', '#facc15', 2.0);
}

let lastHistLen = -1;
function renderState(s) {
  if (!s) return;
  document.getElementById('btnAutoAim').classList.toggle('on', !!s.auto_aim);
  document.getElementById('btnCtrlSpin').classList.toggle('on', !!s.controlled_spin);
  document.getElementById('btnRemSpin').classList.toggle('on', !!s.remote_spin);
  document.getElementById('btnPatrol').classList.toggle('on', !!s.remote_patrol);
  document.getElementById('fpsBadge').textContent = s.max_fps ? `${s.max_fps} FPS` : '不限帧';

  const arenaBadge = document.getElementById('arenaBadge');
  arenaBadge.textContent = s.arena_active ? `平地靶场 ${s.arena_config.distance_m}m` : '常规场地';
  arenaBadge.classList.toggle('run', !!s.arena_active);

  const benchBadge = document.getElementById('benchBadge');
  if (s.bench_running) {
    benchBadge.textContent = s.arena_config.duration_s > 0
      ? `评测中 剩余 ${s.bench_remaining_s.toFixed(1)}s`
      : `录制中 ${s.bench_elapsed_s.toFixed(1)}s`;
    benchBadge.className = 'badge run';
  } else {
    benchBadge.textContent = '未运行';
    benchBadge.className = 'badge';
  }

  ['60','90','120','0'].forEach(v => {
    const el = document.getElementById('fps' + v);
    if (el) el.classList.toggle('on', String(s.max_fps) === v);
  });

  if (typeof s.gimbal_sens === 'number') {
    document.getElementById('sensVal').textContent = s.gimbal_sens.toFixed(2);
    const slider = document.getElementById('sensSlider');
    if (document.activeElement !== slider) slider.value = s.gimbal_sens.toFixed(2);
  }

  document.getElementById('hitRate').textContent = `${(s.hit_rate_pct || 0).toFixed(1)}%`;
  document.getElementById('hitCount').textContent = `${s.hit_count || 0}/${s.launch_count || 0}`;

  const errYaw = s.vision_valid ? angleDiff(s.vision_yaw_deg, s.ec_yaw_deg) : NaN;
  const errPitch = s.vision_valid ? (s.vision_pitch_deg - s.ec_pitch_deg) : NaN;

  document.getElementById('ecYaw').textContent = fmt(s.ec_yaw_deg);
  document.getElementById('ecPitch').textContent = fmt(s.ec_pitch_deg);
  document.getElementById('visYaw').textContent = s.vision_valid ? fmt(s.vision_yaw_deg) : '--';
  document.getElementById('visPitch').textContent = s.vision_valid ? fmt(s.vision_pitch_deg) : '--';
  document.getElementById('errYaw').textContent = fmt(errYaw);
  document.getElementById('errPitch').textContent = fmt(errPitch);

  const visBadge = document.getElementById('visBadge');
  visBadge.textContent = s.vision_valid ? '已锁定' : '未锁定';
  visBadge.classList.toggle('ok', !!s.vision_valid);

  const fireBadge = document.getElementById('fireBadge');
  fireBadge.textContent = s.vision_fire ? '开火中' : '停火';
  fireBadge.classList.toggle('fire', !!s.vision_fire);

  const statusBadge = document.getElementById('statusBadge');
  statusBadge.textContent = s.vision_age_ms < 1000 ? '视觉在线' : '等待视觉';
  statusBadge.classList.toggle('ok', s.vision_age_ms < 1000);

  if (Array.isArray(s.bench_history) && s.bench_history.length !== lastHistLen) {
    lastHistLen = s.bench_history.length;
    const tb = document.getElementById('benchTable');
    if (s.bench_history.length === 0) {
      tb.innerHTML = '<tr><td colspan="6" class="hint">暂无记录</td></tr>';
    } else {
      tb.innerHTML = s.bench_history.map(b => `<tr>
        <td>#${b.id}</td>
        <td>${b.label} (${b.duration_s.toFixed(1)}s)</td>
        <td><b style="color:#4ade80">${b.hit_rate_pct.toFixed(1)}%</b> (${b.hit_count}/${b.launch_count})</td>
        <td>${b.mean_abs_yaw_err.toFixed(2)}&deg; / ${b.mean_abs_pitch_err.toFixed(2)}&deg;</td>
        <td>${b.fire_mean_abs_yaw_err.toFixed(2)}&deg; / ${b.fire_mean_abs_pitch_err.toFixed(2)}&deg;</td>
        <td style="font-size:11px;color:#94a3b8">${b.csv_file}</td>
      </tr>`).join('');
    }
  }

  if (!paused) {
    history.push({
      ecYaw: s.ec_yaw_deg,
      ecPitch: s.ec_pitch_deg,
      visYaw: s.vision_valid ? s.vision_yaw_deg : NaN,
      visPitch: s.vision_valid ? s.vision_pitch_deg : NaN,
      errYaw,
      errPitch,
      fire: !!s.vision_fire,
    });
    if (history.length > MAX_POINTS) history.shift();
    drawAllCharts();
  }
}

async function pollState() {
  try {
    const r = await fetch('/api/state');
    renderState(await r.json());
  } catch {}
}

async function act(action, value) {
  const r = await fetch('/api/action', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({action, value})
  });
  renderState(await r.json());
}
function setFps(v) { act('set_max_fps', v); }
function setSens(v) { act('set_gimbal_sens', parseFloat(v)); }

window.addEventListener('keydown', e => {
  if (['INPUT','SELECT','TEXTAREA'].includes(document.activeElement?.tagName)) return;
  if (e.repeat) return;
  if (['Space','ArrowUp','ArrowDown','ArrowLeft','ArrowRight','Tab','F5','F3'].includes(e.code)) e.preventDefault();
  if (e.code === 'F5') return act('toggle_auto_aim');
  if (e.code === 'KeyQ') return act('toggle_ctrl_spin');
  if (e.code === 'KeyU') return act('toggle_remote_spin');
  if (e.code === 'Tab') return act('switch_slapper');
  if (e.code === 'F3') return act('switch_camera');
  if (e.code === 'KeyG') return act('dart');
  keys.add(e.code);
});
window.addEventListener('keyup', e => keys.delete(e.code));
window.addEventListener('blur', () => keys.clear());
window.addEventListener('resize', drawAllCharts);

function axis(pos, neg) { return (pos ? 1 : 0) - (neg ? 1 : 0); }

setInterval(() => {
  const payload = {
    ctrl_move: [
      axis(keys.has('KeyD'), keys.has('KeyA')),
      axis(keys.has('KeyW'), keys.has('KeyS')),
    ],
    ctrl_gimbal: [
      axis(keys.has('ArrowLeft'), keys.has('ArrowRight')),
      axis(keys.has('ArrowUp'), keys.has('ArrowDown')),
    ],
    ctrl_shoot: keys.has('Space'),
    ctrl_boost: keys.has('ShiftLeft'),
    remote_move: [
      axis(keys.has('KeyL'), keys.has('KeyJ')),
      axis(keys.has('KeyI'), keys.has('KeyK')),
    ],
    remote_gimbal: [0, 0],
    remote_boost: keys.has('ShiftRight'),
  };
  const active = payload.ctrl_move[0] || payload.ctrl_move[1] || payload.ctrl_gimbal[0] || payload.ctrl_gimbal[1]
    || payload.ctrl_shoot || payload.remote_move[0] || payload.remote_move[1];
  if (active) {
    fetch('/api/hold', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify(payload)
    }).catch(() => {});
  }
}, 60);

pollState();
setInterval(pollState, 65);
</script>
</body>
</html>
"#;
