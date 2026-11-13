//! Rerun 3D model of Monoprop, built from basic shapes to match the photos: 3 yellow rods,
//! yellow/white/yellow rings, black base with 3 legs, red motor and 2-blade prop on the gimbal.
//! The shapes are logged once in body frame; each step only moves 3 transforms.

use crate::monoprop::MonopropParams;
use crate::monoprop_sim::MonopropSim;
use nalgebra::{UnitQuaternion, Vector3};
use rerun::*;
use std::f32::consts::TAU;

const BODY: &str = "world/monoprop";
const GIMBAL: &str = "world/monoprop/gimbal";
const PROP: &str = "world/monoprop/gimbal/prop";

const YELLOW: Color = Color(rerun::datatypes::Rgba32(0xF0BE00FF));
const WHITE: Color = Color(rerun::datatypes::Rgba32(0xEBEBEBFF));
const BLACK: Color = Color(rerun::datatypes::Rgba32(0x191919FF));
const RED: Color = Color(rerun::datatypes::Rgba32(0xC81E1EFF));

fn circle(r: f32, z: f32) -> Vec<[f32; 3]> {
    (0..=48).map(|i| {
        let a = i as f32 / 48.0 * TAU;
        [r * a.cos(), r * a.sin(), z]
    }).collect()
}

fn quat(q: &UnitQuaternion<f64>) -> Quaternion {
    Quaternion::from_xyzw([q.i as f32, q.j as f32, q.k as f32, q.w as f32])
}

pub fn log_model(rec: &RecordingStream, p: &MonopropParams, hover_alt: f64) -> RecordingStreamResult<()> {
    let r = p.ring_radius as f32;
    let feet_z = -p.com_height as f32;
    let base_z = feet_z + 0.22; // base plate height above the feet (photo)
    let h = p.frame_height as f32;
    let at = |deg: f32, rad: f32, z: f32| [rad * deg.to_radians().cos(), rad * deg.to_radians().sin(), z];

    rec.log_static("world/ground", &Boxes3D::from_centers_and_half_sizes([(0.0, 0.0, -0.01)], [(5.0, 5.0, 0.01)])
        .with_colors([Color::from_rgb(60, 60, 60)]).with_fill_mode(FillMode::Solid))?;
    rec.log_static("world/hover_target", &Points3D::new([(0.0, 0.0, hover_alt as f32)])
        .with_colors([Color::from_rgb(0, 200, 255)]).with_radii([0.02]))?;

    rec.log_static(format!("{BODY}/rods"), &Cylinders3D::from_lengths_and_radii([h; 3], [0.006; 3])
        .with_centers([0.0, 120.0, 240.0].map(|d| at(d, r, base_z + h / 2.0)))
        .with_colors([YELLOW]).with_fill_mode(FillMode::Solid))?;
    rec.log_static(format!("{BODY}/rings"), &LineStrips3D::new([
            circle(r, base_z + h),        // top, yellow
            circle(r, base_z + 0.75 * h), // middle, white
            circle(r, base_z + 0.35 * h), // lower (electronics), yellow
        ])
        .with_colors([YELLOW, WHITE, YELLOW]).with_radii([0.008]))?;
    rec.log_static(format!("{BODY}/base"), &Cylinders3D::from_lengths_and_radii([0.02], [r * 1.1])
        .with_centers([[0.0, 0.0, base_z]]).with_colors([BLACK]).with_fill_mode(FillMode::Solid))?;
    rec.log_static(format!("{BODY}/legs"), &LineStrips3D::new(
            [60.0, 180.0, 300.0].map(|d| vec![at(d, r, base_z), at(d, p.leg_span as f32, feet_z)]))
        .with_colors([BLACK]).with_radii([0.006]))?;

    // Motor hangs below the gimbal pivot; prop at its bottom
    rec.log_static(format!("{GIMBAL}/motor"), &Cylinders3D::from_lengths_and_radii([0.08], [0.02])
        .with_centers([[0.0, 0.0, -0.03]]).with_colors([RED]).with_fill_mode(FillMode::Solid))?;
    rec.log_static(format!("{PROP}/blades"), &Boxes3D::from_half_sizes([(0.09, 0.012, 0.003)])
        .with_colors([BLACK]).with_fill_mode(FillMode::Solid))?;
    Ok(())
}

/// Per-step pose. `prop_angle` is kept by the caller so the prop only spins while thrusting.
pub fn log_state(rec: &RecordingStream, sim: &MonopropSim, prop_angle: &mut f32) -> RecordingStreamResult<()> {
    let b = &sim.body;
    rec.set_duration_secs("sim_time", sim.t);

    let pos = b.position.map(|v| v as f32);
    rec.log(BODY, &Transform3D::from_translation_rotation([pos.x, pos.y, pos.z], quat(&b.attitude)))?;

    let tilt = UnitQuaternion::rotation_between(&Vector3::z(), &b.thrust_dir()).unwrap_or_else(UnitQuaternion::identity);
    rec.log(GIMBAL, &Transform3D::from_translation_rotation([0.0, 0.0, -b.p.gimbal_arm as f32], quat(&tilt)))?;

    let frac = (b.thrust / b.p.thrust_max) as f32;
    *prop_angle = (*prop_angle + 40.0 * frac * sim.dt as f32) % TAU;
    rec.log(PROP, &Transform3D::from_translation_rotation([0.0, 0.0, -0.08],
        RotationAxisAngle::new([0.0, 0.0, 1.0], Angle::from_radians(*prop_angle))))?;

    // Thrust force, scaled so full thrust is 0.5 m
    rec.log(format!("{GIMBAL}/thrust"), &Arrows3D::from_vectors([[0.0, 0.0, 0.5 * frac]])
        .with_colors([Color::from_rgb(0, 90, 255)]))?;
    Ok(())
}
