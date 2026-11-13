//! Monoprop: the small electric test vehicle (3 rods, 3 rings, base, 3 legs, one gimballed motor + prop).
//! Constant mass, thrust from a propeller with motor lag, gimbal with angle and rate limits.
//! World frame is z-up; body z points from the motor up through the top ring.

use GroundControl::VehicleParams;
use nalgebra::{Matrix3, UnitQuaternion, Vector3};

const G: f64 = 9.81;

/// Measured: mass, max thrust, cage radius and height, gimbal pivot depth (TVC drawing). Everything else is
/// marked `shortcut:` and estimated from photos.
#[derive(Debug, Clone, Copy)]
pub struct MonopropParams {
    pub mass: f64,             // kg
    pub moi: Vector3<f64>,     // kg m^2, diagonal about the center of mass
    pub com_height: f64,       // m, center of mass above the feet when standing
    pub gimbal_arm: f64,       // m, center of mass down to the gimbal pivot
    pub thrust_max: f64,       // N
    pub motor_tau: f64,        // s, first-order lag from commanded to actual thrust
    pub gimbal_limit: f64,     // rad, for each of theta and phi
    pub gimbal_rate: f64,      // rad/s, servo slew limit
    /// Prop reaction torque on yaw per newton of thrust (m). 0 = ignore.
    /// shortcut: a single prop really does spin the frame (~0.015 m); the MPC has no yaw control, so it's off by default.
    pub yaw_torque_per_thrust: f64,
    pub drag_area: f64,        // Cd * A, m^2
    /// Height the flight computer reports while standing on the pad.
    /// shortcut: 0.07 m is inferred from GroundControl's `land_alt`; set it from the real position sensor.
    pub reported_z_on_pad: f64,
    // Geometry for the 3D view (m)
    pub frame_height: f64,     // cage height, base plate up to the top ring
    pub ring_radius: f64,
    pub leg_span: f64,         // foot distance from the centerline
}

impl Default for MonopropParams {
    fn default() -> Self {
        let (mass, radius, height) = (1.05, 0.0762, 0.5715);
        let com_height = 0.30; // shortcut: estimated (battery and electronics sit low); measure by balancing it
        Self {
            mass,
            // shortcut: solid uniform cylinder of the cage's size; replace with a swing test or CAD values
            moi: Vector3::new(
                mass * (3.0 * radius * radius + height * height) / 12.0,
                mass * (3.0 * radius * radius + height * height) / 12.0,
                mass * radius * radius / 2.0,
            ),
            com_height,
            // TVC drawing: pivot is 55.8 mm below the servo horns at the base plate (base ~0.22 m above the feet)
            gimbal_arm: com_height - (0.22 - 0.0558),
            thrust_max: 15.6,
            motor_tau: 0.05,
            gimbal_limit: 15f64.to_radians(),
            gimbal_rate: 6.0,
            yaw_torque_per_thrust: 0.0,
            drag_area: 0.02,
            reported_z_on_pad: 0.07,
            frame_height: height,
            ring_radius: radius,
            leg_span: 0.30,
        }
    }
}

impl MonopropParams {
    /// The same vehicle as the MPC's model sees it
    pub fn mpc_params(&self) -> VehicleParams {
        VehicleParams {
            mass: self.mass,
            ixx: self.moi.x,
            iyy: self.moi.y,
            izz: self.moi.z,
            gimbal_arm: self.gimbal_arm,
            thrust_min: 0.0,
            thrust_max: self.thrust_max,
            gimbal_limit: self.gimbal_limit,
        }
    }
}

pub struct Monoprop {
    pub p: MonopropParams,
    pub position: Vector3<f64>, // center of mass, world
    pub velocity: Vector3<f64>,
    pub attitude: UnitQuaternion<f64>, // body -> world
    pub ang_vel: Vector3<f64>,         // body frame
    pub thrust: f64,                   // actual (lagged) thrust, N
    pub gimbal: (f64, f64),            // actual (theta, phi), rad
    pub on_ground: bool,
}

impl Monoprop {
    /// Standing on the pad at (x, y)
    pub fn on_pad(p: MonopropParams, x: f64, y: f64) -> Self {
        Self::at(p, Vector3::new(x, y, p.com_height), 0.0)
    }

    /// At `position` with thrust already at `thrust` (e.g. hover thrust for an in-air start)
    pub fn at(p: MonopropParams, position: Vector3<f64>, thrust: f64) -> Self {
        Self {
            p,
            position,
            velocity: Vector3::zeros(),
            attitude: UnitQuaternion::identity(),
            ang_vel: Vector3::zeros(),
            thrust,
            gimbal: (0.0, 0.0),
            on_ground: position.z <= p.com_height,
        }
    }

    /// Thrust direction in body frame; same theta/phi convention as `mpc_crate::dynamics`
    pub fn thrust_dir(&self) -> Vector3<f64> {
        let (th, ph) = self.gimbal;
        Vector3::new(th.sin() * ph.cos(), ph.sin(), th.cos() * ph.cos())
    }

    /// Position as the flight computer reports it (z = `reported_z_on_pad` when standing)
    pub fn reported_position(&self) -> Vector3<f64> {
        self.position - Vector3::new(0.0, 0.0, self.p.com_height - self.p.reported_z_on_pad)
    }

    pub fn tilt(&self) -> f64 {
        (self.attitude * Vector3::z()).z.clamp(-1.0, 1.0).acos()
    }

    /// Advance by `dt` with commanded (theta, phi, thrust)
    pub fn step(&mut self, cmd: (f64, f64, f64), dt: f64) {
        let p = self.p;

        // Actuators
        let target = cmd.2.clamp(0.0, p.thrust_max);
        self.thrust += (target - self.thrust) * (dt / p.motor_tau).min(1.0);
        let slew = |cur: f64, want: f64| {
            let want = want.clamp(-p.gimbal_limit, p.gimbal_limit);
            cur + (want - cur).clamp(-p.gimbal_rate * dt, p.gimbal_rate * dt)
        };
        self.gimbal = (slew(self.gimbal.0, cmd.0), slew(self.gimbal.1, cmd.1));

        // Forces (world) and torques (body)
        let f_body = self.thrust * self.thrust_dir();
        let drag = -0.5 * 1.225 * p.drag_area * self.velocity.norm() * self.velocity;
        let accel = self.attitude * f_body / p.mass + drag / p.mass - Vector3::new(0.0, 0.0, G);

        let r_gimbal = Vector3::new(0.0, 0.0, -p.gimbal_arm);
        let torque = r_gimbal.cross(&f_body) + Vector3::new(0.0, 0.0, p.yaw_torque_per_thrust * self.thrust);
        let inertia = Matrix3::from_diagonal(&p.moi);
        let w = self.ang_vel;
        let ang_accel = (torque - w.cross(&(inertia * w))).component_div(&p.moi);

        // Semi-implicit Euler
        self.velocity += accel * dt;
        self.position += self.velocity * dt;
        self.ang_vel += ang_accel * dt;
        self.attitude *= UnitQuaternion::from_scaled_axis(self.ang_vel * dt);

        // shortcut: flat rigid ground, the feet can't tip or slide; add a leg contact model if tip-overs matter.
        self.on_ground = self.position.z <= p.com_height;
        if self.on_ground {
            self.position.z = p.com_height;
            self.velocity = Vector3::zeros();
            self.ang_vel = Vector3::zeros();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn free_fall_matches_gravity() {
        let mut m = Monoprop::at(MonopropParams::default(), Vector3::new(0.0, 0.0, 10.0), 0.0);
        for _ in 0..100 {
            m.step((0.0, 0.0, 0.0), 0.01);
        }
        let expected = 10.0 - 0.5 * G * 1.0 * 1.0;
        assert!((m.position.z - expected).abs() < 0.06, "z = {}", m.position.z);
    }

    #[test]
    fn hover_thrust_holds_height() {
        let p = MonopropParams::default();
        let hover = p.mass * G;
        let mut m = Monoprop::at(p, Vector3::new(0.0, 0.0, 1.0), hover);
        for _ in 0..200 {
            m.step((0.0, 0.0, hover), 0.01);
        }
        assert!((m.position.z - 1.0).abs() < 1e-9 && m.tilt() < 1e-9, "z = {}", m.position.z);
    }
}
