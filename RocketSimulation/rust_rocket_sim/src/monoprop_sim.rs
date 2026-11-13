//! Monoprop flight in the loop with the real ground-side MPC (`GroundControl::MonopropController`).
//! The controller gets the TRUE state as telemetry: no sensor models or estimator, so a failure
//! here is a GNC failure, not an avionics one.

use crate::monoprop::Monoprop;
use Common::DroneTelemetry;
use GroundControl::MonopropController;
use nalgebra::Vector3;

#[derive(Debug, Clone, Copy)]
pub struct Sample {
    pub t: f64,
    pub pos: Vector3<f64>, // reported position (same frame the controller sees)
    pub vel: Vector3<f64>,
    pub tilt: f64,
    pub thrust: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Outcome {
    Landed,
    Crashed(&'static str),
    Timeout,
}

pub struct MonopropSim {
    pub body: Monoprop,
    pub controller: MonopropController,
    pub t: f64,
    pub dt: f64,
    /// Ground-station loop period (main.rs sleeps to 20 ms)
    pub ctrl_period: f64,
    /// When the "land" command is sent
    pub land_time: f64,
    pub max_time: f64,
    pub cmd: (f64, f64, f64), // (theta, phi, thrust) the drone is applying
    pub log: Vec<Sample>,
    /// Last touchdown: (time, position, vertical speed just before contact)
    pub touchdown: Option<(f64, Vector3<f64>, f64)>,
    pub outcome: Option<Outcome>,
    next_ctrl: f64,
    land_sent: bool,
}

impl MonopropSim {
    /// Sends "go" right away, like typing `go` into the ground station
    pub fn new(body: Monoprop) -> Self {
        let mut controller = MonopropController::new(body.p.mpc_params());
        controller.go();
        Self {
            cmd: (0.0, 0.0, body.thrust),
            body,
            controller,
            t: 0.0,
            dt: 0.01,
            ctrl_period: 0.02,
            land_time: 10.0,
            max_time: 30.0,
            log: Vec::new(),
            touchdown: None,
            outcome: None,
            next_ctrl: 0.0,
            land_sent: false,
        }
    }

    pub fn telemetry(&self) -> DroneTelemetry {
        let b = &self.body;
        let q = b.attitude.quaternion();
        let f = |v: Vector3<f64>| [v.x as f32, v.y as f32, v.z as f32];
        DroneTelemetry {
            timestamp_us: (self.t * 1e6) as u64,
            orientation: [q.i as f32, q.j as f32, q.k as f32, q.w as f32],
            position: f(b.reported_position()),
            velocity: f(b.velocity),
            angular_vel: f(b.ang_vel),
        }
    }

    /// One physics tick. Returns false once the flight has an outcome.
    pub fn step(&mut self) -> bool {
        if self.outcome.is_some() {
            return false;
        }
        if !self.land_sent && self.t >= self.land_time {
            self.controller.land();
            self.land_sent = true;
        }
        if self.t >= self.next_ctrl {
            self.next_ctrl += self.ctrl_period;
            let telem = self.telemetry();
            if let Some(c) = self.controller.step(&telem, self.ctrl_period) {
                self.cmd = (c.gimbal_theta_seq[0] as f64, c.gimbal_phi_seq[0] as f64, c.thrust_seq[0] as f64);
            }
            if self.controller.landing_complete {
                self.cmd.2 = 0.0; // the ramp ends at 0; the drone holds that
            }
        }

        let was_on_ground = self.body.on_ground;
        let vz_before = self.body.velocity.z;
        self.body.step(self.cmd, self.dt);
        self.t += self.dt;

        if self.body.on_ground && !was_on_ground {
            self.touchdown = Some((self.t, self.body.position, vz_before));
        }
        self.log.push(Sample {
            t: self.t,
            pos: self.body.reported_position(),
            vel: self.body.velocity,
            tilt: self.body.tilt(),
            thrust: self.body.thrust,
        });

        self.outcome = if self.body.tilt() > 30f64.to_radians() {
            Some(Outcome::Crashed("tilt > 30 deg"))
        } else if self.body.on_ground && !was_on_ground && vz_before < -1.5 {
            Some(Outcome::Crashed("hard touchdown > 1.5 m/s"))
        } else if self.controller.landing_complete && self.body.on_ground {
            Some(Outcome::Landed)
        } else if self.t >= self.max_time {
            Some(Outcome::Timeout)
        } else {
            None
        };
        self.outcome.is_none()
    }

    pub fn run(&mut self) -> Outcome {
        while self.step() {}
        self.outcome.unwrap()
    }
}
