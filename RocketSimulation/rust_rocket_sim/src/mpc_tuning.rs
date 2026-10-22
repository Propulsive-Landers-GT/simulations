use crate::simulation::Simulation;
use rayon::prelude::*;
use rand::Rng;
use nalgebra::Vector3;
use ndarray::{Array1, Array2};
use std::io::Write;

#[derive(Debug, Clone, Copy)]
pub struct MpcWeights {
    pub q_pos_xy: f64,
    pub q_pos_z: f64,
    pub q_vel_xy: f64,
    pub q_vel_z: f64,
    pub q_q_xy: f64,
    pub q_omega_xy: f64,
    pub q_omega_z: f64,

    pub qn_pos_xy: f64,
    pub qn_pos_z: f64,
    pub qn_vel_xy: f64,
    pub qn_vel_z: f64,
    pub qn_q_xy: f64,
    pub qn_omega_xy: f64,
    pub qn_omega_z: f64,

    pub r_gimbal: f64,
    pub r_thrust: f64,
}

impl MpcWeights {
    pub fn random(rng: &mut impl Rng) -> Self {
        Self {
            // 🚀 SMOOTH THE DESCENT: Increase Vz damping, reduce horizontal panic
            q_pos_xy: rng.gen_range(10.0..100.0),      
            q_pos_z: rng.gen_range(500.0..1500.0),
            q_vel_xy: rng.gen_range(100.0..500.0),
            q_vel_z: rng.gen_range(10_000.0..30_000.0), 
            
            // 🚀 PRIORITIZE UPRIGHT: Relaxed slightly so it can tilt to brake horizontally
            q_q_xy: rng.gen_range(500_000.0..2_500_000.0),
            q_omega_xy: rng.gen_range(5000.0..20_000.0),
            q_omega_z: rng.gen_range(100.0..1000.0),

            // 🚀 PREVENT LAST-SECOND PANIC
            qn_pos_xy: rng.gen_range(10.0..100.0), 
            qn_pos_z: rng.gen_range(1000.0..5000.0),
            qn_vel_xy: rng.gen_range(100.0..500.0), 
            qn_vel_z: rng.gen_range(10_000.0..30_000.0), 
            qn_q_xy: rng.gen_range(500_000.0..2_500_000.0), 
            qn_omega_xy: rng.gen_range(5000.0..20_000.0),
            qn_omega_z: rng.gen_range(5000.0..20_000.0),

            r_gimbal: rng.gen_range(10.0..250.0),       
            r_thrust: rng.gen_range(0.01..0.05), // Keep at >= 0.01 to prevent Ill-Conditioned Matrix
        }
    }

    /// (Q, R, QN) diagonals in the MPC's order [x y z | qx qy qz qw | vx vy vz | wx wy wz], [theta phi thrust]
    pub fn matrices(&self) -> (Array2<f64>, Array2<f64>, Array2<f64>) {
        let q_vec = vec![
            self.q_pos_xy, self.q_pos_xy, self.q_pos_z,
            self.q_q_xy, self.q_q_xy, 0.0, 0.0,
            self.q_vel_xy, self.q_vel_xy, self.q_vel_z,
            self.q_omega_xy, self.q_omega_xy, self.q_omega_z
        ];
        let qn_vec = vec![
            self.qn_pos_xy, self.qn_pos_xy, self.qn_pos_z,
            self.qn_q_xy, self.qn_q_xy, 0.0, 0.0,
            self.qn_vel_xy, self.qn_vel_xy, self.qn_vel_z,
            self.qn_omega_xy, self.qn_omega_xy, self.qn_omega_z
        ];
        (
            Array2::<f64>::from_diag(&Array1::from(q_vec)),
            Array2::<f64>::from_diag(&Array1::from(vec![self.r_gimbal, self.r_gimbal, self.r_thrust])),
            Array2::<f64>::from_diag(&Array1::from(qn_vec)),
        )
    }
}

pub struct MPC_Simulation {
    pub population_size: usize,
    pub generations: usize,
    pub mutation_rate: f64,
    pub elite_count: usize,
}

impl MPC_Simulation {
    pub fn new(population_size: usize, generations: usize) -> Self {
        MPC_Simulation {
            population_size,
            generations,
            mutation_rate: 0.30,   
            elite_count: 5,        
        }
    }

    /// Returns the best weights and each generation's (best, median) score
    pub fn run_genetic_algorithm(&self) -> (MpcWeights, Vec<(f64, f64)>) {
        eprintln!("🚀 Starting MPC Genetic Algorithm Tuner...");
        eprintln!("CPUs detected: {}. Preparing parallel pool...", rayon::current_num_threads());

        let mut rng = rand::rng();
        let mut population: Vec<MpcWeights> = (0..self.population_size)
            .map(|_| MpcWeights::random(&mut rng))
            .collect();
        let mut history = Vec::new();

        for generation_idx in 0..self.generations {
            let mut scores: Vec<(usize, f64)> = population
                .par_iter()
                .enumerate()
                .map(|(i, weights)| {
                    let score = Self::evaluate_flight(Some(weights), None).score;
                    (i, score)
                })
                .collect();

            scores.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Greater));

            let best_score = scores[0].1;
            let best_weights = population[scores[0].0].clone();
            let median_score = scores[scores.len() / 2].1;
            history.push((best_score, median_score));

            eprintln!("🧬 Generation {:02} | Best Score: {:.2} | Median: {:.2}", generation_idx, best_score, median_score);
            
            if generation_idx % 5 == 0 || generation_idx == self.generations - 1 {
                eprintln!("   ↳ Best Weights: {:#?}", best_weights);
            }

            let mut next_generation = Vec::with_capacity(self.population_size);
            for i in 0..self.elite_count {
                next_generation.push(population[scores[i].0].clone());
            }

            while next_generation.len() < self.population_size {
                let parent1_idx = rng.gen_range(0..(self.population_size / 2));
                let parent2_idx = rng.gen_range(0..(self.population_size / 2));
                
                let parent1 = &population[scores[parent1_idx].0];
                let parent2 = &population[scores[parent2_idx].0];

                let child = self.crossover_and_mutate(parent1, parent2, &mut rng);
                next_generation.push(child);
            }

            population = next_generation;
        }

        (population[0], history)
    }

    /// Flies one descent. `None` keeps the FSW's own FlightPhase::Descent weights.
    pub fn evaluate_flight(weights: Option<&MpcWeights>, mut log: Option<&mut Vec<Sample>>) -> FlightResult {
        let start_pos = Vector3::new(0.0, 0.0, START_ALTITUDE);
        let mut sim = Simulation::default();
        sim.truth_state = true;
        sim.rocket.position = start_pos;
        sim.start_state = "hover".to_string();
        sim.init();

        // Hold the vehicle at the start point for 1.5 s while the engine settles into hover
        for _ in 0..150 {
            sim.step();
            sim.rocket.position = start_pos;
            sim.rocket.velocity = Vector3::zeros();
            sim.rocket.ang_vel = Vector3::zeros();
        }

        // Switch to Descent with guidance locked out, tracking a fixed vertical reference
        let traj = vertical_reference(START_ALTITUDE, sim.rocket.get_mass());
        sim.fsm.get_state_mut().flight_phase = Lander::state::FlightPhase::Descent;
        sim.fsm.autopilot_mut().set_flight_phase(Lander::state::FlightPhase::Descent);
        sim.fsm.get_state_mut().last_navigation_update = 1e9;
        if let (Some(w), Some(mpc)) = (weights, sim.fsm.autopilot_mut().mpc_mut()) {
            let (q, r, qn) = w.matrices();
            mpc.set_manual_weights(true, Some(q), Some(r), Some(qn));
        }

        let descent_start = sim.current_time;
        let time_limit = traj.time_of_flight_s + 10.0;
        let mut res = FlightResult::default();
        let mut sq_err_sum = 0.0;
        let mut n = 0;
        loop {
            {
                let state = sim.fsm.get_state_mut();
                state.trajectory_state = Some(traj.clone());
                state.trajectory_generation_time = descent_start;
            }

            let pre_step_vel = sim.rocket.velocity;
            let alive = sim.step();
            let t = sim.current_time - descent_start;
            let pos = sim.rocket.position;
            let z_ref = reference_z(&traj, t);
            let err = (pos - Vector3::new(0.0, 0.0, z_ref)).norm();
            sq_err_sum += err * err;
            n += 1;
            let tilt_deg = (sim.rocket.attitude * Vector3::z()).z.clamp(-1.0, 1.0).acos().to_degrees();
            res.max_tilt_deg = res.max_tilt_deg.max(tilt_deg);
            if let Some(log) = log.as_deref_mut() {
                log.push(Sample { t, x: pos.x, y: pos.y, z: pos.z, z_ref, vz: sim.rocket.velocity.z, tilt_deg, thrust: sim.held_control.z });
            }

            if !alive || t > time_limit {
                // step() returns false on ground contact (which zeroes velocity, so use the pre-step value) or FSM termination
                res.landed = !alive && !sim.fsm.get_state().flight_terminated;
                res.touchdown_vz = pre_step_vel.z;
                res.touchdown_vxy = pre_step_vel.xy().norm();
                break;
            }
        }

        res.final_z = sim.rocket.position.z;
        res.miss_xy = sim.rocket.position.xy().norm();
        res.rms_track = (sq_err_sum / n.max(1) as f64).sqrt();
        // Placeholder criteria (m, m/s, deg) until the team agrees on real ones
        res.score = 10.0 * res.rms_track + 20.0 * res.touchdown_vz.abs() + 5.0 * res.touchdown_vxy
            + 2.0 * res.miss_xy + 0.5 * res.max_tilt_deg
            + if res.landed { 0.0 } else { 1000.0 + 10.0 * res.final_z };
        res
    }

    fn crossover_and_mutate(&self, p1: &MpcWeights, p2: &MpcWeights, rng: &mut impl Rng) -> MpcWeights {
        let mut child = MpcWeights {
            q_pos_xy: if rng.random_bool(0.5) { p1.q_pos_xy } else { p2.q_pos_xy },
            q_pos_z: if rng.random_bool(0.5) { p1.q_pos_z } else { p2.q_pos_z },
            q_vel_xy: if rng.random_bool(0.5) { p1.q_vel_xy } else { p2.q_vel_xy },
            q_vel_z: if rng.random_bool(0.5) { p1.q_vel_z } else { p2.q_vel_z },
            q_q_xy: if rng.random_bool(0.5) { p1.q_q_xy } else { p2.q_q_xy },
            q_omega_xy: if rng.random_bool(0.5) { p1.q_omega_xy } else { p2.q_omega_xy },
            q_omega_z: if rng.random_bool(0.5) { p1.q_omega_z } else { p2.q_omega_z },
            
            qn_pos_xy: if rng.random_bool(0.5) { p1.qn_pos_xy } else { p2.qn_pos_xy },
            qn_pos_z: if rng.random_bool(0.5) { p1.qn_pos_z } else { p2.qn_pos_z },
            qn_vel_xy: if rng.random_bool(0.5) { p1.qn_vel_xy } else { p2.qn_vel_xy },
            qn_vel_z: if rng.random_bool(0.5) { p1.qn_vel_z } else { p2.qn_vel_z },
            qn_q_xy: if rng.random_bool(0.5) { p1.qn_q_xy } else { p2.qn_q_xy },
            qn_omega_xy: if rng.random_bool(0.5) { p1.qn_omega_xy } else { p2.qn_omega_xy },
            qn_omega_z: if rng.random_bool(0.5) { p1.qn_omega_z } else { p2.qn_omega_z },
            
            r_gimbal: if rng.random_bool(0.5) { p1.r_gimbal } else { p2.r_gimbal },
            r_thrust: if rng.random_bool(0.5) { p1.r_thrust } else { p2.r_thrust },
        };

        if rng.random_bool(self.mutation_rate) { child.q_pos_xy *= rng.gen_range(0.8..1.2); }
        if rng.random_bool(self.mutation_rate) { child.q_pos_z *= rng.gen_range(0.8..1.2); }
        if rng.random_bool(self.mutation_rate) { child.q_vel_xy *= rng.gen_range(0.8..1.2); }
        if rng.random_bool(self.mutation_rate) { child.q_vel_z *= rng.gen_range(0.8..1.2); }
        if rng.random_bool(self.mutation_rate) { child.q_q_xy *= rng.gen_range(0.8..1.2); }
        if rng.random_bool(self.mutation_rate) { child.q_omega_xy *= rng.gen_range(0.8..1.2); }
        if rng.random_bool(self.mutation_rate) { child.q_omega_z *= rng.gen_range(0.8..1.2); }

        if rng.random_bool(self.mutation_rate) { child.qn_pos_xy *= rng.gen_range(0.8..1.2); }
        if rng.random_bool(self.mutation_rate) { child.qn_pos_z *= rng.gen_range(0.8..1.2); }
        if rng.random_bool(self.mutation_rate) { child.qn_vel_xy *= rng.gen_range(0.8..1.2); }
        if rng.random_bool(self.mutation_rate) { child.qn_vel_z *= rng.gen_range(0.8..1.2); }
        if rng.random_bool(self.mutation_rate) { child.qn_q_xy *= rng.gen_range(0.8..1.2); }
        if rng.random_bool(self.mutation_rate) { child.qn_omega_xy *= rng.gen_range(0.8..1.2); }
        if rng.random_bool(self.mutation_rate) { child.qn_omega_z *= rng.gen_range(0.8..1.2); }

        if rng.random_bool(self.mutation_rate) { child.r_gimbal *= rng.gen_range(0.8..1.2); }
        if rng.random_bool(self.mutation_rate) { child.r_thrust *= rng.gen_range(0.8..1.2); }

        child
    }
}

// Straight vertical descent from 50 m, MPC only: the FSW gets the true state and a fixed reference,
// with its navigator and guidance out of the loop, so the score depends on the weights.
const START_ALTITUDE: f64 = 50.0;
const REF_DT: f64 = 0.1;

/// Trapezoidal vertical profile: speed up to 5 m/s, cruise, brake to 0.5 m/s by 1 m above touchdown
/// (CoM at 1.5 m), then keep descending at 0.5 m/s so the vehicle actually makes contact.
pub fn vertical_reference(z0: f64, mass: f64) -> rust_lossless::TrajectoryResult {
    let (accel, v_max, v_td, z_brake_end, z_end) = (1.0, 5.0, 0.5, 2.5, 1.0);
    let (mut z, mut v, mut braking) = (z0, 0.0_f64, false); // v is downward speed
    let (mut positions, mut velocities, mut thrusts) = (Vec::new(), Vec::new(), Vec::new());
    while z > z_end {
        braking |= z - (v * v - v_td * v_td).max(0.0) / (2.0 * accel) <= z_brake_end;
        let a = if braking { if v > v_td { -accel } else { 0.0 } } else if v < v_max { accel } else { 0.0 };
        positions.push([0.0, 0.0, z]);
        velocities.push([0.0, 0.0, -v]);
        thrusts.push([0.0, 0.0, 9.81 - a]); // per kg, like rust_lossless (the Lander multiplies by mass)
        v = (v + a * REF_DT).min(v_max);
        if braking { v = v.max(v_td); }
        z -= v * REF_DT;
    }
    let n = positions.len();
    rust_lossless::TrajectoryResult {
        time_of_flight_s: (n - 1) as f64 * REF_DT,
        masses: vec![mass; n],
        sigmas: thrusts.iter().map(|t| t[2]).collect(),
        positions,
        velocities,
        thrusts,
    }
}

fn reference_z(traj: &rust_lossless::TrajectoryResult, t: f64) -> f64 {
    let i = (t / REF_DT).max(0.0);
    let last = traj.positions.len() - 1;
    let i0 = (i.floor() as usize).min(last);
    let i1 = (i0 + 1).min(last);
    let frac = (i - i0 as f64).clamp(0.0, 1.0);
    traj.positions[i0][2] + frac * (traj.positions[i1][2] - traj.positions[i0][2])
}

#[derive(Debug, Default, Clone)]
pub struct FlightResult {
    pub landed: bool,
    pub touchdown_vz: f64,
    pub touchdown_vxy: f64,
    pub miss_xy: f64,
    pub rms_track: f64,
    pub max_tilt_deg: f64,
    pub final_z: f64,
    pub score: f64,
}

/// One logged step of a flight, for plotting
pub struct Sample {
    pub t: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub z_ref: f64,
    pub vz: f64,
    pub tilt_deg: f64,
    pub thrust: f64,
}

/// Writes tuning_history.csv and tuning_flights.csv (shipped vs tuned weights) next to Cargo.toml
pub fn write_report(best: &MpcWeights, history: &[(f64, f64)]) -> std::io::Result<()> {
    let mut f = std::fs::File::create("tuning_history.csv")?;
    writeln!(f, "generation,best,median")?;
    for (g, (best_score, median)) in history.iter().enumerate() {
        writeln!(f, "{},{},{}", g, best_score, median)?;
    }

    let mut f = std::fs::File::create("tuning_flights.csv")?;
    writeln!(f, "weights,t,x,y,z,z_ref,vz,tilt_deg,thrust")?;
    for (name, weights) in [("shipped", None), ("tuned", Some(best))] {
        let mut log = Vec::new();
        let res = MPC_Simulation::evaluate_flight(weights, Some(&mut log));
        eprintln!("{:>8}: {:?}", name, res);
        for s in &log {
            writeln!(f, "{},{:.3},{:.4},{:.4},{:.4},{:.4},{:.4},{:.3},{:.2}", name, s.t, s.x, s.y, s.z, s.z_ref, s.vz, s.tilt_deg, s.thrust)?;
        }
    }
    Ok(())
}
