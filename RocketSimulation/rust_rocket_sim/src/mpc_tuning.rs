use crate::simulation::Simulation;
use rayon::prelude::*;
use rand::Rng;
use nalgebra::Vector3;
use ndarray::{Array1, Array2};

/// The 14 specific weights tuned for the MPC descent phase.
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
}

impl MpcWeights {
    pub fn random(rng: &mut impl Rng) -> Self {
        Self {
            // 🛡️ NUMERICALLY STABLE STAGE COSTS
            // 🔥 INCREASED: Force aggressive lateral correction early in the flight
            q_pos_xy: rng.gen_range(1000.0..10_000.0), 
            q_pos_z: rng.gen_range(500.0..5000.0),
            q_vel_xy: rng.gen_range(1000.0..10_000.0),
            q_vel_z: rng.gen_range(5000.0..30_000.0), 
            
            // 🔥 INCREASED: Demand stricter adherence to pointing straight up
            q_q_xy: rng.gen_range(100_000.0..500_000.0), 
            
            // 🔥 LOWERED: Allow it to swing its angular velocity faster to aggressively correct
            q_omega_xy: rng.gen_range(100.0..1000.0),
            q_omega_z: rng.gen_range(100.0..1000.0),

            // 🛡️ NUMERICALLY STABLE TERMINAL COSTS
            qn_pos_xy: rng.gen_range(1000.0..10_000.0), 
            qn_pos_z: rng.gen_range(10_000.0..100_000.0),
            qn_vel_xy: rng.gen_range(1000.0..10_000.0),
            qn_vel_z: rng.gen_range(50_000.0..250_000.0), 
            qn_q_xy: rng.gen_range(50_000.0..200_000.0), 
            qn_omega_xy: rng.gen_range(200.0..5000.0),
            qn_omega_z: rng.gen_range(200.0..5000.0),
        }
    }
}

pub struct MPC_Simulation {
    pub population_size: usize,
    pub generations: usize,
    pub mutation_rate: f64,
    pub elite_count: usize,
}

impl MPC_Simulation {
    pub fn new() -> Self {
        MPC_Simulation {
            population_size: 150,  
            generations: 80,       // Let it bake longer
            mutation_rate: 0.30,   // High mutation for escaping local minima
            elite_count: 5,        
        }
    }

    pub fn run_genetic_algorithm(&self) -> MpcWeights {
        println!("🚀 Starting MPC Genetic Algorithm Tuner...");
        println!("CPUs detected: {}. Preparing parallel pool...", rayon::current_num_threads());

        let mut rng = rand::rng();
        let mut population: Vec<MpcWeights> = (0..self.population_size)
            .map(|_| MpcWeights::random(&mut rng))
            .collect();

        for generation_idx in 0..self.generations {
            let mut scores: Vec<(usize, f64)> = population
                .par_iter()
                .enumerate()
                .map(|(i, weights)| {
                    // 🚀 MULTI-SCENARIO EVALUATION 🚀
                    // The weights must prove they are robust enough to handle 4 different approaches
                    let scenarios = [
                        Vector3::new(0.0, 0.0, 50.0),    // Straight down
                        Vector3::new(5.0, 0.0, 50.0),    // 5m East offset
                        Vector3::new(0.0, 5.0, 50.0),    // 5m North offset
                        Vector3::new(-3.5, -3.5, 50.0),  // ~5m Diagonal offset
                    ];

                    let mut total_score = 0.0;
                    for start_pos in scenarios.iter() {
                        total_score += Self::evaluate_flight(weights, *start_pos);
                    }

                    (i, total_score)
                })
                .collect();

            scores.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Greater));

            let best_score = scores[0].1;
            let best_weights = population[scores[0].0].clone();

            println!("🧬 Generation {:02} | Best Combined Score: {:.2}", generation_idx, best_score);
            
            // With 4 scenarios, a combined score under 4000 is an incredible multi-landing tune
            if best_score < 4000.0 {
                println!("🎉 OPTIMAL ROBUST TUNE FOUND EARLY!");
                return best_weights;
            }

            if generation_idx % 10 == 0 || generation_idx == self.generations - 1 {
                println!("   ↳ Best Weights: {:#?}", best_weights);
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

        population[0]
    }

    /// Runs a single simulation to completion and returns a fitness score.
    fn evaluate_flight(weights: &MpcWeights, start_pos: Vector3<f64>) -> f64 {
        let mut sim = Simulation::default();
        sim.debug = false; 
        
        // 🚀 INJECT SCENARIO STARTING POSITION
        sim.rocket.position = start_pos;
        sim.start_state = "descent".to_string();
        
        sim.init();

        let q_vec = vec![
            weights.q_pos_xy, weights.q_pos_xy, weights.q_pos_z, 
            weights.q_q_xy, weights.q_q_xy, 0.0, 0.0, 
            weights.q_vel_xy, weights.q_vel_xy, weights.q_vel_z, 
            weights.q_omega_xy, weights.q_omega_xy, weights.q_omega_z 
        ];
        
        let qn_vec = vec![
            weights.qn_pos_xy, weights.qn_pos_xy, weights.qn_pos_z,  
            weights.qn_q_xy, weights.qn_q_xy, 0.0, 0.0,              
            weights.qn_vel_xy, weights.qn_vel_xy, weights.qn_vel_z,  
            weights.qn_omega_xy, weights.qn_omega_xy, weights.qn_omega_z 
        ];

        let q_matrix = Array2::<f64>::from_diag(&Array1::from(q_vec));
        let r_matrix = Array2::<f64>::from_diag(&Array1::from(vec![50.0, 50.0, 0.005]));
        let qn_matrix = Array2::<f64>::from_diag(&Array1::from(qn_vec));

        if let Some(mpc) = sim.fsm.autopilot_mut().mpc_mut() {
            mpc.set_manual_weights(true, Some(q_matrix), Some(r_matrix), Some(qn_matrix));
        }

        let mut steps = 0;
        let max_steps = 5_000; 

        // 🚀 METRICS TRACKING 🚀
        let mut tracking_error_sum = 0.0;
        let mut tracking_steps = 0;
        let mut max_tilt = 0.0_f64;

        while sim.step() {
            steps += 1;
            
            // Track max tilt
            let tilt_rad = (sim.rocket.attitude * Vector3::z()).z.clamp(-1.0, 1.0).acos();
            let tilt_deg = tilt_rad.to_degrees();
            if tilt_deg > max_tilt { max_tilt = tilt_deg; }

            // 🚀 INTERPOLATE TRAJECTORY & CALCULATE TRACKING ERROR 🚀
            let state = sim.fsm.get_state_mut();
            if let Some(traj) = &state.trajectory_state {
                let now = state.last_navigation_update;
                let time_since_traj = now - state.trajectory_generation_time;
                
                let target_pos = if time_since_traj >= traj.time_of_flight_s || traj.positions.is_empty() {
                    if let Some(&last_pos) = traj.positions.last() {
                        Vector3::new(last_pos[0], last_pos[1], last_pos[2])
                    } else {
                        Vector3::new(0.0, 0.0, 0.0)
                    }
                } else {
                    let traj_dt = (traj.time_of_flight_s / (traj.positions.len() - 1) as f64).max(1e-4);
                    let exact_idx = time_since_traj / traj_dt;
                    let base_idx = exact_idx.floor() as usize;
                    let safe_idx = base_idx.min(traj.positions.len().saturating_sub(2));
                    let clamped_frac = (exact_idx - safe_idx as f64).clamp(0.0, 1.0);
                    
                    let p0 = traj.positions[safe_idx];
                    let p1 = traj.positions[safe_idx + 1];
                    
                    Vector3::new(
                        p0[0] + clamped_frac * (p1[0] - p0[0]),
                        p0[1] + clamped_frac * (p1[1] - p0[1]),
                        p0[2] + clamped_frac * (p1[2] - p0[2])
                    )
                };
                
                // Compare where the rocket IS to where the Lossless planner WANTS it to be
                let dist = (sim.rocket.position - target_pos).norm();
                tracking_error_sum += dist;
                tracking_steps += 1;
            }

            if steps > max_steps { break; }
        }

        let (hit_angle_limit, _final_state) = sim.finish_sim();
        let mut score = 0.0;

        let final_pos = sim.rocket.position;
        let final_vel = sim.rocket.velocity;

        // --- 📊 PRECISION FITNESS FUNCTION 📊 ---

        // 1. Lossless Tracking Error (Did it trace the glide slope?)
        if tracking_steps > 0 {
            let avg_tracking_error = tracking_error_sum / (tracking_steps as f64);
            score += avg_tracking_error.powi(2) * 50_000.0;
        }

        // 2. Terminal Velocity (Must land softly!)
        let vel_z = final_vel.z.abs();
        score += vel_z.powi(2) * 20_000.0; 
        score += vel_z * 250_000.0; // Linear kicker to force it all the way to 0.0 m/s

        // 3. Final Landing Accuracy (Did it touch the pad?)
        let miss_xy = (final_pos.x.powi(2) + final_pos.y.powi(2)).sqrt();
        score += miss_xy.powi(2) * 5_000.0; 
        
        // 4. Strict Tilt Penalty
        if max_tilt > 10.0 {
            score += (max_tilt - 10.0).powi(2) * 100_000.0;
        }

        // 5. Constraints
        if final_pos.z > 2.0 || steps > max_steps {
            score += 500_000_000.0; // Flew away or timed out
        }
        if hit_angle_limit {
            score += 500_000_000.0; // Tipped over
        }

        score
    }

    /// Breeds two parents and occasionally mutates the offspring
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
        };

        if rng.random_bool(self.mutation_rate) { child.q_pos_xy *= rng.gen_range(0.5..2.0); }
        if rng.random_bool(self.mutation_rate) { child.q_pos_z *= rng.gen_range(0.5..2.0); }
        if rng.random_bool(self.mutation_rate) { child.q_vel_xy *= rng.gen_range(0.5..2.0); }
        if rng.random_bool(self.mutation_rate) { child.q_vel_z *= rng.gen_range(0.5..2.0); }
        if rng.random_bool(self.mutation_rate) { child.q_q_xy *= rng.gen_range(0.5..2.0); }
        if rng.random_bool(self.mutation_rate) { child.q_omega_xy *= rng.gen_range(0.5..2.0); }
        if rng.random_bool(self.mutation_rate) { child.q_omega_z *= rng.gen_range(0.5..2.0); }

        if rng.random_bool(self.mutation_rate) { child.qn_pos_xy *= rng.gen_range(0.5..2.0); }
        if rng.random_bool(self.mutation_rate) { child.qn_pos_z *= rng.gen_range(0.5..2.0); }
        if rng.random_bool(self.mutation_rate) { child.qn_vel_xy *= rng.gen_range(0.5..2.0); }
        if rng.random_bool(self.mutation_rate) { child.qn_vel_z *= rng.gen_range(0.5..2.0); }
        if rng.random_bool(self.mutation_rate) { child.qn_q_xy *= rng.gen_range(0.5..2.0); }
        if rng.random_bool(self.mutation_rate) { child.qn_omega_xy *= rng.gen_range(0.5..2.0); }
        if rng.random_bool(self.mutation_rate) { child.qn_omega_z *= rng.gen_range(0.5..2.0); }

        child
    }
}