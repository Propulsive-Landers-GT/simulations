use crate::simulation::Simulation;
use rayon::prelude::*;
use rand::Rng;
use nalgebra::Vector3;
use ndarray::{Array1, Array2};

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
            population_size: 40,  
            generations: 25,       
            mutation_rate: 0.30,   
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
                    // Fast Tuning: Only simulate the difficult diagonal approach
                    let start_pos = Vector3::new(-3.5, -3.5, 50.0); 
                    let score = Self::evaluate_flight(weights, start_pos);
                    (i, score)
                })
                .collect();

            scores.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Greater));

            let best_score = scores[0].1;
            let best_weights = population[scores[0].0].clone();

            println!("🧬 Generation {:02} | Best Score: {:.2}", generation_idx, best_score);
            
            if generation_idx % 5 == 0 || generation_idx == self.generations - 1 {
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

    fn evaluate_flight(weights: &MpcWeights, start_pos: Vector3<f64>) -> f64 {
        let mut sim = Simulation::default();
        sim.debug = false; 
        
        sim.rocket.position = start_pos;
        
        // 1. Hover initialization
        sim.start_state = "hover".to_string();
        sim.init();

        // 2. Warm start
        let state_dim = 15;
        let initial_p = Array2::<f64>::from_diag(&Array1::from_elem(state_dim, 1e-3));
        let q = Array2::<f64>::from_diag(&Array1::from_elem(state_dim, 1e-5));
        let initial_nominal = Array1::from(vec![
            start_pos.x, start_pos.y, start_pos.z,
            0.0, 0.0, 0.0,
            1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
        ]);
        sim.fsm.autopilot_mut().navigator_mut().inject_state(initial_nominal, initial_p, q);

        {
            let state = sim.fsm.get_state_mut();
            state.vehicle_state.position.x = start_pos.x;
            state.vehicle_state.position.y = start_pos.y;
            state.vehicle_state.position.z = start_pos.z;
            state.vehicle_state.velocity.x = 0.0;
            state.vehicle_state.velocity.y = 0.0;
            state.vehicle_state.velocity.z = 0.0;
        }

        // 3. Sensor Shield (Physical Pinning)
        for _ in 0..150 { 
            sim.step(); 
            
            sim.rocket.position.x = start_pos.x;
            sim.rocket.position.y = start_pos.y;
            sim.rocket.position.z = start_pos.z;
            sim.rocket.velocity.x = 0.0;
            sim.rocket.velocity.y = 0.0;
            sim.rocket.velocity.z = 0.0;
            sim.rocket.ang_vel.x = 0.0;
            sim.rocket.ang_vel.y = 0.0;
            sim.rocket.ang_vel.z = 0.0;
            
            let state = sim.fsm.get_state_mut();
            state.vehicle_state.position.x = start_pos.x;
            state.vehicle_state.position.y = start_pos.y;
            state.vehicle_state.position.z = start_pos.z;
            state.vehicle_state.velocity.x = 0.0;
            state.vehicle_state.velocity.y = 0.0;
            state.vehicle_state.velocity.z = 0.0;
            state.vehicle_state.angular_velocity.x = 0.0;
            state.vehicle_state.angular_velocity.y = 0.0;
            state.vehicle_state.angular_velocity.z = 0.0;
        }

        // 4. Trigger Descent Phase
        sim.fsm.get_state_mut().flight_phase = Lander::state::FlightPhase::Descent;
        sim.fsm.autopilot_mut().set_flight_phase(Lander::state::FlightPhase::Descent);
        
        sim.fsm.get_state_mut().last_navigation_update = -1.0;

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
        let r_matrix = Array2::<f64>::from_diag(&Array1::from(vec![weights.r_gimbal, weights.r_gimbal, weights.r_thrust]));
        let qn_matrix = Array2::<f64>::from_diag(&Array1::from(qn_vec));

        if let Some(mpc) = sim.fsm.autopilot_mut().mpc_mut() {
            mpc.set_manual_weights(true, Some(q_matrix), Some(r_matrix), Some(qn_matrix));
        }

        // CAPTURE START TIME AFTER SHIELD
        let descent_start_time = sim.current_time;

        let mut steps = 0;
        let max_steps = 1_500; 

        let mut tracking_error_sum = 0.0;
        let mut tracking_steps = 0;
        let mut max_tilt = 0.0_f64;
        let mut max_descent_rate = 0.0_f64;

        while sim.step() {
            steps += 1;
            
            let tilt_rad = (sim.rocket.attitude * Vector3::z()).z.clamp(-1.0, 1.0).acos();
            let tilt_deg = tilt_rad.to_degrees();
            if tilt_deg > max_tilt { max_tilt = tilt_deg; }

            // Track Fall Speed
            let descent_rate = -sim.rocket.velocity.z;
            if descent_rate > max_descent_rate { max_descent_rate = descent_rate; }

            // 🚀 HARD FLOOR: Center of mass touches down at 1.5m. Stop simulation!
            if sim.rocket.position.z <= 1.5 {
                break; 
            }

            // Early Aborts
            if sim.rocket.position.z > 60.0 { break; } 
            if max_tilt > 25.0 { break; } 

            let state = sim.fsm.get_state_mut();
            if let Some(traj) = &state.trajectory_state {
                // Fix: Compare current time to when descent ACTUALLY started
                let time_since_traj = sim.current_time - descent_start_time;
                
                let target_pos = if time_since_traj >= traj.time_of_flight_s || traj.positions.is_empty() {
                    if let Some(&last_pos) = traj.positions.last() {
                        Vector3::new(last_pos[0], last_pos[1], last_pos[2])
                    } else {
                        Vector3::new(0.0, 0.0, 0.0)
                    }
                } else {
                    let divisor = traj.positions.len().saturating_sub(1).max(1) as f64;
                    let traj_dt = (traj.time_of_flight_s / divisor).max(1e-4);
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

        if tracking_steps > 0 {
            let avg_tracking_error = tracking_error_sum / (tracking_steps as f64);
            score += avg_tracking_error.powi(2) * 50_000.0;
        }

        // Soft Landing Reward
        let vel_z = final_vel.z.abs();
        if vel_z > 3.0 {
            score += (vel_z - 3.0).powi(2) * 500_000.0; 
        } else {
            score += vel_z * 10_000.0;
        }

        // Punish horizontal misses lightly to prevent wobble
        let miss_xy = (final_pos.x.powi(2) + final_pos.y.powi(2)).sqrt();
        score += miss_xy * 2_000.0; 
        
        // 🚀 Relaxed Tilt Penalty (Allows horizontal braking maneuvers)
        if max_tilt > 10.0 {
            score += (max_tilt - 10.0).powi(2) * 50_000.0;
        }

        // Smooth Drop Penalty
        if max_descent_rate > 8.0 {
            score += (max_descent_rate - 8.0).powi(2) * 100_000.0;
        }

        score += steps as f64 * 100.0;

        // If it failed to reach the 1.5m pad (e.g. hovered away)
        if final_pos.z > 2.0 || steps >= max_steps {
            score += 1_000_000_000.0; 
        }
        
        if hit_angle_limit || max_tilt > 25.0 {
            score += 2_000_000_000.0; 
        }

        score
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