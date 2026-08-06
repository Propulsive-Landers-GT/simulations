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
            // Stage Costs
            q_pos_xy: rng.gen_range(100.0..3000.0),
            q_pos_z: rng.gen_range(500.0..5000.0),
            q_vel_xy: rng.gen_range(100.0..2000.0),
            q_vel_z: rng.gen_range(5000.0..50_000.0), // 🚀 Raised to track descent speed strictly
            q_q_xy: rng.gen_range(20000.0..300000.0), 
            q_omega_xy: rng.gen_range(100.0..2500.0),
            q_omega_z: rng.gen_range(100.0..2500.0),

            // Terminal Costs
            qn_pos_xy: rng.gen_range(1000.0..10_000.0), // 🚀 Raised for pinpoint accuracy
            qn_pos_z: rng.gen_range(50_000.0..300_000.0),
            qn_vel_xy: rng.gen_range(1000.0..10_000.0),
            // 🚨 GIGANTIC BOUND INCREASE FOR TERMINAL Z VELOCITY 🚨
            qn_vel_z: rng.gen_range(50_000.0..1_000_000.0), 
            qn_q_xy: rng.gen_range(50000.0..300000.0), 
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
            population_size: 150,  // Slightly larger population for better diversity
            generations: 60,       
            mutation_rate: 0.25,   
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
                    let score = Self::evaluate_flight(weights);
                    (i, score)
                })
                .collect();

            scores.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Greater));

            let best_score = scores[0].1;
            let best_weights = population[scores[0].0].clone();

            println!("🧬 Generation {:02} | Best Score: {:.2}", generation_idx, best_score);
            
            // If the score is less than 1000, it's essentially a perfect landing!
            if best_score < 1000.0 {
                println!("🎉 OPTIMAL TUNE FOUND EARLY!");
                return best_weights;
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

        println!("   ↳ Final Best Weights: {:#?}", population[0]);
        population[0]
    }

    fn evaluate_flight(weights: &MpcWeights) -> f64 {
        let mut sim = Simulation::default();
        sim.debug = false; 
        
        // Start 50m up, offset by 15m to force it to maneuver back to the pad
        sim.rocket.position = Vector3::new(15.0, 15.0, 50.0);
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

        // Give the sim extra time to perform a slow, soft hover-landing
        let mut steps = 0;
        let max_steps = 6_000; 
        while sim.step() {
            steps += 1;
            if steps > max_steps { break; }
        }

        let (hit_angle_limit, _final_state) = sim.finish_sim();
        let mut score = 0.0;

        let final_pos = sim.rocket.position;
        let final_vel = sim.rocket.velocity;

        // --- 📊 QUARTIC (x^4) COST FUNCTION 📊 ---
        // This violently punishes any landing > 1m off target or > 1m/s fast.

        // 1. Landing Accuracy (Miss Distance) 
        let miss_xy = (final_pos.x.powi(2) + final_pos.y.powi(2)).sqrt();
        score += miss_xy.powi(4) * 20_000.0; 

        // 2. Vertical Touchdown Speed (The Ultimate Priority)
        let vel_z = final_vel.z.abs();
        score += vel_z.powi(4) * 100_000.0; 

        // 3. Lateral Sliding Speed (Kill drifting)
        let vel_xy = (final_vel.x.powi(2) + final_vel.y.powi(2)).sqrt();
        score += vel_xy.powi(4) * 50_000.0;

        // 4. Fuel Efficiency (Very light, linear tie-breaker)
        let initial_fuel = 80.0; 
        let fuel_consumed = initial_fuel - (sim.rocket.nitrous_mass + sim.rocket.fuel_grain_mass);
        score += fuel_consumed * 2.0; 

        // 5. Hard Constraint Violations
        if final_pos.z > 2.0 || steps > max_steps {
            score += 5_000_000.0; // Hovered away or failed to land
        }
        if hit_angle_limit {
            score += 2_000_000.0; // Tipped over and crashed
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
        };

        // 🚀 WIDER MUTATION BOUNDS
        // Allow the weights to mutate anywhere from half (0.5x) to double (2.0x) their size
        // This stops it from getting stuck in a local minimum!
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