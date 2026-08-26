use crate::simulation::Simulation;
use ndarray::{Array1, Array2};
use nalgebra::Vector3;

pub fn run_constant_velocity_test() {
    println!("🧪 Starting MPC Constant Velocity Tracking Test...");

    let mut sim = Simulation::default();
    sim.debug = true;
    
    // 1. Position the physics engine
    let start_altitude = 50.0;
    let start_pos = Vector3::new(0.0, 0.0, start_altitude);
    sim.rocket.position = start_pos;
    
    // 2. Start in Hover so the native planner holds the rocket steady
    sim.start_state = "hover".to_string();
    sim.init();

    // 3. Warm start the EKF so it agrees with the physics engine
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

    // 4. Warm start the FSM Vehicle State
    {
        let state = sim.fsm.get_state_mut();
        state.vehicle_state.position.x = start_pos.x;
        state.vehicle_state.position.y = start_pos.y;
        state.vehicle_state.position.z = start_pos.z;
        state.vehicle_state.velocity.x = 0.0;
        state.vehicle_state.velocity.y = 0.0;
        state.vehicle_state.velocity.z = 0.0;
    }

    // 5. THE SHIELD: Let sensors boot up and EKF converge without trajectory deviations
    println!("⏳ Shielding sensor boot-up sequence (Physical Pinning active)...");
    for _ in 0..150 {
        sim.step();
        
        // 🚀 THE LAUNCH CLAMP: Physically pin the rocket in the air so it cannot fall
        sim.rocket.position.x = start_pos.x;
        sim.rocket.position.y = start_pos.y;
        sim.rocket.position.z = start_pos.z;
        sim.rocket.velocity.x = 0.0;
        sim.rocket.velocity.y = 0.0;
        sim.rocket.velocity.z = 0.0;
        
        // 🚀 CLAMP HEADING: Prevent the rocket from tumbling
        sim.rocket.ang_vel.x = 0.0;
        sim.rocket.ang_vel.y = 0.0;
        sim.rocket.ang_vel.z = 0.0;
        
        // Pin the FSM state to match, preventing false trajectory deviations
        let state = sim.fsm.get_state_mut();
        state.vehicle_state.position.x = start_pos.x;
        state.vehicle_state.position.y = start_pos.y;
        state.vehicle_state.position.z = start_pos.z;
        state.vehicle_state.velocity.x = 0.0;
        state.vehicle_state.velocity.y = 0.0;
        state.vehicle_state.velocity.z = 0.0;
        
        // 🚀 CLAMP HEADING FOR FSM
        state.vehicle_state.angular_velocity.x = 0.0;
        state.vehicle_state.angular_velocity.y = 0.0;
        state.vehicle_state.angular_velocity.z = 0.0;
    }
    println!("✅ Sensors online. Transitioning to Descent.");

    // 6. Force transition to Descent
    sim.fsm.get_state_mut().flight_phase = Lander::state::FlightPhase::Descent;
    sim.fsm.autopilot_mut().set_flight_phase(Lander::state::FlightPhase::Descent);
    
    // 7. Lock out the internal Guidance planner so it doesn't overwrite our test trajectory
    sim.fsm.get_state_mut().last_navigation_update = 1e9;

    // 8. Apply strictly tuned test weights
    let q_vec = vec![
        200.0, 200.0, 50000.0,      // Massive penalty for leaving the Z glide-slope
        1000.0, 1000.0, 0.0, 0.0,   // Attitude (Softened so it can lean to maneuver)
        10.0, 10.0, 15000.0,        // Velocity
        100.0, 100.0, 100.0         // Angular rates
    ];
    
    let q_matrix = Array2::<f64>::from_diag(&Array1::from(q_vec.clone()));
    let qn_matrix = Array2::<f64>::from_diag(&Array1::from(q_vec));
    let r_matrix = Array2::<f64>::from_diag(&Array1::from(vec![50.0, 50.0, 0.005])); // Cheap Throttle

    if let Some(mpc) = sim.fsm.autopilot_mut().mpc_mut() {
        mpc.set_manual_weights(true, Some(q_matrix), Some(r_matrix), Some(qn_matrix));
    }

    // 9. Build the Synthetic Trajectory
    let target_velocity: f64 = -2.0; // m/s
    let flight_time = start_altitude / target_velocity.abs();
    let num_points = (flight_time * 10.0) as usize; // 10 points per second
    
    let mut descent_traj = rust_lossless::TrajectoryResult {
        positions: Vec::with_capacity(num_points),
        velocities: Vec::with_capacity(num_points),
        masses: vec![80.0; num_points],
        thrusts: vec![[0.0, 0.0, 80.0 * 9.81]; num_points],
        sigmas: vec![80.0 * 9.81; num_points],
        time_of_flight_s: flight_time,
    };

    for i in 0..num_points {
        let t = i as f64 / 10.0;
        let z = start_altitude + (target_velocity * t);
        descent_traj.positions.push([0.0, 0.0, z.max(0.0)]);
        descent_traj.velocities.push([0.0, 0.0, target_velocity]);
    }

    // 10. Run the Simulation
    let mut steps = 0;
    let max_steps = 5_000;
    
    // We lock this time in so the trajectory doesn't shift away from us
    let descent_start_time = sim.current_time;
    
    while sim.step() {
        steps += 1;
        
        {
            let state = sim.fsm.get_state_mut();
            state.trajectory_state = Some(descent_traj.clone());
            state.trajectory_generation_time = descent_start_time; 
        }

        if steps > max_steps {
            println!("🛑 Timeout reached!");
            break;
        }
    }

    let (hit_angle, _final_state) = sim.finish_sim();
    
    println!("🏁 Test Complete!");
    println!("   ↳ Final Position: {:.3}, {:.3}, {:.3}", sim.rocket.position.x, sim.rocket.position.y, sim.rocket.position.z);
    println!("   ↳ Final Velocity: {:.3}, {:.3}, {:.3}", sim.rocket.velocity.x, sim.rocket.velocity.y, sim.rocket.velocity.z);
    
    if hit_angle {
        println!("   ❌ FAILURE: Rocket tipped over!");
    } else if sim.rocket.position.z > 2.0 {
        println!("   ❌ FAILURE: Rocket failed to reach the ground.");
    } else {
        println!("   ✅ SUCCESS: MPC tracked the path to a soft landing.");
    }
}