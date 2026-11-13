pub mod rocket_dynamics;
pub mod device_sim;
pub mod algorithms;
pub mod sloshing_sim;
pub mod fluid_dynamics;
pub mod wind_sim;
pub mod aero_tables;
pub mod simulation;
pub mod mpc_test;
use crate::rocket_dynamics::*;
use crate::device_sim::*;
use crate::algorithms::*;
use crate::sloshing_sim::*;
use crate::fluid_dynamics::*;
use crate::wind_sim::*;
use crate::aero_tables::*;
use crate::simulation::*;
use nalgebra::{Matrix3, Vector3, Vector4, UnitQuaternion};
use ndarray::{Array1, Array2};
use std::fs::File;
use std::io::{Write, BufWriter, Result};
pub mod mpc_tuning; // 🚀 ADD THIS LINE
use crate::mpc_tuning::MPC_Simulation; // 🚀 ADD THIS IMPORT
pub mod ground_station;
pub mod monoprop;
pub mod monoprop_sim;
pub mod monoprop_viz;
pub mod batch;


pub fn export_imu_to_csv(
    filename: &str, 
    times: &[f64], 
    readings: &[IMUReading],
    attitudes: &[UnitQuaternion<f64>] // Added the attitudes slice!
) -> Result<()> {
    // Safety check: Ensure all arrays match in length so we don't truncate data
    assert_eq!(
        times.len(), 
        readings.len(), 
        "Times and readings vectors must be the exact same length!"
    );
    assert_eq!(
        times.len(),
        attitudes.len(),
        "Times and attitudes vectors must be the exact same length!"
    );

    // 1. Open the file and wrap it in a BufWriter
    let file = File::create(filename)?;
    let mut writer = BufWriter::new(file);

    // 2. Write the CSV header (Now includes quaternion columns)
    writeln!(
        writer, 
        "time,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,q_x,q_y,q_z,q_w"
    )?;

    // 3. Iterate through all three vectors simultaneously!
    // Chaining .zip() nests the tuples like this: ((t, reading), attitude)
    for ((t, reading), q) in times.iter().zip(readings.iter()).zip(attitudes.iter()) {
        writeln!(
            writer,
            "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.9},{:.9},{:.9},{:.6},{:.6},{:.6},{:.6}",
            t,
            reading.accel.x, reading.accel.y, reading.accel.z,
            reading.gyro.x, reading.gyro.y, reading.gyro.z,
            reading.mag.x, reading.mag.y, reading.mag.z, // Tesla kept at high precision
            q.coords[0], q.coords[1], q.coords[2], q.coords[3] // nalgebra's [x, y, z, w] format
        )?;
    }

    // 4. Flush the buffer to ensure everything is saved to disk
    writer.flush()?;

    println!("✅ Successfully exported {} IMU/Attitude rows to {}", readings.len(), filename);
    Ok(())
}

fn main() {
    // let mut sim = Simulation::default();
    // sim.debug = true;
    // // sim.rocket.position = Vector3::new(0.0, 0.0, 49.0);
    // // sim.start_state = "hover".to_string();
    // // sim.min_time = 3.0;

    // sim.init();

    // while sim.step() {}

    // sim.finish_sim();

    // export_imu_to_csv("flight_data.csv", &sim.rocket.debug_info.times, &sim.rocket.debug_info.imu_readings, &sim.rocket.debug_info.attitudes).unwrap();
    
    // crate::mpc_test::run_constant_velocity_test();
    // return;
    
    // `cargo run --release -- --ground-station [--port 8888] [--loop] [--rerun]`:
    // real-time flight from the pad, commanded from the ground station GUI (see ground_station.rs)
    let args: Vec<String> = std::env::args().collect();
    if args.iter().any(|a| a == "--ground-station") {
        ground_station::run(ground_station::Options::from_args(&args));
        return;
    }

    // `cargo run --release -- --vehicle monoprop`: one Monoprop flight (pad -> 1 m hover -> land) in the Rerun viewer
    if args.windows(2).any(|w| w[0] == "--vehicle" && w[1] == "monoprop") {
        run_monoprop_flight();
        return;
    }
    // `cargo run --release -- --batch starts.csv`: headless Monoprop runs, graded into batch_results.csv
    if let Some(i) = args.iter().position(|a| a == "--batch") {
        let file = args.get(i + 1).expect("--batch needs a CSV of start positions");
        if let Err(e) = batch::run(file) {
            eprintln!("Batch failed: {e}");
            std::process::exit(1);
        }
        return;
    }

    let mode_tune = false; // Set to true to run the genetic algorithm, false for a standard flight

    if mode_tune {
        let tuner = MPC_Simulation::new();
        let best_weights = tuner.run_genetic_algorithm();
        
        println!("\n🏆 TUNING COMPLETE! Optimal MPC Descent Configuration 🏆");
        println!("{:#?}", best_weights);
        println!("Copy these values into your `FlightPhase::Descent` initialization block!");
        
    } else {
        // Your original simulation logic
        let mut sim = Simulation::default();
        sim.debug = true;
        // sim.rocket.position = Vector3::new(0.0, 0.0, 49.0);
        // sim.start_state = "hover".to_string();
        // sim.min_time = 3.0;

        sim.init();

        while sim.step() {}

        sim.finish_sim();

        export_imu_to_csv("flight_data.csv", &sim.rocket.debug_info.times, &sim.rocket.debug_info.imu_readings, &sim.rocket.debug_info.attitudes).unwrap();
    }
}

fn run_monoprop_flight() {
    let p = monoprop::MonopropParams::default();
    let mut sim = batch::sim_from(p, Vector3::new(0.0, 0.0, 0.0), Vector3::zeros());
    let rec = rerun::RecordingStreamBuilder::new("monoprop_sim")
        .connect_grpc()
        .expect("Failed to connect to the Rerun viewer (start it first)");
    monoprop_viz::log_model(&rec, &p, sim.controller.hover_alt).expect("Rerun logging failed");

    let mut prop_angle = 0.0;
    let start = std::time::Instant::now();
    while sim.step() {
        let _ = monoprop_viz::log_state(&rec, &sim, &mut prop_angle);
        // Pace to real time so the viewer plays the flight at 1x (batch runs stay full speed)
        let ahead = sim.t - start.elapsed().as_secs_f64();
        if ahead > 0.0 {
            std::thread::sleep(std::time::Duration::from_secs_f64(ahead));
        }
        if (sim.t * 100.0).round() as usize % 100 == 0 {
            let b = &sim.body;
            let pos = b.reported_position();
            println!("Time: {:.2}s | Pos: [{:.2}, {:.2}, {:.2}] | Vel: [{:.2}, {:.2}, {:.2}] | Thrust: {:.1} N | Tilt: {:.1} deg",
                sim.t, pos.x, pos.y, pos.z, b.velocity.x, b.velocity.y, b.velocity.z,
                b.thrust, b.tilt().to_degrees());
        }
    }
    let _ = monoprop_viz::log_state(&rec, &sim, &mut prop_angle);
    println!("Outcome: {:?}\n{:#?}", sim.outcome.unwrap(), batch::grade(&sim));
}
