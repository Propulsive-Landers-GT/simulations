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

    // `cargo run --release -- --tune [population] [generations] > /dev/null`: headless MPC weight tuning
    // on a straight vertical descent (progress goes to stderr; the FSW's per-step prints go to stdout)
    let mode_tune = args.iter().any(|a| a == "--tune");

    if mode_tune {
        let arg = |i: usize, default: usize| args.get(i).and_then(|s| s.parse().ok()).unwrap_or(default);
        let tuner = MPC_Simulation::new(arg(2, 22), arg(3, 5));
        let (best_weights, history) = tuner.run_genetic_algorithm();

        eprintln!("\n🏆 TUNING COMPLETE! Optimal MPC Descent Configuration 🏆");
        eprintln!("{:#?}", best_weights);
        mpc_tuning::write_report(&best_weights, &history).unwrap();

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
