//! Headless Monoprop batch: many start positions in parallel, one graded row per run.
//!   cargo run --release -- --batch starts.csv     (header row, then x,y,z[,vx,vy,vz])
//! Writes batch_results.csv next to Cargo.toml.

use crate::monoprop::{Monoprop, MonopropParams};
use crate::monoprop_sim::{MonopropSim, Outcome};
use nalgebra::Vector3;
use rayon::prelude::*;
use std::error::Error;

#[derive(Debug, Clone, Copy)]
pub struct Metrics {
    pub outcome: Outcome,
    pub touchdown_xy_err: f64, // m from the pad (0, 0); NaN if it never touched down
    pub touchdown_vz: f64,     // m/s, negative = down
    pub max_tilt_deg: f64,
    pub hover_rms_err: f64,    // m from the hover target over the 3 s before "land"
    pub flight_time: f64,
    pub peak_thrust: f64,      // N
}

pub fn grade(sim: &MonopropSim) -> Metrics {
    let target = Vector3::new(0.0, 0.0, sim.controller.hover_alt);
    let hover: Vec<f64> = sim.log.iter()
        .filter(|s| s.t > sim.land_time - 3.0 && s.t <= sim.land_time)
        .map(|s| (s.pos - target).norm_squared())
        .collect();
    let (td_xy, td_vz) = sim.touchdown.map_or((f64::NAN, f64::NAN), |(_, p, vz)| (p.xy().norm(), vz));
    Metrics {
        outcome: sim.outcome.unwrap_or(Outcome::Timeout),
        touchdown_xy_err: td_xy,
        touchdown_vz: td_vz,
        max_tilt_deg: sim.log.iter().map(|s| s.tilt).fold(0.0, f64::max).to_degrees(),
        hover_rms_err: (hover.iter().sum::<f64>() / hover.len() as f64).sqrt(),
        flight_time: sim.t,
        peak_thrust: sim.log.iter().map(|s| s.thrust).fold(0.0, f64::max),
    }
}

/// `pos` is in the reported frame (what the controller sees). Starts in the air at hover thrust,
/// or on the pad when z is at or below the pad height.
pub fn sim_from(p: MonopropParams, pos: Vector3<f64>, vel: Vector3<f64>) -> MonopropSim {
    let mut body = if pos.z <= p.reported_z_on_pad {
        Monoprop::on_pad(p, pos.x, pos.y)
    } else {
        let com = pos + Vector3::new(0.0, 0.0, p.com_height - p.reported_z_on_pad);
        Monoprop::at(p, com, p.mass * 9.81)
    };
    body.velocity = vel;
    MonopropSim::new(body)
}

pub fn run(starts_csv: &str) -> Result<(), Box<dyn Error>> {
    let mut starts = Vec::new();
    let mut rdr = csv::ReaderBuilder::new().flexible(true).trim(csv::Trim::All).from_path(starts_csv)?;
    for (i, row) in rdr.records().enumerate() {
        let v: Vec<f64> = row?.iter().map(str::parse).collect::<Result<_, _>>()
            .map_err(|e| format!("{starts_csv} row {}: {e}", i + 2))?;
        if v.len() != 3 && v.len() != 6 {
            return Err(format!("{starts_csv} row {}: want x,y,z or x,y,z,vx,vy,vz, got {} values", i + 2, v.len()).into());
        }
        let vel = if v.len() == 6 { Vector3::new(v[3], v[4], v[5]) } else { Vector3::zeros() };
        starts.push((Vector3::new(v[0], v[1], v[2]), vel));
    }

    println!("Running {} Monoprop flights on {} threads...", starts.len(), rayon::current_num_threads());
    let p = MonopropParams::default();
    let results: Vec<Metrics> = starts.par_iter().map(|&(pos, vel)| {
        let mut sim = sim_from(p, pos, vel);
        sim.run();
        grade(&sim)
    }).collect();

    let mut w = csv::Writer::from_path("batch_results.csv")?;
    w.write_record(["x0", "y0", "z0", "vx0", "vy0", "vz0", "outcome", "touchdown_xy_err_m", "touchdown_vz_mps",
        "max_tilt_deg", "hover_rms_err_m", "flight_time_s", "peak_thrust_n"])?;
    let mut landed = 0;
    for ((pos, vel), m) in starts.iter().zip(&results) {
        landed += (m.outcome == Outcome::Landed) as usize;
        let outcome = match m.outcome {
            Outcome::Landed => "landed".to_string(),
            Outcome::Crashed(why) => format!("crashed: {why}"),
            Outcome::Timeout => "timeout".to_string(),
        };
        let nums = [pos.x, pos.y, pos.z, vel.x, vel.y, vel.z];
        let mut rec: Vec<String> = nums.iter().map(|v| v.to_string()).collect();
        rec.push(outcome);
        rec.extend([m.touchdown_xy_err, m.touchdown_vz, m.max_tilt_deg, m.hover_rms_err, m.flight_time, m.peak_thrust]
            .iter().map(|v| format!("{v:.4}")));
        w.write_record(&rec)?;
    }
    w.flush()?;
    println!("{landed}/{} landed. Results in batch_results.csv", results.len());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::monoprop_sim::Sample;

    #[test]
    fn grade_hand_built_flight() {
        let p = MonopropParams::default();
        let mut sim = sim_from(p, Vector3::new(0.0, 0.0, 1.0), Vector3::zeros());
        sim.land_time = 3.0;
        // Hover 0.1 m off target in x for 3 s, one tilted sample, touchdown 0.2 m off the pad at -0.5 m/s
        for i in 1..=300 {
            let t = i as f64 * 0.01;
            let tilt = if i == 150 { 10f64.to_radians() } else { 0.0 };
            sim.log.push(Sample { t, pos: Vector3::new(0.1, 0.0, 1.0), vel: Vector3::zeros(), tilt, thrust: 12.0 });
        }
        sim.touchdown = Some((5.0, Vector3::new(0.2, 0.0, p.com_height), -0.5));
        sim.outcome = Some(Outcome::Landed);
        sim.t = 5.0;

        let m = grade(&sim);
        assert!((m.hover_rms_err - 0.1).abs() < 1e-9);
        assert!((m.touchdown_xy_err - 0.2).abs() < 1e-9 && m.touchdown_vz == -0.5);
        assert!((m.max_tilt_deg - 10.0).abs() < 1e-9 && m.peak_thrust == 12.0);
    }
}

