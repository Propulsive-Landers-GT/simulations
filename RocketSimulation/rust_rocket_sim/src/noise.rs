//! Opt-in, time-keyed sensor noise. Legacy ThreadRng paths remain the default.
//! Reproduction contract: scheme v1, exact time bits, pinned rand/rand_distr versions.
use nalgebra::Vector3;
use rand::{SeedableRng, rngs::StdRng};
use rand_distr::{Distribution, Normal};

pub const SCHEME: &str = "gtpl-sensor-noise-v1-std-rng-time-bits";
pub const IMU: u64 = 1;
pub const GPS: u64 = 2;
pub const UWB: u64 = 3;
pub const M2_PRESSURE: u64 = 4;
pub const O_PRESSURE: u64 = 5;
pub const OA_PRESSURE: u64 = 6;

fn mix(mut value: u64) -> u64 {
    value = (value ^ (value >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94d049bb133111eb);
    value ^ (value >> 31)
}

pub fn sensor_seed(master_seed: u64, sensor_id: u64) -> u64 {
    mix(master_seed ^ mix(sensor_id.wrapping_add(0x9e3779b97f4a7c15)))
}

fn sample_rng(seed: u64, time_s: f64, channel: u64) -> StdRng {
    StdRng::seed_from_u64(mix(seed ^ mix(time_s.to_bits()) ^ mix(channel)))
}

pub fn standard_3d(seed: u64, time_s: f64, channel: u64) -> [f64; 3] {
    let mut rng = sample_rng(seed, time_s, channel);
    let normal = Normal::new(0.0, 1.0).unwrap();
    [
        normal.sample(&mut rng),
        normal.sample(&mut rng),
        normal.sample(&mut rng),
    ]
}

pub fn standard_1d(seed: u64, time_s: f64) -> f64 {
    Normal::new(0.0, 1.0)
        .unwrap()
        .sample(&mut sample_rng(seed, time_s, 0))
}

pub(crate) fn noise_3d(seed: u64, time_s: f64, channel: u64, sigma: &Vector3<f64>) -> Vector3<f64> {
    let mut rng = sample_rng(seed, time_s, channel);
    Vector3::new(
        Normal::new(0.0, sigma.x).unwrap().sample(&mut rng),
        Normal::new(0.0, sigma.y).unwrap().sample(&mut rng),
        Normal::new(0.0, sigma.z).unwrap().sample(&mut rng),
    )
}

pub(crate) fn noise_1d(seed: u64, time_s: f64, sigma: f64) -> f64 {
    Normal::new(0.0, sigma)
        .unwrap()
        .sample(&mut sample_rng(seed, time_s, 0))
}
