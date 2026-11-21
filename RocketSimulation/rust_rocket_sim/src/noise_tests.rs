use crate::device_sim::{GPS, IMU, PressureTransducer, UWB};
use crate::noise;
use nalgebra::{UnitQuaternion, Vector3};

#[test]
fn imu_seed_repeats_samples_preserves_hold_and_uses_existing_dt_scaling() {
    let mut a = IMU::default();
    let mut b = IMU::default();
    let seed = noise::sensor_seed(20261103, noise::IMU);
    a.set_noise_seed(Some(seed));
    b.set_noise_seed(Some(seed));
    let mut time = 0.0;
    for _ in 0..20 {
        time += 0.01;
        let ra = a.update(
            Vector3::zeros(),
            Vector3::zeros(),
            UnitQuaternion::identity(),
            time,
        );
        let rb = b.update(
            Vector3::zeros(),
            Vector3::zeros(),
            UnitQuaternion::identity(),
            time,
        );
        assert_eq!(ra.accel, rb.accel);
        assert_eq!(ra.gyro, rb.gyro);
        assert_eq!(ra.mag, rb.mag);
        let held = a.update(
            Vector3::repeat(500.0),
            Vector3::repeat(20.0),
            UnitQuaternion::identity(),
            time,
        );
        assert_eq!(held.accel, ra.accel);
        if time == 0.01 {
            let z = noise::standard_3d(seed, time, 0);
            let gyro_z = noise::standard_3d(seed, time, 1);
            let mag_z = noise::standard_3d(seed, time, 2);
            for axis in 0..3 {
                let truth = if axis == 2 { 9.81 } else { 0.0 };
                let expected = truth + a.accel_noise_sigma[axis] / 0.01_f64.sqrt() * z[axis];
                assert!((ra.accel[axis] - expected).abs() < 1e-14);
                let expected_gyro = a.gyro_noise_sigma[axis] / 0.01_f64.sqrt() * gyro_z[axis];
                let expected_mag =
                    a.earth_mag[axis] + a.mag_noise_sigma[axis] / 0.01_f64.sqrt() * mag_z[axis];
                assert!((ra.gyro[axis] - expected_gyro).abs() < 1e-14);
                assert!((ra.mag[axis] - expected_mag).abs() < 1e-18);
            }
        }
    }
    let mut other = IMU::default();
    other.set_noise_seed(Some(seed ^ 1));
    assert_ne!(
        other
            .update(
                Vector3::zeros(),
                Vector3::zeros(),
                UnitQuaternion::identity(),
                time
            )
            .accel,
        a.last_reading.accel
    );
}

#[test]
fn time_keys_keep_uwb_noise_aligned_after_a_state_dependent_dropout() {
    let mut a = UWB::new(
        Vector3::repeat(0.1),
        Vector3::zeros(),
        Vector3::zeros(),
        8.0,
        5.0,
    );
    let mut b = a.clone();
    a.set_noise_seed(Some(42));
    b.set_noise_seed(Some(42));
    a.update(Vector3::repeat(0.1), 0.25);
    b.update(Vector3::repeat(50.0), 0.25); // B deliberately makes no noise draw.
    assert_ne!(a.last_reading.position, b.last_reading.position);
    assert_eq!(
        a.update(Vector3::repeat(0.2), 0.5).position,
        b.update(Vector3::repeat(0.2), 0.5).position
    );
    let z = noise::standard_3d(42, 0.5, 0);
    for axis in 0..3 {
        assert!((a.last_reading.position[axis] - (0.2 + 0.1 * z[axis])).abs() < 1e-14);
    }
}

#[test]
fn position_and_pressure_seeds_are_repeatable_and_isolated() {
    let mut gps_a = GPS::new(Vector3::repeat(0.2), Vector3::repeat(0.01), 5.0);
    let mut gps_b = gps_a.clone();
    gps_a.set_noise_seed(Some(noise::sensor_seed(42, noise::GPS)));
    gps_b.set_noise_seed(Some(noise::sensor_seed(42, noise::GPS)));
    let mut pt_a = PressureTransducer::default_with_label("m2-pt");
    let mut pt_b = pt_a.clone();
    pt_a.set_noise_seed(Some(noise::sensor_seed(42, noise::M2_PRESSURE)));
    pt_b.set_noise_seed(Some(noise::sensor_seed(42, noise::M2_PRESSURE)));
    for k in 1..10 {
        let time = k as f64 * 0.25;
        let pressure = pt_a.update(50.0, time).pressure_bar;
        let seed = noise::sensor_seed(42, noise::M2_PRESSURE);
        assert_eq!(pressure, 50.0 + 0.005 * noise::standard_1d(seed, time));
        assert_eq!(pressure, pt_b.update(50.0, time).pressure_bar);
        assert_eq!(
            gps_a.update(Vector3::repeat(1.0), time).position,
            gps_b.update(Vector3::repeat(1.0), time).position
        );
        let z = noise::standard_3d(noise::sensor_seed(42, noise::GPS), time, 0);
        for axis in 0..3 {
            assert!((gps_a.last_reading.position[axis] - (1.01 + 0.2 * z[axis])).abs() < 1e-14);
        }
    }
    assert_ne!(
        noise::standard_3d(noise::sensor_seed(42, noise::GPS), 1.0, 0),
        noise::standard_3d(noise::sensor_seed(42, noise::UWB), 1.0, 0)
    );
}

#[test]
fn keyed_normal_process_has_expected_scale_and_independent_channels() {
    let count = 5000;
    let mut sum = 0.0;
    let mut squares = 0.0;
    let mut cross = 0.0;
    for k in 1..=count {
        let t = k as f64 * 0.01;
        let z = noise::standard_3d(123, t, 0);
        sum += z[0];
        squares += z[0] * z[0];
        cross += z[0] * z[1];
    }
    assert!((sum / count as f64).abs() < 0.05);
    assert!((squares / count as f64 - 1.0).abs() < 0.08);
    assert!((cross / count as f64).abs() < 0.06);
    assert_ne!(
        noise::standard_3d(123, 1.0, 0),
        noise::standard_3d(123, 1.0, 1)
    );
}

#[test]
fn existing_public_noise_helper_signatures_are_preserved() {
    let _: fn(&Vector3<f64>, &mut rand::rngs::ThreadRng) -> Vector3<f64> =
        crate::device_sim::noise_3_d;
    let _: fn(f64, &mut rand::rngs::ThreadRng) -> f64 = crate::device_sim::noise_1_d;
}
