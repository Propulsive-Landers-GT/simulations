use crate::contact::PhysicalContactReason;
use crate::rocket_dynamics::Rocket;
use nalgebra::{UnitQuaternion, Vector3, Vector4};

#[test]
fn contact_preserves_pre_clamp_truth_and_discrete_time_bracket() {
    let mut rocket = Rocket::default();
    rocket.position = Vector3::new(0.4, -0.2, 1.49);
    rocket.velocity = Vector3::new(1.2, -0.7, -6.0);
    rocket.attitude = UnitQuaternion::from_euler_angles(0.12, -0.08, 0.2);
    rocket.ang_vel = Vector3::new(0.2, -0.15, 0.1);
    let start = rocket.truth_snapshot();
    let dt = 0.02;
    let input = Vector4::new(0.02, -0.01, 400.0, 0.0);
    assert!(!rocket.step(input, Vector3::zeros(), Vector3::zeros(), dt));
    let event = &rocket.contact_events()[0];
    assert_eq!(event.start_state, start);
    assert_eq!(event.step_index, 0);
    assert_eq!(event.step_start_time_s, 0.0);
    assert_eq!(event.step_end_time_s, dt);
    assert_eq!(event.dt_s, dt);
    assert!(!event.started_penetrating);
    assert_eq!(
        event.termination_reason,
        PhysicalContactReason::GroundPlanePenetration
    );
    assert_eq!(event.control_input, [0.02, -0.01, 400.0, 0.0]);
    for axis in 0..3 {
        let expected_v = start.velocity_world_mps[axis] + rocket.accel[axis] * dt;
        let expected_p = start.position_world_m[axis] + expected_v * dt;
        assert_eq!(
            event.pre_clamp_state.velocity_world_mps[axis].to_bits(),
            expected_v.to_bits()
        );
        assert_eq!(
            event.pre_clamp_state.position_world_m[axis].to_bits(),
            expected_p.to_bits()
        );
        assert_eq!(
            event.pre_clamp_state.angular_velocity_body_radps[axis],
            rocket.ang_vel[axis]
        );
    }
    assert!(event.pre_clamp_state.velocity_world_mps[2] < -5.0);
    assert!(event.bottom_position_world_m[2] < 0.0);
    let q = rocket.attitude.coords;
    assert_eq!(
        event.pre_clamp_state.orientation_xyzw,
        [q[0], q[1], q[2], q[3]]
    );
    assert_ne!(
        event.start_state.orientation_xyzw,
        event.pre_clamp_state.orientation_xyzw
    );
    assert_eq!(event.pre_clamp_state.mass_kg, rocket.get_mass());
    assert_eq!(rocket.velocity, Vector3::zeros());
    assert!(rocket.position.z > event.pre_clamp_state.position_world_m[2]);
}

#[test]
fn contact_history_is_owned_and_does_not_label_initial_overlap_as_a_crossing() {
    let mut rocket = Rocket::default();
    assert!(!rocket.step(Vector4::zeros(), Vector3::zeros(), Vector3::zeros(), 0.01));
    let first = rocket.contact_events()[0].clone();
    assert!(first.started_penetrating);
    rocket.position.z = 1.51;
    rocket.velocity.z = -6.0;
    assert!(!rocket.step(Vector4::zeros(), Vector3::zeros(), Vector3::zeros(), 0.01));
    assert_eq!(rocket.contact_events().len(), 2);
    assert_eq!(rocket.contact_events()[0], first);
    assert!(!rocket.contact_events()[1].started_penetrating);
    assert_eq!(
        rocket.contact_events()[1].step_start_time_s,
        first.step_end_time_s
    );
}

#[test]
fn free_flight_produces_no_contact_event() {
    let mut rocket = Rocket::default();
    rocket.position.z = 20.0;
    rocket.velocity.z = -3.0;
    assert!(rocket.step(Vector4::zeros(), Vector3::zeros(), Vector3::zeros(), 0.01));
    assert!(rocket.contact_events().is_empty());
}
