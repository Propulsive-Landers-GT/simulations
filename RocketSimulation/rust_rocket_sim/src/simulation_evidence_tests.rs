use crate::contact::SimulationStopReason;
use crate::simulation::Simulation;

#[test]
fn initial_overlap_is_recorded_without_bypassing_existing_min_time() {
    let mut sim = Simulation::default();
    sim.start_state = "standby".into();
    sim.init();
    assert!(sim.step());
    assert!(sim.rocket.contact_events()[0].started_penetrating);
    assert!(sim.stop_reason().is_none());
    assert_eq!(sim.current_time, sim.dt);
}

#[test]
fn terminal_contact_records_reason_even_though_the_simulation_clock_does_not_advance() {
    let mut sim = Simulation::default();
    sim.start_state = "standby".into();
    sim.init();
    sim.min_time = -1.0; // Test fixture selects the existing contact-stop branch.
    sim.rocket.position.z = 1.51;
    sim.rocket.velocity.z = -6.0;
    assert!(!sim.step());
    assert_eq!(sim.current_time, 0.0); // Existing timing behavior is preserved.
    assert_eq!(sim.rocket.contact_events()[0].step_end_time_s, sim.dt);
    assert_eq!(
        sim.stop_reason(),
        Some(&SimulationStopReason::PhysicalContact { event_index: 0 })
    );
}

#[test]
fn fsm_termination_is_not_misreported_as_physical_contact() {
    let mut sim = Simulation::default();
    sim.fsm.abort(0.0);
    assert!(!sim.step());
    assert!(sim.rocket.contact_events().is_empty());
    assert_eq!(
        sim.stop_reason(),
        Some(&SimulationStopReason::FlightTerminated {
            reason: Some("Operator abort".into())
        })
    );
}

#[test]
fn landed_phase_stop_is_distinct_from_contact_or_flight_termination() {
    let mut sim = Simulation::default();
    sim.fsm.get_state_mut().flight_phase = Lander::state::FlightPhase::Landed;
    assert!(!sim.step());
    assert_eq!(sim.stop_reason(), Some(&SimulationStopReason::LandedPhase));
    assert!(sim.rocket.contact_events().is_empty());
}
