//! Observations of the existing discrete contact rule, not landing acceptance criteria.

#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct PhysicalTruthState {
    pub position_world_m: [f64; 3],
    pub velocity_world_mps: [f64; 3],
    /// Body-to-world quaternion, x/y/z/w ordering.
    pub orientation_xyzw: [f64; 4],
    pub angular_velocity_body_radps: [f64; 3],
    pub acceleration_world_mps2: [f64; 3],
    pub mass_kg: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
pub enum PhysicalContactReason {
    GroundPlanePenetration,
}

#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct PhysicalContactEvent {
    pub step_index: usize,
    /// The physical integrator's clock; Simulation::current_time can lag on exit.
    pub step_start_time_s: f64,
    pub step_end_time_s: f64,
    pub dt_s: f64,
    pub start_state: PhysicalTruthState,
    /// Exact discrete state after integration, immediately before the existing clamp.
    /// This is not an interpolated continuous-time impact state.
    pub pre_clamp_state: PhysicalTruthState,
    pub start_bottom_position_world_m: [f64; 3],
    pub bottom_position_world_m: [f64; 3],
    /// Distinguishes e.g. the stock initial COM-at-origin pad overlap from a crossing.
    pub started_penetrating: bool,
    /// Actual argument to Rocket::step: gimbal theta/phi [rad], thrust [N], RCS.
    pub control_input: [f64; 4],
    /// Why the physical step returned false; the simulation may ignore early contacts.
    pub termination_reason: PhysicalContactReason,
}

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
pub enum SimulationStopReason {
    PhysicalContact { event_index: usize },
    FlightTerminated { reason: Option<String> },
    LandedPhase,
}
