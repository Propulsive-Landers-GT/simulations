//! `--ground-station` run mode: the sim plays the vehicle for the GTPL ground station.
//!
//! The rocket starts on the pad in Standby, the loop is paced to real time, and the same
//! `Lander::telemetry::GroundLink` that flies on the vehicle handles commands and telemetry.
//! On top of what the vehicle sends, the sim adds ground truth (`FlightTelemetry::truth`) and
//! stand telemetry from the propulsion model.
//!
//!   cargo run --release -- --ground-station [--port 8888] [--loop] [--rerun]

use crate::rocket_dynamics::{Rocket, RocketDebugInfo};
use crate::simulation::Simulation;
use gs_protocol::{Source, StandChannel, StandTelemetry, TruthState, ValveId, ValveState, ValveStatus};
use nalgebra::{UnitQuaternion, Vector3, Vector4};
use std::time::{Duration, Instant};
use Lander::state::FlightPhase;
use Lander::telemetry::{valve_tag, GroundLink};

/// Keep publishing this long after Landed / flight termination so the GUI sees the end state
/// (and, after an abort in flight, the truth state falling).
const POST_FLIGHT_S: f64 = 8.0;

pub struct Options {
    pub port: u16,
    /// After a flight, rebuild the sim and go back to Standby instead of exiting
    pub repeat: bool,
    /// Also log to the rerun viewer (`sim.debug`); needs a running viewer
    pub rerun: bool,
}

impl Options {
    pub fn from_args(args: &[String]) -> Self {
        let port = args.iter().position(|a| a == "--port")
            .and_then(|i| args.get(i + 1))
            .and_then(|p| p.parse().ok())
            .unwrap_or(gs_protocol::DEFAULT_VEHICLE_PORT);
        Self {
            port,
            repeat: args.iter().any(|a| a == "--loop"),
            rerun: args.iter().any(|a| a == "--rerun"),
        }
    }
}

pub fn run(options: Options) {
    let mut link = match GroundLink::bind(options.port, Source::Sim) {
        Ok(link) => link,
        Err(e) => {
            eprintln!("Could not bind the ground-station link on UDP port {}: {}", options.port, e);
            return;
        }
    };
    println!("Ground-station mode: listening on UDP port {}, waiting for a ground station heartbeat.", options.port);
    println!("Start the bridge (ground-station repo: cargo run --release --bin gs-bridge), then Arm and Launch from the GUI.");

    loop {
        let mut sim = pad_simulation(options.rerun);
        // A rebuilt vehicle is back on default parameters: tell a ground station that is already listening
        link.send_params(&mut sim.fsm);
        let pacer = run_flight(&mut sim, &mut link);
        sim.finish_sim();
        println!(
            "Flight over at t = {:.2}s (phase {:?}, terminated: {:?}). Real-time factor {:.3}.",
            sim.current_time,
            sim.fsm.get_state().flight_phase,
            sim.fsm.get_state().termination_reason,
            pacer.real_time_factor(sim.current_time),
        );
        if !options.repeat {
            break;
        }
        println!("--loop: rebuilding the simulation, back to Standby (mission clock restarts at 0).");
    }
}

/// A fresh vehicle sitting on the pad in Standby.
fn pad_simulation(rerun: bool) -> Simulation {
    let mut sim = Simulation::default();
    sim.debug = rerun;
    sim.start_state = "standby".to_string();
    // `Simulation::step` ends the run once the rocket touches the ground after `min_time`.
    // Here the rocket sits on the ground for as long as the operator likes.
    sim.min_time = f64::INFINITY;
    sim.rocket.position = pad_position(&sim.rocket);
    sim.init();
    sim
}

/// CoM position with the rocket standing on the pad (the floor check keeps the bottom at z = 0)
fn pad_position(rocket: &Rocket) -> Vector3<f64> {
    Vector3::new(0.0, 0.0, -rocket.com_to_ground.z)
}

/// Runs one flight: Standby on the pad until the operator flies it, then until Landed or
/// termination plus `POST_FLIGHT_S`.
fn run_flight(sim: &mut Simulation, link: &mut GroundLink) -> Pacer {
    let mut pacer = Pacer::new(sim.current_time);
    let mut flight_over_at: Option<f64> = None;
    let mut steps: u64 = 0;
    let mut lifted_off = false;

    loop {
        link.handle_commands(&mut sim.fsm, sim.current_time);
        apply_valve_overrides(sim);

        let phase = sim.fsm.get_state().flight_phase;
        if sim.fsm.get_state().flight_terminated || phase == FlightPhase::Landed {
            // The flight is over (`Simulation::step` would refuse to step). Keep the physics and
            // telemetry going with zero controls: after an abort the engine is off and the vehicle falls.
            let ended_at = *flight_over_at.get_or_insert(sim.current_time);
            if sim.current_time - ended_at >= POST_FLIGHT_S {
                return pacer;
            }
            if phase == FlightPhase::Landed {
                // Landed is a live FSM phase (navigation keeps running, controls are zeroed)
                sim.fsm.step(&sim.sensor_data());
            }
            sim.rocket.step(Vector4::zeros(), Vector3::zeros(), Vector3::zeros(), sim.dt);
            sim.current_time += sim.dt;
        } else {
            sim.step();
            // Hold-down clamps release once the engine carries the vehicle (thrust > weight);
            // the FSM ramps thrust up from zero over the first fraction of a second of Ascent.
            if phase == FlightPhase::Ascent && sim.rocket.thrust_vector.z > sim.rocket.get_mass() * 9.81 {
                lifted_off = true;
            }
            if phase == FlightPhase::Standby || phase == FlightPhase::Armed || (phase == FlightPhase::Ascent && !lifted_off) {
                hold_on_pad(&mut sim.rocket);
            } else if phase == FlightPhase::Descent && has_touched_down(&sim.rocket) {
                // The FSM's own Landed check (estimated altitude <= 0.1 m) cannot trigger in the sim:
                // positions are of the CoM, which is 1.5 m above the ground at touchdown (the descent
                // trajectory also targets z = 1.5). The sim knows the legs are on the ground, so it
                // acts as the touchdown sensor.
                println!("Touchdown at t = {:.2}s (estimated vertical speed {:.2} m/s)", sim.current_time, sim.fsm.get_state().vehicle_state.velocity.z);
                sim.fsm.set_flight_phase(FlightPhase::Landed, sim.current_time);
            }
        }

        let now = sim.current_time;
        let sensor_data = sim.sensor_data();
        let diagnostics: Vec<String> = sim.fsm.get_state_mut().diagnostics_queue.drain(..).collect();
        link.publish_events(&diagnostics, now);
        link.publish(&sim.fsm, &sensor_data, Some(truth_state(&sim.rocket)), now);
        link.publish_stand(stand_telemetry(sim, now));

        // `Rocket::step` appends to ~30 debug vectors every tick. They are only used for the CSV /
        // rerun debug output, so without it drop them regularly (this mode can run for hours).
        steps += 1;
        if !sim.debug && steps % 1000 == 0 {
            sim.rocket.debug_info = RocketDebugInfo::default();
        }

        pacer.wait_until(now);
    }
}

/// Hold-down clamps. `Rocket` has no ground reaction force (the floor check only clips the
/// position), so an unclamped rocket on the pad is in free fall as far as its IMU can tell.
/// Pinning it with zero acceleration makes the IMU read 1 g up, like a vehicle at rest.
fn hold_on_pad(rocket: &mut Rocket) {
    rocket.position = pad_position(rocket);
    rocket.velocity = Vector3::zeros();
    rocket.accel = Vector3::zeros();
    rocket.attitude = UnitQuaternion::identity();
    rocket.ang_vel = Vector3::zeros();
    rocket.ang_accel = Vector3::zeros();
}

fn has_touched_down(rocket: &Rocket) -> bool {
    let bottom = rocket.position + rocket.attitude.transform_vector(&rocket.com_to_ground);
    bottom.z <= 1e-3
}

/// The sim stands in for the valve hardware layer Lander does not have yet: operator valve
/// commands (`valve_overrides`, Standby only) drive the matching valves of the feed-system model.
fn apply_valve_overrides(sim: &mut Simulation) {
    let overrides = &sim.fsm.get_state().valve_overrides;
    let rocket = &mut sim.rocket;
    for (id, valve) in [
        (ValveId::OIso, &mut rocket.o_iso),
        (ValveId::OVnt, &mut rocket.o_vnt),
        (ValveId::PuIso, &mut rocket.r_mv),
        (ValveId::Rcs1, &mut rocket.rcs1_mv),
        (ValveId::Rcs2, &mut rocket.rcs2_mv),
    ] {
        if let Some(open) = overrides.get(valve_tag(id)) {
            valve.is_open = *open;
        }
    }
}

fn truth_state(rocket: &Rocket) -> TruthState {
    // nalgebra quaternion coords are [i, j, k, w] = the protocol's [x, y, z, w]
    let q = rocket.attitude.coords;
    TruthState {
        position: [rocket.position.x as f32, rocket.position.y as f32, rocket.position.z as f32],
        velocity: [rocket.velocity.x as f32, rocket.velocity.y as f32, rocket.velocity.z as f32],
        attitude: [q[0] as f32, q[1] as f32, q[2] as f32, q[3] as f32],
        angular_velocity: [rocket.ang_vel.x as f32, rocket.ang_vel.y as f32, rocket.ang_vel.z as f32],
    }
}

/// Stand telemetry from the propulsion model. Only what the model really has:
///   o-pt (run tank)                 -> O-PT
///   m2-pt (downstream of the MTV)   -> M2-PT
///   chamber pressure (fluid solver) -> E-PT
///   oa-pt (upstream of the run tank, after r_mv: the pressurant line) -> PU-PT,
///        the closest tag the protocol has; there is no OA-PT channel
///   |thrust|                        -> load cell
fn stand_telemetry(sim: &Simulation, now: f64) -> StandTelemetry {
    let rocket = &sim.rocket;
    let mut channels = vec![
        (StandChannel::Opt, rocket.o_pt.last_reading.pressure_bar as f32),
        (StandChannel::M2, rocket.m2_pt.last_reading.pressure_bar as f32),
        (StandChannel::Pupt, rocket.oa_pt.last_reading.pressure_bar as f32),
        (StandChannel::Thrust, rocket.thrust_vector.norm() as f32),
    ];
    if let Some(pc_bar) = rocket.debug_info.chamber_pressures.last() {
        channels.push((StandChannel::Ept, *pc_bar as f32));
    }

    let on_off = |id: ValveId, open: bool| ValveStatus {
        id,
        state: if open { ValveState::Open } else { ValveState::Closed },
        position_deg: None,
    };
    // Same rule as `Rocket::step`: an RCS valve is open if held open or if the roll command fires it
    let rcs_command = sim.fsm.get_state().last_rcs_command;
    let mut valves = vec![
        on_off(ValveId::OIso, rocket.o_iso.is_open),
        on_off(ValveId::OVnt, rocket.o_vnt.is_open),
        on_off(ValveId::PuIso, rocket.r_mv.is_open), // r_mv: N2 storage -> run tank isolation
        on_off(ValveId::Rcs1, rocket.rcs1_mv.is_open || rcs_command > 0.0),
        on_off(ValveId::Rcs2, rocket.rcs2_mv.is_open || rcs_command < 0.0),
    ];
    if let Some(angle_deg) = rocket.debug_info.valve_angles.last() {
        // The fluid solver's thrust -> angle map goes slightly negative (about -1.4 deg) at zero thrust
        let angle_deg = &angle_deg.max(0.0);
        valves.push(ValveStatus {
            id: ValveId::Mtv,
            state: if *angle_deg > 0.5 { ValveState::Open } else { ValveState::Closed },
            position_deg: Some(*angle_deg as f32),
        });
    }

    StandTelemetry { time_s: now, source: Source::Sim, channels, valves }
}

/// Paces simulated time to the wall clock.
struct Pacer {
    wall_start: Instant,
    sim_start: f64,
    /// Wall-clock time given up on because the sim could not keep up
    dropped: Duration,
    last_warning: Option<Instant>,
}

impl Pacer {
    fn new(sim_time: f64) -> Self {
        Self { wall_start: Instant::now(), sim_start: sim_time, dropped: Duration::ZERO, last_warning: None }
    }

    fn wait_until(&mut self, sim_time: f64) {
        let target = Duration::from_secs_f64((sim_time - self.sim_start).max(0.0)) + self.dropped;
        let elapsed = self.wall_start.elapsed();
        if elapsed < target {
            std::thread::sleep(target - elapsed);
            return;
        }
        let behind = elapsed - target;
        if behind > Duration::from_millis(200) {
            // Do not race to catch up (that would run faster than real time); accept the slip.
            self.dropped += behind;
            if self.last_warning.map_or(true, |t| t.elapsed() > Duration::from_secs(5)) {
                self.last_warning = Some(Instant::now());
                println!("[Warning] Simulation cannot keep up with real time (fell {:.0} ms behind)", behind.as_secs_f64() * 1000.0);
            }
        }
    }

    fn real_time_factor(&self, sim_time: f64) -> f64 {
        (sim_time - self.sim_start) / self.wall_start.elapsed().as_secs_f64().max(1e-9)
    }
}
