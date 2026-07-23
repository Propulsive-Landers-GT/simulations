# simulations

Vehicle-agnostic **dynamics models & flight simulation** for GT Propulsive Landers —
the plant models controllers and estimators are developed and tested against.

## Contents

| Path | What it is |
|------|------------|
| `DynamicsModel/` | 1D/6-DOF rocket & monocopter dynamics (Python + MATLAB), incl. `FHL-DM/` and legacy models |
| `RocketSimulation/` | Full-vehicle rocket flight simulation (Rust) |
| `SampleData/` | Reference eulers / monocopter / quaternion datasets |
| `simulation_results.csv` | Reference simulation output |
| `docs/` | Legacy documentation |

## Sibling libraries

[`navigation`](../navigation) · [`control`](../control) · [`guidance`](../guidance)
