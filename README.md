# dynamics

Vehicle-agnostic **dynamics models & flight simulation** for GT Propulsive Landers —
the plant models controllers and estimators are developed and tested against.

> **Provenance:** Reorganized **by function** in July 2026 from the monolithic
> [`Propulsive-Landers-GT/MonopropUAV`](https://github.com/Propulsive-Landers-GT/MonopropUAV)
> repo. Full commit history is preserved. See the
> [GTPL-test root README](../README.md) for the full mapping.

## Contents

| Path | What it is | Was |
|------|------------|-----|
| `DynamicsModel/` | 1D/6-DOF rocket & monocopter dynamics (Python + MATLAB), incl. `FHL-DM/` and legacy models | `Algorithms/DynamicsModel` |
| `RocketSimulation/` | Full-vehicle rocket flight simulation (Rust) | `Algorithms/RocketSimulation` |
| `SampleData/` | Reference eulers / monocopter / quaternion datasets | loose `Algorithms/*.csv` |
| `simulation_results.csv` | Reference simulation output | repo-root data |
| `docs/` | Original `Algorithms` README |

## Sibling libraries

[`navigation`](../navigation) · [`control`](../control) · [`guidance`](../guidance)
