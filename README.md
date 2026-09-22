# Propulsive Landers `simulations`: Beginner's Guide (Windows & macOS)

This repo holds the physics models ("plants") used to test our flight software before it flies.
It has three parts:

| Part | Language | What it does | Status |
|---|---|---|---|
| `RocketSimulation/rust_rocket_sim/` | Rust |  A full 6-DOF lander flight (ascent, then hover, then descent) running the real flight software in the loop, with a live 3D view | Active |
| `DynamicsModel/` | Python / MATLAB | Older, simpler models: a 1D fuel sim and the "FHL" 6-DOF + MPC model | Legacy |
| `SampleData/` | CSV | Old monocopter runs from the MATLAB model, so you can test plotting or estimator code without running a sim first | Reference |

---

## 1. One-time setup

You need Git, Rust (`rustup`), and Python 3.10+.

> Every step here was run on Windows. The macOS steps are what the build expects but haven't been tried yet.
> If one is wrong, fix it here.

```bash
git clone https://github.com/Propulsive-Landers-GT/simulations
```

> On Windows, these instructions assume the GNU Rust toolchain, which is what the sim has been built with.
> Run `rustup show` to check; if it says `msvc`, switch with `rustup default stable-x86_64-pc-windows-gnu`.

### a) OpenBLAS (a math library the Rust sim links against)
Neither OS ships with it, so the build fails with `cannot find -lopenblas` unless you install it.

On macOS, Homebrew is all you need. The build finds it on its own:

```bash
brew install openblas libomp
```

Install both. The build looks for `libomp` too and stops if it's missing.

On Windows there's no package manager, so grab the prebuilt zip:

1. Download `OpenBLAS-<version>-x64.zip` from https://github.com/OpenMathLib/OpenBLAS/releases (not the `x64-64` one).
2. Unzip it somewhere permanent, for example `C:\Tools\openblas`.
   It contains `lib\`, which you need to build, and `bin\libopenblas.dll`, which you need to run.
3. Tell Windows where it is, so you don't have to repeat the path in every command.
   Open Start, search for "Edit the system environment variables", click Environment Variables,
   and under *User variables*:
   - Click New, name it `LIBRARY_PATH`, and set it to `C:\path\to\openblas\lib`. This is what lets the build find it.
     (If `LIBRARY_PATH` already exists, add the folder to it, separated by `;`.)
   - Select `Path`, click Edit, click New, and add `C:\path\to\openblas\bin`. This is what lets the sim run.

   Then close and reopen your terminal, or the new values won't be there. Check with `echo $LIBRARY_PATH`.
   ([Microsoft's guide to environment variables](https://learn.microsoft.com/en-us/powershell/module/microsoft.powershell.core/about/about_environment_variables))

   If you'd rather not change your environment, you can pass both paths per command instead.
   See [Without the environment variables](#without-the-environment-variables-windows).

### b) Python environment + Rerun viewer
From the `simulations` folder:

```bash
python -m venv .venv
```

```bash
.venv/Scripts/python -m pip install "rerun-sdk==0.28.*" numpy scipy matplotlib sympy casadi do-mpc pandas plotly
```

`rerun-sdk` gives you the Rerun viewer (`.venv/Scripts/rerun.exe`), the 3D window the Rust sim draws into.
Its version must match the `rerun = "0.28"` line in `Cargo.toml`.

> On macOS the venv puts things in `bin/` rather than `Scripts/`, so use `.venv/bin/python` and `.venv/bin/rerun`,
> and `python3` if plain `python` isn't found. That holds for every venv command below.

---

## 2. Run the Rust flight sim

Step 1: build. The first build takes about 10 minutes; later ones are quick.
Run this from `RocketSimulation/rust_rocket_sim`:

```bash
cargo build --release
```

Step 2: open the viewer in its own terminal and leave it running:

```bash
.venv/Scripts/rerun.exe
```

Step 3: run the sim from `RocketSimulation/rust_rocket_sim`:

```bash
./target/release/rust_rocket_sim.exe
```

(On macOS, `.venv/bin/rerun` and `./target/release/rust_rocket_sim`.)

### What you'll see
- Terminal: one summary line per simulated second, plus a lot of per-step debug output:
  ```
  Time: 9.00s | Phase: Hover | Pos: [0.59, 0.28, 51.06] | Vel: [...] | Mass: 76.74kg
  ```
- Rerun window: the rocket as an arrow (green is fine, red means tilted past about 15°), a blue thrust vector,
  an orange heading arrow, and the planned descent trajectory. Drag the timeline to scrub.
- Files written next to `Cargo.toml`:
  - `simulation.csv`: full telemetry (thrust, inertia, slosh, tank masses, chamber pressure, wind, aero, and so on)
  - `flight_data.csv`: simulated IMU readings plus the true attitude quaternion, good for testing estimators

<!-- TODO: add a screenshot of the Rerun viewer mid-flight here -->

> Both CSVs are tracked in git, so every run shows them as modified. Run `git checkout -- *.csv` to reset them.

### Expected result (as of 2026-09-22)
The sim climbs to about 50 m, hovers, and starts descending. Around t = 24.5 s the flight software aborts
with a "Tilt angle > 30°" termination just before touchdown. Your setup is fine. That's how the controller
behaves right now, and it's what the team is tuning (see the recent "MPC tuning" commits).

You'll also see `Warning: Failed to load aero lookup table (aero_table.csv: expected 50 rows (5α × 10M), got 10);
falling back to static drag.` on startup. Also expected for now: the checked-in table is the wrong shape, so the
sim uses a simpler drag model.

### Without the environment variables (Windows)
If you skipped step 3 of the OpenBLAS setup, pass the paths explicitly every time (Git Bash):

```bash
LIBRARY_PATH="C:\path\to\openblas\lib" cargo build --release
```

```bash
PATH="/c/path/to/openblas/bin:$PATH" ./target/release/rust_rocket_sim.exe
```

In PowerShell: `$env:LIBRARY_PATH = "C:\path\to\openblas\lib"` and `$env:PATH = "C:\path\to\openblas\bin;$env:PATH"`.

---

## 3. Changing what the sim does

Everything is configured in code:

| Want to... | Edit | Change |
|---|---|---|
| Start in hover or descent instead of on the pad | `src/main.rs` | uncomment `sim.start_state = "hover".to_string();` plus the `sim.rocket.position` line |
| Run the MPC genetic-algorithm tuner | `src/main.rs` | `let mode_tune = true;` (slow; prints the best weights at the end) |
| Run the vertical/constant-velocity descent test | `src/main.rs` | uncomment `crate::mpc_test::run_constant_velocity_test(); return;` |
| Change timestep / sim length | `src/simulation.rs` | `dt` (0.01 s) and `min_time` in `Simulation::default()` |
| Change vehicle physics | `src/rocket_dynamics.rs` | the `Rocket` struct and its `default()` |
| Sensors, wind, slosh, propellant | `device_sim.rs`, `wind_sim.rs`, `sloshing_sim.rs`, `fluid_dynamics.rs` | |

Where the "brains" live: the guidance, navigation, and control code is not in this repo. Cargo pulls it from
the team's other repos (`monoprop-flight-software` provides `Lander`, `control` provides `MPC`, and `guidance`
provides `rust_lossless`). `Cargo.lock` is git-ignored, so a fresh build always uses the latest `main` of those
repos. If a teammate's change breaks something, that's usually where it came from.

One sim step (`Simulation::step`) goes: simulated sensors produce a `SensorData`, the flight state machine
(`fsm.step`) turns that into a thrust and gimbal command, `rocket.step` applies the physics, and the new state
is logged to Rerun.

---

## 4. Python models (optional / legacy)

Run these from their own folders, since they write output into the current folder.

The 1D fuel sim is a simple up, hover, down thrust profile. It prints a log, writes `thrust_profile.txt`,
and shows a plot. From `DynamicsModel/`:

```bash
../.venv/Scripts/python 1D_Rocket_Fuel_Sim.py
```

The FHL 6-DOF + MPC model runs 30 s and writes `simulation_results.csv`. From `DynamicsModel/FHL-DM/`:

```bash
PYTHONUTF8=1 ../../.venv/Scripts/python fhl-dyn-model.py
```

`PYTHONUTF8=1` avoids a Windows crash on the final `→` print; macOS doesn't need it.
`wslPlotterFHL.py` plots that CSV with Plotly, but it was written for WSL, so it may need path tweaks on Windows.

The `.m` files need MATLAB.

---

## 5. Troubleshooting

| Symptom | Fix |
|---|---|
| `cannot find -lopenblas` at the end of the build | Windows: `LIBRARY_PATH` isn't set to `openblas\lib`, or you didn't reopen the terminal after setting it. macOS: `brew install openblas libomp` |
| Exe exits instantly with no output (exit code 127) | Windows: `openblas\bin` isn't on `PATH`, so `libopenblas.dll` can't be found |
| `brew not installed` or `brew --prefix failed` during the build | macOS: install Homebrew, then `brew install openblas libomp` |
| `vcpkg failed to find OpenBLAS package` during the build | Windows: you're on the MSVC toolchain; switch to GNU (see section 1) |
| Nothing appears in Rerun | Start the viewer before the sim. Make sure the viewer and crate are both 0.28 |
| Build suddenly broken after `git pull` or a fresh clone | An upstream repo (`control`, `guidance`, `monoprop-flight-software`) changed. Check its latest commits |
| The whole thing rebuilds when you expected it to be quick | Build flags changed between runs. Setting `LIBRARY_PATH` once, instead of varying it per command, avoids this |
| `UnicodeEncodeError` in Python | Set `PYTHONUTF8=1` |
