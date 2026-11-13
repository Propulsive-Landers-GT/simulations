# Propulsive Landers `simulations`: Beginner's Guide (Windows & macOS)

This repo holds the physics models ("plants") used to test our flight software before it flies.
It has three parts:

| Part | Language | What it does | Status |
|---|---|---|---|
| `RocketSimulation/rust_rocket_sim/` | Rust |  A full 6-DOF lander flight (ascent, then hover, then descent) running the real flight software in the loop, with a live 3D view. Flies Monarch (the rocket) by default, or Monoprop (the small electric test vehicle) with `--vehicle monoprop` | Active |
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

### Windows: one-time toolchain setup (GNU + MSYS2)
The GNU Rust toolchain needs MinGW tools (`gcc`, `dlltool`) and a system OpenBLAS. The easiest way to get all
of them is MSYS2. Do this once:

1. **Use the GNU Rust toolchain** (PowerShell):
   ```powershell
   rustup default stable-x86_64-pc-windows-gnu
   ```
2. **Install MSYS2** from https://www.msys2.org. The default location is `C:\msys64`; any folder works
   (for example `C:\Tools\MSYS2`). Swap in your folder wherever `C:\msys64` appears below.
3. **Install the compiler, pkg-config, and OpenBLAS.** Open the **MSYS2 UCRT64** app from the Start menu and run:
   ```bash
   pacman -S --needed mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-pkgconf mingw-w64-ucrt-x86_64-openblas
   ```
4. **Add `C:\msys64\ucrt64\bin` to your `Path`.** Open Start, search for "Edit the system environment variables",
   click Environment Variables, select `Path` under *User variables*, click Edit, click New, and paste
   `C:\msys64\ucrt64\bin`. This one folder gives the build `gcc`, `dlltool`, and `pkg-config`, and gives the sim
   `libopenblas.dll` when it runs. You don't need `LIBRARY_PATH` with this setup.

### Step 1: build
The first build takes a few minutes; later ones are quick. From `RocketSimulation\rust_rocket_sim`:

```powershell
cargo build --release
```

Dozens of `warning:` lines (unused variables and so on) are normal. It worked if it ends with `Finished release`.

### Step 2: Terminal 1, start the viewer and leave it running
From the `simulations` folder:

```powershell
.venv\Scripts\rerun.exe
```

### Step 3: Terminal 2, run the sim
Open a second terminal:

```powershell
cd RocketSimulation\rust_rocket_sim
cargo run --release
```

`cargo run --release` rebuilds if you changed code, then runs. `.\target\release\rust_rocket_sim.exe` runs the
last build directly.

(On macOS: `.venv/bin/rerun` in terminal 1, and `cargo run --release` in terminal 2.)

### Errors we hit on Windows, and the fixes
| Error during `cargo build` | Cause | Fix |
|---|---|---|
| `error calling dlltool 'dlltool.exe': program not found` | GNU toolchain, but MinGW isn't installed or isn't on `Path` | Setup steps 2–5 |
| `openblas-src` build script panics with `Failed to exec cygpath` | OpenBLAS isn't installed in MSYS2, so `pkg-config` can't find it and the build falls back to `cygpath` | In PowerShell: `C:\msys64\usr\bin\pacman.exe -S --needed mingw-w64-ucrt-x86_64-openblas` (or `pacman -S --needed mingw-w64-ucrt-x86_64-openblas` in the MSYS2 UCRT64 shell or `C:\Tools\MSYS2\usr\bin\pacman.exe -S mingw-w64-ucrt-x86_64-openblas` ), then rebuild |
| `could not find Cargo.toml` | You're in the wrong folder | `cd RocketSimulation\rust_rocket_sim` first |

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

### Monoprop: the small test vehicle
The same program can fly Monoprop instead of Monarch: the electric test stand with the yellow rods, 3 rings,
3 legs, and a gimballed motor and propeller at the bottom. It runs Monoprop's real ground-station MPC
(`GroundControl` in `monoprop-flight-software`) in the loop. It feeds the controller the true state instead of
simulated sensors, so if a flight fails here, the problem is in GNC, not avionics.

**One-time setup (until the `sim-lib` branch is merged):** the sim needs the `sim-lib` branch of
`monoprop-flight-software`, cloned next to this repo:

```bash
# from the folder that contains `simulations`
git clone -b sim-lib https://github.com/Propulsive-Landers-GT/monoprop-flight-software
```

`RocketSimulation/rust_rocket_sim/.cargo/config.toml` points the build at that folder. Once `sim-lib` is merged
into `main`, delete that file and skip this step.

**Watch one flight:** start the viewer in terminal 1 exactly as for Monarch, then in terminal 2, from
`RocketSimulation/rust_rocket_sim`:

```bash
cargo run --release -- --vehicle monoprop
```

The `--` on its own matters: what comes after it goes to the sim, not to cargo. Without `--vehicle monoprop`,
you get Monarch.

The flight plays in real time and takes about 13 s. Monoprop starts on the pad, climbs to a 1 m hover,
gets the land command at t = 10 s, and lands. In Rerun you'll see the vehicle model. The motor tilts with the
gimbal, the prop spins with thrust, the blue arrow is the thrust, and the small cyan dot is the hover target.
Monoprop is under 1 m tall, so if you can't see it, double-click `world/monoprop` in the left panel to zoom in.
To slow it down further, set the Rerun timeline to `sim_time` and lower the playback speed.

The terminal prints one line per second, then the outcome (`Landed`, `Crashed: <why>`, or `Timeout`) and the
grades: touchdown distance from the pad, touchdown speed, max tilt, hover error, flight time, and peak thrust.

> The solver prints `OpEn QP solve frequency ...` on every step. To hide it, add `| grep -v "OpEn QP"`
> to the end of the command (macOS/Linux/Git Bash).

**Run many flights at once (for MPC tuning):** make a CSV of start positions. It needs a header row, then
one start per row, as `x,y,z` or `x,y,z,vx,vy,vz` (meters, m/s). A `z` of 0 means on the pad.

```csv
x,y,z
0,0,0
0.5,0,1
1,1,2
-1,0.5,1.5,0.5,0,0
```

```bash
cargo run --release -- --batch starts.csv
```

All the flights run in parallel, with no viewer needed, in well under a second. Each flight gets one row in
`batch_results.csv`, with the same grades as above.

Heights are given the way the flight computer reports them: about 0.07 m when Monoprop is standing on the pad,
matching the landing height in GroundControl.


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
| Change Monoprop's mass, size, thrust, gimbal | `src/monoprop.rs` | `MonopropParams::default()`. Mass, max thrust (15.6 N), cage radius, cage height and gimbal pivot are measured; values marked `shortcut:` are still estimates |
| Change Monoprop's land time or flight length | `src/monoprop_sim.rs` | `land_time` (10 s) and `max_time` (30 s) in `MonopropSim::new` |
| Change how Monoprop flights are graded | `src/batch.rs` | `grade()` |

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
