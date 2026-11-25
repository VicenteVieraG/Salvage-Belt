# Salvage Belt

Salvage Belt is a 3D collision simulation built for the *Análisis y Diseño de Algoritmos* course at ITESO. Its asteroid belt doubles as a benchmark: how does the cost of finding collisions change when the same bodies fly through open space, gather in clusters, or crowd into a small volume?

The C++ core models asteroids as spheres and debris as axis-aligned boxes. Three broad-phase algorithms generate candidate pairs: brute force, k-d tree, and sweep and prune. All three use the same contact tests and physics. The headless CLI lets you compare them on the same seeded world, and the terminal dashboard lets you inspect live runs and saved reports.

## Build

Use CMake 3.30 or newer, Ninja, Git, and a C/C++ toolchain with C++26 mode and a recent standard library. The Linux build has been verified with GCC 15.2 and CMake 4.2.3. The commands below use a Release build for benchmarking.

### Get the source

On either platform, clone the repository with its dependencies:

```sh
git clone --recurse-submodules https://github.com/VicenteVieraG/Salvage-Belt.git
cd Salvage-Belt
```

For an existing Git checkout, run `git submodule update --init --recursive` from the repository root. If you downloaded a source ZIP, use the recursive clone above to obtain the submodules.

The headless CLI also requires CLI11. If `external/cli11/CMakeLists.txt` is missing after the submodule update, run these commands from the repository root to get the revision used for the verified build:

```sh
git clone https://github.com/CLIUtils/CLI11.git external/cli11
git -C external/cli11 checkout 4fa080f70b6768d30e2fcf58ab5a9de162d5f8fd
```

### Windows 11

With Scoop installed, open PowerShell and install the build tools. Run the CMake commands from the repository root. Selecting GCC explicitly keeps the Ninja build on the MinGW toolchain.

```powershell
scoop install git cmake ninja gcc
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++
cmake --build build --parallel
```

### Linux

On Ubuntu 26.04, install the prerequisites, then configure and compile from the repository root:

```bash
sudo apt install git cmake ninja-build gcc-15 g++-15
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc-15 -DCMAKE_CXX_COMPILER=g++-15
cmake --build build --parallel
```

On other distributions, install equivalent packages and use the names of the installed compilers. To build only the benchmark executable, append `--target salvage-headless` to the build command.

## Run

The Ninja build places the executables in `build/bin`. Run the following commands from the repository root:

| Task | Linux | Windows PowerShell |
| --- | --- | --- |
| Run one simulation step and print body, candidate, and contact counts | `./build/bin/Salvage_Belt` | `.\build\bin\Salvage_Belt.exe` |
| Benchmark all three algorithms with default settings | `./build/bin/salvage-headless` | `.\build\bin\salvage-headless.exe` |
| Open the live terminal dashboard with 256 bodies | `./build/bin/salvage-analytics --bodies 256` | `.\build\bin\salvage-analytics.exe --bodies 256` |

## Headless CLI guide

### Compare algorithms on one field

Start with a short run. The examples use Linux executable paths. In PowerShell, substitute the corresponding executable from the table above and keep the same arguments.

```bash
./build/bin/salvage-headless --algorithm all --bodies 128 --seed 42 --warmup 2 --steps 10 --repetitions 1
```

Each algorithm starts from the same field, runs two unmeasured ticks, then records ten. Read the validation result first. A passing variant matched the independent all-pairs reference check. The summary reports median microseconds per tick (lower is faster) and, when a validated single-worker brute-force baseline is present, speedup relative to that baseline.

### Compare worker counts and export the results

```bash
./build/bin/salvage-headless --mode replay --algorithm all --scenario "uniform,clustered" --workers "1,4" --bodies 256 --seed 42 --json belt.json --csv ticks.csv
./build/bin/salvage-analytics --report belt.json
```

This runs 12 variants: three algorithms × two scenarios × two worker counts. Replay feeds every variant the same recorded frames for a given scene, so the timings isolate collision detection. Use the default `--mode simulation` to measure the full physics tick instead. JSON includes metadata, samples, and summaries. CSV has one row per measured tick. Both files are written to the current directory, and the dashboard can open either format.

### Choose what to vary

| Option | What it controls |
| --- | --- |
| `--algorithm` | `all` (default), or a list drawn from `brute-force`, `kd-tree`, and `sweep-and-prune`. |
| `--scenario`, `--seed` | Field presets: `uniform` (default), `clustered`, `heterogeneous`, `fast`, `slow`, `dense`, and `aligned`. The default seed is `1`. |
| `--bodies` | Total bodies, divided evenly between shapes with any extra body assigned to asteroids. Defaults to 128. For a custom mix, replace this option with e.g. `--asteroids 96 --debris 32`. |
| `--scale` | When changing body count, use `volume` (default) to keep the spawn region fixed or `density` to resize it proportionally in volume. |
| `--workers` | CPU workers per variant, including the calling thread. Defaults to 1. |
| `--warmup`, `--steps`, `--repetitions` | Defaults to 10 unmeasured ticks, 60 measured ticks, and 3 independent cold starts per variant. |
| `--fixed-step`, `--response` | Simulated seconds per tick (default 1/120). Contacts change velocities in `resolve` mode (default). Use `detection-only` to detect contacts without resolving them. |
| `--no-validate` | Skip the correctness check. Validation is enabled by default and runs separately from the timed trials. |

Algorithms, scenarios, seeds, body counts, and worker counts accept comma-separated lists. Quote each list as one argument. Use `--json -` or `--csv -` to write one export to standard output while the human-readable summary goes to standard error. Run `salvage-headless --help` using the executable path above for the full option reference.

## Salvage Belt visualizer (upcoming)

The planned raylib visualizer will share the simulation core with the CLI. A third-person view will follow a salvage ship through the belt, while a free-camera sandbox will show k-d tree partitions, candidate pairs, and live metrics. Selecting another algorithm will restart the scene with the same settings and seed so its behavior can be compared from the same starting point.
