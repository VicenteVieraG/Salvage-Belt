# Salvage Belt

Salvage Belt is a headless C++26 library for comparing 3D collision algorithms on reproducible scenes of spheres and axis-aligned boxes.

## Build and run

```sh
cmake -S . -B build
cmake --build build --target Salvage_Belt
./build/bin/Salvage_Belt
```

The example executable advances one tick and prints body, candidate, and contact counts. To build just the library, use the target `salvage-core`. The default build uses the C++ standard library and native threads. A sequential-only build is available with `-DSALVAGE_ENABLE_THREADS=OFF`.

To run the regression suite, initialize the existing Catch2 submodule.

```sh
git submodule update --init tests/catch2
cmake -S . -B build -DBUILD_TESTS=ON
cmake --build build --target tests
ctest --test-dir build --output-on-failure
```

## Using the core

Link a consumer with `target_link_libraries(my_target PRIVATE salvage::core)`. A CLI can step a run an exact number of times and read counters without copying the scene. A visualizer can request an owned snapshot when it needs a frame.

```cpp
#include "salvage/core/simulation/Simulation.hpp"

using namespace salvage::core;

SimulationConfig config;
config.seed = 42;
config.scene = FieldSettings{.asteroids = 150, .debris = 50};
config.algorithm = Algorithm::kdTree;
config.workers = 4;
Simulation sim(config);
for (int i = 0; i < 120; ++i) sim.step();
auto frame = sim.snapshot();
auto lastTick = sim.metrics();
sim.restartWith(Algorithm::sweepAndPrune, 1); // Same initial bodies, tick zero.
```

| Contract | What a consumer can rely on |
| --- | --- |
| `WorldView` | Read-only borrowed bodies with dense scene-local IDs. Reacquire after a step or reset. |
| `Simulation::step()` | Advances one fixed tick. Integrates motion, updates the index, finds ordered pairs, confirms geometry, then optionally resolves contacts. The caller owns pause and wall-clock pacing. |
| `SimulationSnapshot` | Owns completed body states and the sorted candidates and contacts that led to them. Contact geometry precedes positional correction. |
| `SimulationMetrics` | Reports initialization and per-stage wall-clock seconds, examined pairs, raw and unique candidates, and confirmed contacts. Total step time includes worker dispatch, synchronization, and merging. Memory estimates use reserved index and pair element capacities. |
| `Simulation::debug()` | Supplies the indexed k-d split regions or sweep intervals on demand. All extraction, formatting, and file I/O can happen outside the timed step. |
| `reset()` and `restartWith()` | Recreate the world and strategy and clear the tick, contacts, and step metrics. A failed restart preserves the active run. |

Use `FieldSettings` for generated scenes or supply a `std::vector<Body>` in the configuration for a hand-built scenario. Generation uses one SplitMix64 stream per body, seeded from the run seed and its ID. Workers generate the initial proposals independently. Placement commits in ID order, retries conflicting positions from the same body's stream, and fails explicitly if the placement budget runs out. Spawn bounds constrain initial placement. The simulated space is open, and static box bodies can represent walls.

Brute force checks every unordered pair. The k-d tree rebuilds on each step, cycles through the three axes, and splits on a median center down to leaves of eight bodies. Bodies that span a split stay at the parent. Sweep and prune retains its ordering by minimum x between ticks, scans inclusive x intervals, and checks all three axes. Each algorithm uses the same inclusive AABB test and shared sphere/sphere, sphere/box and box/box narrow phase. Tangency counts as contact. The brute-force work counter therefore equals `N * (N - 1) / 2`.

A run with multiple workers reuses a CPU pool for proposals, integration, broad-phase scans and narrow-phase batches. The impulse and position solver runs in sorted pair order. Inputs can apply a one-tick force or impulse. Zero inverse mass fixes a body in space. Contact correction revisits detected pairs, and new pairs created by correction are detected on the next tick. Use `Response::detectionOnly` to observe contacts as bodies pass through each other. Detection is discrete, so fast bodies can cross between ticks.

## Dependencies
Building the project requires CMake 3.30 or later and a C++ compiler with C++26 mode support.

The easiest way to install these dependencies is using a package manager. Here are my preferred package managers, but feel free to use anyone you want, or even install them manually.
- [APT](https://wiki.debian.org/Apt)
- [Scoop](https://scoop.sh)
- [Winget](https://learn.microsoft.com/en-us/windows/package-manager/winget/)
- [Chocolately](https://community.chocolatey.org)

### CMake
CMake is the build system and base of the template. A build system is a tool for automating the process of building a project. It handles the compilation of source code, linking of libraries, and other tasks required to create an executable or library.

You can install CMake in different ways:
- __apt__
  ```bash
  sudo apt update
  sudo apt install cmake -y
  ```
- __Scoop__
  ```shell
  scoop bucket add main
  scoop install main/cmake
  ```
- __Winget__
  ```shell
  winget install -e --id Kitware.CMake
  ```
- __Chocolately__
  ```shell
  choco install cmake
  ```
- __Manual installation__
  - [CMake official web page](https://cmake.org/download/)

### Toolchains/Compilers
CMake requires a build kit to create all the necessary files, folders and configurations for the project. The build kits include a C/C++ compiler, linker, debugger and other tools that are necessary for the project to be built.

#### MinGW (Windows Only)
- __Scoop__
  ```shell
    scoop bucket add main
    scoop install main/mingw
  ```
- __Chocolately__
  ```shell
    choco install mingw
  ```
- __Manual installation__
  - [Official website](https://www.mingw-w64.org)

#### Clang (LLVM)
- __apt__
  ```bash
  sudo apt update
  sudo apt install clang clangd -y
  ```
- __Scoop__
  ```shell
    scoop bucket add main
    scoop install main/llvm
  ```
- __Winget__
  ```shell
  winget install -e --id LLVM.LLVM
  ```
- __Chocolately__
  ```shell
    choco install llvm
  ```
- __Manual installation__
  - [Official website](https://clang.llvm.org)

#### MSCV (Windows Only)
This build kit can only be installed through the Visual Studio installer. You need to install Visual Studio and select the descktop development with C++ workload or manually select the MSVC package in the Visual Studio installer.
- Download Visual Studio
  - Winget
    ```shell
      winget install -e --id Microsoft.VisualStudio.2022.Community.Preview
    ```
  - [Visual Studio official website](https://visualstudio.microsoft.com/es/downloads/)

### Clang-Tidy (Optional)
Linter for C/C++ source files. This template includes an option to enable per-target Clang-tidy analysis via `ENABLE_CLANG_TIDY`.
- __apt__
  ```bash
  sudo apt update
  sudo apt install clang-tidy -y
  ```
- included in [LLVM toolchain](#clang-llvm) for Windows.

### Doxygen (Optional)
Documentation is generated from Doxygen comments in the project headers and source files.

Use `/** ... */` blocks with `@brief`, `@param`, `@return`, and `@throws` as applicable.
Document member fields with `///<` comments, and use `@note` or `@pre` for lifetime
rules and caller requirements. Keep API contracts on declarations in the public
headers so generated documentation and editor tooltips share the same source.

After configuring CMake, generate the documentation with:

```sh
cmake --build build --target docs
```

Alternatively, run `doxygen Doxyfile` from the `docs` directory. Open
`docs/html/index.html` to browse the result. The configuration excludes bundled
Catch2 sources and warns about undocumented API members and parameters.
- __apt__
  ```bash
  sudo apt update
  sudo apt install doxygen -y
  ```
- __Scoop__
  ```shell
  scoop install doxygen
  ```
- __Winget__
  ```shell
  winget install -e --id DimitriVanHeesch.Doxygen
  ```
- __Chocolately__
  ```shell
  choco install doxygen.install
  ```
- __Manual Instalation__
  [Link to official web page](https://www.doxygen.nl/index.html)

## Advanced Features
The template ships with reusable CMake utility modules under [`cmake/`](cmake/) and a set of root options in [`CMakeLists.txt`](CMakeLists.txt) that let you enable tooling, diagnostics, and build optimizations.

### Root CMake options (`CMakeLists.txt`)
Use these options at configure time (for example with `-D<OPTION>=ON`):

| Option                       | Default   | Description                                                                                                     |
| ---------------------------- | --------- | --------------------------------------------------------------------------------------------------------------- |
| `BUILD_TESTS`                | `OFF`     | Enables the test tree and CTest integration (`enable_testing()`).                                               |
| `BUILD_TESTS_WITH_MAIN`      | `OFF`      | Builds Catch2 tests with a custom `main` function (used by the tests subtree).                                  |
| `ENABLE_CLANG_TIDY`          | `OFF`     | Enables clang-tidy support for targets using `target_enable_clang_tidy(<target>)`.                              |
| `ENABLE_WARNINGS`            | `OFF`      | Enables compiler warning profiles for targets using `target_set_warnings(<target> <enabled_as_errors>)`.        |
| `ENABLE_WARNINGS_AS_ERRORS`  | `OFF`     | Promotes warnings to errors when used with `target_set_warnings()`.                                             |
| `ENABLE_SANITIZERS`          | `OFF`     | Enables sanitizer helpers so targets can opt into compiler/linker sanitizer flags.                              |
| `SANITIZER_PRESET`           | `default` | Default sanitizer profile used by `target_enable_sanitizers()`: `default`, `debug`, `thread`, `memory`, `leak`. |
| `ENABLE_LTO_GLOBALY`         | `OFF`     | Enables IPO/LTO globally for `Release` and `RelWithDebInfo` when supported.                                     |

### Utility modules in `cmake/`
#### `cmake/ClangTidy.cmake`
- `target_enable_clang_tidy(<target>)`: Applies clang-tidy to one target when `ENABLE_CLANG_TIDY=ON`.
- Configures both `C_CLANG_TIDY` and `CXX_CLANG_TIDY` target properties.
- Validates argument count and target existence, and fails fast if `clang-tidy` is requested but not installed.

#### `cmake/Warnings.cmake`
- `target_set_warnings(<target> <enabled_as_errors>)`: Applies warning presets per compiler when `ENABLE_WARNINGS=ON`.
- Uses:
  - MSVC: `/W4`, `/permissive-`, and optionally `/WX`
  - Clang/GCC: `-Wall`, `-Wextra`, `-Wpedantic`, and optionally `-Werror`

#### `cmake/Sanitizer.cmake`
- `target_enable_sanitizers(<target> [preset])`: Enables sanitizer flags for a specific target. Uses `SANITIZER_PRESET` by default or an explicit per-target preset when provided.
- Supported presets:
  - `default` (MSVC: `address`; Clang/GCC: `address`, `undefined`)
  - `debug` (Clang/GCC: `address`, `undefined`, `leak`)
  - `thread` (Clang/GCC)
  - `memory` (Clang)
  - `leak` (Clang/GCC)
- Notes:
  - Executable, shared, module, and interface targets receive both compile and link flags.
  - Static and object libraries receive compile flags only, so the final linked target should also enable sanitizers.

#### `cmake/LTO.cmake`
- `target_enable_lto(<target>)`: Enables IPO/LTO for a specific target in `Release` and `RelWithDebInfo`.
- If `ENABLE_LTO_GLOBALY=ON`, enables IPO/LTO globally for supported toolchains.
- Uses `CheckIPOSupported` and emits a warning when the toolchain does not support IPO/LTO.

#### `cmake/Docs.cmake`
- Adds a `docs` custom target when Doxygen is available:
  - Runs Doxygen from `${CMAKE_SOURCE_DIR}/docs`
  - Generates HTML documentation as configured by your Doxygen project files.

#### `cmake/AddGitSubmodule.cmake`
- `add_git_submodule(<relative_dir>)`:
  - Initializes/updates a git submodule if needed.
  - Automatically calls `add_subdirectory()` when the submodule contains a `CMakeLists.txt`.

### Example advanced configurations

Enable warnings + warnings as errors:
```shell
cmake -S . -B build -DENABLE_WARNINGS=ON -DENABLE_WARNINGS_AS_ERRORS=ON
```

Enable sanitizers with the debug preset:
```shell
cmake -S . -B build -DENABLE_SANITIZERS=ON -DSANITIZER_PRESET=debug
```

Then opt targets into the configured sanitizer preset:
```cmake
target_enable_sanitizers(${CMAKE_PROJECT_NAME})
target_enable_sanitizers(tests leak)
```

Enable global IPO/LTO for release-like builds:
```shell
cmake -S . -B build -DENABLE_LTO_GLOBALY=ON
```
