# Rowing Boat Sensor System

Unified repo for the rowing boat sensor project. We moved from Arduino IDE to **PlatformIO** to avoid merge conflicts, library version mismatches, and people breaking each other's code while working on DMS, Gyro, and Bluetooth in parallel.

## Arduino IDE vs PlatformIO

| | Arduino IDE | PlatformIO |
|---|---|---|
| Dependency management | Manual install per machine, versions drift | Pinned in `platformio.ini`, auto-downloaded |
| Project structure | One `.ino` per folder, no real separation | Standard C++ with `src/`, `lib/`, `include/` |
| Multiple build targets | One upload button | Separate environments per component (`test_dms`, `prod`, ...) |
| Version control | Messy to git cleanly | Reproducible, everything tracked |
| Editor | Barebones built-in editor | Runs inside VS Code -> IntelliSense, autocomplete, real debugger |
| Setup friction | Zero, good for beginners | Needs VS Code + extension install |

For a solo blink sketch, Arduino IDE is fine. For a team project with parallel components, FreeRTOS, and multiple test environments, PlatformIO is the right tool.

## Project Structure

- `lib/` -> your component code goes here. One folder per component (e.g. `lib/DMS/`), `.cpp` and `.h` files inside.
- `test/` -> standalone test files per component (e.g. `test_dms.cpp`), each with its own `setup()` and `loop()`.
- `src/main.cpp` -> integration only. Just includes headers from `lib/` to run the final boat software, no logic here.
- `platformio.ini` -> project config, handles all library downloads automatically.

## Installation

### 1. VS Code + PlatformIO

1. Install [Visual Studio Code](https://code.visualstudio.com/).
2. Open Extensions (`Ctrl+Shift+X`), search **PlatformIO IDE**, install, restart.
3. `File > Open Folder` and select the repo root (the folder with `platformio.ini` in it). PlatformIO pulls all dependencies automatically.

### 2. Serial Plotter - Teleplot

- [https://github.com/nesnes/teleplot]

1. Extensions -> search **Teleplot**, install.
2. Format your output like this: `>VariableName: 85.2` (Check out teleplot repo)
3. Click the Teleplot icon to get zoomable, high-speed, saveable graphs.

## Compile & Upload

Use **PlatformIO Project Tasks** (the Ant Head icon in the left sidebar).

Documentation: [https://docs.platformio.org/en/latest/what-is-platformio.html]
> Check also the READMEs in the subfolders (Short descriptions from PlatformIO)

Available environments:

- `prod_nodemcu-32s` -> builds `src/main.cpp`
- `test_dms` -> builds the DMS test file only
- `test_gyro` -> builds the Gyro test file only
- create more tests environments in `test/test_file.cpp` and log in `platformio.ini`

Open your environment, click **General**, then **Upload and Monitor**. (ctrl B (build) and then ctrl U (upload))
