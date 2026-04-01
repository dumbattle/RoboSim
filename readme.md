# Robot Exploration Simulator

A grid-based robot exploration simulator built in C++ with real-time SFML visualization
The robot navigates a procedurally generated map of walls on a limited battery budget.

**Score** = number of unique tiles visited before the battery dies.

Originally built as a teaching tool to compare exploration algorithms,
as well as to benchmark LLM code generation abilities.

---

## How It Works

A few features that aren't obvious from the API alone:

**Sensor error model**
The robot's sensor is imperfect. Each wall type has its own false-positive rate (reports a wall when there isn't one) and false-negative rate (misses a wall that is there), both ranging from 0–50%. These rates drift independently over time — a sensor that's reliable now may degrade, and vice versa. `GetErrorRates()` gives the current values; `GetErrorDeltas()` tells you whether they're getting better or worse.

**Bayesian confidence**
Every tile stores a probability distribution across all tile types (empty + each wall type). It starts uniform (fully uncertain). Each `ScanAhead()` call performs a Bayesian update on the tile ahead using the live error rates. Moving onto a tile — or hitting a wall — sets that tile's confidence to 100% (ground truth). Once any type's probability crosses a threshold (default 0.95), the tile is *revealed*: its distribution snaps to certainty. Revealed tiles cost no battery to scan. Use `GetTileConfidence()` and `GetEntropy()` to read the current state of the robot's belief.

**Obstacle collision damage**
Walls don't kill the robot — they drain battery. Each wall type has its own collision damage, queryable with `GetObstacleDamage()`. Depending on the parameters, it can be worth scanning carefully before entering unknown territory, or it may be cheaper to just walk in and take the hit. `MoveForward()` returns the wall type ID (1..`WALL_TYPE_COUNT`) on a collision, so you can react accordingly.

**Chunk reveal**
Walls that are completely surrounded by other wall tiles (unreachable from any open tile) can never be visited, so the robot has no way to confirm them by walking in. These isolated clusters are tracked separately: once every wall tile on the cluster's border becomes certain via scanning, the entire cluster is revealed all at once. This is visible in the display as a brightness change.

---

## Writing Your Own Algorithm

See [reference.md](reference.md) for the full environment description and API reference.

Create `solutions/YourSolution.cpp`, open it in VS Code, and press `Ctrl+Shift+B` to build.

```cpp
#include "../include/robot_api.h"

int main() {
    Reset();

    while (HasBattery()) {
        // your algorithm here
    }

    PrintResults();
    return 0;
}
```

---

## Tuning the Simulation

Edit `include/robot_params.h` to adjust map size, battery budget, obstacle behaviour,
and animation speed.

```cpp
// Default Values
// Map
const int MAP_WIDTH   = 150;
const int MAP_HEIGHT  = 100;

// Battery
const int MAX_BATTERY       = 50'000;
const int BATTERY_MOVE      = 5;
const int BATTERY_TURN      = 3;
const int BATTERY_QUERY_MIN = 1;
const int BATTERY_QUERY_MAX = 3;
```

---

## AI Benchmark Results

---

## Building on Windows (VS Code + MSVC)

### Prerequisites
- [Visual Studio Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/)
  with the **MSVC C++ compiler** component
- [VS Code](https://code.visualstudio.com/) with the
  [C/C++ extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
- [SFML 2.6.2 - Visual C++ 64-bit](https://www.sfml-dev.org/download/sfml/2.6.2/)

### Setup

**1. Install SFML**

Download the **Visual C++ 64-bit** package from the link above and extract it to:
```
C:\SFML-2.6.2\
```
The path must be exactly `C:\SFML-2.6.2\` - the build task is configured to look there.

**2. Copy the SFML DLLs**

Inside `C:\SFML-2.6.2\bin\` you'll find several `.dll` files. Copy the following
into the `build\` folder next to the `.exe` after your first build:

```
sfml-graphics-2.dll
sfml-window-2.dll
sfml-system-2.dll
```

**3. Create the build folder**

```
mkdir build
```

**4. Build**

Open your solution file in VS Code and press `Ctrl+Shift+B`. Output will be at `build\<name>.exe`.

> **Important:** Always build from a **Developer Command Prompt** session, or launch
> VS Code from one, so that `cl.exe` is on the PATH.

**5. Run**

```
build\main.exe
```

---

## Project Structure

```
├── readme.md
├── .gitignore
├── .vscode/
│   └── tasks.json
├── include/
│   ├── robot_api.h         <- Public API declarations
│   ├── robot_params.h      <- Simulation parameters (edit to tune)
│   ├── state_machine.h     <- Optional state machine helper
│   └── debug_api.h         <- Debug/test API (SetTile, SetErrors, etc.)
├── src/
│   ├── robot_sim.cpp       <- API implementation
│   ├── state_machine.cpp   <- State machine implementation
│   ├── robot.cpp           <- Robot state
│   ├── world.cpp           <- Map generation
│   ├── display.cpp         <- SFML rendering
│   └── debug_api.cpp       <- Debug API implementation
└── solutions/              <- Benchmark solutions and test scripts
```