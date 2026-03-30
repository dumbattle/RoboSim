# Robot Exploration Simulator

A grid-based robot exploration simulator built in C++ with real-time SFML visualizatio
The robot navigates a procedurally generated map of walls and cliffs on a limited battery budget.

**Score** = number of unique tiles visited before the battery dies.

Originally built as a teaching tool to compare exploration algorithms,
as well as to benchmark LLM code generation abilities.

---

## How It Works

The world is a 2D grid procedurally populated with multiple obstacle types that cluster
by type. The robot starts at a random tile and can only perceive the tile directly in front
of it - there is no global map, so it must build one from sensor data.

**Battery costs**
Every action and sensor query drains battery. The robot stops when battery hits 0.
Moving into a wall does not advance the robot, but still costs movement battery plus
a damage penalty specific to that obstacle type.

**Obstacle types**
Each obstacle type has a configurable damage value (battery drained on collision).
ObObstacle types are indexed `0..WALL_TYPE_COUNT-1`. Use `GetObstacleDamage(index)` to
query damage at runtime.

**Sensor error model**
Sensors are imperfect. Each obstacle type has independent false-positive and
false-negative rates (0–50%) that drift slowly over time via a random-walk transition
system. A rate of 50% is pure noise. 0% is completely accurate. 

Internally each tile stores a Bayesian confidence distribution over all tile types.
Every `ScanAhead()` call updates the distribution for the tile ahead using the current
error rates. Moving onto or into a tile sets its confidence to 100% (ground truth).

Use `GetErrorRates()` and `GetErrorDeltas()` to observe how reliable the sensor currently
is and whether it is getting better or worse. Use `GetExpectedInfoGain()` to decide
whether a scan is actually worth the battery cost before committing.

These are provided to simplify implementations.

---

## Writing Your Own Algorithm

### Full API Reference

**Setup**
```cpp
Reset();           // generates the world and places the robot - call once at the start
PrintResults();    // prints score and shows final state - call at the end
```

**Movement** *(cost battery)*
```cpp
MoveForward();              // move one tile forward
                            //   open tile -> robot advances, tile confidence set to 100%
                            //   obstacle  -> robot stays, battery drained by move + damage
TurnLeft();                 // rotate 90° counter-clockwise
TurnRight();                // rotate 90° clockwise
TurnToDirection(Direction); // rotate to face Direction using minimum turns (0, 1, or 2)
```

**Sensor scanning** *(costs battery)*
```cpp
// Scans the tile ahead for obstacle type obstacleIndex. Updates tile confidence.
// FREE if tile is already 100% certain or error rate is 50%.
void ScanAhead(int obstacleIndex);
```

**Sensor error rates** *(free)*
```cpp
// Current false-positive and false-negative rates for an obstacle type (0–50%).
void GetErrorRates(int obstacleIndex, int& falsePositive, int& falseNegative);

// Rate of change of the error rates (% per tick). Positive = getting worse.
void GetErrorDeltas(int obstacleIndex, float& fpDelta, float& fnDelta);
```

**Tile confidence** *(free)*
```cpp
// Single confidence value: P(tile (x,y) is obstacleIndex). obstacleIndex -1 = empty.
float GetTileConfidence(int x, int y, int obstacleIndex);

// Full distribution: results[0]=P(empty), results[i]=P(obstacle type i-1).
void GetTileConfidence(int x, int y, float (&results)[TILE_TYPE_COUNT]);

// Expected information gained by scanning tile (x,y) for obstacleIndex.
// Returns probabilities and entropy deltas for both scan outcomes.
InfoGain GetExpectedInfoGain(int x, int y, int obstacleIndex);
```

**Robot state queries** *(free)*
```cpp
void GetPosition(int& x, int& y);  // current grid position
Direction GetDirection();           // current facing direction
int  GetBattery();                  // remaining battery
bool HasBattery();                  // true if battery > 0
int  GetScore();                    // unique tiles visited so far
bool InRange(int x, int y);         // true if (x,y) is within map bounds
bool TileVisited(int x, int y);     // true if robot has stepped on (x,y)
int  GetObstacleDamage(int index);  // battery damage for obstacle type index
int  GetMapWidth();                 // map width in tiles  (== MAP_WIDTH)
int  GetMapHeight();                // map height in tiles (== MAP_HEIGHT)
```

**Direction helpers** *(free)*
```cpp
Direction Left(Direction d);                     // 90° left of d
Direction Right(Direction d);                    // 90° right of d
void ToVector(Direction d, int& dx, int& dy);    // unit vector for d
void Translate(int& x, int& y, Direction d);     // shift (x,y) one step in d
bool TryToDirection(int dx, int dy, Direction&); // vector -> direction (cardinal only)
std::string ToString(Direction d);               // "North" / "East" / "South" / "West"
```

**Direction enum**
```cpp
NORTH   // +Y
EAST    // +X
SOUTH   // -Y
WEST    // -X
```

---

## Tuning the Simulation

Edit `include/robot_params.h` to adjust map size, battery budget, obstacle behaviour,
and animation speed.

```cpp
// Map
const int MAP_WIDTH   = 150;
const int MAP_HEIGHT  = 100;

// Battery
const int MAX_BATTERY       = 50'000;
const int BATTERY_MOVE      = 5;
const int BATTERY_TURN      = 3;
const int BATTERY_QUERY_MIN = 1;
const int BATTERY_QUERY_MAX = 3;

// Seed
const unsigned int SEED = 31415;

// Obstacle types - add/remove entries to change the number of types
const WallData WALL_DATA[] = {
    {
        50,            // damage on collision
        { {25, 55} },  // error rate ranges (random-walk target ranges, %)
        25,            // transition time (ticks to interpolate to new rate)
        { 15, 25 }     // rest period range (ticks between transitions)
    },
    {
        500,
        { {15, 35} },
        20,
        { 25, 40 }
    }
};

// Obstacle layer layout - one entry per layer, value = WALL_DATA index
const int WALL_LAYER_TYPES[] = { 0, 0, 1 };
const int WALL_LAYERS[]      = { 40, 30, 30 };  // % of map per layer

// Animation
const int SLEEP_MILLISECONDS = 25;
const int PRINT_INTERVAL     = 100;
const int TILE_PX            = 8;
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
│   ├── debug_api.h         <- Debug/test API (SetTile, SetErrors, etc.)
│   ├── robot_params.h      <- Simulation parameters (edit to tune)
│   └── ...
├── src/
│   ├── robot_sim.cpp       <- API implementation
│   ├── debug_api.cpp       <- Debug API implementation
│   ├── robot.cpp           <- Robot state
│   ├── world.cpp           <- Map generation
│   └── display.cpp         <- SFML rendering
└── solutions/              <- Benchmark solutions and test scripts
```
tion
│   ├── robot.cpp           <- Robot state
│   ├── world.cpp           <- Map generation
│   └── display.cpp         <- SFML rendering
└── solutions/              <- AI benchmark solutions + my own

```