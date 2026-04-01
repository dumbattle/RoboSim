# Robot Exploration Simulator — API Reference

## The Environment

The world is a 2D grid procedurally populated with multiple wall types that cluster by type.
The robot starts at a random open tile and can only perceive the tile directly in front of it —
there is no global map, so it must build one from sensor data.

**Goal:** visit as many unique tiles as possible before the battery runs out.
**Score** = number of distinct tiles stepped on.

---

**Battery costs**
Every action and sensor query drains battery. The robot stops when battery hits 0.
Moving into a wall does not advance the robot, but still costs movement battery plus
a damage penalty specific to that wall type.

**Wall types**
There are `WALL_TYPE_COUNT` distinct wall types with IDs `1..WALL_TYPE_COUNT` (0 = empty).
Each type has its own collision damage. Use `GetObstacleDamage(wallID)` to query at runtime.
Walls of the same type tend to cluster together.

**Sensor error model**
Sensors are imperfect. Each wall type has independent false-positive and false-negative rates
(0–50%) that drift slowly over time. A rate of 50% is pure noise; 0% is fully accurate.

Each tile stores a Bayesian confidence distribution over all tile types. Every `ScanAhead()`
call updates the distribution for the tile ahead using the current error rates. Moving onto
or into a tile sets its confidence to 100% (ground truth).

Use `GetErrorRates()` and `GetErrorDeltas()` to observe how reliable the sensor currently is
and whether it is getting better or worse. Use `GetExpectedInfoGain()` to decide whether a
scan is worth the battery cost before committing.

**Confidence**

Confidence is tracked automatically — no setup required. Every tile starts with a uniform
distribution across all tile types (fully uncertain). It updates in three ways:

- **`ScanAhead()`** — performs a Bayesian update on the tile ahead using the current sensor
  error rates. Repeated scans on the same tile accumulate evidence.
- **Moving onto a tile** — sets that tile's confidence to 100% (ground truth, fully certain).
- **Moving into a wall** — sets that tile's confidence to 100% for the wall type hit.
- **Threshold reveal** — once any tile type's confidence crosses `CONFIDENCE_COMPLETION_THRESH`
  (default 0.95), the tile is fully revealed: that type is set to 1.0, all others to 0.0.
  This can happen from scanning alone, without physically visiting the tile.

Once a tile is fully revealed, further `ScanAhead()` calls on it are free (no battery cost).

---

## API Reference

### Setup
```cpp
Reset();           // generate the world and place the robot — call once at the start
Reset(seed);       // same, but overrides SEED from robot_params.h with a specific seed
PrintResults();    // print final stats and close the window — call at the end
```

---

### Movement *(costs battery)*
```cpp
int MoveForward();          // move one tile forward, tile confidence set to 100%
                            //   open tile                      -> robot advances, returns 0
                            //   wall                           -> robot stays, battery drained by move + damage, returns wall type (1..WALL_TYPE_COUNT)
                            //   out of bounds / battery empty  -> returns -1
void TurnLeft();                 // rotate 90° counter-clockwise
void TurnRight();                // rotate 90° clockwise
void TurnToDirection(Direction); // rotate to face Direction using minimum turns (0, 1, or 2)
```

---

### Sensor — scanning *(costs battery)*
```cpp
// Scans the tile directly ahead for the given wall type. Updates tile confidence.
// FREE if the tile is already certain (confidence == 1) or the error rate is 50%.
// wallID: [1, WALL_TYPE_COUNT]
void ScanAhead(int wallID);
```

---

### Sensor — error rates *(free)*
```cpp
// Current false-positive and false-negative rates for a wall type (0–50%).
// wallID: [1, WALL_TYPE_COUNT]
void GetErrorRates(int wallID, int& falsePositive, int& falseNegative);

// Rate of change of the error rates (% per tick). Positive = getting worse.
// wallID: [1, WALL_TYPE_COUNT]
void GetErrorDeltas(int wallID, float& fpDelta, float& fnDelta);
```

---

### Tile confidence *(free)*
```cpp
// Shannon entropy of tile (x,y)'s confidence distribution, normalized to [0,1].
// 0 = fully certain, 1 = fully uncertain (uniform across all tile types).
float GetEntropy(int x, int y);

// Confidence (0–1) that tile (x,y) is a given type.
// wallID: 0 = empty, 1..WALL_TYPE_COUNT = wall type ID.
float GetTileConfidence(int x, int y, int wallID);

// Full distribution: results[0]=P(empty), results[i]=P(wall type i).
void GetTileConfidence(int x, int y, float (&results)[TILE_TYPE_COUNT]);

// Expected information gained by scanning tile (x,y) for wall type wallID.
// wallID: [1, WALL_TYPE_COUNT]. Returns probabilities and entropy deltas for both possible scan outcomes.
InfoGain GetExpectedInfoGain(int x, int y, int wallID);
```

`InfoGain` fields:
```cpp
float expected;          // expected entropy reduction (weighted average)   [0, 1]
float probPositive;      // P(scan fires — obstacle detected)               [0, 1]
float deltaIfPositive;   // entropy change if scan fires                    [-1, 1]
float probNegative;      // P(scan silent — no obstacle detected)           [0, 1]
float deltaIfNegative;   // entropy change if scan is silent                [-1, 1]
```

---

### Robot state *(free)*
```cpp
void GetPosition(int& x, int& y);   // current grid position
Direction GetDirection();            // current facing direction
int  GetBattery();                   // remaining battery
bool HasBattery();                   // true if battery > 0
int  GetScore();                     // unique tiles visited so far
bool InRange(int x, int y);          // true if (x,y) is within map bounds
bool TileVisited(int x, int y);      // true if robot has previously stepped on (x,y)
int  GetObstacleDamage(int wallID);  // battery damage for wall type ID (1..WALL_TYPE_COUNT)
int  GetMapWidth();                  // map width in tiles
int  GetMapHeight();                 // map height in tiles
```

---

### Direction helpers *(free)*
```cpp
Direction Left(Direction d);                      // 90° left of d
Direction Right(Direction d);                     // 90° right of d
void ToVector(Direction d, int& dx, int& dy);     // unit step vector for d
void Translate(int& x, int& y, Direction d);      // shift (x,y) one step in direction d
bool TryToDirection(int dx, int dy, Direction&);  // axis-aligned vector -> Direction (false if diagonal/zero)
std::string ToString(Direction d);                // "North" / "East" / "South" / "West"
```

Direction enum:
```cpp
NORTH   // +Y
EAST    // +X
SOUTH   // -Y
WEST    // -X
```

---

## State Machine

`state_machine.h` provides a lightweight named-state dispatcher for organising multi-phase algorithms.

```cpp
#include "../include/state_machine.h"

StateMachine sm;

sm.Add("explore", []{ /* runs every tick in explore mode */ });
sm.OnEnter("explore", []{ /* called once when entering explore */ });
sm.OnExit("explore",  []{ /* called once when leaving explore */ });

sm.Add("return", []{ /* runs every tick in return mode */ });

sm.SetMode("explore");
while (HasBattery()) sm.Tick();
```

```cpp
void Add(name, fn);      // register a tick function for the named mode
void OnEnter(name, fn);  // optional: called once on mode entry
void OnExit(name, fn);   // optional: called once on mode exit
void SetMode(name);      // switch modes (fires OnExit then OnEnter); throws if mode unknown
void Tick();             // execute the current mode's tick function once
```

---

## Writing a Solution

Create `solutions/YourSolution.cpp`:

```cpp
#include "robot_api.h"

int main() {
    Reset();

    while (HasBattery()) {
        // your algorithm here
    }

    PrintResults();
    return 0;
}
```

Open the file in VS Code and press `Ctrl+Shift+B` to build. Output: `build\YourSolution.exe`.
