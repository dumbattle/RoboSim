# Robot Exploration Simulator

A grid-based robot exploration simulator built in C++ with real-time SFML visualization.
The robot navigates a procedurally generated map of walls, cliffs, and traps on a limited
battery budget. 

Score = number of unique tiles visited before the battery dies.

Originally built as a teaching tool to compare exploration algorithms,
as well as to benchmark LLM coge generation abilities.

---

## How It Works

The world is a 2D grid randomly populated with walls. The robot
starts at a random tile and can only perceive exactly 1 tile in front of it. 
There is no accessible global map, the robot must build one itself.

**Battery costs**
All actions and sensor queries cost energy. 

The robot crashes (and the run ends) if it moves into a wall or runs
out of battery.

---

## Writing Your Own Algorithm

Open a new .cpp and write your exploration logic using the robot API:

```cpp
#include "robot_api.h"

int main() {
    Reset();

    while (true) {
        if (IsWallAhead() || IsCliffAhead()) {
            TurnLeft();
        } 
        else {
            MoveForward();
        }
    }

    PrintResults();
}
```

### Full API Reference

**Setup**
```cpp
Reset();           // call at the start - generates the world and places the robot
PrintResults();    // call at the end - prints score and shows final frame.
```

**Movement** *(cost battery)*
```cpp
MoveForward();     // move one tile forward - crashes on walls
TurnLeft();        // rotate 90° counter-clockwise
TurnRight();       // rotate 90° clockwise
```

**Sensors** *(cost battery, may have error if SENSOR_ERROR_PROB > 0)*
```cpp
bool IsWallAhead();    // true if wall is directly ahead
```

**Info** *(free - no battery cost)*
```cpp
void GetPosition(int& x, int& y);    // current grid position
Direction GetDirection();            // current facing direction
int GetBattery();                    // remaining battery
int GetScore();                      // unique tiles visited so far
```

**Direction helpers** *(free - to maintain consistancy with internal implementation)*
```cpp
Direction Left(Direction d);                        // direction 90° left of d
Direction Right(Direction d);                       // direction 90° right of d
void ToVector(Direction d, int& dx, int& dy);       // unit vector for d
void Translate(int& x, int& y, Direction d);        // shift (x,y) one step in d
```

**Direction enum**
```cpp
Direction::NORTH   // y + 1
Direction::EAST    // x + 1
Direction::SOUTH   // y - 1
Direction::WEST    // x - 1
```

---

## Tuning the Simulation

Edit `include/robot_params.h` to change map size, battery budget, obstacle density, sensor error, and animation speed.

```cpp
// Default Values
const int MAP_WIDTH           = 150;
const int MAP_HEIGHT          = 100;
const int MAX_BATTERY         = 50'000;
const int BATTERY_MOVE        = 5;
const int BATTERY_TURN        = 3;
const int BATTERY_QUERY_MIN   = 1;
const int BATTERY_QUERY_MAX   = 3;
const double SENSOR_ERROR_PROB = 0.0;   // 0.0–1.0
const int WALL_LAYERS[]       = [40, 30, 30]; // target % of map per layer
const int CLIFF_PROBABILITY   = 30;
const int OBS_X_PROBABILITY   = 30;
const unsigned int SEED       = 3141;
```

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
 into the `build\` folder next to the `.exe` after your
first build. The ones you need are:

```
sfml-graphics-2.dll
sfml-window-2.dll
sfml-system-2.dll
```

**3. Create the build folder**

Create an empty `build\` folder inside the project root. The compiler may not create it
automatically.

```
mkdir build
```

**4. Build**

Open your file, or any of the benchmark solutions, in VS Code and press `Ctrl+Shift+B`. The output exe will be at
`build\main.exe`.

> **Important:** Always build from a **Developer Command Prompt** session, or launch
> VS Code from one, so that `cl.exe` is on the PATH. You can find it in the Start menu
> under *Visual Studio -> Developer Command Prompt*.

**5. Run**

```
build\main.exe
```

---

## Project Structure

```
├── README.md
├── .gitignore
├── .vscode/
│   └── tasks.json
├── include/
│   ├── robot_api.h         <- API declarations
│   ├── robot_params.h      <- Simulation parameters (edit to tune)
│   └── ...
├── src/
│   ├── robot_sim.cpp       <- API implementation
│   ├── robot.cpp           <- Robot state
│   ├── world.cpp           <- Map generation
│   └── display.cpp         <- SFML rendering
└── solutions/              <- AI benchmark solutions + my own

```