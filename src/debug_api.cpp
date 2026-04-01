#include "debug_api.h"
#include "robot_api.h"
#include "robot_params.h"
#include "robot.h"
#include "world.h"
#include <algorithm>

using namespace std;


// ----------------------
// World / Tile
// ----------------------

void SetTile(int x, int y, int wallID) {
    if (!InRange(x, y)) return;
    world[y][x] = wallID - 1;
}

int GetTile(int x, int y) {
    if (!InRange(x, y)) return -1;
    return world[y][x] + 1;
}

void FillRect(int x1, int y1, int x2, int y2, int wallID) {
    int lx = min(x1, x2), hx = max(x1, x2);
    int ly = min(y1, y2), hy = max(y1, y2);
    for (int y = ly; y <= hy; y++)
        for (int x = lx; x <= hx; x++)
            SetTile(x, y, wallID);
}

void ClearTile(int x, int y) {
    SetTile(x, y, 0);
}

void SetVisited(int x, int y, bool visited) {
    if (!InRange(x, y)) return;
    _visited[y][x] = visited;
}


// ----------------------
// Confidence
// ----------------------

void SetConfidenceCertain(int x, int y, int tileTypeIndex) {
    if (!InRange(x, y)) return;
    auto& conf = _confidence[y][x];
    for (int i = 0; i < TILE_TYPE_COUNT; i++)
        conf[i] = (i == tileTypeIndex) ? 1.0f : 0.0f;
}

void ResetConfidence(int x, int y) {
    if (!InRange(x, y)) return;
    auto& conf = _confidence[y][x];
    float uniform = 1.0f / TILE_TYPE_COUNT;
    for (int i = 0; i < TILE_TYPE_COUNT; i++)
        conf[i] = uniform;
}

void ResetAllConfidence() {
    for (int y = 0; y < MAP_HEIGHT; y++)
        for (int x = 0; x < MAP_WIDTH; x++)
            ResetConfidence(x, y);
}


// ----------------------
// Error Rates
// ----------------------

// Locks by setting current = start = target and using a huge timer
// so the transition system won't change the value during testing.
void SetErrors(int wallID, int falsePositive, int falseNegative) {
    falsePositive = max(0, min(50, falsePositive));
    falseNegative = max(0, min(50, falseNegative));

    auto& e = sensorErrors[wallID - 1];

    e.falsePositive.currentRate = falsePositive;
    e.falsePositive.startRate   = falsePositive;
    e.falsePositive.targetRate  = falsePositive;
    e.falsePositive.timer       = INT_MAX;

    e.falseNegative.currentRate = falseNegative;
    e.falseNegative.startRate   = falseNegative;
    e.falseNegative.targetRate  = falseNegative;
    e.falseNegative.timer       = INT_MAX;
}

void ClearErrors() {
    for (int i = 1; i <= WALL_TYPE_COUNT; i++)
        SetErrors(i, 0, 0);
}


// ----------------------
// Robot State
// ----------------------

void SetRobotPosition(int x, int y) {
    if (!InRange(x, y)) return;
    robot.x = x;
    robot.y = y;
}

void SetRobotDirection(Direction d) {
    robot.dir = d;
}

void SetBattery(int amount) {
    robot.battery = max(0, amount);
}


// ----------------------
// World Inspection
// ----------------------

int CountTiles(int wallID) {
    int count = 0;
    for (int y = 0; y < MAP_HEIGHT; y++)
        for (int x = 0; x < MAP_WIDTH; x++)
            if (world[y][x] == wallID - 1) count++;
    return count;
}

bool PeekAhead(int wallID) {
    int nx = robot.x, ny = robot.y;
    Translate(nx, ny, robot.dir);
    if (!InRange(nx, ny)) return false;
    return world[ny][nx] == wallID - 1;
}
