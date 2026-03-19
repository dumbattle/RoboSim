#include "robot_api.h"
#include "robot_params.h"
#include "robot.h"
#include "world.h"
#include "display.h"
#include <iostream>
#include <cstdlib>
#include <algorithm>

using namespace std;

// ----------------------
// Internal helpers
// ----------------------
static bool sensorError() {
    return (static_cast<double>(rand()) / RAND_MAX) < SENSOR_ERROR_PROB;
}

[[noreturn]] static void crash(const string& msg) {
    cout << "\n*** EVENT: " << msg << " ***\n";
    robot.battery = 0;
    PrintResults();
    closeDisplay();
    exit(0);
}

static void drainBattery(int cost, const char* depletedMsg) {
    if (robot.battery < cost) crash(depletedMsg);
    robot.battery -= cost;
}

// ----------------------
// Public Helpers
// ----------------------

Direction Left(Direction d) {
    return static_cast<Direction>((d + 3) % 4);
}
Direction Right(Direction d) {
    return static_cast<Direction>((d + 1) % 4);
}

bool TryToDirection(int x, int y, Direction& outDirection) {
    if (x == 0 && y > 0) { outDirection = NORTH; return true; }
    if (x > 0 && y == 0) { outDirection = EAST;  return true; }
    if (x == 0 && y < 0) { outDirection = SOUTH; return true; }
    if (x < 0 && y == 0) { outDirection = WEST;  return true; }

    return false; // not a cardinal direction
}

// Convert a direction into a unit vector
void ToVector(Direction d, int& resultX, int& resultY) {
    switch (d)     {
        case NORTH: resultX = 0;  resultY = 1;  break;
        case EAST:  resultX = 1;  resultY = 0;  break;
        case SOUTH: resultX = 0;  resultY = -1; break;
        case WEST:  resultX = -1; resultY = 0;  break;
    }
}

// Move a point (x, y) one step in direction d
void Translate(int& x, int& y, Direction d) {
    int dx, dy;
    ToVector(d, dx, dy);
    x += dx;
    y += dy;
}

string ToString(Direction d) {
    switch (d) {
        case NORTH: return "North";
        case EAST:  return "East";
        case SOUTH: return "South";
        case WEST:  return "West";
    }
    return "INVALID DIRECTION";
}

// ----------------------
// Public API
// ----------------------

void Reset(long randomSeed) {
    if (randomSeed < 0) {
        randomSeed = SEED;
    }
    initDisplay();

    _visited.assign(MAP_HEIGHT, vector<bool>(MAP_WIDTH, false));
    _seen.assign(MAP_HEIGHT, vector<bool>(MAP_WIDTH, false));
    generateWorld(randomSeed);

    pair<int,int> start = randomEmptyTile();
    robot.x       = start.first;
    robot.y       = start.second;
    robot.dir     = EAST;
    robot.battery = MAX_BATTERY;
    world[robot.y][robot.x] = EMPTY;
    _visited[robot.y][robot.x] = true;

    numMoves = numTurns = numScans = 0;
    printMap();
}

void MoveForward() {
    static int score = 0;
    numMoves++;
    drainBattery(BATTERY_MOVE, "Battery depleted!");
    int dx, dy;
    ToVector(robot.dir, dx, dy);
    int nx = robot.x + dx;
    int ny = robot.y + dy;
    if (nx < 0 || nx >= MAP_WIDTH || ny < 0 || ny >= MAP_HEIGHT)
        crash("Robot fell off the map!");

    switch (world[ny][nx]) {
        case WALL:  crash("Robot ran into a wall!");
        default:
            robot.x = nx;
            robot.y = ny;

            if (!_visited[robot.y][robot.x]) {
                score++;
                if ((score % SCORE_REPORT_INTERVAL == 0) && SCORE_REPORT_INTERVAL > 0) {
                    cout << "SCORED " << score << " at " << MAX_BATTERY - robot.battery << " energy used." << endl;
                }
            }
            
            _visited[robot.y][robot.x] = true;
            _seen[robot.y][robot.x] = true;
            break;
    }

    printMap();
}

void TurnLeft() {
    numTurns++;
    drainBattery(BATTERY_TURN, "Battery depleted!");
    robot.dir = Left(robot.dir);
    printMap();
}

void TurnRight() {
    numTurns++;
    drainBattery(BATTERY_TURN, "Battery depleted!");
    robot.dir = Right(robot.dir);
    printMap();
}

bool IsWallAhead() {
    numScans++;
    int cost = BATTERY_QUERY_MIN + rand() % (BATTERY_QUERY_MAX - BATTERY_QUERY_MIN + 1);
    drainBattery(cost, "Battery depleted!");
    
    int dx, dy;
    ToVector(robot.dir, dx, dy);
    int nx = robot.x + dx;
    int ny = robot.y + dy;

    bool result = !inRange(nx, ny) || world[ny][nx] == WALL;
    if (sensorError()) result = !result;
    if (inRange(nx, ny)) {
        _seen[ny][nx] = true;
    }
    return result;
}

void GetPosition(int& x, int& y) {
    x = robot.x;
    y = robot.y;
}
Direction GetDirection() {
    return robot.dir;
}

int GetBattery() {
    return robot.battery;
}
int GetScore() {
    int n = 0;
    for (size_t r = 0; r < _visited.size(); r++)
        for (size_t c = 0; c < _visited[r].size(); c++)
            if (_visited[r][c]) n++;
    return n;
}

void PrintResults() {
    
    printStatus();
}
void PrintStatus() {
    PrintResults();
}

// ----------------------
// Educational
// ----------------------

// randomly turn (without moving) or move forward. 
// if forward is blocked, will guarantee turn
// chanceTurn: 0–100 (% chance to turn)
void ForwardOrLeft(int chanceTurn) {
    int r = rand() % 100;

    // If blocked, must turn
    if (IsWallAhead()) {
        TurnLeft();
        return;
    }

    if (r < chanceTurn) {
        TurnLeft();
    }
    else {
        MoveForward();
    }
}

void ForwardOrRight(int chanceTurn) {
    int r = rand() % 100;

    // If blocked, must turn
    if (IsWallAhead()) {
        TurnRight();
        return;
    }

    if (r < chanceTurn) {
        TurnRight();
    }
    else {
        MoveForward();
    }
}

void RandomSafeMove() {
    // Create all 4 directions
    std::vector<Direction> dirs = { NORTH, EAST, SOUTH, WEST };

    // Shuffle directions randomly
    std::shuffle(dirs.begin(), dirs.end(), rng);

    // Try each direction in random order
    for (Direction d : dirs) {
        // Turn to face that direction
        while (GetDirection() != d) {
            TurnRight();
        }

        // Check if safe
        if (!IsWallAhead()) {
            MoveForward();
            return;
        }
    }
}