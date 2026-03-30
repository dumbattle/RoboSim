#include "robot_api.h"
#include "robot_params.h"
#include "robot.h"
#include "world.h"
#include "display.h"
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <algorithm>

using namespace std;

// ----------------------
// Internal helpers
// ----------------------

static void UpdateErrors(ErrorData& data, int wallID) { 
    if (data.targetRate != data.currentRate) {
        data.timer -= 1;

        double t = 1.0 * data.timer / WALL_DATA[wallID].transitionTime;

        data.currentRate = (int)(data.targetRate * (1 - t) + data.startRate * (t));

        if (data.timer <= 0) { 
            data.currentRate = data.targetRate;
            int min = WALL_DATA[wallID].restPeriod[0];
            int max = WALL_DATA[wallID].restPeriod[1];
            data.timer = (rand() % (max - min + 1)) + min;
        }
    }
    else {
        data.timer -= 1;
        if (data.timer <= 0) {
            data.timer = WALL_DATA[wallID].transitionTime;

            auto range = WALL_DATA[wallID].errorRanges[rand() % WALL_DATA[wallID].errorRanges.size()];
            data.targetRate = range[0] + rand() % (range[1] - range[0] + 1);
            data.startRate = data.currentRate;
        }
    }
}


static void Tick() {
    for(int i = 0; i < WALL_TYPE_COUNT; i++) {
        auto& errors = sensorErrors[i];
        UpdateErrors(errors.falsePositive, i);
        UpdateErrors(errors.falseNegative, i);
    }
}

static void crash(const string& msg) {
    cout << "\n*** EVENT: " << msg << " ***\n";
    robot.battery = 0;
}

// Returns false and drains battery to 0 if insufficient; returns true and deducts cost otherwise.
static bool drainBattery(int cost) {
    if (robot.battery <= 0 || robot.battery < cost) {
        robot.battery = 0;
        return false;
    }
    robot.battery -= cost;
    return true;
}

// ----------------------
// Sensor API
// ----------------------

void GetErrorRates(int obstacleIndex, int& falsePositive, int& falseNegative) {
    auto& errors = sensorErrors[obstacleIndex];
    falsePositive = errors.falsePositive.currentRate;
    falseNegative = errors.falseNegative.currentRate;

    if (falsePositive < 0) falsePositive = 0;
    if (falseNegative < 0) falseNegative = 0;
    if (falsePositive > 50) falsePositive = 50;
    if (falseNegative > 50) falseNegative = 50;
}


void GetErrorDeltas(int obstacleIndex, float& falsePositive, float& falseNegative) {
    auto errors = sensorErrors[obstacleIndex];
    auto& fp = errors.falsePositive;
    auto& fn = errors.falseNegative;

    falsePositive = fp.timer == 0 ? 0 : (fp.targetRate - fp.currentRate) / fp.timer;
    falseNegative = fn.timer == 0 ? 0 : (fn.targetRate - fn.currentRate) / fn.timer;
}



// ----------------------
// Direction API
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
    _confidence.assign(MAP_HEIGHT, vector<vector<float>>(MAP_WIDTH, vector<float>(TILE_TYPE_COUNT, 1.0 /  TILE_TYPE_COUNT)));
    generateWorld(randomSeed);

    pair<int,int> start = randomEmptyTile();
    robot.x       = start.first;
    robot.y       = start.second;
    robot.dir     = EAST;
    robot.battery = MAX_BATTERY;
    world[robot.y][robot.x] = -1;
    _visited[robot.y][robot.x] = true;

    numMoves = numTurns = numScans = 0;



    for(int i = 0; i < 100; i++) {
        Tick();
    }



    printMap();
}

void MoveForward() {
    Tick();

    static int score = 0;
    numMoves++;
    if (!drainBattery(BATTERY_MOVE)) return;
    int dx, dy;
    ToVector(robot.dir, dx, dy);
    int nx = robot.x + dx;
    int ny = robot.y + dy;

    if (nx < 0 || nx >= MAP_WIDTH || ny < 0 || ny >= MAP_HEIGHT) {
        crash("Robot fell off the map!");
        return;
    }

    int wallType = world[ny][nx];

        
    for (int i = 0; i < TILE_TYPE_COUNT; i++) {
        _confidence[ny][nx][i] = 0; 
    }
    _confidence[ny][nx][wallType + 1] = 1;


    if (world[ny][nx] >= 0) {
        // drainBattery(WALL_TYPE_DAMAGE[wallType]);
    }
    else {
        robot.x = nx;
        robot.y = ny;

        if (!_visited[robot.y][robot.x]) {
            _visited[robot.y][robot.x] = true;
            score++;
            if ((score % SCORE_REPORT_INTERVAL == 0) && SCORE_REPORT_INTERVAL > 0) {
                cout << "SCORED " << score << " at " << MAX_BATTERY - robot.battery << " energy used." << endl;
            }
        }
    }
   
    printMap();
}

void TurnLeft() {
    Tick();
    numTurns++;
    if (!drainBattery(BATTERY_TURN)) return;
    robot.dir = Left(robot.dir);
    printMap();
}

void TurnRight() {
    Tick();
    numTurns++;
    if (!drainBattery(BATTERY_TURN)) return;
    robot.dir = Right(robot.dir);
    printMap();
}

void ScanAhead(int obstacleIndex) {
    Tick();

    int dx, dy;
    ToVector(robot.dir, dx, dy);
    int nx = robot.x + dx;
    int ny = robot.y + dy;

    if (!InRange(nx, ny)) return;

    auto& conf = _confidence[ny][nx];

    // Skip: tile already certain (confidence == 1 for any type)
    for (int i = 0; i < TILE_TYPE_COUNT; i++) {
        if (conf[i] >= 1.0f) return;
    }

    // Cost battery
    numScans++;
    int cost = BATTERY_QUERY_MIN + rand() % (BATTERY_QUERY_MAX - BATTERY_QUERY_MIN + 1);
    if (!drainBattery(cost)) return;

    // Get error rates
    int fp, fn;
    GetErrorRates(obstacleIndex, fp, fn);

    // Skip: pure noise — scan carries no information
    if (fp >= 50 || fn >= 50) return;

    float fpRate = fp / 100.0f;
    float fnRate = fn / 100.0f;

    // Apply error to ground truth to get sensor result
    bool isObstacle = (world[ny][nx] == obstacleIndex);
    bool result = isObstacle
        ? (rand() % 100) >= fn   // true unless false negative fires
        : (rand() % 100) < fp;   // false unless false positive fires

    // Bayesian update: multiply each prior by likelihood of this result
    float total = 0.0f;
    for (int i = 0; i < TILE_TYPE_COUNT; i++) {
        bool isTarget = (i == obstacleIndex + 1);
        float likelihood = result
            ? (isTarget ? (1.0f - fnRate) : fpRate)          // sensor said YES
            : (isTarget ? fnRate           : (1.0f - fpRate));// sensor said NO
        conf[i] *= likelihood;
        total += conf[i];
    }

    // Normalize
    if (total > 0.0f) {
        for (int i = 0; i < TILE_TYPE_COUNT; i++)
            conf[i] /= total;
    }

    printMap();
}

float GetTileConfidence(int x, int y, int obstacleIndex) {
    return _confidence[y][x][obstacleIndex];
}

// index 0 => empty tile
// index N => wall type N-1
void GetTileConfidence(int x, int y, float (&results)[TILE_TYPE_COUNT]) {
    for (int i = 0; i < TILE_TYPE_COUNT; i++) {
        results[i] = _confidence[y][x][i];
    }
}

InfoGain GetExpectedInfoGain(int x, int y, int obstacleIndex) {
    const float MAX_ENTROPY = log2f((float)TILE_TYPE_COUNT);
    const float* prior = _confidence[y][x].data();

    // Current entropy
    float hBefore = 0.0f;
    for (int i = 0; i < TILE_TYPE_COUNT; i++)
        if (prior[i] > 0.0f) hBefore -= prior[i] * log2f(prior[i]);

    int fp, fn;
    GetErrorRates(obstacleIndex, fp, fn);
    float fpRate = fp / 100.0f;
    float fnRate = fn / 100.0f;

    // P(scan fires) = P(true positive) + P(false positive)
    float pPos = prior[obstacleIndex + 1] * (1.0f - fnRate)
               + (1.0f - prior[obstacleIndex + 1]) * fpRate;
    float pNeg = 1.0f - pPos;

    // Compute entropy of posterior for a given outcome (true/false)
    auto posteriorEntropy = [&](bool outcome) -> float {
        float post[TILE_TYPE_COUNT];
        float total = 0.0f;
        for (int i = 0; i < TILE_TYPE_COUNT; i++) {
            bool isTarget = (i == obstacleIndex + 1);
            float likelihood = outcome
                ? (isTarget ? (1.0f - fnRate) : fpRate)
                : (isTarget ? fnRate           : (1.0f - fpRate));
            post[i] = prior[i] * likelihood;
            total += post[i];
        }
        float h = 0.0f;
        if (total > 0.0f)
            for (int i = 0; i < TILE_TYPE_COUNT; i++) {
                float p = post[i] / total;
                if (p > 0.0f) h -= p * log2f(p);
            }
        return h;
    };

    float hIfPos = posteriorEntropy(true);
    float hIfNeg = posteriorEntropy(false);

    // Entropy deltas, normalized to [-1, 1]
    float dPos = (hBefore - hIfPos) / MAX_ENTROPY;
    float dNeg = (hBefore - hIfNeg) / MAX_ENTROPY;

    // Expected info gain, normalized to [0, 1]
    float expected = (pPos * dPos + pNeg * dNeg);

    return { expected, pPos, dPos, pNeg, dNeg };
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

bool HasBattery() {
    return robot.battery > 0;
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

