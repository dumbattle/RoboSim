#include "robot_api.h"
#include "robot_params.h"
#include "robot.h"
#include "world.h"
#include "display.h"
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <queue>

using namespace std;

// ----------------------
// Chunk reveal system
// ----------------------

struct Chunk {
    vector<pair<int,int>> tiles;
    int uncertainBorderCount;
    bool revealed;
};

static vector<Chunk>            chunks;
static vector<vector<int>>      tileChunkID;   // -1 = reachable (not in any chunk)
       vector<vector<bool>>     isBorderTile;  // wall tile adjacent to a reachable empty tile
static vector<vector<bool>>     borderCertain; // border tile already counted as certain

static void revealChunk(int id) {
    Chunk& chunk = chunks[id];
    if (chunk.revealed) return;
    chunk.revealed = true;
    for (auto [x, y] : chunk.tiles) {
        auto& conf = _confidence[y][x];

        if (conf[world[y][x] + 1] < CONFIDENCE_COMPLETION_THRESH) {
            const float HIGHLIGHT_STRENGTH = 1;
            for (int i = 0; i < TILE_TYPE_COUNT; i++) conf[i] = (1 - HIGHLIGHT_STRENGTH) / TILE_TYPE_COUNT;
            conf[world[y][x] + 1] = HIGHLIGHT_STRENGTH; // +1: index 0 = empty, index N = wall type N-1
        }
    }
}

static void checkChunkReveal(int x, int y) {
    int id = tileChunkID[y][x];
    if (id == -1 || !isBorderTile[y][x]) return;
    if (borderCertain[y][x]) return;
    Chunk& chunk = chunks[id];
    if (chunk.revealed) return;

    borderCertain[y][x] = true;
    if (--chunk.uncertainBorderCount <= 0)
        revealChunk(id);
}

static void buildChunks() {
    tileChunkID.assign(MAP_HEIGHT, vector<int>(MAP_WIDTH, -1));
    isBorderTile.assign(MAP_HEIGHT, vector<bool>(MAP_WIDTH, false));
    borderCertain.assign(MAP_HEIGHT, vector<bool>(MAP_WIDTH, false));
    chunks.clear();

    const int dx[4] = {1, -1, 0,  0};
    const int dy[4] = {0,  0, 1, -1};

    for (int sy = 0; sy < MAP_HEIGHT; sy++)
    for (int sx = 0; sx < MAP_WIDTH;  sx++) {
        if (reachable[sy][sx] || tileChunkID[sy][sx] != -1) continue;

        int id = (int)chunks.size();
        chunks.push_back({{}, 0, false});
        Chunk& chunk = chunks.back();

        queue<pair<int,int>> q;
        q.push({sx, sy});
        tileChunkID[sy][sx] = id;

        while (!q.empty()) {
            auto [x, y] = q.front(); q.pop();
            chunk.tiles.push_back({x, y});

            // Border: wall tile with at least one reachable empty neighbor
            if (world[y][x] >= 0) {
                for (int i = 0; i < 4; i++) {
                    int nx = x + dx[i], ny = y + dy[i];
                    if (InRange(nx, ny) && reachable[ny][nx]) {
                        isBorderTile[y][x] = true;
                        chunk.uncertainBorderCount++;
                        break;
                    }
                }
            }

            for (int i = 0; i < 4; i++) {
                int nx = x + dx[i], ny = y + dy[i];
                if (InRange(nx, ny) && !reachable[ny][nx] && tileChunkID[ny][nx] == -1) {
                    tileChunkID[ny][nx] = id;
                    q.push({nx, ny});
                }
            }
        }

      
    }
}

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

void GetErrorRates(int wallID, int& falsePositive, int& falseNegative) {
    auto& errors = sensorErrors[wallID - 1];
    falsePositive = errors.falsePositive.currentRate;
    falseNegative = errors.falseNegative.currentRate;

    if (falsePositive < 0) falsePositive = 0;
    if (falseNegative < 0) falseNegative = 0;
    if (falsePositive > 50) falsePositive = 50;
    if (falseNegative > 50) falseNegative = 50;
}


void GetErrorDeltas(int wallID, float& falsePositive, float& falseNegative) {
    auto errors = sensorErrors[wallID - 1];
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
    buildChunks();
    robot.x       = start.first;
    robot.y       = start.second;
    robot.dir     = EAST;
    robot.battery = MAX_BATTERY;
    world[robot.y][robot.x] = -1;
    _visited[robot.y][robot.x] = true;

    numMoves = numTurns = numScans = 0;



    for(int i = 0; i < 1000; i++) {
        Tick();
    }



    printMap();
}

int MoveForward() {
    Tick();

    static int score = 0;
    numMoves++;
    if (!drainBattery(BATTERY_MOVE)) return -1;
    int dx, dy;
    ToVector(robot.dir, dx, dy);
    int nx = robot.x + dx;
    int ny = robot.y + dy;

    if (nx < 0 || nx >= MAP_WIDTH || ny < 0 || ny >= MAP_HEIGHT) {
        crash("Robot fell off the map!");
        return -1;
    }

    int wallType = world[ny][nx];

        
    for (int i = 0; i < TILE_TYPE_COUNT; i++) {
        _confidence[ny][nx][i] = 0;
    }
    _confidence[ny][nx][wallType + 1] = 1;
    checkChunkReveal(nx, ny);

    if (world[ny][nx] >= 0) {
        drainBattery(WALL_DATA[wallType].damage);
        numCrashes[world[ny][nx]] += 1;
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
    return wallType + 1;
}

void TurnLeft() {
    numTurns++;
    if (!drainBattery(BATTERY_TURN)) return;
    robot.dir = Left(robot.dir);
    printMap();
}

void TurnRight() {
    numTurns++;
    if (!drainBattery(BATTERY_TURN)) return;
    robot.dir = Right(robot.dir);
    printMap();
}

void TurnToDirection(Direction d) {
    if (robot.dir == d) {
        return;
    }

    if (Right(robot.dir) == d) {
        TurnRight();
    }
    else if(Left(robot.dir) == d)  {
        TurnLeft();
    }
    else {
        TurnRight();
        TurnRight();
    }
}



void ScanAhead(int wallID) {
    int dx, dy;
    ToVector(robot.dir, dx, dy);
    int nx = robot.x + dx;
    int ny = robot.y + dy;

    if (!InRange(nx, ny)) return;

    auto& conf = _confidence[ny][nx];

    // Skip: tile already certain (confidence == 1 for any type)
    for (int i = 0; i < TILE_TYPE_COUNT; i++) {
        if (conf[i] >= CONFIDENCE_COMPLETION_THRESH) return;
    }

    // Cost battery
    numScans++;
    int cost = BATTERY_QUERY_MIN + rand() % (BATTERY_QUERY_MAX - BATTERY_QUERY_MIN + 1);
    if (!drainBattery(cost)) return;

    // Get error rates
    int fp, fn;
    GetErrorRates(wallID, fp, fn);

    // Skip: pure noise — scan carries no information
    if (fp >= 50 && fn >= 50) return;

    float fpRate = fp / 100.0f;
    float fnRate = fn / 100.0f;

    // Apply error to ground truth to get sensor result
    bool isObstacle = (world[ny][nx] == wallID - 1);
    bool result = isObstacle
        ? (rand() % 100) >= fn   // true unless false negative fires
        : (rand() % 100) < fp;   // false unless false positive fires

    // Bayesian update: multiply each prior by likelihood of this result
    float total = 0.0f;
    for (int i = 0; i < TILE_TYPE_COUNT; i++) {
        bool isTarget = (i == wallID);
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

    // If close enough to truth, snap to certain and check for chunk reveal
    if (conf[world[ny][nx] + 1] > CONFIDENCE_COMPLETION_THRESH) {
        for (int i = 0; i < TILE_TYPE_COUNT; i++) conf[i] = 0;
        conf[world[ny][nx] + 1] = 1;
        checkChunkReveal(nx, ny);
    }
    printMap();
}

float GetEntropy(int x, int y) {
    static const float MAX_ENTROPY = log2f((float)TILE_TYPE_COUNT);
    const auto& conf = _confidence[y][x];
    float h = 0.0f;
    for (int i = 0; i < TILE_TYPE_COUNT; i++)
        if (conf[i] > 0.0f) h -= conf[i] * log2f(conf[i]);
    return h / MAX_ENTROPY;
}

float GetTileConfidence(int x, int y, int wallID) {
    return _confidence[y][x][wallID];
}

// index 0 => empty tile
// index N => wall type N-1
void GetTileConfidence(int x, int y, float (&results)[TILE_TYPE_COUNT]) {
    for (int i = 0; i < TILE_TYPE_COUNT; i++) {
        results[i] = _confidence[y][x][i];
    }
}

InfoGain GetExpectedInfoGain(int x, int y, int wallID) {
    const float MAX_ENTROPY = log2f((float)TILE_TYPE_COUNT);
    const float* prior = _confidence[y][x].data();

    // Current entropy
    float hBefore = 0.0f;
    for (int i = 0; i < TILE_TYPE_COUNT; i++)
        if (prior[i] > 0.0f) hBefore -= prior[i] * log2f(prior[i]);

    int fp, fn;
    GetErrorRates(wallID, fp, fn);
    float fpRate = fp / 100.0f;
    float fnRate = fn / 100.0f;

    // P(scan fires) = P(true positive) + P(false positive)
    float pPos = prior[wallID] * (1.0f - fnRate)
               + (1.0f - prior[wallID]) * fpRate;
    float pNeg = 1.0f - pPos;

    // Compute entropy of posterior for a given outcome (true/false)
    auto posteriorEntropy = [&](bool outcome) -> float {
        float post[TILE_TYPE_COUNT];
        float total = 0.0f;
        for (int i = 0; i < TILE_TYPE_COUNT; i++) {
            bool isTarget = (i == wallID);
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

bool TileVisited(int x, int y) {
    return _visited[y][x];
}

int GetObstacleDamage(int wallID) {
    return WALL_DATA[wallID - 1].damage;
}

int GetMapWidth()  { return MAP_WIDTH;  }
int GetMapHeight() { return MAP_HEIGHT; }


void PrintResults() {
    printStatus();
}

