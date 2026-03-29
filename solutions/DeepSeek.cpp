#include "robot_api.h"
#include "robot_params.h"
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <limits>
#include <functional>
#include <iostream>

using namespace std;

struct Cell {
    bool isExplored = false;
    bool isWall = false;
    bool isVisited = false;
    double uncertainty = 1.0;
    int distance = numeric_limits<int>::max();
    bool inOpenSet = false;
};

struct PathNode {
    int x, y, g, h, f;
    PathNode(int x_, int y_, int g_, int h_)
        : x(x_), y(y_), g(g_), h(h_), f(g_ + h_) {}
    bool operator>(const PathNode& other) const { return f > other.f; }
};

struct ExplorationTarget {
    int x, y, distance;
    double infoGain, value;
    double forwardBias;  // bonus for being in the direction robot is already facing

    ExplorationTarget() : x(0), y(0), distance(0), infoGain(0.0), value(0.0), forwardBias(0.0) {}
    ExplorationTarget(int x_, int y_, double gain, int dist, double bias)
        : x(x_), y(y_), infoGain(gain), distance(dist), forwardBias(bias) {
        value = ((dist > 0) ? gain / dist : gain * 100) + bias;
    }
    bool operator<(const ExplorationTarget& other) const { return value < other.value; }
};

class AdaptiveExplorer {
private:
    vector<vector<Cell>> map;
    int width, height;
    int robotX, robotY;
    Direction robotDir;
    int battery;

    int spiralLength   = 1;
    int spiralStep     = 0;
    int spiralDirIndex = 0;

    const double UNCERTAINTY_DECAY   = 0.3;
    const double INFO_GAIN_THRESHOLD = 0.1;
    const int    PLANNING_HORIZON    = 30;

    // FIX: Direction vectors consistent with engine:
    // NORTH(0)=y+1, EAST(1)=x+1, SOUTH(2)=y-1, WEST(3)=x-1
    static constexpr int DX[4] = {0, 1, 0, -1};
    static constexpr int DY[4] = {1, 0, -1, 0};

public:
    AdaptiveExplorer() {
        width  = MAP_WIDTH;
        height = MAP_HEIGHT;
        map.resize(height, vector<Cell>(width));
        for (int y = 0; y < height; y++)
            for (int x = 0; x < width; x++)
                map[y][x].uncertainty = 1.0;
    }

    void updateState() {
        GetPosition(robotX, robotY);
        robotDir = GetDirection();
        battery  = GetBattery();
        if (isInBounds(robotX, robotY)) {
            map[robotY][robotX].isVisited  = true;
            map[robotY][robotX].isExplored = true;
            map[robotY][robotX].uncertainty = 0.0;
        }
    }

    bool isInBounds(int x, int y) {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    // Returns true if tile ahead is a wall. Uses memory first, only
    // queries sensor if the tile is still unexplored.
    bool senseAhead() {
        if (battery <= 0) return true;
        int aheadX = robotX, aheadY = robotY;
        Translate(aheadX, aheadY, robotDir);

        // Out of bounds = wall, no sensor needed
        if (!isInBounds(aheadX, aheadY)) return true;

        // Already explored — use memory, skip sensor
        if (map[aheadY][aheadX].isExplored)
            return map[aheadY][aheadX].isWall;

        // Unknown cell — query sensor
        bool wall = IsWallAhead();
        map[aheadY][aheadX].isExplored  = true;
        map[aheadY][aheadX].isWall       = wall;
        map[aheadY][aheadX].uncertainty  = 0.0;
        propagateUncertainty(aheadX, aheadY);
        return wall;
    }

    void propagateUncertainty(int x, int y) {
        for (int i = 0; i < 4; i++) {
            int nx = x + DX[i], ny = y + DY[i];
            if (isInBounds(nx, ny) && !map[ny][nx].isExplored)
                map[ny][nx].uncertainty = max(UNCERTAINTY_DECAY,
                                              map[ny][nx].uncertainty * 0.8);
        }
    }

    double calculateInfoGain(int x, int y) {
        if (!isInBounds(x, y) || map[y][x].isExplored) return 0.0;
        double gain = map[y][x].uncertainty;
        for (int i = 0; i < 4; i++) {
            int nx = x + DX[i], ny = y + DY[i];
            if (isInBounds(nx, ny) && map[ny][nx].isExplored) { gain *= 1.5; break; }
        }
        int wallCount = 0;
        for (int i = 0; i < 4; i++) {
            int nx = x + DX[i], ny = y + DY[i];
            if (!isInBounds(nx, ny) || map[ny][nx].isWall) wallCount++;
        }
        if (wallCount >= 3) gain *= 2.0;
        return gain;
    }

    vector<vector<int>> computeDistanceMap() {
        vector<vector<int>> dist(height, vector<int>(width, -1));
        queue<pair<int,int>> q;
        dist[robotY][robotX] = 0;
        q.push({robotX, robotY});
        while (!q.empty()) {
            auto [cx, cy] = q.front(); q.pop();
            for (int i = 0; i < 4; i++) {
                int nx = cx + DX[i], ny = cy + DY[i];
                if (!isInBounds(nx, ny)) continue;
                if (dist[ny][nx] != -1) continue;
                if (map[ny][nx].isExplored && map[ny][nx].isWall) continue;
                dist[ny][nx] = dist[cy][cx] + 1;
                q.push({nx, ny});
            }
        }
        return dist;
    }

    vector<ExplorationTarget> findExplorationTargets() {
        vector<ExplorationTarget> targets;

        int fdx = 0, fdy = 0;
        ToVector(robotDir, fdx, fdy);

        auto distMap = computeDistanceMap();

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                if (map[y][x].isExplored || map[y][x].isWall) continue;

                bool onFrontier = false;
                for (int i = 0; i < 4; i++) {
                    int nx = x + DX[i], ny = y + DY[i];
                    if (isInBounds(nx, ny) && map[ny][nx].isExplored && !map[ny][nx].isWall) {
                        onFrontier = true;
                        break;
                    }
                }
                if (!onFrontier) continue;

                double gain = calculateInfoGain(x, y);
                if (gain < INFO_GAIN_THRESHOLD) continue;

                int actualDist = distMap[y][x];
                if (actualDist > 0) {
                    double tdx = x - robotX;
                    double tdy = y - robotY;
                    double dot = tdx * fdx + tdy * fdy;
                    double bias = (actualDist > 0) ? 0.001 * dot / actualDist : 0.0;
                    targets.emplace_back(x, y, gain, actualDist, bias);
                }
            }
        }

        sort(targets.begin(), targets.end(),
            [](const ExplorationTarget& a, const ExplorationTarget& b) {
                return a.value > b.value;
            });

        if (targets.size() > 10) targets.resize(10);
        return targets;
    }
    vector<pair<int,int>> planPath(int targetX, int targetY) {
        vector<pair<int,int>> path;
        if (targetX == robotX && targetY == robotY) return path;

        vector<vector<pair<int,int>>> parent(width, vector<pair<int,int>>(height, {-1,-1}));
        vector<vector<int>> gScore(width, vector<int>(height, numeric_limits<int>::max()));
        vector<vector<bool>> closed(width, vector<bool>(height, false));

        priority_queue<PathNode, vector<PathNode>, greater<PathNode>> open;

        auto heuristic = [&](int x, int y) {
            return abs(x - targetX) + abs(y - targetY);
        };

        gScore[robotX][robotY] = 0;
        open.emplace(robotX, robotY, 0, heuristic(robotX, robotY));

        while (!open.empty()) {
            PathNode cur = open.top(); open.pop();

            if (closed[cur.x][cur.y]) continue;
            closed[cur.x][cur.y] = true;

            if (cur.x == targetX && cur.y == targetY) {
                int x = targetX, y = targetY;
                while (!(x == robotX && y == robotY)) {
                    path.emplace_back(x, y);
                    auto [px, py] = parent[x][y];
                    if (px == -1 || py == -1) { path.clear(); return path; }
                    x = px; y = py;
                }
                reverse(path.begin(), path.end());
                return path;
            }

            for (int i = 0; i < 4; i++) {
                int nx = cur.x + DX[i];
                int ny = cur.y + DY[i];

                if (!isInBounds(nx, ny)) continue;
                if (closed[nx][ny]) continue;
                if (map[ny][nx].isExplored && map[ny][nx].isWall) continue;

                int tentG = gScore[cur.x][cur.y] + 1;
                if (tentG < gScore[nx][ny]) {
                    parent[nx][ny] = {cur.x, cur.y};
                    gScore[nx][ny] = tentG;
                    open.emplace(nx, ny, tentG, heuristic(nx, ny));
                }
            }
        }

        return path;
    }

    void turnToward(Direction targetDir) {
        while (robotDir != targetDir && battery > 0) {
            int diff = (targetDir - robotDir + 4) % 4;
            if (diff <= 2) TurnRight();
            else           TurnLeft();
            updateState();
        }
    }

    void moveAlongPath(const vector<pair<int,int>>& path) {
        for (const auto& [nextX, nextY] : path) {
            if (battery <= 0) return;

            int dx = nextX - robotX;
            int dy = nextY - robotY;

            // FIX: Direction mapping consistent with engine
            // NORTH = y+1, SOUTH = y-1
            Direction targetDir;
            if      (dx ==  1) targetDir = EAST;
            else if (dx == -1) targetDir = WEST;
            else if (dy ==  1) targetDir = NORTH;
            else if (dy == -1) targetDir = SOUTH;
            else continue;

            turnToward(targetDir);

            if (battery <= 0) return;

            // Sense before moving — uses memory if already known
            bool wallAhead = senseAhead();
            updateState();

            if (wallAhead) {
                if (isInBounds(nextX, nextY)) {
                    map[nextY][nextX].isWall = true;
                    map[nextY][nextX].isExplored = true;
                }
                return; // path blocked; replan
            }

            MoveForward();
            updateState();
            map[robotY][robotX].isVisited = true;
        }
    }

    void performSpiralSearch() {
        if (battery <= 0) return;

        for (int i = 0; i < 4 && battery > 0; i++) {
            Direction targetDir = static_cast<Direction>((spiralDirIndex + i) % 4);
            turnToward(targetDir);

            bool wallAhead = senseAhead();
            updateState();

            if (battery > 0 && !wallAhead) {
                MoveForward();
                updateState();
                if (++spiralStep >= spiralLength) {
                    spiralStep = 0;
                    spiralDirIndex = (spiralDirIndex + 1) % 4;
                    if (spiralDirIndex % 2 == 0) spiralLength++;
                }
                return;
            }
        }

        if (battery > 0) { TurnRight(); updateState(); }
    }

    // Try to step forward without pathfinding. Returns true if moved.
    bool greedyForward() {
        // Check ahead first (cheapest - no turn needed)
        if (!senseAhead()) {
            int ax = robotX, ay = robotY;
            Translate(ax, ay, robotDir);
            if (isInBounds(ax, ay) && !map[ay][ax].isVisited) {
                MoveForward();
                updateState();
                return true;
            }
        }
        return false;
    }

    void run() {
        updateState();

        while (battery > 0) {
            // Greedy: if tile ahead is open and unvisited, just go
            if (greedyForward()) continue;

            // Sense forward only if unknown — free if already explored
            senseAhead();
            updateState();

            auto targets = findExplorationTargets();
            if (targets.empty()) {
                performSpiralSearch();
                continue;
            }

            const auto& best = targets[0];
            auto path = planPath(best.x, best.y);

            if (path.empty()) {
                map[best.y][best.x].isWall     = true;
                map[best.y][best.x].isExplored = true;
                continue;
            }

            moveAlongPath(path);
        }

        PrintResults();
    }
};

int main() {
    Reset();
    AdaptiveExplorer explorer;
    explorer.run();
    return 0;
}