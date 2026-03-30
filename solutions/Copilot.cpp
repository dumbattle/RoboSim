#include "robot_api.h"
#include "robot_params.h"
#include <queue>
#include <map>
#include <set>
#include <vector>

struct Node {
    int x, y;
};

// For BFS
struct BFSState {
    int x, y;
    std::vector<Direction> path;
};

std::set<std::pair<int,int>> visited;
std::set<std::pair<int,int>> knownWalls;

// ----------------------
// Helpers
// ----------------------

bool IsKnownWall(int x, int y) {
    return knownWalls.count({x,y}) > 0;
}

bool IsVisited(int x, int y) {
    return visited.count({x,y}) > 0;
}

bool IsInsideMap(int x, int y) {
    return x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT;
}

void faceDirection(Direction target) {
    int diff = (target - GetDirection() + 4) % 4;
    if (diff == 1) TurnRight();
    else if (diff == 2) { TurnRight(); TurnRight(); }
    else if (diff == 3) TurnLeft();
}

// Check if tile in direction d is free (using sensor + memory)
bool CanMove(Direction d) {
    int x, y;
    GetPosition(x, y);

    int dx, dy;
    ToVector(d, dx, dy);
    int nx = x + dx;
    int ny = y + dy;

    if (!IsInsideMap(nx, ny)) return false;
    if (IsKnownWall(nx, ny))  return false;

    // Only check sensor if needed
    faceDirection(d);
    Direction cur = GetDirection();

    bool wall = IsWallAhead();
    if (wall) {
        knownWalls.insert({nx, ny});
        return false;
    }
    return true;
}


// Move forward safely (record visited)
void SafeForward() {
    int x, y;
    GetPosition(x, y);
    visited.insert({x, y});
    MoveForward();
    GetPosition(x, y);
    visited.insert({x, y});
}

// ----------------------
// BFS to nearest unseen tile
// ----------------------

bool BFS_FindPath(std::vector<Direction>& outPath) {
    int sx, sy;
    GetPosition(sx, sy);

    std::queue<BFSState> q;
    std::set<std::pair<int,int>> seen;

    q.push({sx, sy, {}});
    seen.insert({sx, sy});

    while (!q.empty()) {
        BFSState cur = q.front();
        q.pop();

        // Found an unseen tile
        if (!IsVisited(cur.x, cur.y)) {
            outPath = cur.path;
            return true;
        }

        // Explore neighbors
        for (Direction d : {NORTH, EAST, SOUTH, WEST}) {
            int dx, dy;
            ToVector(d, dx, dy);
            int nx = cur.x + dx;
            int ny = cur.y + dy;

            if (!IsInsideMap(nx, ny)) continue;
            if (IsKnownWall(nx, ny)) continue;
            if (seen.count({nx, ny})) continue;

            // BFS does not use sensors; it trusts memory
            BFSState next = cur;
            next.x = nx;
            next.y = ny;
            next.path.push_back(d);

            q.push(next);
            seen.insert({nx, ny});
        }
    }

    return false;
}

// ----------------------
// Main exploration logic
// ----------------------

void Explore() {
    int x, y;
    GetPosition(x, y);
    visited.insert({x, y});

    while (HasBattery()) {

        // 1) Try right, forward, left
        Direction d = GetDirection();
        Direction right = Right(d);
        Direction left = Left(d);

        struct Option { Direction d; };
        std::vector<Option> options = {
            {right},
            {d},
            {left}
        };

        bool moved = false;

        for (auto& opt : options) {
            int cx, cy;
            GetPosition(cx, cy);

            int dx, dy;
            ToVector(opt.d, dx, dy);
            int nx = cx + dx;
            int ny = cy + dy;

            if (!IsInsideMap(nx, ny)) continue;
            if (IsVisited(nx, ny)) continue;
            if (IsKnownWall(nx, ny)) continue;

            if (CanMove(opt.d)) {
                faceDirection(opt.d);
                SafeForward();
                moved = true;
                break;
            }
        }

        if (moved) continue;

        // 2) BFS to nearest unseen tile
        std::vector<Direction> path;
        if (!BFS_FindPath(path)) {
            // 3) No unseen tiles remain
            break;
        }

        // Option B: Move only one step toward BFS target
        if (!path.empty()) {
            Direction step = path[0];
            if (CanMove(step)) {
                faceDirection(step);
                SafeForward();
            } else {
                // If BFS says it's free but sensor says wall, update memory
                int cx, cy;
                GetPosition(cx, cy);
                int dx, dy;
                ToVector(step, dx, dy);
                knownWalls.insert({cx + dx, cy + dy});
            }
        }
    }
}

int main() {
    Reset(SEED);
    Explore();
    PrintResults();
    return 0;
}
