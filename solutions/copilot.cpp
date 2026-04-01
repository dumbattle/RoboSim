// solutions/GreedyExplorer.cpp

#include "robot_api.h"
#include "robot_params.h"

#include <vector>
#include <algorithm>

struct TileInfo {
    bool knownWall = false;
    bool isWall    = false;
};

Direction directions[4] = { NORTH, EAST, SOUTH, WEST };

void GetStep(Direction d, int& dx, int& dy) {
    ToVector(d, dx, dy);
}

int main() {
    Reset();

    const int width  = GetMapWidth();
    const int height = GetMapHeight();

    // Local map of known walls to avoid repeatedly colliding with them
    std::vector<TileInfo> grid(width * height);

    auto idx = [width](int x, int y) {
        return y * width + x;
    };

    while (HasBattery()) {
        int x, y;
        GetPosition(x, y);
        Direction curDir = GetDirection();

        // Choose best neighbor direction
        bool foundCandidate = false;
        Direction bestDir   = curDir;
        int bestScore       = -1;

        for (Direction d : directions) {
            int dx, dy;
            GetStep(d, dx, dy);
            int nx = x + dx;
            int ny = y + dy;

            if (!InRange(nx, ny)) continue;

            TileInfo& t = grid[idx(nx, ny)];
            if (t.knownWall && t.isWall) continue; // don't walk into known walls

            int score = 0;
            if (!TileVisited(nx, ny)) {
                // Strongly prefer new tiles
                score += 100;
            } else {
                // Mild preference for closer revisits over being stuck
                score += 10;
            }

            // Small bias toward not turning if scores tie
            if (d == curDir) score += 1;

            if (!foundCandidate || score > bestScore) {
                foundCandidate = true;
                bestScore = score;
                bestDir = d;
            }
        }

        if (!foundCandidate) {
            // Surrounded by walls or out of bounds — nothing useful to do
            break;
        }

        // Face chosen direction
        if (bestDir != curDir) {
            TurnToDirection(bestDir);
            if (!HasBattery()) break;
        }

        // Compute ahead tile before moving (for wall marking on collision)
        int dx, dy;
        GetStep(bestDir, dx, dy);
        int ax = x + dx;
        int ay = y + dy;

        // Attempt to move
        int result = MoveForward();
        if (!HasBattery()) break;

        if (result > 0) {
            // Hit a wall: mark it so we avoid it in future
            if (InRange(ax, ay)) {
                TileInfo& t = grid[idx(ax, ay)];
                t.knownWall = true;
                t.isWall    = true;
            }
            // Slight change of heading to escape dead-ends over time
            TurnRight();
        } else if (result == 0) {
            // Successful move: tile visitation is tracked by the simulator
            // Optionally, we could mark non-wall, but confidence is already 100% internally
            if (!HasBattery()) break;
        } else {
            // result == -1: out of bounds or battery just died
            break;
        }
    }

    PrintResults();
    return 0;
}
