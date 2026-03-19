#include "robot_api.h"
#include "robot_params.h"

#include <cstring>  // memset

// ───────────────────────────────────────────────
// Grid & state
// ───────────────────────────────────────────────
bool visited[MAP_HEIGHT][MAP_WIDTH];
int curX = 0, curY = 0;
Direction curDir = NORTH;

bool hasSeenObstacle = false;   // renamed for clarity

int tilesVisited = 0;

void markVisited(int x, int y) {
    if (x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT) return;
    if (!visited[y][x]) {
        visited[y][x] = true;
        tilesVisited++;
    }
}

void updatePose() {
    GetPosition(curX, curY);
    curDir = GetDirection();
    markVisited(curX, curY);
}

bool canMoveForward() {
    if (IsWallAhead() || IsCliffAhead()) return false;

    int nx = curX, ny = curY;
    Translate(nx, ny, curDir);   // use API helper to compute next cell

    // Treat visited cells as "soft wall" only after we've seen real obstacle
    if (hasSeenObstacle && visited[ny][nx]) {
        return false;
    }

    return true;
}

bool tryForward() {
    if (canMoveForward()) {
        MoveForward();
        updatePose();
        return true;
    }
    return false;
}

bool tryRight() {
    TurnRight();
    if (canMoveForward()) {
        MoveForward();
        updatePose();
        return true;
    }
    TurnLeft(); // undo
    return false;
}

bool tryLeft() {
    TurnLeft();
    if (canMoveForward()) {
        MoveForward();
        updatePose();
        return true;
    }
    TurnRight(); // undo
    return false;
}

bool isBatteryOk() {
    return GetBattery() > 400;   // conservative — adjust as needed
}

int main() {
    Reset();  // deterministic

    std::memset(visited, 0, sizeof(visited));
    updatePose();
    markVisited(curX, curY);

    while (isBatteryOk()) {
        // ───────────────────────────────
        // Phase 1: Go straight until obstacle
        // ───────────────────────────────
        if (!hasSeenObstacle) {
            if (tryForward()) {
                continue;
            }

            // Hit real obstacle → turn LEFT so wall is now on RIGHT
            TurnLeft();
            hasSeenObstacle = true;

            // Now fall through to wall-following logic
        }

        // ───────────────────────────────
        // Phase 2: Right-hand wall follow
        // with visited cells treated as soft walls
        // Priority: right > forward > left > backtrack
        // ───────────────────────────────
        if (tryRight())  continue;
        if (tryForward()) continue;
        if (tryLeft())   continue;

        // No unvisited direction → backtrack (turn 180°)
        TurnLeft();
        TurnLeft();

        if (!tryForward()) {
            // Cannot even backtrack → dead-end or whole map explored
            // In small maps this often means we're done
            break;
        }
    }

    PrintResults();
    return 0;
}