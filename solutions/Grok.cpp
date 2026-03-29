#include "robot_api.h"
#include "robot_params.h"

#include <cstring>
#include <stack>

// ───────────────────────────────────────────────
// Grid & state
// ───────────────────────────────────────────────
bool visited[MAP_HEIGHT][MAP_WIDTH];
bool blocked[MAP_HEIGHT][MAP_WIDTH];
int curX = 0, curY = 0;
Direction curDir = NORTH;

void updatePose() {
    GetPosition(curX, curY);
    curDir = GetDirection();
    visited[curY][curX] = true;
}

bool inBounds(int x, int y) {
    return x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT;
}

void faceDirection(Direction target) {
    int diff = (target - curDir + 4) % 4;
    if (diff == 1) TurnRight();
    else if (diff == 2) { TurnRight(); TurnRight(); }
    else if (diff == 3) TurnLeft();
    curDir = target;
}

bool moveDir(Direction d) {
    int nx = curX, ny = curY;
    Translate(nx, ny, d);
    if (!inBounds(nx, ny)) return false;
    if (blocked[ny][nx]) return false;

    faceDirection(d);

    if (!visited[ny][nx]) {
        if (IsWallAhead()) {
            blocked[ny][nx] = true;
            return false;
        }
    }

    MoveForward();
    updatePose();
    return true;
}

struct Pos { int x, y; };
std::stack<Pos> path;

int main() {
    Reset();

    std::memset(visited, 0, sizeof(visited));
    std::memset(blocked, 0, sizeof(blocked));
    updatePose();
    path.push({curX, curY});

    while (GetBattery() > 50) {
        // Right-hand wall following: right > forward > left
        Direction prio[3];
        prio[0] = Right(curDir);
        prio[1] = curDir;
        prio[2] = Left(curDir);

        bool moved = false;
        for (int i = 0; i < 3; i++) {
            int nx = curX, ny = curY;
            Translate(nx, ny, prio[i]);
            if (!inBounds(nx, ny)) continue;
            if (blocked[ny][nx]) continue;
            if (visited[ny][nx]) continue;

            if (moveDir(prio[i])) {
                path.push({curX, curY});
                moved = true;
                break;
            }
        }

        if (!moved) {
            // Backtrack
            path.pop();
            if (path.empty()) break;

            Pos back = path.top();
            int dx = back.x - curX;
            int dy = back.y - curY;

            Direction backDir;
            if (dx == 1) backDir = EAST;
            else if (dx == -1) backDir = WEST;
            else if (dy == 1) backDir = NORTH;
            else backDir = SOUTH;

            faceDirection(backDir);
            MoveForward();
            updatePose();
        }
    }

    PrintResults();
    return 0;
}