#include "robot_api.h"
#include "robot_params.h"
#include <vector>
#include <queue>
#include <cmath>
#include <limits>

struct Node {
    int x, y;
};

int dx[4] = {0, 1, 0, -1};
int dy[4] = {1, 0, -1, 0};

Direction dirs[4] = {NORTH, EAST, SOUTH, WEST};


// --- Utility ---
Direction DirFromIndex(int i) {
    return dirs[i];
}

int DirIndex(Direction d) {
    for (int i = 0; i < 4; i++) if (dirs[i] == d) return i;
    return 0;
}


// --- Heuristic: how "good" is moving into tile ---
float EvaluateTile(int x, int y) {
    if (!InRange(x, y)) return -1e9;

    float entropy = GetEntropy(x, y);

    float wallProb = 0.0f;
    for (int i = 1; i <= WALL_TYPE_COUNT; i++) {
        wallProb += GetTileConfidence(x, y, i);
    }

    // Penalize walls
    float riskPenalty = wallProb * 2.0f;

    // Reward uncertainty (exploration)
    return entropy - riskPenalty;
}


// --- Decide whether to scan ---
bool ShouldScan(int x, int y) {
    float entropy = GetEntropy(x, y);

    if (entropy < 0.2f) return false; // already confident

    float bestGain = 0.0f;

    for (int i = 1; i <= WALL_TYPE_COUNT; i++) {
        InfoGain ig = GetExpectedInfoGain(x, y, i);
        bestGain = std::max(bestGain, ig.expected);
    }

    return bestGain > 0.05f;
}


// --- Perform best scan ---
void DoBestScan(int x, int y) {
    float bestGain = 0.0f;
    int bestType = 1;

    for (int i = 1; i <= WALL_TYPE_COUNT; i++) {
        InfoGain ig = GetExpectedInfoGain(x, y, i);
        if (ig.expected > bestGain) {
            bestGain = ig.expected;
            bestType = i;
        }
    }

    ScanAhead(bestType);
}


// --- Choose best direction to move ---
Direction ChooseDirection() {
    int x, y;
    GetPosition(x, y);

    float bestScore = -1e9;
    Direction bestDir = GetDirection();

    for (int i = 0; i < 4; i++) {
        int nx = x + dx[i];
        int ny = y + dy[i];

        float score = EvaluateTile(nx, ny);

        // Prefer unvisited tiles
        if (!TileVisited(nx, ny)) score += 1.0f;

        if (score > bestScore) {
            bestScore = score;
            bestDir = dirs[i];
        }
    }

    return bestDir;
}


// --- Main ---
int main() {
    Reset();

    while (HasBattery()) {
        int x, y;
        GetPosition(x, y);

        Direction dir = GetDirection();

        int fx = x, fy = y;
        Translate(fx, fy, dir);

        // --- Scan if useful ---
        if (InRange(fx, fy) && ShouldScan(fx, fy)) {
            DoBestScan(fx, fy);
        }

        // --- Choose movement ---
        Direction bestDir = ChooseDirection();

        TurnToDirection(bestDir);

        int result = MoveForward();

        // --- If hit wall: adapt ---
        if (result > 0) {
            // learned it's a wall automatically
            // next loop will avoid it via confidence
        }

        if (result == -1) break;
    }

    PrintResults();
    return 0;
}