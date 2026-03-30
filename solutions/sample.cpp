#include "robot_api.h"
#include <iostream>
using namespace std;

// Threshold for deciding if a tile is safe enough
const float SAFE_THRESHOLD = 0.2f;

// Check probability that tile is empty
float GetEmptyConfidence(int x, int y) {
    return GetTileConfidence(x, y, 0); // index 0 = empty
}

// Try scanning all obstacle types once
void ScanTile(int x, int y) {
    for (int i = 0; i < WALL_TYPE_COUNT; i++) {
        InfoGain info = GetExpectedInfoGain(x, y, i);

        // Only scan if useful
        if (info.expected > 0.05f) {
            ScanAhead(i);
        }
    }
}

// Try to move forward if safe
bool TryMoveForward() {
    int x, y;
    GetPosition(x, y);

    Direction d = GetDirection();
    int nx = x, ny = y;
    Translate(nx, ny, d);

    if (!InRange(nx, ny)) return false;

    float emptyProb = GetEmptyConfidence(nx, ny);

    if (emptyProb > SAFE_THRESHOLD) {
        MoveForward();
        return true;
    }

    // If uncertain, scan
    ScanTile(nx, ny);

    emptyProb = GetEmptyConfidence(nx, ny);
    if (emptyProb > SAFE_THRESHOLD) {
        MoveForward();
        return true;
    }

    return false;
}

// Turn toward an unvisited or least-known tile
void ExploreTurn() {
    int x, y;
    GetPosition(x, y);

    Direction bestDir = GetDirection();
    float bestScore = -1.0f;

    for (int i = 0; i < 4; i++) {
        Direction d = (Direction)i;

        int nx = x, ny = y;
        Translate(nx, ny, d);

        if (!InRange(nx, ny)) continue;

        float score = 0.0f;

        // Prefer unvisited tiles
        if (!TileVisited(nx, ny)) score += 1.0f;

        // Prefer tiles likely empty
        score += GetEmptyConfidence(nx, ny);

        if (score > bestScore) {
            bestScore = score;
            bestDir = d;
        }
    }

    TurnToDirection(bestDir);
}

int main() {
    Reset();

    while (HasBattery()) {
        if (!TryMoveForward()) {
            ExploreTurn();
        }
    }

    PrintResults();
    return 0;
}