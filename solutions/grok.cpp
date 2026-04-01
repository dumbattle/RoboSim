/*
Detailed Behavior (per tick while battery > 0):

Sparse Scanning (≈10% chance per tick)
Checks expected information gain for the tile directly ahead.
Only performs ScanAhead() if the scan is expected to meaningfully reduce uncertainty (expected > 0.1).
Skips scans automatically if the tile is already fully certain or sensor noise is 50%.

Attempt Forward Movement
Calls MoveForward().
Success (returns 0): Robot moves to new tile.
With small probability (~5%), randomly turns left or right to increase exploration diversity.

Collision (returns wallID > 0): Robot stays in place.
Immediately turns right (implements right-hand rule wall following).

Out of bounds / No battery (returns -1): Do nothing.

Confidence-Based Proactive Turning
If the tile ahead has low entropy (< 0.3) and high probability of being a wall (> 70%), turn right early.
This avoids wasting movement battery by bumping into predictable walls.

*/
#include "robot_api.h"
#include "state_machine.h"
#include <cstdlib>  // for rand()
#include <ctime>

// Simple random helper
int RandomInt(int min, int max) {
    return min + (rand() % (max - min + 1));
}

int main() {
    Reset();  // Generate world and place robot

    StateMachine sm;

    // Main "explore" mode
    sm.Add("explore", [&]() {
        if (!HasBattery()) return;

        int x, y;
        GetPosition(x, y);
        Direction dir = GetDirection();

        // Get info about tile ahead
        int ahead_x = x, ahead_y = y;
        Translate(ahead_x, ahead_y, dir);

        bool in_bounds = InRange(ahead_x, ahead_y);

        // Occasionally scan for expected info gain (cheap if tile is certain or noise=50%)
        if (RandomInt(0, 10) == 0) {  // ~10% chance per tick
            // Scan for wall types that might give good info
            for (int wid = 1; wid < TILE_TYPE_COUNT; ++wid) {  // TILE_TYPE_COUNT includes 0?
                InfoGain ig = GetExpectedInfoGain(ahead_x, ahead_y, wid);
                if (ig.expected > 0.1f) {  // Arbitrary threshold - tune this
                    ScanAhead(wid);
                    break;
                }
            }
        }

        // Try to move forward
        int result = MoveForward();

        if (result == -1) {
            // Out of bounds or no battery - nothing to do
            return;
        } else if (result > 0) {
            // Hit a wall (result = wall type)
            // Bounce: turn right (simple right-hand wall following)
            TurnRight();
        } else {
            // Successfully moved to open tile
            // Sometimes explore randomly instead of always going straight
            if (RandomInt(0, 20) == 0) {  // Rare random turn
                if (RandomInt(0, 1) == 0) {
                    TurnLeft();
                } else {
                    TurnRight();
                }
            }
        }

        // If we're in a tight spot or facing a probable wall, turn proactively
        // Use confidence to decide
        float entropy = GetEntropy(ahead_x, ahead_y);
        if (in_bounds && entropy < 0.3f) {  // Fairly certain
            float conf_wall = 0.0f;
            for (int wid = 1; wid < TILE_TYPE_COUNT; ++wid) {
                conf_wall += GetTileConfidence(ahead_x, ahead_y, wid);
            }
            if (conf_wall > 0.7f) {
                TurnRight();  // Likely wall ahead -> turn
            }
        }
    });

    sm.SetMode("explore");

    while (HasBattery()) {
        sm.Tick();
    }

    PrintResults();
    return 0;
}