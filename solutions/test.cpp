#include "robot_api.h"
#include "robot_params.h"
#include <iostream>
#include <string>

using namespace std;

static int passed = 0;
static int failed = 0;

static void check(const string& name, bool condition) {
    if (condition) {
        cout << "  [PASS] " << name << "\n";
        passed++;
    } else {
        cout << "  [FAIL] " << name << "\n";
        failed++;
    }
}

// ----------------------
// Test: Battery costs
// ----------------------
static void testBatteryCosts() {
    cout << "\n-- Battery Costs --\n";
    Reset(SEED);

    int b0 = GetBattery();
    MoveForward();
    check("MoveForward costs BATTERY_MOVE", GetBattery() == b0 - BATTERY_MOVE);

    b0 = GetBattery();
    TurnLeft();
    check("TurnLeft costs BATTERY_TURN", GetBattery() == b0 - BATTERY_TURN);

    b0 = GetBattery();
    TurnRight();
    check("TurnRight costs BATTERY_TURN", GetBattery() == b0 - BATTERY_TURN);

    b0 = GetBattery();
    IsWallAhead(0);
    int cost = b0 - GetBattery();
    check("IsWallAhead cost in [QUERY_MIN, QUERY_MAX]",
          cost >= BATTERY_QUERY_MIN && cost <= BATTERY_QUERY_MAX);
}

// ----------------------
// Test: Wall hit penalty
// ----------------------
static void testWallHit() {
    cout << "\n-- Wall Hit Penalty --\n";
    Reset(SEED);

    // Find a wall in some direction
    Direction dirs[4] = { NORTH, EAST, SOUTH, WEST };
    bool foundWall = false;

    for (int i = 0; i < 4; i++) {
        while (GetDirection() != dirs[i]) TurnRight();

        if (IsWallAhead(0) || IsWallAhead(1)) {
            int wallType = IsWallAhead(1) ? 1 : 0;

            int x0, y0;
            GetPosition(x0, y0);
            int b0 = GetBattery();

            MoveForward();

            int x1, y1;
            GetPosition(x1, y1);

            check("Robot does not move into wall", x0 == x1 && y0 == y1);
            check("Wall hit drains WALL_TYPE_DAMAGE",
                  GetBattery() <= b0 - BATTERY_MOVE - WALL_TYPE_DAMAGE[wallType]);

            foundWall = true;
            break;
        }
    }

    if (!foundWall)
        cout << "  [SKIP] No adjacent wall found at spawn — wall hit test skipped\n";
}

// ----------------------
// Test: Score counting
// ----------------------
static void testScore() {
    cout << "\n-- Score --\n";
    Reset(SEED);

    int s0 = GetScore();
    MoveForward();
    int s1 = GetScore();
    check("Score increments on new tile", s1 == s0 + 1);

    // Turn around and go back to starting tile
    TurnRight(); TurnRight();
    MoveForward();
    int s2 = GetScore();
    check("Score does not increment on revisit", s2 == s1);
}

// ----------------------
// Test: HasBattery
// ----------------------
static void testHasBattery() {
    cout << "\n-- HasBattery --\n";
    Reset(SEED);
    check("HasBattery true at start", HasBattery());

    // Drain battery to 0 via wall hits
    while (HasBattery()) {
        if (!IsWallAhead(0) && !IsWallAhead(1)) {
            MoveForward();
        } else {
            TurnRight();
        }
    }
    check("HasBattery false when depleted", !HasBattery());
}

// ----------------------
// Main
// ----------------------
int main() {
    testBatteryCosts();
    testWallHit();
    testScore();
    testHasBattery();

    cout << "\n============================\n";
    cout << "Results: " << passed << " passed, " << failed << " failed\n";
    cout << "============================\n";

    PrintResults();
    return failed > 0 ? 1 : 0;
}
