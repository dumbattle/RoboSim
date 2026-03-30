// Tests that repeated ScanAhead converges confidence toward the true tile type
#include "robot_api.h"
#include "robot_params.h"
#include "debug_api.h"
#include "test_helpers.h"

static void testConvergence(int wallType, const char* label) {
    cout << "\n-- Convergence: " << label << " --\n";
    Reset(SEED);
    ClearErrors();  // 0% error so scans are always correct

    // Place target tile directly ahead
    int x, y;
    GetPosition(x, y);
    int ax = x, ay = y;
    Translate(ax, ay, GetDirection());
    SetTile(ax, ay, wallType);
    ResetConfidence(ax, ay);

    float confBefore[TILE_TYPE_COUNT];
    GetTileConfidence(ax, ay, confBefore);

    // Scan many times
    for (int i = 0; i < 20 && HasBattery(); i++)
        for (int t = 0; t < WALL_TYPE_COUNT; t++)
            ScanAhead(t);

    float confAfter[TILE_TYPE_COUNT];
    GetTileConfidence(ax, ay, confAfter);

    // The correct slot index: -1 (open) = 0, wall type N = N+1
    int slot = wallType + 1;

    check("Correct type confidence increased",    confAfter[slot] > confBefore[slot]);
    check("Correct type is now most likely",       confAfter[slot] == *max_element(confAfter, confAfter + TILE_TYPE_COUNT));
}

int main() {
    testConvergence(-1, "Open tile");
    for (int t = 0; t < WALL_TYPE_COUNT; t++)
        testConvergence(t, ("Wall type " + to_string(t)).c_str());

    summary();
    return _failed > 0 ? 1 : 0;
}
