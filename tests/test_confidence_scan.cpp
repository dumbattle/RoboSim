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
            ScanAhead(t + 1);

    float confAfter[TILE_TYPE_COUNT];
    GetTileConfidence(ax, ay, confAfter);

    int slot = wallType;

    check("Correct type confidence increased",    confAfter[slot] > confBefore[slot]);
    check("Correct type is now most likely",       confAfter[slot] == *max_element(confAfter, confAfter + TILE_TYPE_COUNT));
}

int main() {
    testConvergence(0, "Open tile");
    for (int t = 0; t < WALL_TYPE_COUNT; t++)
        testConvergence(t + 1, ("Wall type " + to_string(t)).c_str());

    summary();
    return _failed > 0 ? 1 : 0;
}
