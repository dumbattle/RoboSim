// Tests that MoveForward sets the destination tile to 100% confidence
#include "robot_api.h"
#include "robot_params.h"
#include "debug_api.h"
#include "test_helpers.h"

int main() {
    cout << "-- Confidence: Movement --\n";
    Reset(SEED);

    // Place known open tile ahead, move onto it
    int x, y;
    GetPosition(x, y);
    int ax = x, ay = y;
    Translate(ax, ay, GetDirection());
    SetTile(ax, ay, -1);

    // Verify confidence starts non-certain (uniform prior)
    float conf[TILE_TYPE_COUNT];
    GetTileConfidence(ax, ay, conf);
    check("Confidence starts below 1.0 (uniform prior)", conf[0] < 1.0f);

    MoveForward();

    GetTileConfidence(ax, ay, conf);
    check("Open tile: confidence[0] = 1.0 after move", conf[0] == 1.0f);
    check("Open tile: no wall confidence after move",   conf[1] == 0.0f);

    // Place a wall ahead, bump into it
    GetPosition(x, y);
    int wx = x, wy = y;
    Translate(wx, wy, GetDirection());
    SetTile(wx, wy, 0);
    ResetConfidence(wx, wy);

    MoveForward();  // hits wall

    GetTileConfidence(wx, wy, conf);
    check("Wall tile: confidence[1] = 1.0 after collision", conf[1] == 1.0f);
    check("Wall tile: no open confidence after collision",  conf[0] == 0.0f);

    summary();
    return _failed > 0 ? 1 : 0;
}
