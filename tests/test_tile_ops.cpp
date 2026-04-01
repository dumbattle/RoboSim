// Tests SetTile, GetTile, FillRect, ClearTile
#include "robot_api.h"
#include "robot_params.h"
#include "debug_api.h"
#include "test_helpers.h"

int main() {
    cout << "-- Tile Operations --\n";
    Reset(SEED);

    // SetTile / GetTile roundtrip
    SetTile(10, 10, 1);
    check("SetTile/GetTile wall type 1",  GetTile(10, 10) == 1);
    SetTile(10, 10, 2);
    check("SetTile/GetTile wall type 2",  GetTile(10, 10) == 2);
    SetTile(10, 10, 0);
    check("SetTile/GetTile empty (0)",   GetTile(10, 10) == 0);

    // ClearTile
    SetTile(20, 20, 1);
    ClearTile(20, 20);
    check("ClearTile sets tile to 0",    GetTile(20, 20) == 0);

    // Out-of-bounds safety
    SetTile(-1, -1, 0);
    check("SetTile out-of-bounds is safe", GetTile(-1, -1) == -1);

    // FillRect — count tiles before and after
    FillRect(5, 5, 9, 9, 0);
    int before = CountTiles(1);
    FillRect(5, 5, 9, 9, 1);   // 5x5 = 25 tiles
    check("FillRect adds correct tile count", CountTiles(1) == before + 25);

    // Clear the rect and verify
    FillRect(5, 5, 9, 9, 0);
    check("FillRect clear restores tile count", CountTiles(1) == before);

    // CountTiles on empty
    int emptyBefore = CountTiles(0);
    SetTile(30, 30, 1);
    check("CountTiles decreases on SetTile", CountTiles(0) == emptyBefore - 1);

    summary();
    return _failed > 0 ? 1 : 0;
}
