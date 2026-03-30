// Tests wall collision: robot stays put, battery drained by WALL_DATA damage
#include "robot_api.h"
#include "robot_params.h"
#include "debug_api.h"
#include "test_helpers.h"

static void testWallType(int wallType) {
    cout << "\n-- Wall Type " << wallType << " (damage=" << WALL_DATA[wallType].damage << ") --\n";
    Reset(SEED);

    // Place the chosen wall type directly ahead
    int x, y;
    GetPosition(x, y);
    int wx = x, wy = y;
    Translate(wx, wy, GetDirection());
    SetTile(wx, wy, wallType);

    int b = GetBattery();
    MoveForward();

    int x2, y2;
    GetPosition(x2, y2);

    check("Robot does not move into wall", x == x2 && y == y2);
    check("Battery drained by MOVE + WALL_DATA damage",
          GetBattery() == b - BATTERY_MOVE - WALL_DATA[wallType].damage);
}

int main() {
    for (int t = 0; t < WALL_TYPE_COUNT; t++)
        testWallType(t);

    summary();
    return _failed > 0 ? 1 : 0;
}
