// Tests SetRobotPosition, SetRobotDirection, SetBattery
#include "robot_api.h"
#include "robot_params.h"
#include "debug_api.h"
#include "test_helpers.h"

int main() {
    cout << "-- Robot State --\n";
    Reset(SEED);

    // SetRobotPosition
    SetRobotPosition(50, 50);
    int x, y;
    GetPosition(x, y);
    check("SetRobotPosition sets X", x == 50);
    check("SetRobotPosition sets Y", y == 50);

    // Out-of-bounds teleport should be ignored
    SetRobotPosition(-1, -1);
    GetPosition(x, y);
    check("SetRobotPosition OOB ignored", x == 50 && y == 50);

    // SetRobotDirection
    SetRobotDirection(NORTH);
    check("SetRobotDirection NORTH", GetDirection() == NORTH);
    SetRobotDirection(WEST);
    check("SetRobotDirection WEST",  GetDirection() == WEST);

    // SetBattery
    SetBattery(1000);
    check("SetBattery sets exact value", GetBattery() == 1000);
    SetBattery(0);
    check("SetBattery to 0 disables HasBattery", !HasBattery());
    SetBattery(-50);
    check("SetBattery negative clamps to 0", GetBattery() == 0);

    // PeekAhead
    SetRobotPosition(40, 40);
    SetRobotDirection(EAST);
    SetTile(41, 40, 1);
    check("PeekAhead detects wall type 1 ahead", PeekAhead(1));
    check("PeekAhead returns false for wrong type", !PeekAhead(0));
    ClearTile(41, 40);
    check("PeekAhead false after ClearTile", !PeekAhead(1));

    summary();
    return _failed > 0 ? 1 : 0;
}
