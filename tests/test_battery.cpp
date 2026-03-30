// Tests exact battery deductions for every action type
#include "robot_api.h"
#include "robot_params.h"
#include "debug_api.h"
#include "test_helpers.h"

int main() {
    cout << "-- Battery Costs --\n";
    Reset(SEED);

    // MoveForward
    int b = GetBattery();
    MoveForward();
    check("MoveForward costs BATTERY_MOVE", GetBattery() == b - BATTERY_MOVE);

    // TurnLeft
    b = GetBattery();
    TurnLeft();
    check("TurnLeft costs BATTERY_TURN", GetBattery() == b - BATTERY_TURN);

    // TurnRight
    b = GetBattery();
    TurnRight();
    check("TurnRight costs BATTERY_TURN", GetBattery() == b - BATTERY_TURN);

    // ScanAhead — should cost within [QUERY_MIN, QUERY_MAX]
    ClearErrors();
    b = GetBattery();
    ScanAhead(0);
    int cost = b - GetBattery();
    check("ScanAhead cost >= BATTERY_QUERY_MIN", cost >= BATTERY_QUERY_MIN);
    check("ScanAhead cost <= BATTERY_QUERY_MAX", cost <= BATTERY_QUERY_MAX);

    // TurnToDirection — should cost 0 if already facing that direction
    b = GetBattery();
    TurnToDirection(GetDirection());
    check("TurnToDirection same dir costs nothing", GetBattery() == b);

    // TurnToDirection — 180 should cost 2x BATTERY_TURN
    b = GetBattery();
    Direction opposite = Right(Right(GetDirection()));
    TurnToDirection(opposite);
    check("TurnToDirection 180 costs 2x BATTERY_TURN", GetBattery() == b - 2 * BATTERY_TURN);

    // No battery left — actions should no-op
    SetBattery(0);
    b = GetBattery();
    MoveForward();
    TurnLeft();
    TurnRight();
    check("Actions no-op at 0 battery", GetBattery() == 0);

    summary();
    return _failed > 0 ? 1 : 0;
}
