// Tests SetErrors, ClearErrors, GetErrorRates, and clamping
#include "robot_api.h"
#include "robot_params.h"
#include "debug_api.h"
#include "test_helpers.h"

int main() {
    cout << "-- Error Rates --\n";
    Reset(SEED);

    // SetErrors and readback
    SetErrors(1, 20, 35);
    int fp, fn;
    GetErrorRates(1, fp, fn);
    check("SetErrors: fp reads back correctly", fp == 20);
    check("SetErrors: fn reads back correctly", fn == 35);

    // SetErrors on second obstacle type
    SetErrors(2, 10, 5);
    GetErrorRates(2, fp, fn);
    check("SetErrors type 2: fp correct", fp == 10);
    check("SetErrors type 2: fn correct", fn == 5);

    // Clamping: values above 50 should clamp to 50
    SetErrors(1, 99, 99);
    GetErrorRates(1, fp, fn);
    check("SetErrors clamps fp to 50", fp == 50);
    check("SetErrors clamps fn to 50", fn == 50);

    // Clamping: negative values clamp to 0
    SetErrors(1, -10, -5);
    GetErrorRates(1, fp, fn);
    check("SetErrors clamps negative fp to 0", fp == 0);
    check("SetErrors clamps negative fn to 0", fn == 0);

    // ClearErrors sets all to 0
    SetErrors(1, 30, 30);
    SetErrors(2, 25, 25);
    ClearErrors();
    for (int i = 1; i < WALL_TYPE_COUNT + 1; i++) {
        GetErrorRates(i, fp, fn);
        check("ClearErrors fp == 0 for type " + to_string(i), fp == 0);
        check("ClearErrors fn == 0 for type " + to_string(i), fn == 0);
    }

    // Rates stay locked (no drift) after SetErrors
    SetErrors(1, 15, 15);
    for (int i = 0; i < 100; i++) TurnLeft();  // trigger many Ticks
    GetErrorRates(1, fp, fn);
    check("SetErrors locks rate — no drift after 100 ticks", fp == 15 && fn == 15);

    summary();
    return _failed > 0 ? 1 : 0;
}
