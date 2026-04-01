// Tests GetExpectedInfoGain returns sensible values
#include "robot_api.h"
#include "robot_params.h"
#include "debug_api.h"
#include "test_helpers.h"

int main() {
    cout << "-- Expected Info Gain --\n";
    Reset(SEED);
    ClearErrors();

    int x, y;
    GetPosition(x, y);
    int ax = x, ay = y;
    Translate(ax, ay, GetDirection());
    SetTile(ax, ay, 0);

    // Uncertain tile (uniform prior) with 0% error — should have high gain
    ResetConfidence(ax, ay);
    InfoGain ig = GetExpectedInfoGain(ax, ay, 1);
    check("Uncertain tile: expected > 0",           ig.expected > 0.0f);
    check("Uncertain tile: probPositive in [0,1]",  ig.probPositive >= 0.0f && ig.probPositive <= 1.0f);
    check("Uncertain tile: probNeg + probPos == 1",
          abs(ig.probPositive + ig.probNegative - 1.0f) < 0.001f);

    // Already certain tile — expected gain should be 0
    SetConfidenceCertain(ax, ay, 0);
    ig = GetExpectedInfoGain(ax, ay, 1);
    check("Certain tile: expected == 0", ig.expected == 0.0f);

    // 50% error rates — scan is pure noise, expected gain should be ~0
    ResetConfidence(ax, ay);
    SetErrors(1, 50, 50);
    ig = GetExpectedInfoGain(ax, ay, 1);
    check("50% error: expected near 0", ig.expected < 0.01f);
    ClearErrors();

    // Higher error rate -> less info gain than 0% error
    ResetConfidence(ax, ay);
    InfoGain igClean = GetExpectedInfoGain(ax, ay, 1);
    SetErrors(1, 30, 30);
    InfoGain igNoisy = GetExpectedInfoGain(ax, ay, 1);
    check("Higher error -> less expected gain", igNoisy.expected < igClean.expected);
    ClearErrors();

    // deltaIfPositive and deltaIfNegative are in [-1, 1]
    ResetConfidence(ax, ay);
    ig = GetExpectedInfoGain(ax, ay, 1);
    check("deltaIfPositive in [-1, 1]", ig.deltaIfPositive >= -1.0001f && ig.deltaIfPositive <= 1.0001f);
    check("deltaIfNegative in [-1, 1]", ig.deltaIfNegative >= -1.0001f && ig.deltaIfNegative <= 1.0001f);

    summary();
    return _failed > 0 ? 1 : 0;
}
