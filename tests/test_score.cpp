// Tests score: increments on new tile, no increment on revisit
#include "robot_api.h"
#include "robot_params.h"
#include "debug_api.h"
#include "test_helpers.h"

int main() {
    cout << "-- Score Counting --\n";
    Reset(SEED);

    // Ensure tile ahead is open
    int x, y;
    GetPosition(x, y);
    int ax = x, ay = y;
    Translate(ax, ay, GetDirection());
    SetTile(ax, ay, -1);

    int s = GetScore();
    MoveForward();
    check("Score increments on new tile", GetScore() == s + 1);

    // Walk back and return — should not increment again
    TurnToDirection(Right(Right(GetDirection())));
    SetTile(x, y, -1);
    MoveForward();
    TurnToDirection(Right(Right(GetDirection())));
    MoveForward();

    check("Score does not increment on revisit", GetScore() == s + 1);

    // TileVisited reflects score
    check("TileVisited true after move", TileVisited(ax, ay));
    check("TileVisited false on unvisited tile", !TileVisited(ax + 5, ay + 5));

    summary();
    return _failed > 0 ? 1 : 0;
}
