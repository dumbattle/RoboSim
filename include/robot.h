#pragma once
#include <vector>
#include "robot_api.h"


struct Robot {
    int x, y;
    Direction dir;
    int battery;
};

extern Robot robot;
extern std::vector<std::vector<bool>> _visited;
extern std::vector<std::vector<bool>> _seen;
extern int numMoves;
extern int numTurns;
extern int numScans;

// Direction delta tables (indexed by Direction enum)
extern const int DX[4];
extern const int DY[4];

char getRobotChar();