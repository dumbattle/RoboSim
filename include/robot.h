#pragma once
#include <vector>
#include "robot_api.h"


struct Robot {
    int x, y;
    Direction dir;
    int battery;
};

struct ErrorData {
    int currentRate;
    int startRate;
    int targetRate;
    int timer;
};

struct SensorError {
    ErrorData falsePositive;
    ErrorData falseNegative;
};

extern Robot robot;
extern std::vector<std::vector<bool>> _visited;
extern std::vector<std::vector<std::vector<float>>> _confidence;

extern std::vector<SensorError> sensorErrors;


extern int numMoves;
extern int numTurns;
extern int numScans;
extern int numCrashes[WALL_TYPE_COUNT];

// Direction delta tables (indexed by Direction enum)
extern const int DX[4];
extern const int DY[4];

char getRobotChar();