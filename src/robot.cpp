#include "robot.h"
#include "robot_params.h"

using namespace std;

Robot robot;
vector<vector<bool>> _visited;
std::vector<std::vector<std::vector<float>>> _confidence;
std::vector<SensorError> sensorErrors(WALL_TYPE_COUNT);

int numMoves = 0;
int numTurns = 0;
int numScans = 0;


char getRobotChar() {
    switch (robot.dir) {
        case NORTH: return '^';
        case EAST:  return '>';
        case SOUTH: return 'v';
        case WEST:  return '<';
    }
    return '?';
}