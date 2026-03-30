#ifndef ROBOT_API_H
#define ROBOT_API_H
#include <string>
using namespace std;

// ----------------------
// Data Helpers
// ----------------------

enum Direction { NORTH, EAST, SOUTH, WEST };

Direction Left(Direction d);
Direction Right(Direction d);

bool TryToDirection(int x, int y, Direction& outDirection);
void ToVector(Direction d, int& resultX, int& resultY);
void Translate(int& x, int& y, Direction d);

std::string ToString(Direction d);

// ----------------------
// Sensors
// ----------------------

// obstacle index in range [0, WALL_TYPE_COUNT)
// error rates in range [0-50]%
void GetErrorRates(int obstacleIndex, int& falsePositive, int& falseNegative);

void GetErrorDeltas(int obstacleIndex, float& falsePositiveRate, float& falseNegativeRate);

// ----------------------
// API
// ----------------------

// Call this at the start of your script
void Reset(long seed = -1);

// Movement - These all cost energy
void MoveForward();
void TurnLeft();    // only turns, does not move forawrd after
void TurnRight();

// Sensors 
// These cost energy, make sure they are not wasted!
bool IsWallAhead(int wallType);

// Info
// Do not cost energy
void GetPosition(int& x, int& y);
int GetBattery();
bool HasBattery();
Direction GetDirection();
int GetScore();

// Call this at end of script
void PrintResults();

#endif

