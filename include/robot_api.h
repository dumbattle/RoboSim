#ifndef ROBOT_API_H
#define ROBOT_API_H

#include <string>
#include "robot_params.h"

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

// obstacleIndex in range [0, WALL_TYPE_COUNT)
// Skips (free) if tile is already certain, or if error rate is 50% (pure noise).
void ScanAhead(int obstacleIndex);

// ----------------------
// Tile Queries
// ----------------------

float GetTileConfidence(int x, int y, int obstacleIndex);

// index 0 => empty tile
// index N => wall type N-1
void GetTileConfidence(int x, int y, float (&results)[TILE_TYPE_COUNT]);


// ----------------------
// API
// ----------------------


// These all cost energy
void MoveForward();
void TurnLeft();    // only turns, does not move forward after
void TurnRight();


// ----------------------
// Robot Queries
// ----------------------
// Do not cost energy

void GetPosition(int& x, int& y);
int GetBattery();
bool HasBattery();
Direction GetDirection();
int GetScore();
bool InRange(int x, int y);
// ----------------------
// Boilerplate
// ----------------------


// Call this at the start of your script
void Reset(long seed = -1);
// Call this at end of script
void PrintResults();

#endif

