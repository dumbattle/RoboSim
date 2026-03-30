#pragma once
#include "robot_api.h"

// ============================================================
// Debug API  —  for test scripts only, not for benchmark solutions
// ============================================================


// ----------------------
// World / Tile
// ----------------------

// Set a tile directly. obstacleType: -1 = empty, 0+ = obstacle index
void SetTile(int x, int y, int obstacleType);

// Read the raw tile value at (x, y)
int GetTile(int x, int y);

// Fill a rectangular region with a tile type
void FillRect(int x1, int y1, int x2, int y2, int obstacleType);

// Clear a tile (set to empty)
void ClearTile(int x, int y);

// Mark a tile as visited/unvisited in the tracking grid
void SetVisited(int x, int y, bool visited);


// ----------------------
// Confidence
// ----------------------

// Force 100% confidence for one tile type, 0% for all others
void SetConfidenceCertain(int x, int y, int tileTypeIndex);

// Reset a tile's confidence to the uniform prior
void ResetConfidence(int x, int y);

// Reset all tiles' confidence to uniform prior
void ResetAllConfidence();


// ----------------------
// Error Rates
// ----------------------

// Set error rates for an obstacle type directly.
// Locks rates in place (no transitions) until changed again.
// fp / fn in range [0, 50].
void SetErrors(int obstacleIndex, int falsePositive, int falseNegative);

// Set all obstacle error rates to 0
void ClearErrors();


// ----------------------
// Robot State
// ----------------------

// Teleport robot to (x, y). Does not cost battery.
void SetRobotPosition(int x, int y);

// Set robot direction. Does not cost battery.
void SetRobotDirection(Direction d);

// Set battery to an exact value
void SetBattery(int amount);


// ----------------------
// World Inspection
// ----------------------

// Count tiles of a given type across the whole map. -1 = count empty tiles.
int CountTiles(int obstacleType);

// Returns true if the tile ahead of the robot is the given type (bypasses sensor)
bool PeekAhead(int obstacleType);
