#pragma once
#include "robot_api.h"

// ============================================================
// Debug API  —  for test scripts only, not for benchmark solutions
// ============================================================


// ----------------------
// World / Tile
// ----------------------

// Set a tile directly. wallID: 0 = empty, 1..WALL_TYPE_COUNT = wall type ID
void SetTile(int x, int y, int wallID);

// Read the raw tile value at (x, y)
int GetTile(int x, int y);

// Fill a rectangular region with a tile type
void FillRect(int x1, int y1, int x2, int y2, int wallID);

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

// Set error rates for a wall type directly.
// Locks rates in place (no transitions) until changed again.
// fp / fn in range [0, 50].
void SetErrors(int wallID, int falsePositive, int falseNegative);

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

// Count tiles of a given wall ID across the whole map. 0 = count empty tiles.
int CountTiles(int wallID);

// Returns true if the tile ahead of the robot has the given wall ID (bypasses sensor)
bool PeekAhead(int wallID);
