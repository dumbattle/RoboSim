#ifndef ROBOT_API_H
#define ROBOT_API_H

#include <string>
#include "robot_params.h"

using namespace std;

// ============================================================
// Robot Exploration Simulator - Public API
// ============================================================
//
// Goal: visit as many unique tiles as possible before battery runs out.
// Score = number of distinct tiles stepped on.
//
// Key constants (see robot_params.h):
//   MAP_WIDTH, MAP_HEIGHT     - map dimensions
//   MAX_BATTERY               - starting battery
//   BATTERY_MOVE              - cost per MoveForward()
//   BATTERY_TURN              - cost per TurnLeft() / TurnRight()
//   BATTERY_QUERY_MIN/MAX     - cost per ScanAhead()
//   WALL_TYPE_COUNT           - number of distinct wall types
//   TILE_TYPE_COUNT           - WALL_TYPE_COUNT + 1  (ID 0 = empty)
//   SEED                      - default map seed
//
// Tips:
//   - Obstacles cluster by type - nearby tiles are likely the same type.
//   - Sensor error rates drift over time; use GetErrorDeltas() to track trends.
//   - Moving into a wall drains damage battery but does not advance the robot.
//   - ScanAhead() is free if the tile is already certain (confidence == 1) or sensor is completely unreliable.


// ============================================================
// Direction
// ============================================================

// Cardinal directions. NORTH = +Y, EAST = +X, SOUTH = -Y, WEST = -X.
enum Direction { NORTH, EAST, SOUTH, WEST };

// Returns the direction 90° to the left / right of d.
Direction Left(Direction d);
Direction Right(Direction d);

// Converts a unit axis-aligned offset (dx, dy) to a Direction.
// Returns false if the vector is not a cardinal direction (e.g. diagonal or zero).
bool TryToDirection(int x, int y, Direction& outDirection);

// Fills (resultX, resultY) with the unit step vector for direction d.
// NORTH -> (0,+1)  EAST -> (+1,0)  SOUTH -> (0,-1)  WEST -> (-1,0)
void ToVector(Direction d, int& resultX, int& resultY);

// Advances (x, y) one step in direction d (equivalent to x+=dx, y+=dy).
void Translate(int& x, int& y, Direction d);

// Returns a human-readable name for d ("North", "East", "South", "West").
std::string ToString(Direction d);


// ============================================================
// Sensor - error model
// ============================================================
//
// Each wall type has independent false-positive and false-negative rates
// that drift over time (range 0–50%). Rates are integers representing percent.
// A rate of 50% is pure noise - ScanAhead() skips automatically.

// Fills (falsePositive, falseNegative) with the current error rates (0–50) for
// the given wall type. wallID: [1, WALL_TYPE_COUNT]. Free - no battery cost.
void GetErrorRates(int wallID, int& falsePositive, int& falseNegative);

// Fills (falsePositiveRate, falseNegativeRate) with the current rate-of-change
// of the error rates (units: % per tick). Positive = error is worsening.
// wallID: [1, WALL_TYPE_COUNT]. Free - no battery cost.
void GetErrorDeltas(int wallID, float& falsePositiveRate, float& falseNegativeRate);


// ============================================================
// Sensor - scanning
// ============================================================

// Scans the tile directly ahead for the presence of wall type wallID.
// Performs a Bayesian update on the tile's confidence distribution.
//
// Costs BATTERY_QUERY_MIN–BATTERY_QUERY_MAX battery (random each call).
// FREE (no battery, no state change) if:
//   - the tile is already certain (any type has confidence == 1), or
//   - the current error rate is 50% (scan would carry no information).
//
// wallID: [1, WALL_TYPE_COUNT]
void ScanAhead(int wallID);


// ============================================================
// Tile confidence
// ============================================================
//
// Each tile stores a probability distribution over TILE_TYPE_COUNT outcomes:
//   ID 0        -> empty
//   ID 1..N     -> wall type N
// Initially uniform. Updated by ScanAhead() and revealed on move.
// If the correct type's confidence crosses a threshold, the tile is fully revealed (snapped to 1/0).

// Returns the Shannon entropy of tile (x, y)'s confidence distribution, normalized to [0, 1].
// 0 = fully certain (one outcome has probability 1).
// 1 = fully uncertain (uniform distribution across all tile types).
float GetEntropy(int x, int y);

// Returns the confidence (0–1) that tile (x, y) is the given type.
// wallID: 0 = empty, 1..WALL_TYPE_COUNT = wall type ID.
float GetTileConfidence(int x, int y, int wallID);

// Fills results[0..TILE_TYPE_COUNT-1] with the full confidence distribution for
// tile (x, y). results[0] = P(empty), results[i] = P(wall type i).
void GetTileConfidence(int x, int y, float (&results)[TILE_TYPE_COUNT]);

// Describes the information value of one ScanAhead() call on a specific tile.
// All entropy deltas normalized to [-1, 1]; probabilities and expected to [0, 1].
struct InfoGain {
    float expected;          // Expected entropy reduction (weighted average)     [0, 1]
    float probPositive;      // P(scan fires - obstacle detected)                 [0, 1]
    float deltaIfPositive;   // Entropy reduction if scan fires                   [-1, 1]
    float probNegative;      // P(scan silent - no obstacle detected)             [0, 1]
    float deltaIfNegative;   // Entropy reduction if scan is silent               [-1, 1]
};

// Returns the expected information gain of scanning tile (x, y) for wall type wallID,
// given the current confidence and live sensor error rates.
// wallID: [1, WALL_TYPE_COUNT]. Pure query - no battery cost, no state changes.
InfoGain GetExpectedInfoGain(int x, int y, int wallID);


// ============================================================
// Actions  (all cost battery)
// ============================================================

// Moves the robot one tile in its current direction. Costs BATTERY_MOVE.
// If the destination is a wall, the wall's damage is also subtracted from battery
// and the robot does NOT advance. Reveals the destination tile's true type.
// Crashes (battery -> 0) if the robot would move off the map.
// Returns 0 if successful, -1 if out of bounds or out of battery,
// or 1..WALL_TYPE_COUNT = the wall type ID that was hit.
int MoveForward();

// Rotates the robot 90° left / right in place. Costs BATTERY_TURN each.
// Does not move the robot forward.
void TurnLeft();
void TurnRight();

// Rotates the robot to face direction d using the minimum number of turns.
// Costs BATTERY_TURN per turn (0, 1, or 2 turns total).
void TurnToDirection(Direction d);


// ============================================================
// Robot state queries  (all free - no battery cost)
// ============================================================

// Fills (x, y) with the robot's current grid position.
void GetPosition(int& x, int& y);

// Returns the robot's remaining battery.
int GetBattery();

// Returns true if battery > 0.
bool HasBattery();

// Returns the direction the robot is currently facing.
Direction GetDirection();

// Returns the number of unique tiles visited so far (the score).
int GetScore();

// Returns true if (x, y) is within map bounds [0, MAP_WIDTH) × [0, MAP_HEIGHT).
bool InRange(int x, int y);

// Returns true if the robot has previously stepped on tile (x, y).
bool TileVisited(int x, int y);

// Returns the battery damage dealt when colliding with wall type wallID.
// wallID: [1, WALL_TYPE_COUNT]
int GetObstacleDamage(int wallID);

// Returns the map dimensions in tiles.
int GetMapWidth();
int GetMapHeight();


// ============================================================
// Boilerplate
// ============================================================

// Initialises the world and robot. Must be called once before anything else.
// Uses SEED from robot_params.h if seed is omitted or negative.
void Reset(long seed = -1);

// Prints final stats and closes the display window. Call at the end of main().
void PrintResults();

#endif

