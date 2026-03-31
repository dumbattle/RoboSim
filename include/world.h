#pragma once
#include <vector>
#include <utility>
#include <random>
#include <cstdint>

// ----------------------
// Tile / Map Types
// ----------------------

extern std::vector<std::vector<int>> world;
extern std::vector<std::vector<bool>>   isBorderTile;
extern std::mt19937 rng;

// True for every tile in the largest connected empty region (where the robot can spawn/navigate).
extern std::vector<std::vector<bool>> reachable;

void generateWorld(unsigned int seed);

// Returns a random empty tile from the largest connected empty region.
// Also populates the global `reachable` grid as a side effect.
// Returns {-1, -1} if none exists.
std::pair<int,int> randomEmptyTile();