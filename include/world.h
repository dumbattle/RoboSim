#pragma once
#include <vector>
#include <utility>
#include <random>
#include <cstdint>

// ----------------------
// Tile / Map Types
// ----------------------

extern std::vector<std::vector<int>> world;
extern std::mt19937 rng;


void generateWorld(unsigned int seed);

// Returns a random empty tile from the largest connected empty region.
// Returns {-1, -1} if none exists.
std::pair<int,int> randomEmptyTile();