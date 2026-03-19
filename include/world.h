#pragma once
#include <vector>
#include <utility>
#include <random>
#include <cstdint>

// ----------------------
// Tile / Map Types
// ----------------------
enum Tile { EMPTY, WALL };

extern std::vector<std::vector<Tile>> world;
extern std::mt19937 rng;

bool inRange(int x, int y);
void generateWorld(unsigned int seed);

// Returns a random empty tile from the largest connected empty region.
// Returns {-1, -1} if none exists.
std::pair<int,int> randomEmptyTile();