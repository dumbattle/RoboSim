#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <cmath>
#include "robot_api.h"
#include "robot_params.h"

enum GemTile { G_UNKNOWN, G_EMPTY, G_WALL };

// Global state for the controller
GemTile g_grid[MAP_WIDTH][MAP_HEIGHT];

struct Node {
    int x, y;
    int cost;
    bool operator>(const Node& other) const { return cost > other.cost; }
};

// Helper to check bounds
bool inBounds(int x, int y) {
    return x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT;
}

// Find the path to a target using Dijkstra based on energy costs
std::vector<Direction> findPath(int startX, int startY, Direction startDir, int targetX, int targetY) {
    // This is a simplified BFS for pathfinding to keep the logic concise
    // In a real scenario, we'd account for Turn costs in the Dijkstra weights
    struct State { int x, y; std::vector<Direction> path; };
    std::queue<State> q;
    q.push({startX, startY, {}});
    
    bool visited[MAP_WIDTH][MAP_HEIGHT] = {false};
    visited[startX][startY] = true;

    while (!q.empty()) {
        State curr = q.front(); q.pop();
        if (curr.x == targetX && curr.y == targetY) return curr.path;

        for (int i = 0; i < 4; ++i) {
            Direction d = static_cast<Direction>(i);
            int nx = curr.x, ny = curr.y;
            Translate(nx, ny, d);
            if (inBounds(nx, ny) && g_grid[nx][ny] == G_EMPTY && !visited[nx][ny]) {
                visited[nx][ny] = true;
                std::vector<Direction> nextPath = curr.path;
                nextPath.push_back(d);
                q.push({nx, ny, nextPath});
            }
        }
    }
    return {};
}

void FaceDirection(Direction target) {
    while (GetDirection() != target) {
        // Optimization: decide best turn direction
        int diff = (target - GetDirection() + 4) % 4;
        if (diff == 3) TurnLeft();
        else TurnRight();
    }
}

int main() {
    Reset();
    
    // Initialize map
    for(int i=0; i<MAP_WIDTH; ++i)
        for(int j=0; j<MAP_HEIGHT; ++j) g_grid[i][j] = G_UNKNOWN;

    int curX, curY;
    GetPosition(curX, curY);
    g_grid[curX][curY] = G_EMPTY;

    while (HasBattery()) {
        GetPosition(curX, curY);
        
        // 1. Identify Frontiers
        std::vector<std::pair<int, int>> frontiers;
        for (int x = 0; x < MAP_WIDTH; ++x) {
            for (int y = 0; y < MAP_HEIGHT; ++y) {
                if (g_grid[x][y] == G_EMPTY) {
                    for (int i = 0; i < 4; ++i) {
                        int nx = x, ny = y;
                        Translate(nx, ny, static_cast<Direction>(i));
                        if (inBounds(nx, ny) && g_grid[nx][ny] == G_UNKNOWN) {
                            frontiers.push_back({nx, ny});
                            break; 
                        }
                    }
                }
            }
        }

        if (frontiers.empty()) break; // Map fully explored or inaccessible

        // 2. Select best frontier (Nearest-Neighbor with a slight bias for centers)
        std::pair<int, int> bestTarget = {-1, -1};
        int minCost = 1e9;
        
        for (auto& f : frontiers) {
            int dist = abs(f.first - curX) + abs(f.second - curY);
            if (dist < minCost) {
                minCost = dist;
                bestTarget = f;
            }
        }

        // 3. Travel to the tile ADJACENT to the frontier
        // Find the EMPTY neighbor of the target that is closest to the robot
        int adjX = -1, adjY = -1;
        Direction lookDir = NORTH;
        int bestAdjDist = 1e9;
        for(int i=0; i<4; ++i) {
            int tx = bestTarget.first, ty = bestTarget.second;
            Direction d = static_cast<Direction>(i);
            Direction opp = static_cast<Direction>((i + 2) % 4);
            Translate(tx, ty, opp); // step back from unknown tile to find empty neighbor
            if (inBounds(tx, ty) && g_grid[tx][ty] == G_EMPTY) {
                int dist = abs(tx - curX) + abs(ty - curY);
                if (dist < bestAdjDist) {
                    bestAdjDist = dist;
                    adjX = tx; adjY = ty;
                    lookDir = d;
                }
            }
        }

        // If robot is already on the adjacent tile, skip pathfinding
        if (adjX == curX && adjY == curY) {
            // Already here, just sense
        } else if (adjX != -1) {
            std::vector<Direction> path = findPath(curX, curY, GetDirection(), adjX, adjY);
            for (Direction d : path) {
                FaceDirection(d);
                if (GetBattery() < BATTERY_MOVE) break;
                MoveForward();
            }
        } else {
            // No reachable adjacent tile, skip this frontier
            g_grid[bestTarget.first][bestTarget.second] = G_WALL;
            continue;
        }

        // Update position after moving
        GetPosition(curX, curY);
        g_grid[curX][curY] = G_EMPTY;

        // 4. Sense the frontier
        FaceDirection(lookDir);
        if (IsWallAhead()) {
            g_grid[bestTarget.first][bestTarget.second] = G_WALL;
        } else {
            g_grid[bestTarget.first][bestTarget.second] = G_EMPTY;
            // If it's not a wall, we might as well move into it to save a future "search"
            if (GetBattery() > BATTERY_MOVE) MoveForward();
        }
    }

    PrintResults();
    return 0;
}