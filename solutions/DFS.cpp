#include "../include/robot_api.h"
#include <stack>
#include <vector>
#include <utility>
#include <iostream>
using namespace std;

// Navigate from current position to (tx, ty) using BFS pathfinding.
// Returns false if no path found or battery too low.
bool NavigateTo(int tx, int ty) {
    int sx, sy;
    GetPosition(sx, sy);
    if (sx == tx && sy == ty) return true;

    int w = GetMapWidth(), h = GetMapHeight();

    // BFS over known-safe tiles
    vector<vector<int>> prev(w, vector<int>(h, -1));
    vector<vector<bool>> queued(w, vector<bool>(h, false));

    // Encode direction as 0=N,1=E,2=S,3=W stored as prev value
    // Store parent encoded as (px*h + py)*4 + dir
    struct Cell { int x, y; };
    vector<Cell> frontier;
    frontier.push_back({sx, sy});
    queued[sx][sy] = true;

    // BFS parent tracking: store (parentX, parentY) per cell
    vector<vector<pair<int,int>>> parent(w, vector<pair<int,int>>(h, {-1,-1}));

    bool found = false;
    for (int i = 0; i < (int)frontier.size() && !found; i++) {
        auto [cx, cy] = frontier[i];
        Direction dirs[4] = {NORTH, EAST, SOUTH, WEST};
        for (Direction d : dirs) {
            int nx = cx, ny = cy;
            Translate(nx, ny, d);
            if (!InRange(nx, ny) || queued[nx][ny]) continue;
            // Only traverse tiles that are visited (known safe) or the target
            if (!TileVisited(nx, ny) && !(nx == tx && ny == ty)) continue;
            // Skip if confidence suggests wall (non-empty confidence > 0.5)
            float emptyConf = GetTileConfidence(nx, ny, 0);
            if (emptyConf < 0.4f && !(nx == tx && ny == ty)) continue;

            parent[nx][ny] = {cx, cy};
            queued[nx][ny] = true;
            if (nx == tx && ny == ty) { found = true; break; }
            frontier.push_back({nx, ny});
        }
    }
    if (!found) return false;

    // Reconstruct path
    vector<pair<int,int>> path;
    int cx = tx, cy = ty;
    while (cx != sx || cy != sy) {
        path.push_back({cx, cy});
        auto [px, py] = parent[cx][cy];
        cx = px; cy = py;
    }
    reverse(path.begin(), path.end());

    // Execute path
    for (auto [nx, ny] : path) {
        int rx, ry;
        GetPosition(rx, ry);
        int dx = nx - rx, dy = ny - ry;
        Direction d;
        if (!TryToDirection(dx, dy, d)) return false;
        TurnToDirection(d);
        MoveForward();
        GetPosition(rx, ry);
        if (rx != nx || ry != ny) return false; // hit a wall
        if (!HasBattery()) return false;
    }
    return true;
}

int main() {
    try {
        Reset();

        // DFS stack stores positions to backtrack to
        stack<pair<int, int>> dfsStack;

        int x, y;
        GetPosition(x, y);
        dfsStack.push({x, y});

        while (HasBattery() && !dfsStack.empty()) {
            GetPosition(x, y);

            // Find an unvisited neighbor that looks passable
            Direction dirs[4] = {NORTH, EAST, SOUTH, WEST};
            int bestNx = -1, bestNy = -1;
            Direction bestDir = NORTH;

            for (Direction d : dirs) {
                int nx = x, ny = y;
                Translate(nx, ny, d);
                if (!InRange(nx, ny)) continue;
                if (TileVisited(nx, ny)) continue;

                // Must face direction d before scanning
                TurnToDirection(d);
                for (int i = 0; i < WALL_TYPE_COUNT; i++) {

                    for (size_t s = 0; s < 100; s++) {
                        InfoGain ig = GetExpectedInfoGain(nx, ny, i);
                        if (ig.expected > 0.01f) ScanAhead(i);
                    }
                    
                }

                float emptyConf = GetTileConfidence(nx, ny, 0);
                if (emptyConf >= 0.4f) {
                    bestNx = nx;
                    bestNy = ny;
                    bestDir = d;
                    break; // take first viable unvisited neighbor
                }
            }

            if (bestNx != -1) {
                // Push current position onto stack before moving
                dfsStack.push({x, y});

                TurnToDirection(bestDir);
                MoveForward();

                // Check we actually moved (didn't hit a wall)
                int nx, ny;
                GetPosition(nx, ny);
                if (nx == x && ny == y) {
                    // Didn't move — wall hit, pop the push we just did
                    dfsStack.pop();
                }
            } 
            else {
                // No unvisited neighbors — backtrack
                if (dfsStack.empty()) break;

                auto [bx, by] = dfsStack.top();
                NavigateTo(bx, by);
                dfsStack.pop();
                // if (!NavigateTo(bx, by)) {
                //     // Can't navigate back — try popping more
                //     dfsStack.pop();
                // }
            }
        }

        PrintResults();
            // Code that might throw an exception
    } catch (const std::exception& e) {
       cout << e.what(); 
    } 
        return 0;
}
