/*
The Strategy: "Cluster-Aware Frontier Explorer"

State: DISCOVER: Find the nearest unvisited tile using a BFS (Breadth-First Search) on the known map.

State: SCAN: Before moving into an unknown tile, check if the sensor error is low enough to justify a scan. If we suspect a wall with high collision damage, we scan first.

State: RECOVERY: If we hit a wall, we mark it and recalculate the path.

*/

#include "robot_api.h"
#include "state_machine.h"
#include <vector>
#include <queue>
#include <map>

// Struct to store our internal map data
struct Node {
    int x, y;
    bool operator<(const Node& other) const {
        return x < other.x || (x == other.x && y < other.y);
    }
};

StateMachine sm;
std::map<Node, bool> wallMap; // Persistent memory of where walls are

// Helper: Get coordinate of the tile directly in front
void GetFrontTile(int& fx, int& fy) {
    GetPosition(fx, fy);
    Translate(fx, fy, GetDirection());
}

// Logic: Find the nearest tile that hasn't been visited and isn't a known wall
Direction GetNextStepToFrontier() {
    int startX, startY;
    GetPosition(startX, startY);

    std::queue<std::pair<Node, Direction>> q;
    std::map<Node, bool> visited;

    // Check all 4 adjacent directions to start BFS
    for (int i = 0; i < 4; ++i) {
        Direction d = static_cast<Direction>(i);
        int nx = startX, ny = startY;
        Translate(nx, ny, d);
        if (InRange(nx, ny) && wallMap.find({nx, ny}) == wallMap.end()) {
            q.push({{nx, ny}, d});
            visited[{nx, ny}] = true;
        }
    }

    while (!q.empty()) {
        auto [curr, firstDir] = q.front();
        q.pop();

        if (!TileVisited(curr.x, curr.y)) return firstDir;

        for (int i = 0; i < 4; ++i) {
            Direction d = static_cast<Direction>(i);
            int nx = curr.x, ny = curr.y;
            Translate(nx, ny, d);
            if (InRange(nx, ny) && !visited[{nx, ny}] && wallMap.find({nx, ny}) == wallMap.end()) {
                visited[{nx, ny}] = true;
                q.push({{nx, ny}, firstDir});
            }
        }
    }
    return GetDirection(); // Default
}

void TickExplore() {
    int fx, fy;
    GetFrontTile(fx, fy);

    // 1. Bayesian Safety Check: If the tile ahead is highly uncertain and we have 
    // enough battery, perform a scan to avoid high-damage walls.
    float entropy = GetEntropy(fx, fy);
    if (entropy > 0.5f) {
        // Only scan if error rates aren't pure noise (50%)
        int fp, fn;
        GetErrorRates(1, fp, fn); // Check representative wall type
        if (fp < 45) {
            ScanAhead(1); 
            // If confidence it's a wall is high after scan, don't move.
            if (GetTileConfidence(fx, fy, 0) < 0.3f) {
                TurnRight(); // Pivot to find a new path
                return;
            }
        }
    }

    // 2. Attempt Movement
    int result = MoveForward();
    
    if (result > 0) { // Hit a wall!
        wallMap[{fx, fy}] = true;
        TurnLeft(); // Reroute
    } else if (result == -1) {
        // Edge of world or dead battery
        TurnRight();
    }
    // result == 0 means success, continue in next tick
}

int main() {
    Reset();
    
    sm.Add("explore", TickExplore);
    sm.SetMode("explore");

    while (HasBattery()) {
        // Every few ticks, check if we should change direction based on BFS
        // This keeps us moving toward "Frontiers" (unvisited areas)
        int cx, cy;
        GetPosition(cx, cy);
        
        Direction nextDir = GetNextStepToFrontier();
        if (nextDir != GetDirection()) {
            TurnToDirection(nextDir);
        }

        sm.Tick();
    }

    PrintResults();
    return 0;
}