#include "robot_api.h"
#include "robot_params.h"
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <limits>
#include <functional>

using namespace std;

// Structure to represent a cell in our internal map
struct Cell {
    bool isExplored = false;
    bool isWall = false;
    bool isVisited = false;  // Robot has physically been here
    double uncertainty = 1.0;  // 1 = completely unknown, 0 = fully known
    
    // For pathfinding
    int distance = numeric_limits<int>::max();
    bool inOpenSet = false;
};

// Structure for pathfinding nodes
struct PathNode {
    int x, y;
    int g;  // Cost from start
    int h;  // Heuristic to target
    int f;  // Total cost
    
    PathNode(int x_, int y_, int g_, int h_) : x(x_), y(y_), g(g_), h(h_), f(g_ + h_) {}
    
    bool operator>(const PathNode& other) const {
        return f > other.f;
    }
};

// Structure for exploration targets
struct ExplorationTarget {
    int x, y;
    double infoGain;  // Expected information gain
    int distance;     // Distance from current position
    double value;     // infoGain / distance (or energy cost)
    
    ExplorationTarget(int x_, int y_, double gain, int dist) 
        : x(x_), y(y_), infoGain(gain), distance(dist) {
        value = (dist > 0) ? gain / dist : gain * 100;  // Current position gets boost
    }
    
    bool operator<(const ExplorationTarget& other) const {
        return value < other.value;
    }
};

class AdaptiveExplorer {
private:
    vector<vector<Cell>> map;
    int width, height;
    int robotX, robotY;
    Direction robotDir;
    int battery;
    
    // Exploration parameters
    const double UNCERTAINTY_DECAY = 0.3;  // How much uncertainty reduces when sensed
    const double INFO_GAIN_THRESHOLD = 0.1;  // Minimum info gain to consider target
    const int PLANNING_HORIZON = 30;  // Max steps to plan ahead
    
public:
    AdaptiveExplorer() {
        width = MAP_WIDTH;
        height = MAP_HEIGHT;
        map.resize(height, vector<Cell>(width));
        
        // Initialize all cells as unexplored
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                map[y][x].uncertainty = 1.0;
            }
        }
    }
    
    void updateState() {
        GetPosition(robotX, robotY);
        robotDir = GetDirection();
        battery = GetBattery();
        
        // Mark current cell as visited
        if (isInBounds(robotX, robotY)) {
            map[robotY][robotX].isVisited = true;
            map[robotY][robotX].isExplored = true;
            map[robotY][robotX].uncertainty = 0.0;
        }
    }
    
    bool isInBounds(int x, int y) {
        return x >= 0 && x < width && y >= 0 && y < height;
    }
    
    void senseAhead() {
        if (battery <= 0) return;
        
        // Query the tile ahead
        bool isWall = IsWallAhead();
        
        // Calculate where that tile is
        int aheadX = robotX, aheadY = robotY;
        ToVector(robotDir, aheadX, aheadY);
        Translate(aheadX, aheadY, robotDir);
        
        if (isInBounds(aheadX, aheadY)) {
            map[aheadY][aheadX].isExplored = true;
            map[aheadY][aheadX].isWall = isWall;
            map[aheadY][aheadX].uncertainty = 0.0;
            
            // Propagate uncertainty reduction to neighboring cells
            propagateUncertainty(aheadX, aheadY);
        }
    }
    
    void propagateUncertainty(int x, int y) {
        // When we know a cell, it slightly reduces uncertainty in adjacent cells
        int dx[] = {0, 1, 0, -1};
        int dy[] = {-1, 0, 1, 0};
        
        for (int i = 0; i < 4; i++) {
            int nx = x + dx[i];
            int ny = y + dy[i];
            
            if (isInBounds(nx, ny) && !map[ny][nx].isExplored) {
                // Reduce uncertainty but don't go below threshold
                map[ny][nx].uncertainty = max(UNCERTAINTY_DECAY, 
                                             map[ny][nx].uncertainty * 0.8);
            }
        }
    }
    
    double calculateInfoGain(int x, int y) {
        if (!isInBounds(x, y) || map[y][x].isExplored) {
            return 0.0;
        }
        
        double gain = map[y][x].uncertainty;
        
        // Bonus for exploring frontiers (adjacent to explored areas)
        int dx[] = {0, 1, 0, -1};
        int dy[] = {-1, 0, 1, 0};
        
        for (int i = 0; i < 4; i++) {
            int nx = x + dx[i];
            int ny = y + dy[i];
            
            if (isInBounds(nx, ny) && map[ny][nx].isExplored) {
                gain *= 1.5;  // Frontier bonus
                break;
            }
        }
        
        // Bonus for unexplored areas surrounded by walls (rooms)
        int wallCount = 0;
        for (int i = 0; i < 4; i++) {
            int nx = x + dx[i];
            int ny = y + dy[i];
            
            if (!isInBounds(nx, ny) || (isInBounds(nx, ny) && map[ny][nx].isWall)) {
                wallCount++;
            }
        }
        
        if (wallCount >= 3) {
            gain *= 2.0;  // Potential room discovery
        }
        
        return gain;
    }
    
    vector<ExplorationTarget> findExplorationTargets() {
        vector<ExplorationTarget> targets;
        priority_queue<ExplorationTarget> pq;
        
        // Search in expanding squares around robot
        int searchRadius = min(PLANNING_HORIZON, max(width, height));
        
        for (int dy = -searchRadius; dy <= searchRadius; dy++) {
            for (int dx = -searchRadius; dx <= searchRadius; dx++) {
                int tx = robotX + dx;
                int ty = robotY + dy;
                
                if (!isInBounds(tx, ty) || map[ty][tx].isExplored) {
                    continue;
                }
                
                double gain = calculateInfoGain(tx, ty);
                if (gain < INFO_GAIN_THRESHOLD) {
                    continue;
                }
                
                // Calculate Manhattan distance as heuristic
                int distance = abs(dx) + abs(dy);
                
                // Adjust for walls using BFS from current position
                int actualDist = findPathDistance(tx, ty);
                if (actualDist > 0) {
                    targets.emplace_back(tx, ty, gain, actualDist);
                }
            }
        }
        
        // Sort by value (info gain per distance)
        sort(targets.begin(), targets.end(), 
             [](const ExplorationTarget& a, const ExplorationTarget& b) {
                 return a.value > b.value;
             });
        
        // Return top targets (prune)
        if (targets.size() > 10) {
            targets.resize(10);
        }
        
        return targets;
    }
    
    int findPathDistance(int targetX, int targetY) {
        // Simple A* to find path distance to target
        if (targetX == robotX && targetY == robotY) return 0;
        
        vector<vector<bool>> closed(width, vector<bool>(height, false));
        priority_queue<PathNode, vector<PathNode>, greater<PathNode>> open;
        
        // Heuristic function
        auto heuristic = [&](int x, int y) {
            return abs(x - targetX) + abs(y - targetY);
        };
        
        open.emplace(robotX, robotY, 0, heuristic(robotX, robotY));
        
        int dx[] = {0, 1, 0, -1};
        int dy[] = {-1, 0, 1, 0};
        
        while (!open.empty()) {
            PathNode current = open.top();
            open.pop();
            
            if (current.x == targetX && current.y == targetY) {
                return current.g;
            }
            
            if (closed[current.x][current.y]) continue;
            closed[current.x][current.y] = true;
            
            for (int i = 0; i < 4; i++) {
                int nx = current.x + dx[i];
                int ny = current.y + dy[i];
                
                if (!isInBounds(nx, ny)) continue;
                if (closed[nx][ny]) continue;
                
                // Check if cell is traversable
                if (map[ny][nx].isExplored && map[ny][nx].isWall) continue;
                
                int ng = current.g + 1;
                int nh = heuristic(nx, ny);
                
                open.emplace(nx, ny, ng, nh);
            }
        }
        
        return -1;  // No path found
    }
    
    vector<pair<int, int>> planPath(int targetX, int targetY) {
        vector<pair<int, int>> path;
        
        if (targetX == robotX && targetY == robotY) {
            return path;
        }
        
        // A* to reconstruct path
        vector<vector<pair<int, int>>> parent(width, vector<pair<int, int>>(height, {-1, -1}));
        vector<vector<int>> gScore(width, vector<int>(height, numeric_limits<int>::max()));
        
        priority_queue<PathNode, vector<PathNode>, greater<PathNode>> open;
        
        auto heuristic = [&](int x, int y) {
            return abs(x - targetX) + abs(y - targetY);
        };
        
        gScore[robotX][robotY] = 0;
        open.emplace(robotX, robotY, 0, heuristic(robotX, robotY));
        
        int dx[] = {0, 1, 0, -1};
        int dy[] = {-1, 0, 1, 0};
        
        while (!open.empty()) {
            PathNode current = open.top();
            open.pop();
            
            if (current.x == targetX && current.y == targetY) {
                // Reconstruct path
                int x = targetX, y = targetY;
                while (!(x == robotX && y == robotY)) {
                    path.emplace_back(x, y);
                    auto [px, py] = parent[x][y];
                    x = px; y = py;
                }
                reverse(path.begin(), path.end());
                return path;
            }
            
            for (int i = 0; i < 4; i++) {
                int nx = current.x + dx[i];
                int ny = current.y + dy[i];
                
                if (!isInBounds(nx, ny)) continue;
                if (map[ny][nx].isExplored && map[ny][nx].isWall) continue;
                
                int tentativeG = gScore[current.x][current.y] + 1;
                
                if (tentativeG < gScore[nx][ny]) {
                    parent[nx][ny] = {current.x, current.y};
                    gScore[nx][ny] = tentativeG;
                    open.emplace(nx, ny, tentativeG, heuristic(nx, ny));
                }
            }
        }
        
        return path;  // Empty if no path
    }
    
    void moveAlongPath(const vector<pair<int, int>>& path) {
        for (const auto& [nextX, nextY] : path) {
            if (battery <= 0) return;
            
            // Determine direction to next cell
            int dx = nextX - robotX;
            int dy = nextY - robotY;
            
            Direction targetDir;
            if (dx == 1 && dy == 0) targetDir = EAST;
            else if (dx == -1 && dy == 0) targetDir = WEST;
            else if (dx == 0 && dy == -1) targetDir = NORTH;
            else if (dx == 0 && dy == 1) targetDir = SOUTH;
            else continue;  // Invalid step
            
            // Turn to face correct direction
            while (robotDir != targetDir && battery > 0) {
                // Determine shortest turn
                int diff = (targetDir - robotDir + 4) % 4;
                if (diff <= 2) {
                    TurnRight();
                } else {
                    TurnLeft();
                }
                updateState();
            }
            
            // Move forward
            if (battery > 0) {
                MoveForward();
                updateState();
                
                // Mark new cell as visited
                map[robotY][robotX].isVisited = true;
            }
        }
    }
    
    void run() {
        Reset();
        updateState();
        
        // Main exploration loop
        while (battery > 0) {
            // Sense the tile ahead
            if (battery > 0) {
                senseAhead();
                updateState();
            }
            
            // Find best exploration targets
            auto targets = findExplorationTargets();
            
            if (targets.empty()) {
                // If no good targets, do spiral search pattern
                performSpiralSearch();
                continue;
            }
            
            // Pick best target and plan path
            const auto& best = targets[0];
            auto path = planPath(best.x, best.y);
            
            if (path.empty()) {
                // Mark as unreachable and try next target
                map[best.y][best.x].isWall = true;  // Treat as wall
                continue;
            }
            
            // Move along path, sensing as we go
            moveAlongPath(path);
        }
        
        PrintResults();
    }
    
    void performSpiralSearch() {
        // Simple spiral pattern when no good targets found
        static int spiralStep = 1;
        static int spiralLength = 1;
        static int stepCount = 0;
        static int directionIndex = 0;
        
        int dx[] = {0, 1, 0, -1};
        int dy[] = {-1, 0, 1, 0};
        
        if (battery <= 0) return;
        
        // Sense ahead before moving
        senseAhead();
        updateState();
        
        // Try to move in spiral pattern
        for (int i = 0; i < 4 && battery > 0; i++) {
            int targetDirIdx = (directionIndex + i) % 4;
            Direction targetDir = static_cast<Direction>(targetDirIdx);
            
            // Turn to face direction
            while (robotDir != targetDir && battery > 0) {
                int diff = (targetDir - robotDir + 4) % 4;
                if (diff <= 2) {
                    TurnRight();
                } else {
                    TurnLeft();
                }
                updateState();
            }
            
            // Check if we can move
            if (battery > 0 && !IsWallAhead()) {
                MoveForward();
                updateState();
                
                stepCount++;
                if (stepCount >= spiralLength) {
                    stepCount = 0;
                    directionIndex = (directionIndex + 1) % 4;
                    if (directionIndex % 2 == 0) {
                        spiralLength++;
                    }
                }
                return;
            }
        }
        
        // If stuck, turn randomly
        if (battery > 0) {
            TurnRight();
            updateState();
        }
    }
};

int main() {
    AdaptiveExplorer explorer;
    explorer.run();
    return 0;
}