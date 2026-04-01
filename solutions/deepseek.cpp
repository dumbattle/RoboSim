#include "robot_api.h"
#include "state_machine.h"
#include <vector>
#include <queue>
#include <map>
#include <set>
#include <algorithm>
#include <cmath>

struct Position {
    int x, y;
    Position(int x = 0, int y = 0) : x(x), y(y) {}
    bool operator<(const Position& other) const {
        return x < other.x || (x == other.x && y < other.y);
    }
    bool operator==(const Position& other) const {
        return x == other.x && y == other.y;
    }
};

struct MapCell {
    bool visited;  // robot has physically been here
    bool certain;  // confidence >= threshold
    float confidence[TILE_TYPE_COUNT];
    int mostLikelyType;
    
    MapCell() : visited(false), certain(false), mostLikelyType(0) {
        for (int i = 0; i < TILE_TYPE_COUNT; i++) {
            confidence[i] = 1.0f / TILE_TYPE_COUNT;
        }
    }
};

class ExplorationRobot {
private:
    std::map<Position, MapCell> worldMap;
    std::set<Position> frontier;
    Position robotPos;
    Direction robotDir;
    
    void updateCellConfidence(int x, int y) {
        float maxConf = 0;
        int bestType = 0;
        bool isCertain = false;
        
        for (int i = 0; i < TILE_TYPE_COUNT; i++) {
            float conf = GetTileConfidence(x, y, i);
            worldMap[Position(x, y)].confidence[i] = conf;
            if (conf > maxConf) {
                maxConf = conf;
                bestType = i;
            }
            if (conf >= CONFIDENCE_COMPLETION_THRESH) {
                isCertain = true;
            }
        }
        
        worldMap[Position(x, y)].mostLikelyType = bestType;
        worldMap[Position(x, y)].certain = isCertain;
        
        // If tile is now certain, we don't need to scan it again
        if (isCertain) {
            // Remove from frontier if it was there
            frontier.erase(Position(x, y));
        }
    }
    
    void updateFrontier(int x, int y) {
        Position pos(x, y);
        
        // Don't add visited or certain tiles to frontier
        if (worldMap[pos].visited || worldMap[pos].certain) {
            return;
        }
        
        // Check if this unknown tile is adjacent to a visited tile
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;
                if (!(dx == 0 || dy == 0)) continue;  // Only cardinal directions
                
                Position neighbor(x + dx, y + dy);
                if (worldMap[neighbor].visited) {
                    frontier.insert(pos);
                    return;
                }
            }
        }
    }
    
    bool scanTile(int x, int y) {
        if (!InRange(x, y)) return false;
        
        Position pos(x, y);
        
        // Skip if already certain (scan is free anyway)
        if (worldMap[pos].certain) {
            return true;
        }
        
        // Get the most likely wall type to test
        int bestType = worldMap[pos].mostLikelyType;
        if (bestType == 0) {
            // If most likely is empty, test a random wall type
            bestType = 1;
        }
        
        // Check expected info gain before scanning
        InfoGain gain = GetExpectedInfoGain(x, y, bestType);
        
        // Only scan if expected entropy reduction is significant
        if (gain.expected > 0.1f && HasBattery()) {
            ScanAhead(bestType);
            updateCellConfidence(x, y);
            return true;
        }
        
        return false;
    }
    
    Direction getDirectionTo(Position target) {
        int dx = target.x - robotPos.x;
        int dy = target.y - robotPos.y;
        
        if (dx > 0) return EAST;
        if (dx < 0) return WEST;
        if (dy > 0) return NORTH;
        if (dy < 0) return SOUTH;
        return robotDir;  // Already there
    }
    
    bool moveTo(Position target) {
        if (robotPos.x == target.x && robotPos.y == target.y) {
            return true;
        }
        
        Direction targetDir = getDirectionTo(target);
        
        // Turn to face target direction
        if (robotDir != targetDir) {
            TurnToDirection(targetDir);
            robotDir = GetDirection();
            return false;  // Didn't move this tick
        }
        
        // Try to move forward
        Position nextPos = robotPos;
        Translate(nextPos.x, nextPos.y, robotDir);
        
        if (!InRange(nextPos.x, nextPos.y)) {
            return false;
        }
        
        int moveResult = MoveForward();
        
        if (moveResult == 0) {
            // Successfully moved
            robotPos = nextPos;
            
            // Mark as visited and update
            worldMap[robotPos].visited = true;
            updateCellConfidence(robotPos.x, robotPos.y);
            
            // Update frontier around new position
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (dx == 0 && dy == 0) continue;
                    if (!(dx == 0 || dy == 0)) continue;
                    
                    int nx = robotPos.x + dx;
                    int ny = robotPos.y + dy;
                    if (InRange(nx, ny)) {
                        updateFrontier(nx, ny);
                    }
                }
            }
            return true;
        } else if (moveResult > 0) {
            // Hit a wall
            worldMap[nextPos].visited = false;
            updateCellConfidence(nextPos.x, nextPos.y);
            // Wall is now certain
            frontier.erase(nextPos);
            return false;
        }
        
        return false;
    }
    
    Position selectBestFrontier() {
        if (frontier.empty()) {
            return robotPos;
        }
        
        // Select frontier tile with highest information value
        Position best = *frontier.begin();
        float bestValue = -1;
        
        for (const auto& pos : frontier) {
            float entropy = GetEntropy(pos.x, pos.y);
            int manhattanDist = abs(pos.x - robotPos.x) + abs(pos.y - robotPos.y);
            
            // Value = high uncertainty / distance
            float value = (entropy + 0.1f) / (manhattanDist + 1);
            
            if (value > bestValue) {
                bestValue = value;
                best = pos;
            }
        }
        
        return best;
    }
    
    std::vector<Position> findPath(Position target) {
        std::map<Position, Position> cameFrom;
        std::queue<Position> q;
        std::set<Position> visited;
        
        q.push(robotPos);
        visited.insert(robotPos);
        
        while (!q.empty()) {
            Position current = q.front();
            q.pop();
            
            if (current.x == target.x && current.y == target.y) {
                // Reconstruct path
                std::vector<Position> path;
                Position pos = target;
                while (!(pos.x == robotPos.x && pos.y == robotPos.y)) {
                    path.push_back(pos);
                    pos = cameFrom[pos];
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
            
            // Check all 4 directions
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (dx == 0 && dy == 0) continue;
                    if (!(dx == 0 || dy == 0)) continue;
                    
                    Position neighbor(current.x + dx, current.y + dy);
                    
                    if (!InRange(neighbor.x, neighbor.y)) continue;
                    if (visited.count(neighbor)) continue;
                    
                    // Check if tile is passable (not a wall)
                    if (worldMap[neighbor].certain && 
                        worldMap[neighbor].mostLikelyType != 0) {
                        continue;  // Wall
                    }
                    
                    visited.insert(neighbor);
                    cameFrom[neighbor] = current;
                    q.push(neighbor);
                }
            }
        }
        
        return std::vector<Position>();
    }
    
public:
    ExplorationRobot() {
        int x, y;
        GetPosition(x, y);
        robotPos = Position(x, y);
        robotDir = GetDirection();
        
        // Mark starting position as visited
        worldMap[robotPos].visited = true;
        updateCellConfidence(robotPos.x, robotPos.y);
        
        // Initialize frontier from starting position
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;
                if (!(dx == 0 || dy == 0)) continue;
                
                int nx = robotPos.x + dx;
                int ny = robotPos.y + dy;
                if (InRange(nx, ny)) {
                    updateFrontier(nx, ny);
                }
            }
        }
    }
    
    void tick() {
        // Update position and direction
        GetPosition(robotPos.x, robotPos.y);
        robotDir = GetDirection();
        
        // First, try to scan adjacent frontier tiles
        Position bestFrontier = selectBestFrontier();
        
        // If we're at a frontier tile, scan it
        if (frontier.count(robotPos)) {
            scanTile(robotPos.x, robotPos.y);
        }
        
        // Scan adjacent tiles to the frontier
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;
                if (!(dx == 0 || dy == 0)) continue;
                
                int nx = robotPos.x + dx;
                int ny = robotPos.y + dy;
                Position adj(nx, ny);
                
                if (frontier.count(adj)) {
                    scanTile(nx, ny);
                }
            }
        }
        
        // Move towards the best frontier if we're not already there
        if (bestFrontier.x != robotPos.x || bestFrontier.y != robotPos.y) {
            std::vector<Position> path = findPath(bestFrontier);
            if (!path.empty()) {
                moveTo(path[0]);  // Move one step along the path
            } else {
                // If no path, try to scan to discover path
                // Scan in all directions to find openings
                Direction originalDir = robotDir;
                for (int i = 0; i < 4; i++) {
                    int nx = robotPos.x, ny = robotPos.y;
                    Translate(nx, ny, robotDir);
                    if (InRange(nx, ny)) {
                        scanTile(nx, ny);
                    }
                    TurnRight();
                    robotDir = GetDirection();
                }
                TurnToDirection(originalDir);
                robotDir = GetDirection();
            }
        }
    }
    
    bool hasFrontier() const {
        return !frontier.empty();
    }
};

int main() {
    Reset();
    
    ExplorationRobot robot;
    
    while (HasBattery() && robot.hasFrontier()) {
        robot.tick();
    }
    
    PrintResults();
    return 0;
}