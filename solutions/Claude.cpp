// robot_explore.cpp
//
// ALGORITHM OVERVIEW
// ------------------
// Maintains an internal grid of cell states (UNKNOWN, OPEN, WALL)
// and a visited map tracking which cells the robot has physically stood on.
//
// Each iteration:
//   1. GREEDY STEP — check the 3 immediate neighbors (forward, left, right).
//      If any is unvisited or has unknown neighbors, move there directly
//      without replanning. This keeps the robot moving in straight runs.
//
//   2. REPLAN (only if greedy fails) — run Dijkstra over all reachable OPEN
//      cells to find every frontier (unvisited cell, or OPEN cell with at
//      least one UNKNOWN neighbor), weighted by actual move + turn cost.
//      Navigate to the cheapest frontier using a second Dijkstra for the path.
//
//   3. SENSE — after each move, scan only directions that are still UNKNOWN
//      to minimise wasted turns.
//
// Sensing uses lazy evaluation: sensors are only queried for UNKNOWN cells,
// and the grid is never re-queried for already-known cells.

#include "robot_api.h"
#include "robot_params.h"
#include <vector>
#include <queue>
#include <algorithm>
#include <cstring>

// ---------------------------------------------------------------------------
// Grid — direct map coordinates, no offset needed
// ---------------------------------------------------------------------------
enum CellState : uint8_t { UNKNOWN = 0, OPEN, WALL };

static CellState g_grid[MAP_WIDTH][MAP_HEIGHT];
static bool g_visited[MAP_WIDTH][MAP_HEIGHT];

inline bool InGrid(int x, int y) { return x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT; }

static const int DX[4]          = { 0, 1,  0, -1};
static const int DY[4]          = { 1, 0, -1,  0};
static const Direction DIRS[4]  = {NORTH, EAST, SOUTH, WEST};

// ---------------------------------------------------------------------------
// FaceDirection — turn using actual GetDirection() feedback, no math
// ---------------------------------------------------------------------------
static void FaceDirection(Direction target) {
    for (int i = 0; i < 3 && GetDirection() != target; ++i)
        TurnRight();
    if (GetDirection() != target)
        TurnLeft();
}

// ---------------------------------------------------------------------------
// SenseAhead — query sensor only if UNKNOWN, update grid, return state
// ---------------------------------------------------------------------------
static CellState SenseAhead() {
    int rx, ry; GetPosition(rx, ry);
    // Use our own DX/DY instead of Translate() to avoid axis convention mismatch.
    Direction facing = GetDirection();
    int ax = rx, ay = ry;
    for (int d = 0; d < 4; ++d) {
        if (DIRS[d] == facing) { ax += DX[d]; ay += DY[d]; break; }
    }
    if (!InGrid(ax, ay)) return WALL;
    if (g_grid[ax][ay] == UNKNOWN) {
        if      (IsWallAhead())  g_grid[ax][ay] = WALL;
        else                     g_grid[ax][ay] = OPEN;
    }
    return g_grid[ax][ay];
}

// ---------------------------------------------------------------------------
// SenseAllNeighbors — sense all 4 directions without moving
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// StepInDirection — face, sense, move if clear. Returns success.
// ---------------------------------------------------------------------------
static bool StepInDirection(Direction d) {
    FaceDirection(d);
    if (SenseAhead() != OPEN) return false;
    MoveForward();
    int rx, ry; GetPosition(rx, ry);
    g_grid[rx][ry] = OPEN;
    return true;
}

// ---------------------------------------------------------------------------
// PlanPath — Dijkstra through OPEN cells, penalising turns to prefer
// straight movement. Turn cost = BATTERY_TURN, move cost = BATTERY_MOVE.
// ---------------------------------------------------------------------------
struct PathNode { int x, y, parent; Direction move; int cost; };

static std::vector<Direction> PlanPath(int sx, int sy, int tx, int ty) {
    if (sx == tx && sy == ty) return {};

    static int  best[MAP_WIDTH][MAP_HEIGHT];
    static int  parent[MAP_WIDTH][MAP_HEIGHT];
    static Direction parentDir[MAP_WIDTH][MAP_HEIGHT];
    memset(best,   0x7f, sizeof(best));
    memset(parent, -1,   sizeof(parent));

    // Priority queue: (cost, index) — min-heap
    using T = std::pair<int,int>;
    std::priority_queue<T, std::vector<T>, std::greater<T>> pq;

    std::vector<PathNode> nodes;
    nodes.push_back({sx, sy, -1, GetDirection(), 0});
    best[sx][sy] = 0;
    pq.push({0, 0});

    while (!pq.empty()) {
        auto [cost, idx] = pq.top(); pq.pop();
        PathNode cur = nodes[idx];
        if (cost > best[cur.x][cur.y]) continue; // stale entry

        if (cur.x == tx && cur.y == ty) {
            std::vector<Direction> path;
            int i = idx;
            while (nodes[i].parent != -1) {
                path.push_back(nodes[i].move);
                i = nodes[i].parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int d = 0; d < 4; ++d) {
            int nx = cur.x + DX[d];
            int ny = cur.y + DY[d];
            if (!InGrid(nx, ny))       continue;
            if (g_grid[nx][ny] != OPEN) continue;

            // Turn cost: how many turns to face DIRS[d] from cur.move
            int rightSteps = 0;
            Direction tmp = cur.move;
            while (tmp != DIRS[d] && rightSteps < 4) { tmp = Right(tmp); ++rightSteps; }
            int turnSteps = std::min(rightSteps, 4 - rightSteps);
            int newCost = cur.cost + BATTERY_MOVE + turnSteps * BATTERY_TURN;

            if (newCost < best[nx][ny]) {
                best[nx][ny] = newCost;
                int newIdx = (int)nodes.size();
                nodes.push_back({nx, ny, idx, DIRS[d], newCost});
                pq.push({newCost, newIdx});
            }
        }
    }
    return {};
}

// ---------------------------------------------------------------------------
// FindFrontiers — BFS over OPEN cells, return all frontier cells
// (OPEN cells with at least one UNKNOWN neighbor), sorted nearest-first
// ---------------------------------------------------------------------------
struct Frontier { int x, y, cost; };

static std::vector<Frontier> FindFrontiers(int curX, int curY) {
    // Dijkstra so frontier cost reflects actual turn+move expense
    static int       best[MAP_WIDTH][MAP_HEIGHT];
    static Direction bestFacing[MAP_WIDTH][MAP_HEIGHT];
    memset(best, 0x7f, sizeof(best));

    std::vector<Frontier> result;

    // Store index into nodes vector; priority on cost only
    struct Node { int x, y, cost; Direction facing; };
    std::vector<Node> nodes;
    std::priority_queue<std::pair<int,int>,
                        std::vector<std::pair<int,int>>,
                        std::greater<std::pair<int,int>>> pq;

    Direction startDir = GetDirection();
    best[curX][curY] = 0;
    nodes.push_back({curX, curY, 0, startDir});
    pq.push({0, 0});

    while (!pq.empty()) {
        auto [c, idx] = pq.top(); pq.pop();
        Node cur = nodes[idx];
        if (c > best[cur.x][cur.y]) continue;

        // Frontier if: not yet visited, OR has an UNKNOWN neighbor
        bool isFrontier = !g_visited[cur.x][cur.y];
        if (!isFrontier) {
            for (int d = 0; d < 4; ++d) {
                int nx = cur.x + DX[d];
                int ny = cur.y + DY[d];
                if (InGrid(nx, ny) && g_grid[nx][ny] == UNKNOWN) {
                    isFrontier = true;
                    break;
                }
            }
        }
        if (isFrontier)
            result.push_back({cur.x, cur.y, cur.cost});

        for (int d = 0; d < 4; ++d) {
            int nx = cur.x + DX[d];
            int ny = cur.y + DY[d];
            if (!InGrid(nx, ny))         continue;
            if (g_grid[nx][ny] != OPEN)  continue;
            int rightSteps = 0;
            Direction tmp = cur.facing;
            while (tmp != DIRS[d] && rightSteps < 4) { tmp = Right(tmp); ++rightSteps; }
            int turnSteps = std::min(rightSteps, 4 - rightSteps);
            int newCost = cur.cost + BATTERY_MOVE + turnSteps * BATTERY_TURN;
            if (newCost < best[nx][ny]) {
                best[nx][ny] = newCost;
                int newIdx = (int)nodes.size();
                nodes.push_back({nx, ny, newCost, DIRS[d]});
                pq.push({newCost, newIdx});
            }
        }
    }

    std::sort(result.begin(), result.end(),
        [](const Frontier& a, const Frontier& b){ return a.cost < b.cost; });
    return result;
}

// ---------------------------------------------------------------------------
// GreedyStep — rotate through all 4 directions, sensing as we turn.
// Moves to the first unvisited OPEN neighbor found. Scanning is free
// since SenseAhead() is called regardless as part of the turn loop.
// Returns true if a greedy move was made.
// ---------------------------------------------------------------------------
static bool GreedyStep() {
    for (int i = 0; i < 4; i++) {
        Direction d = GetDirection();
        for(int j=0;j<i;j++) {
            d = Right(d);
        }
        // d = (Direction)i;
        int nx, ny;
        GetPosition(nx, ny);
        Translate(nx, ny, d);

        if (g_grid[nx][ny] == UNKNOWN) {
            FaceDirection(d);
            SenseAhead();
        }
        if (g_grid[nx][ny] == OPEN && !g_visited[nx][ny]) {
            StepInDirection(d);
            g_visited[nx][ny] = true;
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main() {
    Reset();

    int rx, ry;
    GetPosition(rx, ry);
    g_grid[rx][ry] = OPEN;
    g_visited[rx][ry] = true;


    while (true) {
        // Try greedy lookahead before expensive replanning
        if (GreedyStep()) continue;

        GetPosition(rx, ry);
        std::vector<Frontier> frontiers = FindFrontiers(rx, ry);
        if (frontiers.empty()) break;

        bool moved = false;
        for (const Frontier& f : frontiers) {
            std::vector<Direction> path = PlanPath(rx, ry, f.x, f.y);
            if (path.empty()) continue;

            for (Direction d : path)
                StepInDirection(d);

            GetPosition(rx, ry);
            g_visited[rx][ry] = true;
            moved = true;
            break;
        }

        if (!moved) break;
    }

    PrintResults();
    return 0;
}