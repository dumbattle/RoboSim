#include "robot_api.h"
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <limits>
#include <cstring>

// ============================================================
// Tile queries
// ============================================================

static bool IsRevealed(int x, int y) {
    return GetEntropy(x, y) < 0.01f;
}

static float PEmpty(int x, int y) {
    return GetTileConfidence(x, y, 0);
}

static bool IsKnownEmpty(int x, int y) {
    return IsRevealed(x, y) && PEmpty(x, y) > 0.5f;
}

static bool IsKnownWall(int x, int y) {
    return IsRevealed(x, y) && PEmpty(x, y) < 0.5f;
}

static bool IsSafeToWalk(int x, int y) {
    return TileVisited(x, y) || IsKnownEmpty(x, y);
}

// ============================================================
// Rejected tiles — never retry these.
// ============================================================

static bool rejected[MAP_WIDTH][MAP_HEIGHT];

static bool IsDone(int x, int y) {
    return TileVisited(x, y) || IsKnownWall(x, y) || rejected[x][y];
}

static void Reject(int x, int y) {
    rejected[x][y] = true;
}

// ============================================================
// Move helper — ALWAYS marks rejected on failure.
// ============================================================

static bool TryMove(int tx, int ty) {
    int rx, ry;
    GetPosition(rx, ry);

    Translate(rx, ry, GetDirection());

    if (IsKnownWall(rx, ry)) {
        return false;
    }
    MoveForward();

    GetPosition(rx, ry);

    if (rx == tx && ry == ty) {
        return true;
    }
    // Failed — wall. Mark so we never retry.
    return false;
}

// ============================================================
// Scanning
// ============================================================

static float MaxObstacleDamage() {
    float mx = 0;
    for (int i = 1; i <= WALL_TYPE_COUNT; i++) {
        float d = (float)GetObstacleDamage(i);
        if (d > mx) mx = d;
    }
    return mx;
}

// Robot must be facing (tx,ty). Returns true = enter.
// On false, marks tile as rejected.
static bool ScanAndDecide(int tx, int ty) {
    if (!InRange(tx, ty)) return false;

    // Already resolved — use what we know.
    if (IsKnownWall(tx, ty)) { Reject(tx, ty); return false; }
    if (IsKnownEmpty(tx, ty)) return true;
    if (rejected[tx][ty]) return false;

    float maxDamage = MaxObstacleDamage();

    // Walls are cheap — just walk in.
    if (maxDamage <= BATTERY_MOVE) return true;

    // Scan until fully revealed or no info gain left.
    for (int iter = 0; iter < 200; iter++) {
        if (!HasBattery()) return false;
        if (IsRevealed(tx, ty)) {
            if (IsKnownEmpty(tx, ty)) return true;
            Reject(tx, ty);
            return false;
        }

        int bestObs = -1;
        float bestGain = 0.0f;
        for (int i = 1; i <= WALL_TYPE_COUNT; i++) {
            InfoGain ig = GetExpectedInfoGain(tx, ty, i);
            if (ig.expected > bestGain) {
                bestGain = ig.expected;
                bestObs = i;
            }
        }

        if (bestObs < 0 || bestGain < 0.01f) break;
        ScanAhead(bestObs);
    }

    // for (size_t s = 0; s < 100; s++) {
    //     for (int i = 0; i < WALL_TYPE_COUNT; i++) {

    //         InfoGain ig = GetExpectedInfoGain(tx, ty, i);
    //         if (ig.expected > 0.01f) ScanAhead(i);
    //     }
        
    // }


    // Check if scanning resolved it.
    if (IsRevealed(tx, ty)) {
        if (IsKnownEmpty(tx, ty)) return true;
        Reject(tx, ty);
        return false;
    }

    // Still uncertain — decide based on risk.
    float pE = PEmpty(tx, ty);
    if (pE > 0.4f) return true;
    if (maxDamage < BATTERY_MOVE * 3 && pE > 0.25f) return true;

    Reject(tx, ty);
    return false;
}

// ============================================================
// BFS — single flood from robot.
// ============================================================

struct Pos { int x, y; };

static int distMap[MAP_WIDTH][MAP_HEIGHT];

static void BFSFromRobot() {
    int rx, ry;
    GetPosition(rx, ry);
    memset(distMap, -1, sizeof(distMap));

    std::queue<Pos> q;
    distMap[rx][ry] = 0;
    q.push({rx, ry});

    while (!q.empty()) {
        Pos cur = q.front(); q.pop();
        for (int d = 0; d < 4; d++) {
            int nx = cur.x, ny = cur.y;
            Translate(nx, ny, (Direction)d);
            if (!InRange(nx, ny) || distMap[nx][ny] >= 0) continue;
            if (!IsSafeToWalk(nx, ny)) continue;
            distMap[nx][ny] = distMap[cur.x][cur.y] + 1;
            q.push({nx, ny});
        }
    }
}

static std::vector<Pos> PathFromDistMap(int gx, int gy) {
    if (distMap[gx][gy] < 0) return {};

    std::vector<Pos> path;
    int cx = gx, cy = gy;
    while (distMap[cx][cy] > 0) {
        path.push_back({cx, cy});
        for (int d = 0; d < 4; d++) {
            int nx = cx, ny = cy;
            Translate(nx, ny, (Direction)d);
            if (InRange(nx, ny) && distMap[nx][ny] == distMap[cx][cy] - 1) {
                cx = nx; cy = ny;
                break;
            }
        }
    }
    std::reverse(path.begin(), path.end());
    return path;
}

static bool WalkPath(const std::vector<Pos>& path) {
    for (auto& p : path) {
        if (!HasBattery()) return false;
        int rx, ry;
        GetPosition(rx, ry);
        if (rx == p.x && ry == p.y) continue;

        Direction dir;
        int dx = p.x - rx, dy = p.y - ry;
        if (!TryToDirection(dx, dy, dir)) return false;

        TurnToDirection(dir);
        TryMove(0,0);
    }
    return true;
}

// ============================================================
// Local flood-fill
// ============================================================

static float LocalTileScore(int rx, int ry, int nx, int ny, Direction currentDir) {
    float score = PEmpty(nx, ny) * 10.0f;

    int unknownNeighbors = 0;
    for (int d = 0; d < 4; d++) {
        int nnx = nx, nny = ny;
        Translate(nnx, nny, (Direction)d);
        if (InRange(nnx, nny) && !IsDone(nnx, nny))
            unknownNeighbors++;
    }
    score += unknownNeighbors * 2.0f;

    Direction toNeighbor;
    int dx = nx - rx, dy = ny - ry;
    if (TryToDirection(dx, dy, toNeighbor) && toNeighbor == currentDir)
        score += 1.5f;

    return score;
}

static int FloodFillLocal() {
    int totalNew = 0;

    while (HasBattery() && GetBattery() >= BATTERY_MOVE + BATTERY_TURN) {
        int rx, ry;
        GetPosition(rx, ry);
        Direction curDir = GetDirection();

        struct Candidate { Direction dir; int x, y; float score; };
        std::vector<Candidate> cands;

        for (int d = 0; d < 4; d++) {
            int nx = rx, ny = ry;
            Translate(nx, ny, (Direction)d);
            if (!InRange(nx, ny)) continue;
            if (IsDone(nx, ny)) continue;

            float sc = LocalTileScore(rx, ry, nx, ny, curDir);
            cands.push_back({(Direction)d, nx, ny, sc});
        }

        if (cands.empty()) break;

        std::sort(cands.begin(), cands.end(),
                  [](const Candidate& a, const Candidate& b) { return a.score > b.score; });

        bool moved = false;
        for (auto& c : cands) {
            if (!HasBattery() || GetBattery() < BATTERY_MOVE + BATTERY_TURN) break;

            TurnToDirection(c.dir);

            if (!ScanAndDecide(c.x, c.y)) continue;  // marks rejected on failure

            if (TryMove(c.x, c.y)) {  // marks rejected on failure
                totalNew++;
                moved = true;
                break;
            }
        }

        if (!moved) break;
    }

    return totalNew;
}

// ============================================================
// Frontier detection — closest first.
// ============================================================

struct Frontier {
    int x, y;
    int accessX, accessY;
    int accessDist;
};

static std::vector<Frontier> FindFrontiers() {
    BFSFromRobot();

    std::vector<Frontier> frontiers;
    static bool seen[MAP_WIDTH][MAP_HEIGHT];
    memset(seen, false, sizeof(seen));

    // Walk reachable tiles, find unknown non-done neighbors.
    for (int x = 0; x < MAP_WIDTH; x++) {
        for (int y = 0; y < MAP_HEIGHT; y++) {
            if (distMap[x][y] < 0) continue;

            for (int d = 0; d < 4; d++) {
                int nx = x, ny = y;
                Translate(nx, ny, (Direction)d);
                if (!InRange(nx, ny)) continue;
                if (IsDone(nx, ny)) continue;

                // Deduplicate: keep closest access point per target.
                if (seen[nx][ny]) continue;

                // Check if this is the closest access or first we've found.
                seen[nx][ny] = true;
                frontiers.push_back({nx, ny, x, y, distMap[x][y]});
            }
        }
    }

    // Sort by access distance — closest first.
    std::sort(frontiers.begin(), frontiers.end(),
              [](const Frontier& a, const Frontier& b) {
                  return a.accessDist < b.accessDist;
              });

    return frontiers;
}

// Navigate to frontier, enter, flood-fill.
static int CommitToFrontier(const Frontier& f) {
    if (IsDone(f.x, f.y)) return 0;

    auto path = PathFromDistMap(f.accessX, f.accessY);

    int tripCost = ((int)path.size() + 1) * BATTERY_MOVE + BATTERY_TURN * 2;
    if (GetBattery() < tripCost + BATTERY_MOVE * 2) return 0;

    if (!WalkPath(path)) return 0;
    if (!HasBattery()) return 0;

    if (IsDone(f.x, f.y)) return 0;

    Direction dir;
    int dx = f.x - f.accessX, dy = f.y - f.accessY;
    if (!TryToDirection(dx, dy, dir)) return 0;

    TurnToDirection(dir);

    if (!ScanAndDecide(f.x, f.y)) return 0;

    if (TryMove(f.x, f.y)) {
        return 1 + FloodFillLocal();
    }
    return 0;
}

// ============================================================
// Main loop
// ============================================================

int main() {
    Reset();
    memset(rejected, false, sizeof(rejected));

    while (HasBattery() && GetBattery() >= BATTERY_MOVE + BATTERY_TURN) {

        FloodFillLocal();

        if (!HasBattery() || GetBattery() < BATTERY_MOVE + BATTERY_TURN) break;

        auto frontiers = FindFrontiers();
        if (frontiers.empty()) break;

        // Frontiers are sorted closest-first.
        bool madeProgress = false;
        for (int i = 0; i < (int)frontiers.size() && HasBattery(); i++) {
            if (IsDone(frontiers[i].x, frontiers[i].y)) continue;

            int minCost = (frontiers[i].accessDist + 1) * BATTERY_MOVE + BATTERY_TURN;
            if (GetBattery() < minCost + BATTERY_MOVE) continue;

            int gained = CommitToFrontier(frontiers[i]);
            if (gained > 0) {
                madeProgress = true;
                break;
            }
        }

        // Fallback: force-enter closest non-done frontier.
        if (!madeProgress) {
            // Recompute BFS since we may have moved during failed commits.
            BFSFromRobot();

            for (int i = 0; i < (int)frontiers.size() && HasBattery(); i++) {
                if (IsDone(frontiers[i].x, frontiers[i].y)) continue;

                // Recheck reachability with fresh BFS.
                if (distMap[frontiers[i].accessX][frontiers[i].accessY] < 0) continue;

                int minCost = (distMap[frontiers[i].accessX][frontiers[i].accessY] + 1)
                            * BATTERY_MOVE + BATTERY_TURN;
                if (GetBattery() < minCost + BATTERY_MOVE) continue;

                auto path = PathFromDistMap(frontiers[i].accessX, frontiers[i].accessY);
                if (!WalkPath(path)) continue;
                if (!HasBattery()) break;

                if (IsDone(frontiers[i].x, frontiers[i].y)) continue;

                Direction dir;
                int dx = frontiers[i].x - frontiers[i].accessX;
                int dy = frontiers[i].y - frontiers[i].accessY;
                if (!TryToDirection(dx, dy, dir)) continue;
                TurnToDirection(dir);

                TryMove(frontiers[i].x, frontiers[i].y);  // marks rejected on failure
                break;  // Either entered or rejected — re-loop to re-evaluate.
            }
        }
    }

    PrintResults();
    return 0;
}