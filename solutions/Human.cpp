#include "robot_api.h"
#include "robot_params.h"
#include <vector>
#include <queue>
#include <cfloat>

// ─────────────────────────────────────────────
//  Tile state
// ─────────────────────────────────────────────
enum TileState {
    UNKNOWN,    // never queried
    SAFE,       // confirmed passable, not yet visited
    COLLECTED,  // visited (stepped on)
    UNSAFE      // wall or cliff
};

struct Vec2 { int x, y; };

// ─────────────────────────────────────────────
//  Tile memory
// ─────────────────────────────────────────────
static TileState tileMemory[MAP_WIDTH * MAP_HEIGHT];

static int tileKey(int x, int y) { return x * MAP_HEIGHT + y; }

static TileState getState(int x, int y) {
    if (x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT) return UNKNOWN;
    return tileMemory[tileKey(x, y)];
}

// Forward declaration — setState also updates frontier bookkeeping
static void setState(int x, int y, TileState s);

// ─────────────────────────────────────────────
//  Stats
// ─────────────────────────────────────────────
static int exploredCount = 0;  // tiles stepped on (score)
static int seenCount     = 0;  // unique tiles entered into memory
static int stepsTaken    = 0;  // MoveForward() calls

// ─────────────────────────────────────────────────────────────────────
//  Frontier group cache
//
//  tileToGroup[k]  : group index owning tile k, or -1
//  groups[]        : stable-index vector; dead slots have tiles.empty()
//  freeList        : indices of dead slots available for reuse
//
//  A group is "dirty" when one or more of its tiles has been collected.
//  Dirty groups are fully discarded and re-flood-filled at the start of
//  the next pathfinder call.
// ─────────────────────────────────────────────────────────────────────
struct FrontierGroup {
    std::vector<Vec2> tiles;      // all SAFE/UNKNOWN tiles in this group
    std::vector<Vec2> dirtyTiles; // tiles collected since last rebuild
    bool dirty;                   // true if any tile was collected

    FrontierGroup() : dirty(false) {}
};

static int                        tileToGroup[MAP_WIDTH * MAP_HEIGHT]; // -1 = none
static std::vector<FrontierGroup> groups;
static std::vector<int>           freeList;

// ─────────────────────────────────────────────
//  Alloc / free a group slot
// ─────────────────────────────────────────────
static int allocGroup() {
    if (!freeList.empty()) {
        int idx = freeList.back();
        freeList.pop_back();
        groups[idx] = FrontierGroup(); // clear the slot
        return idx;
    }
    groups.push_back(FrontierGroup());
    return (int)groups.size() - 1;
}

static void freeGroup(int gi) {
    groups[gi].tiles.clear();
    groups[gi].dirtyTiles.clear();
    groups[gi].dirty = false;
    freeList.push_back(gi);
}

// ─────────────────────────────────────────────
//  Flood-fill a single connected component of SAFE/UNKNOWN tiles
//  starting from (seedX, seedY) and assign it group index gi.
//  Uses the provided visited array (caller owns it).
// ─────────────────────────────────────────────
static void floodFillGroup(int gi, int seedX, int seedY,
                            bool visited[MAP_WIDTH * MAP_HEIGHT]) {
    FrontierGroup& grp = groups[gi];
    std::queue<Vec2> q;
    Vec2 seed; seed.x = seedX; seed.y = seedY;
    q.push(seed);
    int sk = tileKey(seedX, seedY);
    visited[sk] = true;
    tileToGroup[sk] = gi;
    grp.tiles.push_back(seed);

    while (!q.empty()) {
        Vec2 cur = q.front(); q.pop();

        for (int d = 0; d < 4; ++d) {
            int nx = cur.x;
            int ny = cur.y;
            Translate(nx, ny, (Direction)d);

            if (nx < 0 || nx >= MAP_WIDTH || ny < 0 || ny >= MAP_HEIGHT) continue;
            TileState nts = getState(nx, ny);
            if (nts != SAFE && nts != UNKNOWN) continue;
            int nk = tileKey(nx, ny);
            if (visited[nk]) continue;
            visited[nk] = true;
            tileToGroup[nk] = gi;
            Vec2 nv; nv.x = nx; nv.y = ny;
            grp.tiles.push_back(nv);
            q.push(nv);
        }
    }
}

// ─────────────────────────────────────────────
//  Rebuild one dirty group.
//  Clears the old group, then re-flood-fills from any surviving tiles,
//  potentially producing multiple child groups (split detection).
// ─────────────────────────────────────────────
static void rebuildDirtyGroup(int gi) {
    FrontierGroup& grp = groups[gi];

    // Collect surviving (non-COLLECTED, non-UNSAFE) tiles before wiping
    std::vector<Vec2> survivors;
    survivors.reserve(grp.tiles.size());
    for (int i = 0; i < (int)grp.tiles.size(); ++i) {
        Vec2 v = grp.tiles[i];
        TileState ts = getState(v.x, v.y);
        if (ts == SAFE || ts == UNKNOWN) survivors.push_back(v);
    }

    // Clear tile→group mappings for every tile that was in this group
    for (int i = 0; i < (int)grp.tiles.size(); ++i) {
        int k = tileKey(grp.tiles[i].x, grp.tiles[i].y);
        if (tileToGroup[k] == gi) tileToGroup[k] = -1;
    }

    // Release the slot — child groups will claim new indices below
    freeGroup(gi);

    if (survivors.empty()) return; // group fully consumed

    // Pre-mark tiles owned by other (live) groups as visited so the
    // flood-fills below don't steal them
    bool visited[MAP_WIDTH * MAP_HEIGHT] = {false};
    for (int k = 0; k < MAP_WIDTH * MAP_HEIGHT; ++k) {
        if (tileToGroup[k] != -1) visited[k] = true;
    }

    // Flood-fill from each unvisited survivor — each run is a child group
    for (int i = 0; i < (int)survivors.size(); ++i) {
        Vec2 s = survivors[i];
        int sk = tileKey(s.x, s.y);
        if (visited[sk]) continue; // already claimed by an earlier child

        int newGi = allocGroup();
        floodFillGroup(newGi, s.x, s.y, visited);
    }
}

// ─────────────────────────────────────────────
//  Flush all dirty groups before pathfinding.
// ─────────────────────────────────────────────
static void flushDirtyGroups() {
    // Snapshot dirty indices first — rebuilding modifies the groups vector
    std::vector<int> dirty;
    for (int i = 0; i < (int)groups.size(); ++i) {
        if (groups[i].dirty && !groups[i].tiles.empty()) dirty.push_back(i);
    }
    for (int i = 0; i < (int)dirty.size(); ++i) {
        rebuildDirtyGroup(dirty[i]);
    }
}


// ─────────────────────────────────────────────
//  setState — update tile memory and frontier bookkeeping
// ─────────────────────────────────────────────
static void setState(int x, int y, TileState s) {
    if (x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT) return;
    int k = tileKey(x, y);
    TileState prev = tileMemory[k];
    tileMemory[k] = s;

    if (s == SAFE && prev == UNKNOWN) {

    } else if (s == COLLECTED) {
        // Tile consumed — mark its group dirty
        int gi = tileToGroup[k];
        if (gi != -1) {
            Vec2 v; v.x = x; v.y = y;
            groups[gi].dirtyTiles.push_back(v);
            groups[gi].dirty = true;
            tileToGroup[k] = -1;
        }
    }
    // UNSAFE tiles are never in the frontier; nothing to do.
}

// ─────────────────────────────────────────────
//  Safe-check
// ─────────────────────────────────────────────
static bool isTileAheadSafe(int curX, int curY, Direction curDir) {
    int nx = curX;
    int ny = curY;
    Translate(nx, ny, curDir);

    // Memory fast-path: skip sensors if tile state is already known
    if (nx >= 0 && nx < MAP_WIDTH && ny >= 0 && ny < MAP_HEIGHT) {
        TileState mem = getState(nx, ny);
        if (mem == COLLECTED || mem == SAFE) return true;
        if (mem == UNSAFE)                   return false;
        // mem == UNKNOWN: fall through to sensor queries
    }

    if (IsWallAhead())  return false;
    return true;
}

// ─────────────────────────────────────────────
//  Turn to face absDir, update curDir
// ─────────────────────────────────────────────
static void faceDirection(Direction& curDir, Direction absDir) {
    int diff = (absDir - curDir + 4) % 4;
    if      (diff == 1) { TurnRight(); }
    else if (diff == 2) { TurnRight(); TurnRight(); }
    else if (diff == 3) { TurnLeft(); }
    curDir = absDir;
}

// ─────────────────────────────────────────────
//  Check tile in targetAbsDir, turn to face it,
//  record result. Skip if already known.
// ─────────────────────────────────────────────
static TileState checkAndRecord(Direction& curDir, Direction targetAbsDir,
                                int curX, int curY) {
    int nx = curX;
    int ny = curY;
    Translate(nx, ny, targetAbsDir);

    if (nx < 0 || nx >= MAP_WIDTH || ny < 0 || ny >= MAP_HEIGHT)
        return UNSAFE;

    TileState existing = getState(nx, ny);
    if (existing != UNKNOWN) return existing;

    faceDirection(curDir, targetAbsDir);

    TileState result = isTileAheadSafe(curX, curY, curDir) ? SAFE : UNSAFE;
    setState(nx, ny, result);
    if (result == SAFE) ++seenCount;

    return result;
}

// ─────────────────────────────────────────────────────────────────────
//  Pathfinding data structures
// ─────────────────────────────────────────────────────────────────────

// Result of the BFS pathfinder for one group slot
struct GroupTarget {
    int       distToGroup;  // BFS steps from robot to nearest group tile
    Direction firstStepDir; // direction of the first step along that path
    int       groupSize;    // number of SAFE/UNKNOWN tiles in group
    double    score;        // heuristic score (higher = better)
};

// Forward declaration — defined at bottom of file for easy tinkering
static double computeHeuristic(int distToGroup, int groupSize,
                                int totalCollected, int totalSeen,
                                int totalStepsTaken);

// ─────────────────────────────────────────────────────────────────────
//  BFS from (startX, startY) over COLLECTED+SAFE tiles.
//  Uses the global tileToGroup[] array — no rebuild needed here.
//  For each live frontier group, records distToGroup and firstStepDir.
//  Returns false if no group is reachable at all.
// ─────────────────────────────────────────────────────────────────────
struct BFSNode {
    int x, y;
    int dist;
    Direction firstDir;
};

static bool bfsToGroups(int startX, int startY,
                        std::vector<GroupTarget>& outTargets) {

    int numSlots = (int)groups.size();
    outTargets.resize(numSlots);
    int liveCount = 0;
    for (int gi = 0; gi < numSlots; ++gi) {
        outTargets[gi].distToGroup  = -1;
        outTargets[gi].firstStepDir = (Direction)(-1);
        outTargets[gi].groupSize    = (int)groups[gi].tiles.size();
        outTargets[gi].score        = -1.0;
        if (!groups[gi].tiles.empty()) ++liveCount;
    }
    if (liveCount == 0) return false;

    int groupsFound = 0;

    bool visited[MAP_WIDTH * MAP_HEIGHT] = {false};
    std::queue<BFSNode> q;

    visited[tileKey(startX, startY)] = true;

    for (int d = 0; d < 4; ++d) {
        int nx = startX;
        int ny = startY;
        Translate(nx, ny, (Direction)d);

        if (nx < 0 || nx >= MAP_WIDTH || ny < 0 || ny >= MAP_HEIGHT) continue;
        if (getState(nx, ny) == UNSAFE) continue;
        int nk = tileKey(nx, ny);
        if (visited[nk]) continue;
        visited[nk] = true;
        BFSNode node; node.x = nx; node.y = ny; node.dist = 1;
        node.firstDir = (Direction)d;
        q.push(node);
    }

    while (!q.empty() && groupsFound < liveCount) {
        BFSNode cur = q.front(); q.pop();

        int k  = tileKey(cur.x, cur.y);
        int gi = tileToGroup[k];

        if (gi != -1) {
            // Tile belongs to a frontier group — record if first hit
            if (outTargets[gi].distToGroup == -1) {
                outTargets[gi].distToGroup  = cur.dist;
                outTargets[gi].firstStepDir = cur.firstDir;
                ++groupsFound;
            }
            // Do not expand into frontier group interiors
            continue;
        }

        // Expand through COLLECTED space (and un-grouped SAFE tiles)
        for (int d = 0; d < 4; ++d) {
            int nx = cur.x;
            int ny = cur.y;
            Translate(nx, ny, (Direction)d);

            if (nx < 0 || nx >= MAP_WIDTH || ny < 0 || ny >= MAP_HEIGHT) continue;
            if (getState(nx, ny) == UNSAFE) continue;
            int nk = tileKey(nx, ny);
            if (visited[nk]) continue;
            visited[nk] = true;
            BFSNode node; node.x = nx; node.y = ny;
            node.dist     = cur.dist + 1;
            node.firstDir = cur.firstDir;
            q.push(node);
        }
    }

    return groupsFound > 0;
}

// ─────────────────────────────────────────────────────────────────────
//  Full pathfinder entry point.
//  Flushes dirty groups, runs BFS, scores each group, returns the
//  first-step direction toward the best group.
//  Returns -1 if nothing reachable.
// ─────────────────────────────────────────────────────────────────────
static Direction computeBestPathStep(int curX, int curY) {
    flushDirtyGroups();

    std::vector<GroupTarget> targets;
    if (!bfsToGroups(curX, curY, targets)) return (Direction)(-1);

    Direction bestDir   = (Direction)(-1);
    double    bestScore = -DBL_MAX;

    for (int i = 0; i < (int)targets.size(); ++i) {
        GroupTarget& t = targets[i];
        if (t.distToGroup == -1 || t.groupSize == 0) continue;

        t.score = computeHeuristic(t.distToGroup, t.groupSize,
                                   exploredCount, seenCount, stepsTaken);
        if (t.score > bestScore) {
            bestScore = t.score;
            bestDir   = t.firstStepDir;
        }
    }

    return bestDir;
}

// ─────────────────────────────────────────────
//  doExplorationStep()
//  Called once per main-loop iteration.
//  Returns false only when the robot is completely stuck
//  (map exhausted / no reachable frontier).
// ─────────────────────────────────────────────
static bool doExplorationStep(int& curX, int& curY, Direction& curDir) {

    // ── 1. Check forward tile if unknown ────────────────────────────────────
    {
        int fwdX = curX;
        int fwdY = curY;
        Translate(fwdX, fwdY, curDir);

        if (getState(fwdX, fwdY) == UNKNOWN)
            checkAndRecord(curDir, curDir, curX, curY);
    }

    // ── 2. Move to adjacent SAFE tile ───────────────────────────────────────
    {
        Direction prio[4];
        prio[0] = curDir;
        prio[1] = Left(curDir);
        prio[2] = Right(curDir);
        prio[3] = Right(Right(curDir));

        if (rand() % 2 == 0) {
            Direction a = prio[1];
            Direction b = prio[2];
            prio[1] = b;
            prio[2] = a;
        }

        bool moved = false;
        for (int i = 0; i < 4 && !moved; ++i) {
            Direction md = prio[i];
            int mx = curX, my = curY;
            Translate(mx, my, md);

            if (getState(mx, my) == SAFE) {
                faceDirection(curDir, md);
                MoveForward();
                ++stepsTaken;
                curX = mx; curY = my;
                ++exploredCount;
                setState(curX, curY, COLLECTED);
                moved = true;
            }
        }

        // ── 3. Probe remaining UNKNOWN neighbors ─────────────────────────────
        if (!moved) {
            Direction allDirs[4];
            allDirs[0] = curDir;
            allDirs[1] = Left(curDir);
            allDirs[2] = Right(Right(curDir));
            allDirs[3] = Right(curDir);

            if (rand() % 2 == 0) {
                Direction a = prio[1];
                Direction b = prio[3];
                prio[1] = b;
                prio[3] = a;
            }


            for (int i = 0; i < 4 && !moved; ++i) {
                Direction md = allDirs[i];
                int tx = curX, ty = curY;
                Translate(tx, ty, md);

                if (getState(tx, ty) == UNKNOWN) {
                    TileState probed = checkAndRecord(curDir, md, curX, curY);
                    if (probed == SAFE) {
                        MoveForward();
                        ++stepsTaken;
                        curX = tx; curY = ty;
                        ++exploredCount;
                        setState(curX, curY, COLLECTED);
                        moved = true;
                    }
                }
            }
        }

        // ── 4. Full pathfinder ───────────────────────────────────────────────
        //    Greedy failed (no adjacent frontier tile).
        //    Flush dirty groups, BFS to all frontier groups, score with
        //    heuristic, take one step toward the best group.
        if (!moved) {
            Direction bestDir = computeBestPathStep(curX, curY);
            if (bestDir >= 0) {
                int mx = curX, my = curY;
                Translate(mx, my, bestDir);

                // Safety: only step on confirmed COLLECTED tile
                if (getState(mx, my) == COLLECTED) {
                    faceDirection(curDir, bestDir);
                    if (isTileAheadSafe(curX, curY, curDir)) {
                        MoveForward();
                        ++stepsTaken;
                        curX = mx; curY = my;
                        moved = true;
                    }
                    // If not safe: wall between positions, skip without
                    // corrupting the COLLECTED state of the target tile.
                }
            }

            if (!moved) return false; // map fully exhausted or unreachable
        }
    }

    return true;
}

// ─────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────
int main() {
    Reset();

    for (int i = 0; i < MAP_WIDTH * MAP_HEIGHT; ++i) tileToGroup[i] = -1;
    
    bool visited[MAP_WIDTH * MAP_HEIGHT] = {false};
    int newGi = allocGroup();
    floodFillGroup(newGi, 0, 0, visited);

    int curX = 0, curY = 0;
    Direction curDir = EAST;

  

    GetPosition(curX, curY);
    setState(curX, curY, COLLECTED);
    ++exploredCount;
    ++seenCount;

    while (HasBattery()) {
        if (!doExplorationStep(curX, curY, curDir)) break;
    }

    PrintStatus();
    return 0;
}

// ═════════════════════════════════════════════════════════════════════
//  HEURISTIC — edit freely
//
//  Parameters:
//    distToGroup    : BFS steps from robot to nearest tile in this group
//    groupSize      : number of SAFE/UNKNOWN tiles in the group
//    totalCollected : exploredCount (tiles stepped on so far)
//    totalSeen      : seenCount (tiles with known state)
//    totalStepsTaken: stepsTaken (MoveForward calls so far)
//
//  Returns a score; higher = more desirable to visit next.
// ═════════════════════════════════════════════════════════════════════
static double computeHeuristic(int distToGroup, int groupSize,
                                int totalCollected, int totalSeen,
                                int totalStepsTaken) {
    if (distToGroup <= 0 || groupSize <= 0) return -1.0;

    // Yield rate: how many points per seen tile so far
    double yieldRate = (totalSeen > 0)
        ? (double)totalCollected / (double)totalSeen
        : 1.0;

    // Expected collectible points in this group
    double expectedPoints = (double)groupSize * yieldRate;

    // Coverage factor: fraction of map already seen
    // Groups in less-explored areas are proportionally more valuable
    double coverageFactor = (double)totalSeen / (double)(MAP_WIDTH * MAP_HEIGHT);

    // Expected points we will actually collect (scales down if map nearly done)
    double expectedCollect = expectedPoints * coverageFactor;

    // Current efficiency: points per step (avoid div-by-zero)
    double efficiency = (totalStepsTaken > 0 && totalCollected > 0)
        ? (double)totalCollected / (double)totalStepsTaken
        : 1.0;

    // Expected steps: travel + steps to collect the group
    double expectedCollectSteps = (efficiency > 0.0)
        ? expectedCollect / efficiency
        : expectedCollect;
    double expectedSteps = (double)distToGroup + expectedCollectSteps;

    // Heuristic: value per step invested
    if (expectedSteps <= 0.0) return -1.0;
    return expectedCollect / expectedSteps;
}