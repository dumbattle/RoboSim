

#include "robot_api.h"
#include "robot_params.h"
#include <map>
#include <vector>
#include <queue>
#include <cstdlib>
#include <cfloat>
#include <iterator>
#include <algorithm>

// ─────────────────────────────────────────────
//  Tile state
// ─────────────────────────────────────────────
enum TileState {
    UNKNOWN,    // never queried
    SAFE,       // confirmed passable, not yet visited
    COLLECTED,  // visited (stepped on)
    UNSAFE      // wall
};

struct Vec2 { int x, y; };

// ─────────────────────────────────────────────
//  Memory  
// ─────────────────────────────────────────────
static  TileState tileMemory[MAP_WIDTH * MAP_HEIGHT];
// static  std::map<int, TileState> tileMemory;

static int tileKey(int x, int y) { return x * MAP_HEIGHT + y; }

static TileState getState(int x, int y) {
    if (x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT) return UNKNOWN;
    return tileMemory[(tileKey(x, y))];
}

static void setState(int x, int y, TileState s) {
    if (x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT) return;
    tileMemory[tileKey(x, y)] = s;
}


// ─────────────────────────────────────────────
//  Stats
// ─────────────────────────────────────────────
static int exploredCount = 0;  // tiles stepped on (score)
static int seenCount     = 0;  // unique tiles entered into memory
static int stepsTaken    = 0;  // MoveForward() calls

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
        if (mem == COLLECTED || mem == SAFE)   return true;
        if (mem == UNSAFE)                     return false;
        // mem == UNKNOWN: fall through to sensor queries
    }
   
    return !IsWallAhead();
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
                                int curX,   int curY) {
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


// ─────────────────────────────────────────────
//  Returns true if (x,y) has a SAFE or UNKNOWN neighbor
// ─────────────────────────────────────────────
static bool hasFrontierNeighbor(int x, int y) {
    for (int d = 0; d < 4; ++d) {
        int nx = x;
        int ny = y;
        Translate(nx, ny, (Direction)d);

        if (nx < 0 || nx >= MAP_WIDTH || ny < 0 || ny >= MAP_HEIGHT) continue;
        TileState ts = getState(nx, ny);
        if (ts == SAFE || ts == UNKNOWN) return true;
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────
//  Pathfinding data structures
// ─────────────────────────────────────────────────────────────────────

// A frontier group: connected region of SAFE/UNKNOWN tiles
struct FrontierGroup {
    std::vector<Vec2> tiles;  // all SAFE/UNKNOWN tiles in this group
};

// Result of the full pathfinder for one group
struct GroupTarget {
    int       distToGroup;  // BFS steps from robot to nearest group tile
    Direction firstStepDir; // direction of the first step along that path (-1=none)
    int       groupSize;    // number of SAFE/UNKNOWN tiles in group
    double    score;        // heuristic score (higher = better)
};

// Forward declaration — defined at bottom of file for easy tinkering
static double computeHeuristic(int distToGroup, int groupSize,
                                int totalCollected, int totalSeen,
                                int totalStepsTaken);

// ─────────────────────────────────────────────────────────────────────
//  Flood-fill: group all SAFE/UNKNOWN tiles into connected components.
//  Two frontier tiles are in the same group if connected through other
//  SAFE/UNKNOWN tiles (not through COLLECTED — the robot must travel
//  through explored space to reach them, but internally they form one
//  pocket of unexplored territory).
// ─────────────────────────────────────────────────────────────────────
static std::vector<FrontierGroup> buildFrontierGroups() {
    // visited array indexed by tileKey
    // std::map<int,bool> visited;
    int visited[MAP_WIDTH * MAP_HEIGHT] = {0};
  //  bool visited[MAP_WIDTH * MAP_HEIGHT];
    std::vector<FrontierGroup> groups;

    for (int y = 0; y < MAP_HEIGHT; ++y) {
        for (int x = 0; x < MAP_WIDTH; ++x) {
            TileState ts = getState(x, y);
            if (ts != SAFE && ts != UNKNOWN) continue;
            int k = tileKey(x, y);
            if (visited[k]) continue;

            // BFS to collect this connected component
            FrontierGroup grp;
            std::queue<Vec2> q;
            Vec2 start; start.x = x; start.y = y;
            q.push(start);
            visited[k] = true;

            while (!q.empty()) {
                Vec2 cur = q.front(); q.pop();
                grp.tiles.push_back(cur);

                for (int d = 0; d < 4; ++d) {
                    int nx = cur.x;
                    int ny = cur.y ;
                    Translate(nx, ny, (Direction)d);

                    if (nx < 0 || nx >= MAP_WIDTH || ny < 0 || ny >= MAP_HEIGHT) continue;
                    TileState nts = getState(nx, ny);
                    if (nts != SAFE && nts != UNKNOWN) continue;
                    int nk = tileKey(nx, ny);
                    if (visited[nk]) continue;
                    visited[nk] = true;
                    Vec2 nv; nv.x = nx; nv.y = ny;
                    q.push(nv);
                }
            }

            groups.push_back(grp);
        }
    }

    return groups;
}

// ─────────────────────────────────────────────────────────────────────
//  BFS from (startX, startY) over COLLECTED+SAFE tiles.
//  For each frontier group, records:
//    - distToGroup: steps to reach the nearest tile in that group
//    - firstStepDir: direction of the very first step in the path
//
//  Returns false if no group is reachable at all.
//
//  Implementation note: since all edge weights are 1, BFS gives the
//  same optimal distances as Dijkstra at lower cost.
// ─────────────────────────────────────────────────────────────────────
struct BFSNode {
    int x, y;
    int dist;
    Direction firstDir; // direction taken from (startX,startY) to reach this node
};

static bool bfsToGroups(int startX, int startY,
                        const std::vector<FrontierGroup>& groups,
                        std::vector<GroupTarget>& outTargets) {

    if (groups.empty()) return false;

    // Build a reverse-lookup: tileKey -> group index, for O(1) group membership
     std::map<int,int> tileToGroup;
    // bool visited[MAP_WIDTH * MAP_HEIGHT] = {false};
     
    //int tileToGroup[MAP_WIDTH * MAP_HEIGHT];
    for (int gi = 0; gi < (int)groups.size(); ++gi) {
        const FrontierGroup& grp = groups[gi];
        for (int ti = 0; ti < (int)grp.tiles.size(); ++ti) {
            tileToGroup[tileKey(grp.tiles[ti].x, grp.tiles[ti].y)] = gi;
        }
    }

    // Per-group results (dist = -1 means not yet found)
    outTargets.resize(groups.size());
    for (int gi = 0; gi < (int)outTargets.size(); ++gi) {
        outTargets[gi].distToGroup  = -1;
        outTargets[gi].firstStepDir = (Direction)(-1);
        outTargets[gi].groupSize    = (int)groups[gi].tiles.size();
        outTargets[gi].score        = -1.0;
    }

    int groupsFound = 0;

    // BFS — walk on COLLECTED or SAFE tiles (robot's traversable space)
    // std::map<int,bool> visited;
    bool visited[MAP_WIDTH * MAP_HEIGHT] = {false};

  //  int visited[MAP_WIDTH * MAP_HEIGHT];
    std::queue<BFSNode> q;

    // Seed: all 4 neighbors of start that are walkable
    // (start itself is COLLECTED, not in any group)
    visited[tileKey(startX, startY)] = true;

    for (int d = 0; d < 4; ++d) {
        int nx = startX;
        int ny = startY;
        Translate(nx, ny, (Direction)d);


        if (nx < 0 || nx >= MAP_WIDTH || ny < 0 || ny >= MAP_HEIGHT) continue;
        TileState ts = getState(nx, ny);
        if (ts == UNSAFE) continue;
        int nk = tileKey(nx, ny);
        if (visited[nk]) continue;
        visited[nk] = true;
        BFSNode node; node.x = nx; node.y = ny; node.dist = 1; node.firstDir = (Direction)d;
        q.push(node);
    }

    while (!q.empty() && groupsFound < (int)groups.size()) {
        BFSNode cur = q.front(); q.pop();

        int k = tileKey(cur.x, cur.y);
        // Is this tile part of a frontier group?
        
    //    auto git = std::find(std::begin(tileToGroup), std::end(tileToGroup), k);
        std::map<int,int>::iterator git = tileToGroup.find(k);
        if (git != (tileToGroup.end())) {
           // int gi = *git;
            int gi = git->second;
            if (outTargets[gi].distToGroup == -1) {
                outTargets[gi].distToGroup  = cur.dist;
                outTargets[gi].firstStepDir = cur.firstDir;
                ++groupsFound;
            }
            // Don't expand further — group interior is not traversed.
            // (Other groups are reachable only through COLLECTED space,
            //  which is handled when COLLECTED nodes are expanded.)
            continue;
        }

        // Expand neighbors.
        // - COLLECTED tiles: traverse freely (the robot's explored space)
        // - SAFE/UNKNOWN tiles: enqueue so the group-check fires when popped,
        //   but do NOT expand from them (we stop at the group boundary)
        // - UNSAFE: never traverse
        for (int d = 0; d < 4; ++d) {
            int nx = cur.x;
            int ny = cur.y ;
            Translate(nx, ny, (Direction)d);

            if (nx < 0 || nx >= MAP_WIDTH || ny < 0 || ny >= MAP_HEIGHT) continue;
            TileState ts = getState(nx, ny);
            if (ts == UNSAFE) continue;
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
//  Builds groups, runs BFS, scores each group, returns the first-step
//  direction toward the best group.  Returns -1 if nothing reachable.
// ─────────────────────────────────────────────────────────────────────
static Direction computeBestPathStep(int curX, int curY) {
    std::vector<FrontierGroup> groups = buildFrontierGroups();
    if (groups.empty()) return (Direction)(-1);

    std::vector<GroupTarget> targets;
    bool anyReachable = bfsToGroups(curX, curY, groups, targets);
    if (!anyReachable) return (Direction)(-1);

    // Score each reachable group
    Direction bestDir = (Direction)(-1);
    double bestScore = -DBL_MAX;

    for (int i = 0; i < (int)targets.size(); ++i) {
        GroupTarget& t = targets[i];
        if (t.distToGroup == -1) continue; // unreachable

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
//  Runs all 6 sub-steps in order; returns false only when the robot
//  is completely stuck (map exhausted / no reachable frontier).
// ─────────────────────────────────────────────
static bool doExplorationStep(int& curX, int& curY, Direction& curDir) {

    // ── 1. Check FORWARD if unknown ─────────────────────────────────────────
    {
        int fwdX = curX;
        int fwdY = curY ;
        Translate(fwdX, fwdY, curDir);


        if (getState(fwdX, fwdY) == UNKNOWN)
            checkAndRecord(curDir, curDir, curX, curY);
    }


    // ── 3. Move to adjacent SAFE tile ─────────────────────────────────────────
    {
        Direction prio[4];
        prio[0] = curDir;
        prio[1] = Left(curDir);
        prio[2] = Right(curDir);
        prio[3] = Right(Right(curDir));
 

        bool moved = false;
        for (int i = 0; i < 4 && !moved; ++i) {
            Direction md = prio[i];

            int mx = curX;
            int my = curY ;
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

        // ── 4. Probe remaining UNKNOWN neighbors ─────────────────────────
        if (!moved) {
            Direction allDirs[4];
            allDirs[0] = curDir;
            allDirs[1] = Left(curDir);
            allDirs[2] = Right(Right(curDir));
            allDirs[3] = Right(curDir);

            for (int i = 0; i < 4 && !moved; ++i) {
                Direction md = allDirs[i];

                int tx = curX;
                int ty = curY ;
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

        // ── 6. Full pathfinder ─────────────────────────────────────────────────
        //    Greedy failed (no adjacent frontier tile).
        //    Build frontier groups globally, BFS for shortest path
        //    to each, score with heuristic, take one step toward
        //    the best group.  Recalculated every frame.
        if (!moved) {
            Direction bestDir = computeBestPathStep(curX, curY);
            if (bestDir >= 0) {
                int mx = curX;
                int my = curY ;
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

    int curX = 0, curY = 0;
    Direction curDir = EAST;  // assume facing East at start

    GetPosition(curX, curY);
    setState(curX, curY, COLLECTED);
    ++exploredCount;
    ++seenCount;

    while (true) {
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
    // return groupSize;
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