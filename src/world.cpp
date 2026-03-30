#include "world.h"
#include "robot_params.h"
#include <queue>
#include <cmath>
#include <algorithm>
#include <iterator>

using namespace std;

vector<vector<int>> world;
vector<vector<int>> wallLayers;
mt19937 rng;

// ----------------------
// Utilities
// ----------------------
bool InRange(int x, int y) {
    return x >= 0 && y >= 0 && x < MAP_WIDTH && y < MAP_HEIGHT;
}

static vector<pair<int,int>> neighbors4(int x, int y) {
    vector<pair<int,int>> n;
    const int dx[4] = {1, -1, 0,  0};
    const int dy[4] = {0,  0, 1, -1};
    for (int i = 0; i < 4; i++) {
        int nx = x + dx[i], ny = y + dy[i];
        if (InRange(nx, ny)) n.push_back({nx, ny});
    }
    return n;
}

// ----------------------
// Adjacency helpers
// ----------------------
struct Group {
    vector<pair<int,int>> tiles;
    vector<pair<int,int>> queue;
};

// Returns true if (x,y) has an 8-directional neighbour of `type`,
// ignoring tiles that belong to `group` (pass nullptr to skip that check).
static bool adjacent8(int x, int y, int layer, const Group* group = nullptr) {
    for (int dx = -1; dx <= 1; dx++)
    for (int dy = -1; dy <= 1; dy++) {
        if (dx == 0 && dy == 0) continue;
        int nx = x + dx, ny = y + dy;
        if (!InRange(nx, ny)) continue;
        if (wallLayers[ny][nx] != layer) continue;
        if (group) {
            bool inSameGroup = false;
            for (auto& p : group->tiles)
                if (p.first == nx && p.second == ny) { inSameGroup = true; break; }
            if (inSameGroup) continue;
        }
        return true;
    }
    return false;
}

static void addNeighbors(Group& g, int x, int y) {
    for (auto& p : neighbors4(x, y)) g.queue.push_back(p);
}

// ----------------------
// Tile-type placement
// ----------------------
static void generateType(int layer, int expected, int wallType) {
    vector<Group> groups;

    int seeds = static_cast<int>(sqrt(expected));
    uniform_int_distribution<int> xdist(0, MAP_WIDTH  - 1);
    uniform_int_distribution<int> ydist(0, MAP_HEIGHT - 1);

    for (int i = 0; i < seeds; i++) {
        int x = xdist(rng), y = ydist(rng);
        if (world[y][x] != -1) continue;
        if (adjacent8(x, y, layer)) continue;
        Group g;
        world[y][x] = wallType;
        wallLayers[y][x] = layer;
        g.tiles.push_back({x, y});
        addNeighbors(g, x, y);
        groups.push_back(g);
    }

    int attempts = 0;
    while (attempts < expected) {
        if (groups.empty()) break;

        uniform_int_distribution<int> gdist(0, (int)groups.size() - 1);
        Group& g = groups[gdist(rng)];

        if (g.queue.empty()) { attempts++; continue; }

        uniform_int_distribution<int> qdist(0, (int)g.queue.size() - 1);
        int qi = qdist(rng);
        int x = g.queue[qi].first;
        int y = g.queue[qi].second;
        g.queue.erase(g.queue.begin() + qi);

        if (!InRange(x, y) || world[y][x] != -1 || adjacent8(x, y, layer, &g)) {
            attempts++;
            continue;
        }

        world[y][x] = wallType;
        wallLayers[y][x] = layer;
        g.tiles.push_back({x, y});
        addNeighbors(g, x, y);
        attempts++;
    }
}

// ----------------------
// Map generation
// ----------------------
void generateWorld(unsigned int seed) {
    rng = mt19937(seed);
    world.assign(MAP_HEIGHT, vector<int>(MAP_WIDTH, -1));
    wallLayers.assign(MAP_HEIGHT, vector<int>(MAP_WIDTH, -1));
    int total = MAP_WIDTH * MAP_HEIGHT;

    for (int i = 0; i < size(WALL_LAYERS); i++) {
        generateType(i,  total * WALL_LAYERS[i]  / 100, WALL_LAYER_TYPES[i]);
    }
    
}

// ----------------------
// Random empty tile (largest connected region)
// ----------------------
pair<int,int> randomEmptyTile() {
    vector<vector<bool>> visited(MAP_HEIGHT, vector<bool>(MAP_WIDTH, false));
    vector<pair<int,int>> largestRegion;

    for (int y = 0; y < MAP_HEIGHT; y++)
    for (int x = 0; x < MAP_WIDTH;  x++) {
        if (world[y][x] != -1 || visited[y][x]) continue;

        queue<pair<int,int>> q;
        vector<pair<int,int>> region;
        q.push({x, y});
        visited[y][x] = true;

        while (!q.empty()) {
            pair<int,int> p = q.front(); q.pop();
            int cx = p.first, cy = p.second;
            region.push_back({cx, cy});
            vector<pair<int,int>> nbrs = neighbors4(cx, cy);
            for (size_t i = 0; i < nbrs.size(); i++) {
                int nx = nbrs[i].first, ny = nbrs[i].second;
                if (!visited[ny][nx] && world[ny][nx] == -1) {
                    visited[ny][nx] = true;
                    q.push({nx, ny});
                }
            }
        }

        if (region.size() > largestRegion.size()) largestRegion = region;
    }

    if (largestRegion.empty()) return {-1, -1};
    uniform_int_distribution<int> dist(0, (int)largestRegion.size() - 1);
    return largestRegion[dist(rng)];
}