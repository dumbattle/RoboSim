#include "robot_api.h"
#include "robot_params.h"
#include <iostream>
#include <iomanip>

using namespace std;

const int SCANS_PER_TILE = 8;
const int TILES_TO_TEST  = 10;

// ----------------------
// Helpers
// ----------------------

static void printConfidence(int x, int y) {
    float conf[TILE_TYPE_COUNT];
    GetTileConfidence(x, y, conf);

    cout << "    [";
    for (int i = 0; i < TILE_TYPE_COUNT; i++) {
        if (i > 0) cout << "  |  ";
        if (i == 0) cout << "open";
        else        cout << "wall" << (i - 1);
        cout << ": " << setw(5) << fixed << setprecision(1) << conf[i] * 100.0f << "%";
    }
    cout << "]\n";
}

// Turns until an open-looking direction is found, or all 4 are tried
static bool findOpenDirection() {
    for (int attempt = 0; attempt < 4; attempt++) {
        float conf[TILE_TYPE_COUNT];
        int ax, ay;
        GetPosition(ax, ay);
        Translate(ax, ay, GetDirection());

        if (InRange(ax, ay)) {
            GetTileConfidence(ax, ay, conf);

            // If open confidence is highest, try this direction
            bool openLikely = true;
            for (int i = 1; i < TILE_TYPE_COUNT; i++) {
                if (conf[i] >= conf[0]) { openLikely = false; break; }
            }
            if (openLikely) return true;
        }
        TurnRight();
    }
    return false; // surrounded
}

// ----------------------
// Main
// ----------------------

int main() {
    Reset(SEED);

    int tilesVisited = 0;

    while (HasBattery() && tilesVisited < TILES_TO_TEST) {

        // Get tile-ahead coordinates
        int ax, ay;
        GetPosition(ax, ay);
        Translate(ax, ay, GetDirection());

        if (!InRange(ax, ay)) {
            TurnRight();
            continue;
        }

        cout << "\n====================================\n";
        cout << "Tile (" << ax << ", " << ay << ")\n";
        cout << "------------------------------------\n";
        cout << "  Before scans:\n";
        printConfidence(ax, ay);

        // Scan repeatedly, printing confidence after each round
        for (int s = 1; s <= SCANS_PER_TILE && HasBattery(); s++) {
            for (int t = 0; t < WALL_TYPE_COUNT; t++) {
                ScanAhead(t);


                int fp, fn;

                GetErrorRates(t, fp, fn);
                cout << "\n  After scan " << s << "." << t << "  --- P " << fp << "%  --- n " << fn << "%:\n";
                printConfidence(ax, ay);
            }
        }

        // Decide: move forward or turn
        float conf[TILE_TYPE_COUNT];
        GetTileConfidence(ax, ay, conf);

        bool likelyWall = false;
        for (int i = 1; i < TILE_TYPE_COUNT; i++) {
            if (conf[i] > conf[0]) { likelyWall = true; break; }
        }

        if (likelyWall) {
            cout << "  >> Likely wall — turning\n";
            TurnRight();
        } else {
            cout << "  >> Likely open — moving\n";
            MoveForward();
            tilesVisited++;
        }
    }

    cout << "\n====================================\n";
    cout << "Tiles visited: " << tilesVisited << "\n";
    PrintResults();
    return 0;
}
