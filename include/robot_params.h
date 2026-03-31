#ifndef ROBOT_PARAMS_H
#define ROBOT_PARAMS_H
#include <vector>


const float CONFIDENCE_COMPLETION_THRESH = 0.999;
// ----------------------
// Map Parameters
// ----------------------
const int MAP_WIDTH = 150;
const int MAP_HEIGHT = 100;



// ----------------------
// Wall Data
// ----------------------


struct WallData {
    int damage;
    std::vector<std::vector<int>> errorRanges;
    int transitionTime;
    int restPeriod[2];
    int color[3];
};

// const WallData WALL_DATA[] = {
//     {
//         50,                                 // Damage
//         { {-10, 10}, {-5, 25}, {0, 55} },   // errorRanges
//         25,                                 // transitionTime
//         { 15, 25 }                          // restPeriod
//     },
//     {
//         500,                                // Damage
//         { {-10,  5}, {-5, 15}, {0, 35} },   // errorRanges
//         20,                                 // transitionTime
//         { 25, 40 }                          // restPeriod
//     }
// };
const WallData WALL_DATA[] = {
    {
        50,                                 // Damage
        { {0, 25} },                        // errorRanges
        500,                                 // transitionTime
        { 15, 25 },                         // restPeriod
        { 244, 55, 55}                      // Color
    },
    {
        800,                                // Damage
        { {5, 25} },                        // errorRanges
        200,                                 // transitionTime
        { 25, 40 },                         // restPeriod
        { 55, 212, 255}                      // Color
    }
};


const int WALL_TYPE_COUNT = sizeof(WALL_DATA) / sizeof(WALL_DATA[0]);
const int TILE_TYPE_COUNT = WALL_TYPE_COUNT + 1;

// ----------------------
// Walls
// ----------------------

const int WALL_LAYER_TYPES[]  = { 0, 0, 1};   
const int WALL_LAYERS[]       = { 40, 30, 30};   // % chance

// ----------------------
// Battery Costs
// ----------------------
const int MAX_BATTERY = 500'000;        

const int BATTERY_MOVE = 5;           // cost per MoveForward()
const int BATTERY_TURN = 3;           // cost per TurnLeft/TurnRight()
const int BATTERY_QUERY_MIN = 1;      // cost per sensor query
const int BATTERY_QUERY_MAX = 3;      // max cost per sensor query

// ----------------------
// Animation
// ----------------------

const int SLEEP_MILLISECONDS = 25;  // time between prints
const int   PRINT_INTERVAL = 100;  
const int   TILE_PX        = 8;    // pixels per grid tile

// ----------------------
// Logging
// ----------------------

const int SCORE_REPORT_INTERVAL = -1;     // reports energy usage after collecting every multiple of this.
                                          // -1 to disable

// ----------------------
// Seed
// ----------------------

const unsigned int SEED = 31415;



#endif