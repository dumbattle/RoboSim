#ifndef ROBOT_PARAMS_H
#define ROBOT_PARAMS_H


// ----------------------
// Map Parameters
// ----------------------
const int MAP_WIDTH = 150;
const int MAP_HEIGHT = 100;


// GOOD FOR LARGE MAPS
const int WALL_LAYERS[]  = {40, 30, 30};   // % chance

// ----------------------
// Battery Costs
// ----------------------
const int MAX_BATTERY = 50'000;        

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