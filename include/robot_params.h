#ifndef ROBOT_PARAMS_H
#define ROBOT_PARAMS_H


// ----------------------
// Map Parameters
// ----------------------
const int MAP_WIDTH = 600;
const int MAP_HEIGHT = 400;


// GOOD FOR LARGE MAPS
const int WALL_LAYERS[]  = {40, 30, 30};   // % chance

// ----------------------
// Battery Costs
// ----------------------
const int MAX_BATTERY = 1'000'000;        

const int BATTERY_MOVE = 5;           // cost per MoveForward()
const int BATTERY_TURN = 3;           // cost per TurnLeft/TurnRight()
const int BATTERY_QUERY_MIN = 1;      // cost per sensor query
const int BATTERY_QUERY_MAX = 3;      // max cost per sensor query

// ----------------------
// Sensor Error
// ----------------------
const double SENSOR_ERROR_PROB = 0.0;  // probability chance to return incorrect info

// ----------------------
// Animation
// ----------------------

const int SLEEP_MILLISECONDS = 25;  // time between prints
const int   PRINT_INTERVAL = 100;  
const int   TILE_PX        = 2;    // pixels per grid tile

// ----------------------
// Logging
// ----------------------

const int SCORE_REPORT_INTERVAL = -1000;  // reports energy usage after collecting every multiple of this. 
                                          // -1 to disable

// ----------------------
// Seed
// ----------------------

const unsigned int SEED = 31415;



#endif