#ifndef ROBOT_API_H
#define ROBOT_API_H
#include <string>
using namespace std;

// ----------------------
// Data Helpers
// ----------------------

enum Direction { NORTH, EAST, SOUTH, WEST };

Direction Left(Direction d);
Direction Right(Direction d);

bool TryToDirection(int x, int y, Direction& outDirection);
void ToVector(Direction d, int& resultX, int& resultY);
void Translate(int& x, int& y, Direction d);

std::string ToString(Direction d);

// ----------------------
// Educational
// ----------------------

// any of 4 directions
void RandomSafeMove();

// randomly turn (without moving) or move forward. 
// if forward is blocked, will guarantee turn
// chance from 0-100 %
void ForwardOrLeft(int chanceTurn);
void ForwardOrRight(int chanceTurn); 

// ----------------------
// API
// ----------------------

// Call this at the start of your script
void Reset(long seed = -1);

// Movement - These all cost energy
void MoveForward(); // Ends program if you run into a wall or cliff
void TurnLeft();    // only turns, does not move forawrd after
void TurnRight();

// Sensors 
// These cost energy, make sure they are not wasted!
bool IsWallAhead();

// Info
// Do not cost energy
void GetPosition(int& x, int& y);
int GetBattery();
Direction GetDirection();
int GetScore();

// Call this at end of script
void PrintResults();

// OLD VERSION DO NOT USE
void PrintStatus();
#endif

