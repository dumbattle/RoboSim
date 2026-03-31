#pragma once
#include <string>

// Call once before Reset() – creates the SFML window.
void initDisplay();

// Call once at program end (or on crash) – closes the window.
void closeDisplay();

// Redraws the map + HUD.  Called automatically by MoveForward / Turn* / Reset.
// Also pumps the SFML event queue so the window stays responsive.
void printMap(bool force=false);

// Prints final statistics to stdout and redraws the final frame.
void printStatus();