#pragma once
#include <SFML/Graphics.hpp>

// ============================================================
// Tile colours  ← tweak these to experiment
// ============================================================
const sf::Color COLOR_BACKGROUND  = sf::Color(18,  18,  18);   // near-black canvas
const sf::Color COLOR_ROBOT       = sf::Color(222,  234, 255);    // bright green
const sf::Color COLOR_WALL        = sf::Color(210, 60, 60);   // light grey
const sf::Color COLOR_CLIFF       = sf::Color(220, 50,  50);    // red
const sf::Color COLOR_TRAP        = sf::Color(230, 40, 40);    // amber
const sf::Color COLOR_VISITED     = sf::Color(60,  180, 100);   // muted blue
const sf::Color COLOR_UNVISITED   = sf::Color(50,  50,  50);    // dark grey dot

// ============================================================
// HUD colours  ← tweak these to experiment
// ============================================================
const sf::Color COLOR_HUD_TEXT        = sf::Color(220, 220, 220);
const sf::Color COLOR_BATTERY_HIGH    = sf::Color(50,  220, 80);    // > 60 %
const sf::Color COLOR_BATTERY_MED     = sf::Color(230, 180, 30);    // 30–60 %
const sf::Color COLOR_BATTERY_LOW     = sf::Color(220, 50,  50);    // < 30 %
const sf::Color COLOR_BATTERY_EMPTY   = sf::Color(60,  60,  60);    // unfilled segment