#include "display.h"
#include "display_colors.h"
#include "robot.h"
#include "world.h"
#include "robot_params.h"

#include <SFML/Graphics.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>
#include <fstream>
using namespace std;

// ----------------------------------------------------------------
// Layout constants  ← tweak to resize / reposition things
// ----------------------------------------------------------------
static const int   HUD_HEIGHT    = 60;    // pixel height of the top HUD strip
static const int   MARGIN        = 6;     // padding inside the HUD
static const int   BAR_SEGMENTS  = 20;    // number of battery bar blocks
static const int   FONT_SIZE_LG  = 16;    // score / stats text
static const int   FONT_SIZE_SM  = 13;    // battery label


// ----------------------------------------------------------------
// Module-level SFML objects
// ----------------------------------------------------------------
static sf::RenderWindow window;
static sf::Font         font;
static bool             fontLoaded = false;

// ----------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------
static int countVisited() {
    int n = 0;
    for (size_t r = 0; r < _visited.size(); r++)
        for (size_t c = 0; c < _visited[r].size(); c++)
            if (_visited[r][c]) n++;
    return n;
}

static void sleep() {
    this_thread::sleep_for(chrono::milliseconds(SLEEP_MILLISECONDS));
}

// Pump SFML events so the OS doesn't think the window has hung.
static void pollEvents() {
    sf::Event event;
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed)
            window.close();
    }
}

// ----------------------------------------------------------------
// HUD drawing
// ----------------------------------------------------------------
static sf::Color batteryColour(double pct) {
    if (pct > 60) return COLOR_BATTERY_HIGH;
    if (pct > 30) return COLOR_BATTERY_MED;
    return COLOR_BATTERY_LOW;
}
static void drawHUD() {
    const int windowW = window.getSize().x;
 
    // Background strip
    sf::RectangleShape hudBg(sf::Vector2f(windowW, HUD_HEIGHT));
    hudBg.setFillColor(sf::Color(30, 30, 30));
    window.draw(hudBg);
 
    // --- Battery bar ---
    double pct    = 100.0 * robot.battery / MAX_BATTERY;
    int    filled = static_cast<int>(round(pct / 100.0 * BAR_SEGMENTS));
    filled = max(0, min(BAR_SEGMENTS, filled));
 
    const int segH   = 18;
    const int segGap  = 2;
    const int labelW  = fontLoaded ? 75 : 0;  // pixels reserved for "Battery:"
    const int pctW    = fontLoaded ? 46 : 0;  // pixels reserved for "100%"
    const int barY    = MARGIN;
    const int barX    = MARGIN + labelW;
 
    // Derive segment width so all BAR_SEGMENTS fit within the window
    int availW = windowW - barX - pctW - MARGIN;
    int segW   = std::max(4, (availW - segGap * (BAR_SEGMENTS - 1)) / BAR_SEGMENTS);
 
    // "Battery:" label
    if (fontLoaded) {
        sf::Text label("Battery:", font, FONT_SIZE_SM);
        label.setFillColor(COLOR_HUD_TEXT);
        label.setPosition(MARGIN, barY - 1);
        window.draw(label);
    }
 
    for (int i = 0; i < BAR_SEGMENTS; i++) {
        sf::RectangleShape seg(sf::Vector2f(segW, segH));
        seg.setPosition(barX + i * (segW + segGap), barY);
        seg.setFillColor(i < filled ? batteryColour(pct) : COLOR_BATTERY_EMPTY);
        window.draw(seg);
    }
 
    // Percentage text
    if (fontLoaded) {
        int pctX = barX + BAR_SEGMENTS * (segW + segGap) + 4;
        sf::Text pctText(to_string(static_cast<int>(pct)) + "%", font, FONT_SIZE_SM);
        pctText.setFillColor(batteryColour(pct));
        pctText.setPosition(pctX, barY - 1);
        window.draw(pctText);
    }
 
    // --- Score ---
    if (fontLoaded) {
        sf::Text score("Score: " + to_string(countVisited()), font, FONT_SIZE_LG);
        score.setFillColor(COLOR_HUD_TEXT);
        score.setPosition(MARGIN, HUD_HEIGHT / 2 + 4);
        window.draw(score);
    }
}
// ----------------------------------------------------------------
// Map drawing
// ----------------------------------------------------------------

// Returns the colour and character glyph for a tile.
static sf::Color tileColour(int x, int y) {
    if (x == robot.x && y == robot.y) return COLOR_ROBOT;
    sf::Color result;

    if (world[y][x] >= 0) {
        result = COLOR_WALL;
    }
    else {
        result = _visited[y][x] ? COLOR_VISITED : COLOR_UNVISITED;
    }
    auto c = _confidence[y][x];

    float max_c = 0;

    for (int i = 0; i < WALL_TYPE_COUNT + 1; i++) {
        max_c = max_c > c[i] ? max_c : c[i];
    }

    const int MIN = 0;
    const int MAX = 255;

    int t = (int)(max_c * (MAX - MIN) + MIN);
    result *= sf::Color(t, t, t);

    return result;
}

static void drawMap() {
    for (int y = 0; y < MAP_HEIGHT; y++) {
        for (int x = 0; x < MAP_WIDTH; x++) {
            sf::RectangleShape tile(sf::Vector2f(TILE_PX - 1, TILE_PX - 1));
            tile.setPosition(x * TILE_PX, HUD_HEIGHT + (MAP_HEIGHT - 1 - y) * TILE_PX);
            tile.setFillColor(tileColour(x, y));
            window.draw(tile);

            // Draw robot direction arrow as a text glyph on top of the tile
            if (fontLoaded && x == robot.x && y == robot.y) {
                sf::Text arrow(string(1, getRobotChar()), font, TILE_PX - 2);
                arrow.setFillColor(sf::Color::Black);
                arrow.setPosition(
                    x * TILE_PX,
                    HUD_HEIGHT + (MAP_HEIGHT - 1 - y) * TILE_PX - 1
                );
                window.draw(arrow);
            }
        }
    }
}

// ----------------------------------------------------------------
// Public API
// ----------------------------------------------------------------
void initDisplay() {
    int winW = MAP_WIDTH  * TILE_PX;
    int winH = MAP_HEIGHT * TILE_PX + HUD_HEIGHT;

    window.create(
        sf::VideoMode(winW, winH),
        "Robot Simulator",
        sf::Style::Titlebar | sf::Style::Close
    );
    window.setFramerateLimit(60);
    // return;
    // Try to load a font; fall back gracefully if not found.
    // Point FONT_PATH at any .ttf on your system, e.g.:
    //   /usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf   (Linux)
    //   C:/Windows/Fonts/consola.ttf                           (Windows)
    //   /System/Library/Fonts/Menlo.ttc                        (macOS)
    const char* FONT_PATH = "C:/Windows/Fonts/consola.ttf";
    fontLoaded = font.loadFromFile(FONT_PATH);

    if (!fontLoaded){
        cerr << "[display] Warning: could not load font at " << FONT_PATH
            << " — text labels will be hidden.\n";  // doesn't print
    }
}

void closeDisplay() {
    if (window.isOpen()) window.close();
}

void printMap() {
    static int requestCount = -1;
    requestCount += 1;
    if (requestCount % PRINT_INTERVAL != 0) {
        return;
    }

    if (!window.isOpen()) return;

    pollEvents();

    window.clear(COLOR_BACKGROUND);
    drawHUD();
    drawMap();
    window.display();

    sleep();
}

void printStatus() {
    // Always write stats to stdout so they survive after the window closes.
    cout 
         << "\nFinal Score: " << countVisited() << " unique tiles visited.\n"
         << "Battery remaining: " << robot.battery << "\n\n"
         << "Steps Taken  : " << numMoves << "\n"
         << "Turns Made   : " << numTurns << "\n"
         << "Tiles Scanned: " << numScans << "\n"
        //  << "\"score\": "<<countVisited()<<", \"steps\": "<<numMoves<<", \"turns\": "<<numTurns<<", \"scans\": "<<numScans<< endl
    ;

    // Draw a final frozen frame.
    printMap();
}