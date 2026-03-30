#pragma once
#include <iostream>
#include <string>
using namespace std;

static int _passed = 0;
static int _failed = 0;

static void check(const string& name, bool condition) {
    if (condition) { cout << "  [PASS] " << name << "\n"; _passed++; }
    else           { cout << "  [FAIL] " << name << "\n"; _failed++; }
}

static void summary() {
    cout << "\n----------------------------\n";
    cout << _passed << " passed, " << _failed << " failed\n";
    cout << "----------------------------\n";
}
