#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <string>
#include <functional>
#include <unordered_map>
#include <stdexcept>

// ============================================================
// StateMachine — lightweight named-state dispatcher
// ============================================================
//
// Usage:
//   StateMachine sm;
//   sm.Add("explore", []{ /* tick logic */ });
//   sm.OnEnter("explore", []{ /* setup */ });
//   sm.OnExit("explore",  []{ /* teardown */ });
//   sm.SetMode("explore");
//
//   while (HasBattery()) sm.Tick();

class StateMachine {
public:
    using Callback = std::function<void()>;

    // Register a tick function for the named mode.
    // Calling Add() a second time with the same name replaces the tick function.
    void Add(const std::string& name, Callback fn);

    // Register an optional enter/exit listener for the named mode.
    // The mode does not need to be Add()-ed before registering listeners.
    void OnEnter(const std::string& name, Callback fn);
    void OnExit(const std::string& name, Callback fn);

    // Activate a mode. Calls the previous mode's OnExit (if any),
    // then the new mode's OnEnter (if any). Throws std::runtime_error
    // if the mode was never registered with Add().
    void SetMode(const std::string& name);

    // Execute the current mode's tick function once.
    // No-op if no mode has been set yet.
    void Tick();
    
private:
    struct State {
        Callback tick;
        Callback onEnter;
        Callback onExit;
    };

    std::unordered_map<std::string, State> _states;
    std::string _current;
};

#endif
