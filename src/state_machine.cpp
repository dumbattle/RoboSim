#include "state_machine.h"

void StateMachine::Add(const std::string& name, Callback fn) {
    _states[name].tick = fn;
}

void StateMachine::OnEnter(const std::string& name, Callback fn) {
    _states[name].onEnter = fn;
}

void StateMachine::OnExit(const std::string& name, Callback fn) {
    _states[name].onExit = fn;
}

void StateMachine::SetMode(const std::string& name) {
    if (_states.find(name) == _states.end() || !_states[name].tick) {
        throw std::runtime_error("StateMachine: unknown mode \"" + name + "\"");
    }

    // Exit current mode.
    if (!_current.empty()) {
        auto it = _states.find(_current);
        if (it != _states.end() && it->second.onExit)
            it->second.onExit();
    }

    _current = name;

    // Enter new mode.
    if (_states[_current].onEnter)
        _states[_current].onEnter();
}

void StateMachine::Tick() {
    if (_current.empty()) return;
    auto it = _states.find(_current);
    if (it != _states.end() && it->second.tick)
        it->second.tick();
}
