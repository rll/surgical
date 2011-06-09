#include "controller_generic.h"

Controller_generic::Controller_generic(int slave, int state_size, int output_size, Controller** controllers) : Controller(slave, state_size, output_size) {
    _controllers = controllers;
}

Controller_generic::~Controller_generic() {
    for(int i=0;i<state_size;i++) {
        delete _controllers[i];
    }
}

int Controller_generic::control(const double* state, double* goal, double* out) {
    int out_index = 0;
    for (int i=0;i<state_size;i++) {
        out_index += _controllers[i]->control(state + out_index, goal + out_index, out + out_index);
    }
    last_output.clear();
    for(int i=0;i<out_index;i++) {
        last_output.push_back(out[i]);
    }
    return out_index;
}
