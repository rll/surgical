#include "controller_dummy.h"

Controller_dummy::Controller_dummy(int slave, int statesize, int outsize) : Controller(slave, statesize, outsize) { }

Controller_dummy::~Controller_dummy() { }

int Controller_dummy::control(const double* state, double* goal, double* out) {
    last_output.clear();
    for(int i=0;i<output_size;i++) {
        out[i] = goal[i];
        last_output.push_back(out[i]);
    }
    return output_size;
}
