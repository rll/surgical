#ifndef _CONTROLLER_GENERIC_H_
#define _CONTROLLER_GENERIC_H_

#include "shared.h"
#include "controller.h"

class Controller_generic : public Controller {
    public:

    Controller**    _controllers;

    Controller_generic(int slave, int state_size, int output_size, Controller** controllers);
    ~Controller_generic();

    int control(const double* state, double* goal, double* out);

};
#endif
