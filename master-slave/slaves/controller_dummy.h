#ifndef _CONTROLLER_DUMMY_H_
#define _CONTROLLER_DUMMY_H_

#include "shared.h"
#include "controller.h"

class Controller_dummy : public Controller {
public:
    
    Controller_dummy(int slave, int statesize, int outsize);
    
    ~Controller_dummy();

    int control(const double* state, double* goal, double* out);
};
#endif
