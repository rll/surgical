#ifndef _CONTROLLER_LQR_AND_PID_H_
#define _CONTROLLER_LQR_AND_PID_H_


#include "shared.h"
#include "util.h"
#include "controller.h"
#include "controller_lqr.h"
#include "controller_pid.h"
#include "accelera_geometry.h"

class Controller_lqr_and_pid : public Controller {
    public:
    
    Controller_lqr* base_controller;
    Controller_pid* wrist_controller;
    
    Accelera_geometry* _geometry;

    Controller_lqr_and_pid(int slave, int statesize, int outputsize, const char* matrixfilepath, const char* targetfilepath, double* kp, double* ki, double* kd);
    ~Controller_lqr_and_pid();

    int     control(const double* state, double* goal, double* out);
    
    void    reset_time();
    bool    is_complete();
};

#endif
