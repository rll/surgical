#ifndef _CONTROLLER_MOTION_PLANNER_H_
#define _CONTROLLER_MOTION_PLANNER_H_

#include "controller.h"
#include "LowPassFilter.h"
#include "util.h"
#include "controller_pid.h"
#include "accelera_geometry.h"
#include "interpolate_planner.h"
class Controller_motion_planner : public Controller {
public:
    Interpolate_planner* _motion_planner;
    Accelera_geometry* _geometry;
    Controller_pid* _pid_controller;
    
    Controller_motion_planner(int slave_num, int statesize, int outputsize, double* kp, double* ki, double* kd, const double* start_state, const double* goal_state, double duration);

    ~Controller_motion_planner();

    int control(const double* state, double* goal, double* out);
    
    bool is_complete(const double* state); 

};

#endif
