#ifndef _INTERPOLATE_PLANNER_H_
#define _INTERPOLATE_PLANNER_H_

#include "motion_planner.h"

class Interpolate_planner : public Motion_planner{

    public:
    
    double _duration;
    
    Interpolate_planner(int slave_num, int statesize, const double* start_state, const double* goal_state, double time_usec);
    ~Interpolate_planner();
    
    void get_next_state(const double* cur_state, double* next_state);
    void get_trajectory_at_time(double* state, timeval state_time); 
    bool is_complete(const double* state);
    

};

#endif
